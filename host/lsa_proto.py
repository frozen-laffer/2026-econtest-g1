# -*- coding: utf-8 -*-
"""
lsa_proto.py —— 与 STM32 主控板的 TCP 命令/数据中心

板卡默认地址: 192.168.57.200:5000
协议（全部小端 / little-endian）

命令帧 16 字节
    0..3   magic  'LSCO'  = 0x4F43534C
    4      cmd     1=PING 2=CAPTURE 3=STATUS 4=ABORT 5=SET_AFE 6=SELFTEST
    5      a0      CAPTURE: 采样档位 gear 0..3 ; SET_AFE: 通道 0..2
    6      a1      CAPTURE: 0=单路 1=三路交替 ; SET_AFE: 1=AC 耦合
    7      a2      CAPTURE: 通道 0/1/2 (单路时有效) ; SET_AFE: 1=20x 衰减
    8      a3      SET_AFE: 1=20x 增益
    9      rsv     0
    10..11 n       u16  本次采集总点数
    12..15 tok     u32  事务号

应答 / 数据头 32 字节
    0..3   magic  'LSDA' = 0x4144534C
    4      ver
    5      type   0=ACK 1=DATA 2=ERR
    6      gear
    7      mode
    8..11  fs_hz
    12..15 n_total
    16..19 n_this
    20..23 gain_milli
    24..27 tok
    28..29 flags
    30..31 crc16

DATA 报文 = 32 字节头 + n_this 个 int16 原始差分码
"""
import functools
import socket
import threading
import struct
import time

LCMD_MAGIC = 0x4F43534C
LSDA_MAGIC = 0x4144534C

LCMD_LEN = 16
LSDA_HDR_LEN = 32

CMD_PING = 0x01
CMD_CAPTURE = 0x02
CMD_STATUS = 0x03
CMD_ABORT = 0x04
CMD_SET_AFE = 0x05
CMD_SELFTEST = 0x06
CMD_PARAM_BEGIN = 0x07
CMD_PARAM_CHUNK = 0x08
CMD_PARAM_COMMIT = 0x09
CMD_PARAM_LOAD = 0x0A
CMD_PARAM_INFO = 0x0B
CMD_PARAM_CLEAR = 0x0C
CMD_LOG_PUT = 0x0D
CMD_LOG_INFO = 0x0E
CMD_LOG_READ = 0x0F
CMD_LOG_CLEAR = 0x10

TYPE_ACK = 0
TYPE_DATA = 1
TYPE_ERR = 2

ERR_NONE = 0
ERR_PARAM = 1
ERR_BUSY = 2
ERR_START = 3
ERR_TIMEOUT = 4

FLAG_INTER3_OK = 0x0001
FLAG_ERROR = 0x0002
#: 与固件宏名保持一致，便于跨文件引用
LSDA_FLAG_INTER3_OK = FLAG_INTER3_OK

#: 标称采样率（Hz），顺序与固件 gear 0..3 一致
GEAR_RATE_SINGLE = (3541666, 2656250, 2125000, 1770833)   # 21.25MHz / {6,8,10,12}
GEAR_RATE_INTER3 = (10624998, 7968750, 6375000, 5312499)
GEAR_RATE_INTER2 = (7083332, 5312500, 4250000, 3541666)   # 双ADC硬件交替 = 单路 x2
GEAR_COUNT = 4

#: 每路最大点数（固件 SCOPE_MAX_M）
MAX_POINTS_PER_CH = 8192
MAX_POINTS_SINGLE = 24576      # 单路可用满 3x8192 连续缓冲，用于提高分辨率
SINGLE_DEFAULT_N = 16384       # 默认 16384 -> 去掉 EQ 两端裁剪后 df=268 Hz
#: 三路交替时的总点数上限
MAX_POINTS_TOTAL = 3 * MAX_POINTS_PER_CH

_HDR = struct.Struct("<I4B5IHH")     # 4+4+20+2+2 = 32
_CMD = struct.Struct("<I6BHI")       # 4+6+2+4  = 16


class LsaError(Exception):
    pass


def crc16_ccitt(data):
    """CRC-16/CCITT-FALSE, poly 0x1021, init 0xFFFF"""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def unpack_header(buf):
    if len(buf) != LSDA_HDR_LEN:
        raise LsaError("头部长度错误: %d" % len(buf))
    (magic, ver, typ, gear, mode, fs_hz,
     n_total, n_this, gain_milli, tok, flags, crc) = _HDR.unpack(buf)
    if magic != LSDA_MAGIC:
        raise LsaError("头部 magic 错误: 0x%08X" % magic)
    calc = crc16_ccitt(buf[:30])
    return {
        "ver": ver, "type": typ, "gear": gear, "mode": mode,
        "fs_hz": fs_hz, "n_total": n_total, "n_this": n_this,
        "gain_milli": gain_milli, "tok": tok, "flags": flags,
        "crc16": crc, "crc_ok": (calc == crc),
    }


def _locked(fn):
    """把整条 命令->应答 串起来加锁（板子只有一条 TCP 连接）。"""
    @functools.wraps(fn)
    def w(self, *a, **kw):
        with self._lock:
            return fn(self, *a, **kw)
    return w


class LsaClient(object):
    def __init__(self, host="192.168.57.200", port=5000, timeout=3.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None
        self.tok = 1
        # 板子只允许一条 TCP 连接：后台保活线程与采集线程必须串行使用 socket，
        # 否则保活的 ACK 会插进数据流中间，表现为"头部 magic 错误"。
        self._lock = threading.RLock()

    # ── 连接管理 ───────────────────────────────────────────────
    def connect(self, wait=6.0):
        """连接板卡。

        板子只保留一条 socket：上一个连接刚断开时它还停在 ESTABLISHED，
        新 SYN 会被 RST（表现为 Connection refused）。这里容忍这种瞬时拒绝，
        在 wait 秒内反复重试。
        """
        self.close()
        t0 = time.time()
        last = None
        while True:
            try:
                s = socket.create_connection((self.host, self.port), timeout=self.timeout)
                break
            except (ConnectionRefusedError, ConnectionResetError, socket.timeout, OSError) as e:
                last = e
                if time.time() - t0 >= wait:
                    raise
                time.sleep(0.35)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(self.timeout)
        self.sock = s
        return self

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def drain(self, quiet=0.08):
        """丢弃套接字里残留的字节。

        上一次采集超时/中断时，数据流里会剩下半截（比如头已读走、载荷没读完），
        下一次再读就会错位，报"头部 magic 错误"。重连或重试前先清干净。
        """
        if self.sock is None:
            return 0
        n = 0
        try:
            self.sock.settimeout(quiet)
            while True:
                part = self.sock.recv(65536)
                if not part:
                    break
                n += len(part)
        except (socket.timeout, OSError):
            pass
        finally:
            try:
                self.sock.settimeout(self.timeout)
            except OSError:
                pass
        return n

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        self.close()

    # ── 底层收发 ───────────────────────────────────────────────
    def _recv_exact(self, n, timeout=None):
        if self.sock is None:
            raise LsaError("尚未连接")
        if timeout is not None:
            self.sock.settimeout(timeout)
        buf = bytearray()
        while len(buf) < n:
            try:
                part = self.sock.recv(n - len(buf))
            except socket.timeout:
                raise LsaError("等待 %d 字节超时（已收到 %d）" % (n, len(buf)))
            if not part:
                raise LsaError("连接被对端关闭")
            buf += part
        if timeout is not None:
            self.sock.settimeout(self.timeout)
        return bytes(buf)

    def _send_cmd(self, cmd, a0=0, a1=0, a2=0, a3=0, n=0):
        if self.sock is None:
            raise LsaError("尚未连接")
        tok = self.tok
        self.tok = (self.tok + 1) & 0xFFFFFFFF or 1
        frame = _CMD.pack(LCMD_MAGIC, cmd, a0 & 0xFF, a1 & 0xFF,
                          a2 & 0xFF, a3 & 0xFF, 0, n & 0xFFFF, tok)
        assert len(frame) == LCMD_LEN
        self.sock.sendall(frame)
        return tok

    def _recv_header(self, timeout=None):
        return unpack_header(self._recv_exact(LSDA_HDR_LEN, timeout))

    # ── 命令 ──────────────────────────────────────────────────
    @_locked
    def ping(self):
        tok = self._send_cmd(CMD_PING)
        h = self._recv_header()
        h["tok_ok"] = (h["tok"] == tok)
        return h

    @_locked
    def status(self):
        tok = self._send_cmd(CMD_STATUS)
        h = self._recv_header()
        h["tok_ok"] = (h["tok"] == tok)
        return h

    @_locked
    def selftest(self):
        tok = self._send_cmd(CMD_SELFTEST)
        h = self._recv_header(timeout=5.0)
        h["tok_ok"] = (h["tok"] == tok)
        h["inter3_ok"] = bool(h["flags"] & FLAG_INTER3_OK)
        return h

    @_locked
    def abort(self):
        tok = self._send_cmd(CMD_ABORT)
        h = self._recv_header()
        h["tok_ok"] = (h["tok"] == tok)
        return h

    @_locked
    def set_afe(self, ch, ac=False, att20=False, gain20=False):
        """ch: 0..2 (0=CH1)"""
        tok = self._send_cmd(CMD_SET_AFE, a0=ch, a1=int(bool(ac)),
                             a2=int(bool(att20)), a3=int(bool(gain20)))
        h = self._recv_header()
        h["tok_ok"] = (h["tok"] == tok)
        return h

    # ── 掉电日志（板载 W25Q16 环形区） ──────────────────────
    @_locked
    def log_put(self, level, text):
        """写一条日志到板子 Flash（板子同时从调试串口打一遍）。"""
        b = str(text).encode("utf-8")[:120]
        self._send_cmd(CMD_LOG_PUT, n=1 + len(b))
        self.sock.sendall(bytes([int(level) & 0xFF]) + b)
        h = self._recv_header()
        return h["type"] == TYPE_ACK

    @_locked
    def log_info(self):
        """返回 (最大序号, 已用扇区数)。"""
        self._send_cmd(CMD_LOG_INFO)
        h = self._recv_header()
        body = self._recv_exact(h["n_this"]) if h["n_this"] else b""
        if len(body) < 8:
            return 0, 0
        seq, sec = struct.unpack_from("<II", body, 0)
        return seq, sec

    @_locked
    def log_read(self, from_seq=1, max_bytes=800):
        """取 >= from_seq 的日志，返回文本行（'seq\tL<级别>\t正文\n'）。"""
        self._send_cmd(CMD_LOG_READ, a0=from_seq & 0xFF,
                       a1=(from_seq >> 8) & 0xFF,
                       a2=(from_seq >> 16) & 0xFF,
                       a3=(from_seq >> 24) & 0xFF,
                       n=int(max_bytes))
        h = self._recv_header(timeout=6.0)
        if h["type"] != TYPE_DATA or h["n_this"] == 0:
            return ""
        return self._recv_exact(h["n_this"]).decode("utf-8", "replace")

    @_locked
    def log_clear(self):
        self._send_cmd(CMD_LOG_CLEAR)
        return self._recv_header()["type"] == TYPE_ACK

    @_locked
    def capture(self, gear=0, inter3=True, ch=0, n=None, timeout=5.0, inter2=False):
        """
        发起一次采集并取回全部样本。

        gear   : 0..3
        inter3 : True 三路交替（3 倍采样率），False 单路
        ch     : 单路模式下的通道 0/1/2
        n      : 总点数；None 表示用默认值（inter3 24576，单路 16384 -> df=268Hz）

        返回 (info, raw)，raw 为 bytes（int16 小端，交织顺序）
        """
        if inter2:
            inter3 = False
        if n is None:
            n = MAX_POINTS_TOTAL if inter3 else SINGLE_DEFAULT_N
        n = int(n)
        if inter3 and (n % 3) != 0:
            raise LsaError("三路交替模式点数必须是 3 的倍数")
        if inter2 and (n % 2) != 0:
            raise LsaError("双ADC交替模式点数必须是 2 的倍数")
        if n <= 0 or n > MAX_POINTS_TOTAL:
            raise LsaError("点数超出范围 1..%d" % MAX_POINTS_TOTAL)

        a1 = 2 if inter2 else (1 if inter3 else 0)
        tok = self._send_cmd(CMD_CAPTURE, a0=gear, a1=a1, a2=ch, n=n)
        try:
            h = self._recv_header(timeout=timeout)
        except Exception:
            self.drain()
            raise
        h["tok_ok"] = (h["tok"] == tok)

        # 帧新鲜度：事务号必须和本命令一致。板子只回显本次 tok，
        # 若收到的是上一轮遗留的帧（板子卡住、多次点击、重连后残留），
        # tok 必然对不上 —— 直接丢弃并报错，绝不把旧数据当新测量。
        if not h["tok_ok"]:
            self.drain()
            raise LsaError("收到陈旧帧（tok=%d，期望 %d），已丢弃" % (h["tok"], tok))

        if h["type"] == TYPE_ERR:
            raise LsaError("采集被拒绝，错误码 %d" % (h["flags"] & 0x00FF))
        if h["type"] != TYPE_DATA:
            raise LsaError("期望 DATA，收到 type=%d" % h["type"])

        want = h["n_this"] * 2
        raw = self._recv_exact(want, timeout=timeout)

        # 长度自检：n_this 与实际收到的字节数必须严格对应（防半包）
        if len(raw) != want:
            self.drain()
            raise LsaError("数据长度不符：收到 %d 字节，期望 %d" % (len(raw), want))
        return h, raw


def raw_to_int16(raw):
    """把 int16 小端字节流转成 Python list（无 numpy 依赖）"""
    import array
    a = array.array("h")
    a.frombytes(raw)
    if struct.pack("<h", 1) != struct.pack("=h", 1):
        a.byteswap()
    return a


if __name__ == "__main__":
    import sys
    host = sys.argv[1] if len(sys.argv) > 1 else "192.168.57.200"
    with LsaClient(host) as c:
        print("PING  ->", c.ping())
        print("STATUS->", c.status())
