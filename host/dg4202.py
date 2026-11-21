# -*- coding: utf-8 -*-
"""RIGOL DG4202 信号源驱动（USBTMC over WinUSB，SCPI over pyusb）。

为什么单独写一个文件
--------------------
DG4202 没有串口、没有网口，唯一通道是 USB USBTMC（VID 0x1AB1 / PID 0x0641）。
本机 NI-VISA 已损坏（``VI_ERROR_LIBRARY_NFOUND``），所以不能走 pyvisa，只能用
libusb + 手写 USBTMC bulk 协议。实测坑如下，代码里逐条对应：

1. **pyusb 默认后端是 None**：``usb.core.find()`` 找不到任何设备。必须显式
   ``get_backend(find_library=libusb_package.find_library)``，DLL 在
   ``Lib/site-packages/libusb_package/libusb-1.0.dll``。

2. **必须先换 WinUSB 驱动**：仪器出厂绑在 Windows 的 ``usbtmc.sys`` 上，此时
   libusb 能枚举到设备，但 ``set_configuration()/claim_interface()`` 会抛
   ``NotImplementedError: Operation not supported or unimplemented on this
   platform``。用 Zadig 选中 VID 1AB1 / PID 0641 的接口 0 换成 WinUSB，重新插拔
   即可。``--idn`` 会就此给出短提示，而不是抛 traceback。

3. **DG 方言与 GW Instek 不同**：幅度节点是 ``:VOLT``（不是 ``:AMP``），偏置是
   ``:VOLT:OFFS``（不是 ``:DCO``）。RIGOL 长短助记符都收，且**要带前导冒号**；
   本文件统一用短形式，例如 ``:SOUR1:FREQ``。实测还有个坑：扫频起止频率挂在
   **``:FREQ:STAR`` / ``:FREQ:STOP``** 节点，``:SWE:STAR`` / ``:SWE:STOP`` 会
   直接被固件忽略（查询超时）。扫频触发源是 ``:SWE:TRIG:SOUR INT``；写 ``IMM``
   固件回 ``-220,"Parameter error"``。

4. **ARB 是 14 bit / 16 kpts / 500 MSa/s**：数据范围 -8192..8191（AFG 是
   ±511 / 4096 点 / 120 MSa/s，不能照抄）。采样率 = 重复频率 × 点数，超上限
   固件会回 ``-222 Data out of range``。

5. **ARB 顺序必须是 SIN → DATA → FREQ/VOLT/OFFS → USER**：``FUNC USER`` 会把
   FREQ 复位（AFG 手册 P261 的同一个坑）。先切回 SIN 才能改 ARB 数据，FREQ/幅度
   必须在切 USER 之前写好。``:FUNC:ARB:SRAT`` 是否被接受随固件而异，按需探测
   （本机 00.01.14 实测接受）。注意它是**只写节点**：``:FUNC:ARB:SRAT?`` 查询
   本固件完全不应答（3 s 超时），所以代码里只发写命令，从不回读采样率。

6. **跨固件命令漂移**：扫频触发源可能是 ``:SOUR:SWE:TRIG:SOUR`` 或
   ``:TRIG:SOUR``；ARB 数据命令可能是 ``:DATA:DAC`` / ``:DATA`` /
   ``:TRACE:DATA:DAC``。都按「写一条 → 读 ``SYST:ERR?`` → 干净则缓存」的方式
   惰性探测，且只探一次。本机固件实测：``:DATA:DAC VOLATILE,`` 可用，干净时
   ``SYST:ERR?`` 原样回 ``0,"No error"``（不带 ``+``）。

7. **输出用 ON/OFF**：``:OUTP1 1`` 在部分固件上不生效，和 AFG 一样只用
   ``ON`` / ``OFF``。

8. **谐波模式（HARMONIC）**：``:SOUR:FUNC HARM`` 开启；**没有** ``HARM:STAT``
   节点（写它回 ``-113``），开启/关闭就是切 FUNC。次数节点是全称
   ``:HARM:ORDER``（``:HARM:ORD`` 不存在），本机 00.01.14 实测 **2..16**，
   写 >16 会静默钳位到 16 且不报错。每阶幅度/相位用
   ``:HARM:AMPL <order>,<vpp>`` / ``:HARM:PHAS <order>,<deg>``，查询必须带阶数
   （``:HARM:AMPL? 3``）；``:HARM:TYPE`` 取 ``EVEN`` / ``ODD`` / ``ALL`` /
   ``USER``。做"基波 + 第 N 次谐波"要 ``TYPE USER``。用户要的 40 次谐波
   **超出硬件能力**，只能用 ``load_arb`` 把基波与谐波一起合成进 ARB 波形。

9. **幅度单位不是固定 Vpp**：``:VOLT:UNIT`` 可以是 ``VPP`` / ``VRMS`` / ``DBM``，
   ``:VOLT?`` 回的是**当前单位**下的值。实测 ``:VOLT 1VPP`` 被接受，且只按 Vpp
   换算成当前单位、**不改变** ``:VOLT:UNIT``（VRMS 模式下回 0.3536）。所以设点
   前要么先 ``:VOLT:UNIT VPP``，要么给数值带 ``VPP`` 后缀；读回时若单位不是
   ``VPP``（或为空），用 ``:VOLT:HIGH?`` 减 ``:VOLT:LOW?`` 反算 Vpp。

所有探测都是惰性的：``Scpi()`` 可以离线构造，不碰仪器；第一次真正发命令时才开
USB。这样 ``lsa_src`` 在本机没插仪器时也能正常 import。
"""
from __future__ import annotations

import argparse
import glob
import logging
import math
import os
import select
import struct
import sys
import time

try:
    import usb.core
    import usb.util
    import usb.backend.libusb1
    import libusb_package
    _USB_IMPORT_ERR = None
except Exception as _exc:  # pragma: no cover - 只在缺 pyusb 时走到
    usb = None
    libusb_package = None
    _USB_IMPORT_ERR = _exc

log = logging.getLogger("dg4202")

VID = 0x1AB1
PID = 0x0641
SERIAL_DEFAULT = "DG4E202401525"

TIMEOUT_MS = 3000
READ_SIZE = 1024 * 1024          # Bulk-IN 一次最多请求 1 MiB
SRATE_MAX = 500e6                # DG4202 ARB 采样率上限 500 MSa/s
SRATE_MIN = 1e-6                 # 下限 1 uSa/s（手册标称）
ARB_MAX_POINTS = 16384           # 16 kpts
ARB_FULL_SCALE = 8191            # 14 bit，相对零电平的最大偏移
ARB_CODE_MID = 8192              # 偏移二进制的零电平码
ARB_CODE_MAX = 16383             # 0x3FFF，手册给的上限
HARM_MAX_ORDER = 16              # 实测 00.01.14：2..16，>16 静默钳位
HARM_MASK_OFF = "X000000000000000"   # :HARMonic:USER 默认值 = 全部阶次关断
HARM_SETTLE = 3.0                # 写完 USER 掩码后波形重建约需 2.5~4 s


def harm_mask(orders):
    """把阶次列表转成 ``:HARMonic:USER`` 的 16 位掩码字符串。

    手册 ``[:SOURce<n>]:HARMonic:USER <user>``：``X000000000000000`` ..
    ``X111111111111111``，最左位是基波（固定 ``X``，不可关），后面 15 位
    依次对应 **2..16 次**谐波，``1``=输出、``0``=不输出，默认全 ``0``。

    这是 DG4000 谐波模式的**总开关**：只写 ``HARM:AMPL`` 而掩码仍为默认
    全 0 时，仪器接受命令、回读也正确、错误队列干净，但输出里就是一个纯
    基波——本机在这个坑里耗掉了十几轮探针。
    """
    bits = ["0"] * (HARM_MAX_ORDER - 1)
    for k in orders:
        k = int(k)
        if 2 <= k <= HARM_MAX_ORDER:
            bits[k - 2] = "1"
    return "X" + "".join(bits)

WINUSB_HINT = (
    "cannot open DG4202 (VID 0x1AB1 / PID 0x0641): {why}. "
    "NI-VISA is broken on this host (VI_ERROR_LIBRARY_NFOUND), so USBTMC must "
    "go through WinUSB: use Zadig to bind interface 0 of this device to "
    "WinUSB, unplug/replug, then retry."
)


class DgError(RuntimeError):
    """DG4202 驱动错误（打不开 / 超时 / 命令被拒）。"""


# ---------------------------------------------------------------------------
# libusb 后端
# ---------------------------------------------------------------------------
_BACKEND = None


def _get_backend():
    global _BACKEND
    if _BACKEND is None:
        if usb is None:
            raise DgError(
                "pyusb / libusb 不可用（%r）；请先 pip install pyusb libusb-package"
                % (_USB_IMPORT_ERR,))
        try:
            _BACKEND = usb.backend.libusb1.get_backend(
                find_library=libusb_package.find_library)
        except Exception as exc:
            raise DgError("加载 libusb 后端失败：%r" % (exc,))
        if _BACKEND is None:
            raise DgError(
                "libusb 后端为 None：libusb-1.0.dll 没找到。"
                "确认 libusb_package 已安装（find_library 返回的路径应存在）。")
    return _BACKEND


def _safe_attr(dev, name):
    try:
        return getattr(dev, name) or ""
    except Exception:
        return "?"


def list_usb():
    """枚举 USB 设备（只读，不打开仪器），供 ``--list`` 用。"""
    if usb is None:
        raise DgError(
            "pyusb / libusb 不可用（%r）；请先 pip install pyusb libusb-package"
            % (_USB_IMPORT_ERR,))
    backend = _get_backend()
    try:
        devs = list(usb.core.find(find_all=True, backend=backend))
    except Exception as exc:
        raise DgError("枚举 USB 设备失败：%r" % (exc,))
    print("USB 设备（libusb，共 %d 个）：" % len(devs))
    for d in devs:
        mark = ""
        if int(getattr(d, "idVendor", 0)) == VID and int(getattr(d, "idProduct", 0)) == PID:
            mark = "  <= DG4202"
        print("  bus %-3s addr %-3s  %04x:%04x  %s / %s / %s%s" % (
            getattr(d, "bus", "?"), getattr(d, "address", "?"),
            int(getattr(d, "idVendor", 0)), int(getattr(d, "idProduct", 0)),
            _safe_attr(d, "manufacturer"), _safe_attr(d, "product"),
            _safe_attr(d, "serial_number"), mark))
    if not any(int(getattr(d, "idVendor", 0)) == VID
               and int(getattr(d, "idProduct", 0)) == PID for d in devs):
        print("没有看到 1ab1:0641 —— 检查 USB 线/电源，或在 Zadig 里确认设备存在。")
    return devs


# ---------------------------------------------------------------------------
# USBTMC 传输层
# ---------------------------------------------------------------------------
class Usbtmc:
    """纯 Python USBTMC bulk 传输（只实现 DEV_DEP_MSG_OUT / DEV_DEP_MSG_IN）。"""

    VID = VID
    PID = PID
    SERIAL_DEFAULT = SERIAL_DEFAULT

    def __init__(self, serial=None, vid=None, pid=None, timeout_ms=TIMEOUT_MS):
        self.serial = serial or self.SERIAL_DEFAULT
        self.vid = self.VID if vid is None else int(vid)
        self.pid = self.PID if pid is None else int(pid)
        self.timeout_ms = int(timeout_ms)
        self.dev = None
        self._iface = None
        self._ep_out = None
        self._ep_in = None
        self._max_pkt = 64
        self._tag = 0
        self._empty_frames = 0
        self.open()

    # ---- USB 生命周期 -------------------------------------------------
    def _pick_interface(self, cfg):
        for itf in cfg:
            if (itf.bInterfaceClass, itf.bInterfaceSubClass,
                    itf.bInterfaceProtocol) == (0xFE, 0x03, 0x01):
                return itf
        # 兜底：第一个同时有 bulk IN/OUT 的接口
        for itf in cfg:
            out = inn = None
            for ep in itf:
                if usb.util.endpoint_type(ep.bmAttributes) != usb.util.ENDPOINT_TYPE_BULK:
                    continue
                if usb.util.endpoint_direction(ep.bEndpointAddress) == usb.util.ENDPOINT_OUT:
                    out = ep
                else:
                    inn = ep
            if out is not None and inn is not None:
                return itf
        return None

    def open(self):
        if self.dev is not None:
            return self
        if usb is None:
            raise DgError(
                "pyusb / libusb 不可用（%r）；请先 pip install pyusb libusb-package"
                % (_USB_IMPORT_ERR,))
        backend = _get_backend()
        try:
            cands = list(usb.core.find(find_all=True, idVendor=self.vid,
                                       idProduct=self.pid, backend=backend))
        except Exception as exc:
            raise DgError("枚举 USB 失败：%r" % (exc,))
        if not cands:
            raise DgError(
                "没有找到 RIGOL DG4202（VID 0x%04X / PID 0x%04X）。"
                "检查 USB 线、仪器电源，或先跑 `python dg4202.py --list`。"
                % (self.vid, self.pid))

        # 序列号要开设备才能读；在 usbtmc.sys 下可能读不到，读不到就按 VID/PID 用。
        wanted = str(self.serial).strip()
        matched = None
        readable = []
        for d in cands:
            try:
                sn = (d.serial_number or "").strip()
            except Exception:
                sn = ""
            readable.append(sn)
            if sn and sn.lower() == wanted.lower():
                matched = d
                break
        if matched is not None:
            dev = matched
        elif len(cands) == 1 and not readable[0]:
            # 只有序列号读不出来时才退回 VID/PID（usbtmc.sys 下可能读不到）
            dev = cands[0]
        else:
            raise DgError(
                "找到 %d 台 RIGOL（序列号：%s），但没有 %s；用 --serial 指定。"
                % (len(cands), ", ".join(s or "?" for s in readable), wanted))

        try:
            dev.set_configuration()
            cfg = dev.get_active_configuration()
            itf = self._pick_interface(cfg)
            if itf is None:
                raise DgError("USBTMC 接口不存在（既没有 FE/03/01，也没有 bulk IN+OUT）")
            ep_out = ep_in = None
            for ep in itf:
                if usb.util.endpoint_type(ep.bmAttributes) != usb.util.ENDPOINT_TYPE_BULK:
                    continue
                if usb.util.endpoint_direction(ep.bEndpointAddress) == usb.util.ENDPOINT_OUT:
                    ep_out = ep
                else:
                    ep_in = ep
            if ep_out is None or ep_in is None:
                raise DgError("USBTMC 接口缺少 bulk IN/OUT 端点")
            usb.util.claim_interface(dev, itf.bInterfaceNumber)
        except NotImplementedError as exc:
            self.dev = None
            raise DgError(WINUSB_HINT.format(
                why="libusb 无法打开，当前驱动不是 WinUSB（%s）" % exc))
        except usb.core.USBError as exc:
            self.dev = None
            raise DgError(WINUSB_HINT.format(
                why="USB 打开/占用失败（%s）" % exc))
        except DgError:
            self.dev = None
            raise

        self.dev = dev
        self._iface = itf.bInterfaceNumber
        self._ep_out = ep_out
        self._ep_in = ep_in
        self._max_pkt = int(getattr(ep_out, "wMaxPacketSize", 0) or 64)
        log.debug("DG4202 打开：bus=%s addr=%s ep_out=0x%02x ep_in=0x%02x mps=%d",
                  getattr(dev, "bus", "?"), getattr(dev, "address", "?"),
                  ep_out.bEndpointAddress, ep_in.bEndpointAddress, self._max_pkt)
        # 上一次会话可能留下未读的应答（实测会一直排队，导致之后每条查询都错位
        # 一格，看起来像"命令没生效"）。打开后先无条件清空 IN 管道。
        self.flush_input()
        return self

    def flush_input(self, timeout_ms=100, max_reads=128):
        """丢掉 IN 端点里已经排队的旧应答（不发送任何新请求）。"""
        for _ in range(max_reads):
            try:
                self.dev.read(self._ep_in, 4096, timeout=timeout_ms)
            except Exception:
                return

    def _recover_input(self):
        """一次查询超时后的软恢复：清 IN halt，再排空迟到的字节。

        实测扫频刚启动时偶发某条查询 3 s 无应答；若直接重发会把迟到的
        应答错配给下一条命令，因此必须先 clear_halt + flush_input。
        """
        try:
            util = getattr(usb, "util", None)
            if util is not None:
                util.clear_halt(self.dev, self._ep_in)
        except Exception:
            pass
        self.flush_input(timeout_ms=60, max_reads=32)

    def close(self):
        dev, iface = self.dev, self._iface
        self.dev = None
        self._iface = None
        if dev is None:
            return
        try:
            if iface is not None:
                usb.util.release_interface(dev, iface)
        except Exception:
            pass
        try:
            usb.util.dispose_resources(dev)
        except Exception:
            pass

    # ---- 错误包装 -----------------------------------------------------
    def _usb_err(self, exc, action, timeout=None):
        msg = str(exc)
        low = msg.lower()
        eno = getattr(exc, "errno", None)
        if eno == 32 or "pipe" in low or "stall" in low:
            for ep in (self._ep_out, self._ep_in):
                try:
                    if ep is not None:
                        self.dev.clear_halt(ep)
                except Exception:
                    pass
            return DgError("%s 失败：USB 管道错误（已尝试 clear_halt）：%s" % (action, msg))
        if eno == 110 or "timed out" in low or "timeout" in low:
            return DgError("%s 超时（%d ms 内无应答）：%s"
                           % (action, timeout or self.timeout_ms, msg))
        return DgError("%s 失败：%s" % (action, msg))

    def _bulk_out(self, data, timeout=None):
        to = timeout or self.timeout_ms
        # 实测：谐波模式切换时仪器会忙数秒，OUT 端点暂时不收数据（Errno 10060）。
        # 短命令（<=256 字节）允许退避重试；大块 ARB 数据不重试，避免重复下发。
        for attempt in range(3):
            try:
                self.dev.write(self._ep_out, data, timeout=to)
                return
            except NotImplementedError as exc:
                raise DgError(WINUSB_HINT.format(
                    why="libusb 无法使用该端点，驱动可能仍是 usbtmc.sys（%s）" % exc))
            except usb.core.USBError as exc:
                low = str(exc).lower()
                busy = (getattr(exc, "errno", None) == 110
                        or "timed out" in low or "timeout" in low)
                if attempt < 2 and busy and len(data) <= 256:
                    time.sleep(1.0 + attempt)
                    continue
                raise self._usb_err(exc, "写 USBTMC", to)

    def _read_exact(self, n, timeout=None):
        """读满 n 字节；容忍设备在整包之后补的 ZLP（读回 0 字节）与短包。"""
        timeout = timeout or self.timeout_ms
        buf = bytearray()
        deadline = time.time() + timeout / 1000.0
        zero = 0
        while len(buf) < n:
            remain = int((deadline - time.time()) * 1000.0)
            if remain <= 0:
                break
            try:
                chunk = self.dev.read(self._ep_in, n - len(buf), timeout=max(50, remain))
            except usb.core.USBError as exc:
                if buf:
                    break
                raise self._usb_err(exc, "读 USBTMC", timeout)
            if chunk is None:
                break
            chunk = bytes(chunk)
            if not chunk:
                zero += 1
                if zero > 2:
                    break
                continue
            zero = 0
            buf.extend(chunk)
        return bytes(buf)

    # ---- 协议 ---------------------------------------------------------
    def _next_tag(self):
        self._tag = self._tag + 1 if self._tag < 255 else 1
        return self._tag

    @staticmethod
    def _header(msgid, tag, size, attrs):
        # MsgID, bTag, ~bTag, 保留, TransferSize(LE32), bmTransferAttributes, 3 字节保留
        return struct.pack("<BBBBI B 3x", msgid, tag, (~tag) & 0xFF, 0, size, attrs)

    def write(self, data, timeout=None, settle=0.0):
        """发一条 USBTMC DEV_DEP_MSG_OUT；``data`` 可为 str（自动 ASCII）。"""
        if isinstance(data, str):
            data = data.encode("ascii")
        data = bytes(data)
        timeout = timeout or self.timeout_ms
        tag = self._next_tag()
        n = len(data)
        # EOM 放在消息头上（USBTMC 是消息级标志），这里每条 write 就是一条完整消息。
        self._bulk_out(self._header(1, tag, n, 0x01), timeout)
        for off in range(0, n, self._max_pkt):
            self._bulk_out(data[off:off + self._max_pkt], timeout)
        # 若总长正好是 wMaxPacketSize 的整数倍，设备分不清「还有后续」，
        # 必须补一个 ZLP 收尾。
        if n and n % self._max_pkt == 0:
            self._bulk_out(b"", timeout)
        if settle:
            time.sleep(settle)

    def _read_header(self, timeout=None):
        """读 12 字节 USBTMC 头，并跳过报文之间的杂散字节。

        实测 DG4202 在 EOM 之后还会多送 1 个字节（值不固定：NUL / ``o`` /
        ``+``，像是未清空的发送缓冲），而 ``OUTP?`` 又把它算进 TransferSize。
        若按固定 12 字节读，这个字节会顶掉下一次头的第一字节，之后整条链路
        逐字节错位（表现为 MsgID=111 之类的怪值）。合法头 MsgID 只能是 2，
        所以这里丢弃所有非 0x02 的前导字节再凑满 12 字节。
        """
        timeout = timeout or self.timeout_ms
        deadline = time.time() + timeout / 1000.0
        buf = bytearray()
        while len(buf) < 12:
            remain = int((deadline - time.time()) * 1000.0)
            if remain <= 0:
                break
            try:
                chunk = self.dev.read(self._ep_in, 12 - len(buf), timeout=max(50, remain))
            except usb.core.USBError as exc:
                if buf:
                    break
                raise self._usb_err(exc, "读 USBTMC 应答头", timeout)
            if chunk is None:
                break
            chunk = bytes(chunk)
            if not chunk:
                continue
            buf.extend(chunk)
            while buf and buf[0] != 0x02:
                del buf[0]
        return bytes(buf)

    def read(self, timeout=None):
        """发 DEV_DEP_MSG_IN，读回一条完整应答（最多累积 4 个消息）。"""
        timeout = timeout or self.timeout_ms
        tag = self._next_tag()
        self._bulk_out(self._header(2, tag, READ_SIZE, 0x00), timeout)
        out = bytearray()
        for _ in range(4):
            hdr = self._read_header(timeout)
            if len(hdr) < 12:
                raise DgError("USBTMC 应答头不完整（收到 %d/12 字节），仪器可能未就绪"
                              % len(hdr))
            msgid, _rtag, _inv, _rsv, size, attrs = struct.unpack_from("<BBBBI B", hdr, 0)
            if msgid != 2:
                raise DgError("USBTMC 应答头 MsgID=%d（应为 2）" % msgid)
            if size > 4 * 1024 * 1024:
                raise DgError("USBTMC 应答长度异常：%d 字节" % size)
            got = 0
            while got < size:
                part = self._read_exact(size - got, timeout)
                if not part:
                    break
                out.extend(part)
                got += len(part)
            if attrs & 0x01:            # EOM
                break
            if got < size:              # 短读且没有更多数据可等
                break
        return bytes(out)

    # ---- SCPI 便捷层 --------------------------------------------------
    def q(self, cmd, maxlines=8):
        """查询并返回行列表（按 ``\\r\\n`` 切分）。

        空帧保护：固件偶尔回一条只含 ``\\n`` 的应答。这里不把空串当有效值，
        连续 16 次空应答就判定 USBTMC 会话失步并抛 DgError，避免把单位 token
        （``VPP``/``VRMS``）错喂给数值查询后一路返回垃圾。
        """
        cmd = cmd.rstrip("\r\n")
        raw = None
        for attempt in (0, 1):
            try:
                self.write(cmd + "\n")
                raw = self.read()
                break
            except DgError:
                # 超时后仪器可能还在算这一条；把迟到的应答连同 IN 管道一起丢掉，
                # 再重发一次查询。否则残留字节会让之后每条查询都错位。
                if attempt:
                    raise
                log.info("DG4202 查询超时，清空 IN 管道后重试一次：%s", cmd)
                self._recover_input()
                time.sleep(0.2)
        text = raw.decode("ascii", "replace")
        lines = [ln.strip() for ln in text.replace("\r\n", "\n").replace("\r", "\n").split("\n")]
        out = [ln for ln in lines if ln][:maxlines]
        if out:
            self._empty_frames = 0
        else:
            self._empty_frames += 1
            if self._empty_frames >= 16:
                raise DgError(
                    "DG4202 连续 %d 次空应答，USBTMC 会话已失步；"
                    "请 close() 后重新连接（或拔插 USB）" % self._empty_frames)
        return out

    def one(self, cmd, default=""):
        r = self.q(cmd)
        return r[0] if r else default

    def w(self, cmd, settle=0.0):
        self.write((cmd.rstrip("\r\n") + "\n"), settle=settle)


# ---------------------------------------------------------------------------
class UsbtmcDev:
    """POSIX 传输：直接用内核 ``usbtmc`` 驱动给的 ``/dev/usbtmcN`` 设备节点。

    为什么树莓派上不走 pyusb：

    * 树莓派 OS 自带 ``usbtmc`` 内核模块，DG4202 一插上就有 ``/dev/usbtmc0``，
      USBTMC 的 bulk 分包、termchar、clear-halt 全部由内核做完，用户态只管
      ``write()`` 命令、``read()`` 应答，一行 SCPI 一条消息；
    * 反过来用 pyusb 要先 ``detach_kernel_driver(0)`` 把内核驱动踢掉，再自己拼
      12 字节头（即本文件的 ``Usbtmc`` 类），既多一份依赖又容易和内核抢设备；
    * 所以 ``open_transport()`` 在 POSIX 上优先选这里，Windows 上仍走 pyusb。

    权限：``/dev/usbtmc*`` 默认 ``root:root 0600``，普通用户读不了。装一条 udev
    规则即可（见 ``host_app/_pi_setup_dg.sh``，内容是
    ``SUBSYSTEM=="usbmisc", ATTRS{idVendor}=="1ab1", MODE="0666"``），否则只能
    sudo 跑。
    """

    DEV_GLOB = "/dev/usbtmc*"

    def __init__(self, serial=None, timeout_ms=TIMEOUT_MS, path=None):
        self.serial = serial or SERIAL_DEFAULT
        self.timeout_ms = int(timeout_ms)
        self.path = path
        self.fd = None
        self._empty_frames = 0
        self.open()

    # ---- 生命周期 ------------------------------------------------------
    def open(self):
        if self.fd is not None:
            return self
        cands = [self.path] if self.path else sorted(glob.glob(self.DEV_GLOB))
        if not cands:
            raise DgError(
                "没有 %s（DG4202 没插上，或内核 usbtmc 模块没加载）"
                % self.DEV_GLOB)
        want = str(self.serial).strip().lower()
        first_err = None
        for path in cands:
            try:
                fd = os.open(path, os.O_RDWR)
            except OSError as exc:
                first_err = first_err or exc
                continue
            self.fd = fd
            self.path = path
            try:
                idn = self.one("*IDN?").strip()
            except DgError as exc:
                idn = ""
                first_err = first_err or exc
            if not idn:
                # 这个节点不应答，换下一个
                os.close(fd)
                self.fd = None
                continue
            if want and want not in idn.lower():
                # 连上了但不是我们要的那台
                os.close(fd)
                self.fd = None
                first_err = first_err or DgError(
                    "%s 上是 %r，不是序列号 %s" % (path, idn, self.serial))
                continue
            return self
        raise DgError(
            "打不开任何 USBTMC 设备节点（%s）：%r。检查是否插好、"
            "是否有 udev 权限规则（见 _pi_setup_dg.sh）"
            % (", ".join(cands), first_err))

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None

    # ---- 非阻塞收尾 ----------------------------------------------------
    def _set_nonblock(self, on):
        """临时切换 O_NONBLOCK（flush/补读时用；fcntl 只在 POSIX 有）。"""
        import fcntl
        flags = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        new = (flags | os.O_NONBLOCK) if on else (flags & ~os.O_NONBLOCK)
        if new != flags:
            fcntl.fcntl(self.fd, fcntl.F_SETFL, new)

    def _read_available(self, timeout_s=0.1, max_reads=64):
        """在 timeout_s 内把**已经到达**的数据收干净（不会触发新的 IN 传输）。"""
        if self.fd is None:
            return b""
        out = bytearray()
        self._set_nonblock(True)
        try:
            deadline = time.monotonic() + timeout_s
            for _ in range(max_reads):
                remain = deadline - time.monotonic()
                if remain <= 0:
                    break
                r, _, _ = select.select([self.fd], [], [], min(remain, 0.02))
                if not r:
                    continue
                try:
                    chunk = os.read(self.fd, 65536)
                except BlockingIOError:
                    break
                except OSError:
                    break
                if not chunk:
                    break
                out.extend(chunk)
                if out.endswith(b"\n"):
                    break
        finally:
            self._set_nonblock(False)
        return bytes(out)

    def flush_input(self, timeout_ms=100, max_reads=128):
        """把 IN 通道里残留的应答丢掉（不阻塞）。"""
        return len(self._read_available(timeout_s=timeout_ms / 1000.0,
                                        max_reads=max_reads))

    def _recover_input(self):
        return self.flush_input()

    # ---- 读写 ----------------------------------------------------------
    def write(self, data, timeout=None, settle=0.0):
        if self.fd is None:
            raise DgError("USBTMC 设备已关闭")
        if isinstance(data, str):
            data = data.encode("ascii", "replace")
        total = 0
        view = memoryview(data)
        while total < len(data):
            try:
                n = os.write(self.fd, view[total:])
            except OSError as exc:
                raise DgError("写 %s 失败：%r" % (self.path, exc))
            if n <= 0:
                raise DgError("写 %s 返回 0 字节" % self.path)
            total += n
        if settle:
            time.sleep(settle)
        return total

    def read(self, timeout=None):
        """读一条应答。

        **必须直接 read()，不能先 select() 等可读**：内核 usbtmc 的
        ``poll()`` 只在驱动缓冲里已经有数据时才报 POLLIN，它不会替你发起
        bulk IN；先 select 就一直等不到（实测 select 3 s 恒为 False，而
        紧接的 read 立刻拿到 IDN）。真正的 IN 传输是 read() 自己发的，
        驱动内部超时约 5 s，没应答就返回 EIO。
        """
        if self.fd is None:
            raise DgError("USBTMC 设备已关闭")
        try:
            data = os.read(self.fd, 65536)
        except BlockingIOError:
            data = self._read_available(0.2)
        except OSError as exc:
            raise DgError(
                "读 %s 失败：%r（仪器 5 s 内没有应答；写命令或只写节点常见）"
                % (self.path, exc))
        if not data:
            raise DgError("读 %s 返回 0 字节" % self.path)
        buf = bytearray(data)
        if not buf.endswith(b"\n"):
            # 一条消息可能被拆成多次 read，补收一小段
            buf.extend(self._read_available(0.2))
        return bytes(buf)

    # ---- SCPI 便捷层（与 Usbtmc 保持同一套 API）------------------------
    def q(self, cmd, maxlines=8):
        cmd = cmd.rstrip("\r\n")
        raw = None
        for attempt in (0, 1):
            try:
                self.write(cmd + "\n")
                raw = self.read()
                break
            except DgError:
                if attempt:
                    raise
                log.info("DG4202 查询超时，清空 IN 通道后重试一次：%s", cmd)
                self._recover_input()
                time.sleep(0.2)
        text = raw.decode("ascii", "replace")
        lines = [ln.strip()
                 for ln in text.replace("\r\n", "\n").replace("\r", "\n").split("\n")]
        out = [ln for ln in lines if ln][:maxlines]
        if out:
            self._empty_frames = 0
        else:
            self._empty_frames += 1
            if self._empty_frames >= 16:
                raise DgError(
                    "DG4202 连续 %d 次空应答，USBTMC 会话已失步；"
                    "请 close() 后重新连接（或拔插 USB）" % self._empty_frames)
        return out

    def one(self, cmd, default=""):
        r = self.q(cmd)
        return r[0] if r else default

    def w(self, cmd, settle=0.0):
        self.write((cmd.rstrip("\r\n") + "\n"), settle=settle)


def open_transport(serial=None, timeout_ms=TIMEOUT_MS):
    """按平台挑传输层：POSIX 有 /dev/usbtmc* 就用内核驱动，否则 pyusb/WinUSB。"""
    if os.name == "posix" and glob.glob(UsbtmcDev.DEV_GLOB):
        return UsbtmcDev(serial=serial, timeout_ms=timeout_ms)
    return Usbtmc(serial=serial, timeout_ms=timeout_ms)


# ---------------------------------------------------------------------------
# SCPI 方言层
# ---------------------------------------------------------------------------
class Scpi:
    """DG4200 系列的 SCPI 门面；构造时不碰硬件（第一次发命令才开 USB）。"""

    # 跨固件漂移的命令模板；探测成功后缓存
    _SWEEP_TRIG_CMDS = (
        ":SOUR%d:SWE:TRIG:SOUR INT",     # 本机 00.01.14 实测：INT 才认，IMM 回 -220
        ":SOUR%d:SWE:TRIG:SOUR IMM",
        ":TRIG%d:SOUR IMM",
    )
    _ARB_DATA_CMDS = (
        ":SOUR%d:DATA:DAC VOLATILE,",
        ":SOUR%d:DATA VOLATILE,",
        ":SOUR%d:TRACE:DATA:DAC VOLATILE,",
    )

    def __init__(self, serial=None, dev=None, timeout_ms=TIMEOUT_MS):
        self.serial = serial or Usbtmc.SERIAL_DEFAULT
        self._dev = dev
        self._timeout_ms = int(timeout_ms)
        self._sweep_trig_cmd = None      # None=未探测, ""=都不支持
        self._arb_data_cmd = None
        self._arb_srat_cmd = None        # None=未探测, ""=不支持
        self._remote_done = False
        self.last_harm_mask = ""         # 上一次写的 :HARMonic:USER 掩码
        self.last_harm_tones = []        # 上一次的 [(阶次, 幅度Vpp, 相位), ...]

    @property
    def dev(self):
        if self._dev is None:
            self._dev = open_transport(serial=self.serial, timeout_ms=self._timeout_ms)
        return self._dev

    def close(self):
        if self._dev is not None:
            try:
                self._dev.close()
            finally:
                self._dev = None

    # ---- 基础 ---------------------------------------------------------
    def q(self, cmd, maxlines=8):
        return self.dev.q(cmd, maxlines)

    def one(self, cmd, default=""):
        r = self.q(cmd)
        return r[0] if r else default

    def w(self, cmd, settle=0.0):
        self.dev.w(cmd, settle)

    def remote(self):
        """兼容 AFG 驱动的 API；DG4000 没有远程状态。

        实测 00.01.14：``SYST:REM`` 与 ``SYST:LOC`` 都回
        ``-113,"Undefined header; keyword cannot be found"``。AFG-2225 必须
        先进远程才收命令，DG 不需要，所以这里**不发命令**，免得每次设点都往
        错误队列里塞一条 -113。
        """
        self._remote_done = True
        return self

    def local(self):
        self._remote_done = False
        return self

    def clear_status(self):
        self.w("*CLS", 0.2)

    def resync(self, limit=32, drain=True):
        """把仪器与 USBTMC 会话拉回同步，返回被丢弃的错误条数。

        ``-410,"Query INTERRUPTED"`` 是"上一条应答还没读完就又发了命令"留下的
        后遗症：一旦错位，之后每条查询都会拿到**上一条**的答案，看起来数值都
        在合理范围内，其实全错了一格。实测在树莓派上用 select 探针写过几条
        查询没读，就攒出十几条 -410 和一次 -350 队列溢出。

        步骤：``*CLS`` 清状态 → 丢掉 IN 通道残留 → 把错误队列抽空。
        """
        try:
            self.w("*CLS", 0.1)
        except DgError:
            pass
        dev = self._dev
        if drain and dev is not None and hasattr(dev, "flush_input"):
            try:
                dev.flush_input()
            except Exception:
                pass
        dropped = 0
        for _ in range(limit):
            try:
                r = self.one(":SYST:ERR?")
            except DgError:
                break
            if not r:
                break
            txt = r.strip()
            if txt.startswith("+0") or txt.startswith("0,") or "no error" in txt.lower():
                break
            dropped += 1
        return dropped

    def errors(self, limit=12):
        out = []
        for _ in range(limit):
            r = self.q("SYST:ERR?")
            if not r:
                break
            txt = " ".join(r)
            if r[0].startswith("+0") or "no error" in txt.lower():
                break
            out.append(txt)
        return out

    # ---- 状态 ---------------------------------------------------------
    def state(self, ch=1):
        p = ":SOUR%d:" % int(ch)
        unit = self.one(p + "VOLT:UNIT?")
        amp = self.one(p + "VOLT?")
        # 单位可能是 VRMS/DBM，:VOLT? 回的是当前单位下的值；用 HIGH-LOW 反算 Vpp
        if not unit or unit.strip().upper() != "VPP":
            try:
                hi = float(self.one(p + "VOLT:HIGH?", "nan"))
                lo = float(self.one(p + "VOLT:LOW?", "nan"))
                if hi == hi and lo == lo:
                    amp = "%.6E" % (hi - lo)
            except (TypeError, ValueError):
                pass
        st = {
            "func": self.one(p + "FUNC?"),
            "freq_hz": self.one(p + "FREQ?"),
            "amp_vpp": amp,
            "offset_v": self.one(p + "VOLT:OFFS?"),
            "unit": unit,
            "output": self.one(":OUTP%d?" % int(ch)),
            "load": self.one(":OUTP%d:LOAD?" % int(ch)),
            "phase": self.one(p + "PHAS?"),
            "sweep_on": self.one(p + "SWE:STAT?"),
            "sweep_start": self.one(p + "FREQ:STAR?"),
            "sweep_stop": self.one(p + "FREQ:STOP?"),
            "sweep_time": self.one(p + "SWE:TIME?"),
            "sweep_space": self.one(p + "SWE:SPAC?"),
            "sweep_src": self._sweep_src_read(int(ch)),
        }
        if st["func"].upper().startswith("HARM"):
            st["harm_on"] = "ON"
            st["harm_order"] = self.one(p + "HARM:ORDER?")
            st["harm_type"] = self.one(p + "HARM:TYPE?")
            try:
                order = int(float(st["harm_order"]))
            except (TypeError, ValueError):
                order = 2
            st["harm_amp"] = self.one(p + "HARM:AMPL? %d" % order)
            st["harm_phase"] = self.one(p + "HARM:PHAS? %d" % order)
            # :HARMonic:USER? 手册未列、实测是写-only，查询会卡 5 s 楔住 USB，
            # 所以掩码只能记在本进程里，不回读仪器
            st["harm_mask"] = self.last_harm_mask
        else:
            # 非谐波模式下 :HARM:AMPL? 不带阶数会回 -220，这里不去碰仪器
            st["harm_on"] = "OFF"
            st["harm_order"] = ""
            st["harm_type"] = ""
            st["harm_amp"] = ""
            st["harm_phase"] = ""
            st["harm_mask"] = ""
        return st

    def _sweep_src_read(self, ch):
        for tpl in (":SOUR%d:SWE:TRIG:SOUR?", ":TRIG%d:SOUR?"):
            try:
                r = self.q(tpl % ch, maxlines=1)
            except DgError:
                continue
            if r:
                return r[0]
        return ""

    # ---- 设置 ---------------------------------------------------------
    def set_sine(self, hz, vpp, ch=1, offset=0.0):
        p = ":SOUR%d:" % int(ch)
        # 实测：背靠背零间隔写会被固件判 -220 Parameter error，必须留 settle
        self.w(p + "FUNC SIN", 0.3)
        self.w(p + "VOLT:UNIT VPP", 0.2)
        self.w(p + "FREQ %.10g" % float(hz), 0.25)
        self.w(p + "VOLT %.10g" % float(vpp), 0.25)
        self.w(p + "VOLT:OFFS %.10g" % float(offset), 0.25)

    def set_load(self, ohms, ch=1):
        """设置输出负载（``:OUTP<n>:LOAD``），``None`` = ``INFINITY``（高阻）。

        实测（板子高阻输入、firmware 00.01.14）：

        * ``LOAD 50``（出厂默认）时实际输出 **等于设定值**：``:VOLT 0.4`` 测到
          峰值 200 mV、``HARM:AMPL 3,0.1`` 测到峰值 50 mV；
        * ``LOAD INFINITY`` 时实际输出只有设定值的一半。

        所以本机一律保持 ``LOAD 50``，不要想当然按"高阻"去写 INFINITY。
        """
        v = "INFINITY" if ohms is None else ("%d" % int(ohms))
        self.w(":OUTP%d:LOAD %s" % (int(ch), v), 0.2)

    def set_harmonic(self, f1_hz, vpp, order, ch=1, phase_deg=None,
                     harm_amp=None):
        """基波 + 第 ``order`` 次谐波（DG4202 的 HARMONIC 模式）。

        这是 :meth:`set_harmonics` 的单阶快捷方式：``harm_amp`` 缺省取
        基波的一半（即该阶与基波等幅）。

        实测方言（固件 00.01.14，手册 + 真机双向确认）：

        * 谐波模式用 ``:SOUR<n>:APPL:HARM`` 进入（``FUNC HARM`` 也吃）；
          没有 ``HARM:STAT``，退出用 ``FUNC SIN``。
        * **``:HARM:USER`` 掩码是总开关**，默认 ``X000000000000000`` 全关，
          只写 ``HARM:AMPL`` 不会出谐波（见 :func:`harm_mask`）。
        * ``HARM:AMPL <sn>,<Vpp>`` / ``HARM:PHAS <sn>,<deg>`` 必须带阶数；
          单位是绝对 Vpp，不是百分比；查询也要带阶数（``HARM:AMPL? 3``），
          否则回 -220。
        * 次数范围 **2..16**：写 40 不报错但被静默钳位到 16。40 次谐波
          （25 kHz × 40 = 1 MHz）只能 :meth:`load_arb` 合成。
        * 没写过的阶次默认 **1.2647 Vpp**（不是 0），多阶输出前必须显式清零。
        * 写完掩码后波形要 2.5~4 s 才重建完，期间测出来是旧波形。
        """
        order = int(order)
        if order < 2 or order > HARM_MAX_ORDER:
            raise DgError(
                "DG4202 谐波次数实测只能是 2..%d（要 %d 次）；更高次谐波请用 "
                "load_arb 合成基波+谐波" % (HARM_MAX_ORDER, order))
        if harm_amp is None:
            harm_amp = float(vpp) / 2.0
        self.set_harmonics(f1_hz, vpp, [(order, float(harm_amp), phase_deg)],
                           ch=ch)

    def set_wave(self, kind, hz, vpp, ch=1, offset=0.0, phase_deg=None):
        """按波形名设点（仿 afg2225 的 ``set_wave``，全测试管线要用）。

        ``kind`` 收 afg2225 那套名字：``sine``/``sin``、``square``/``squ``、
        ``triangle``/``tri``、``ramp``、``pulse``/``puls``、``noise``/``nois``。

        实测方言（固件 00.01.14）：

        * ``:FUNC SQUARE`` 接受，``:FUNC?`` 回 ``SQU``；
        * DG4200 **没有独立的三角波**：``:FUNC TRIANGLE``/``:FUNC TRI`` 都回
          ``RAMP``（而且不报错，只是待在 RAMP）。三角波 = ``:FUNC RAMP`` +
          ``:FUNC:RAMP:SYMM 50``。注意对称度节点前面必须带 ``FUNC:``，
          ``:SOUR1:RAMP:SYMM`` 会回 -113。
        """
        p = ":SOUR%d:" % int(ch)
        k = str(kind).strip().lower()
        if k in ("sin", "sine"):
            self.w(p + "FUNC SIN", 0.3)
        elif k in ("squ", "square", "sq"):
            self.w(p + "FUNC SQUARE", 0.3)
        elif k in ("tri", "triangle", "ramp"):
            self.w(p + "FUNC RAMP", 0.3)
            self.w(p + "FUNC:RAMP:SYMM 50", 0.2)     # 50% 对称 = 三角波
        elif k in ("puls", "pulse"):
            self.w(p + "FUNC PULSE", 0.3)
        elif k in ("nois", "noise"):
            self.w(p + "FUNC NOISE", 0.3)
        else:
            raise DgError("DG4202 不认识的波形 %r（支持 sine/square/triangle/"
                          "pulse/noise）" % (kind,))
        self.w(p + "VOLT:UNIT VPP", 0.2)
        self.w(p + "FREQ %.10g" % float(hz), 0.25)
        self.w(p + "VOLT %.10g" % float(vpp), 0.25)
        self.w(p + "VOLT:OFFS %.10g" % float(offset), 0.25)
        if phase_deg is not None:
            self.w(p + "PHAS %.10g" % (float(phase_deg) % 360.0), 0.25)

    def set_harmonics(self, f1_hz, vpp, tones, ch=1, zero_unused=True,
                      use_type_user=True):
        """基波 + **多阶**谐波同时输出（验收矩阵里的 h3+h4、h10+h17 这类）。

        ``tones`` = ``[(阶次, 幅度Vpp, 相位deg或None), ...]``。

        实测要点（固件 00.01.14，手册 + 树莓派 /dev/usbtmc0 上真机确认）：

        * **``:HARM:USER`` 掩码是谐波输出的总开关**：默认 ``X000000000000000``
          全关时，``HARM:AMPL`` 写了、回读对、错误队列干净，输出却只有基波。
          本方法会按 ``tones`` 自动拼掩码并在最后一步写入。
        * 每阶幅度**各自独立存储**：写 ``HARM:AMPL 3,0.15`` 不会清掉 ``4,0.10``，
          改 ``HARM:ORDER`` 也不清；
        * 但**没写过的阶次默认不是 0，而是 1.2647 Vpp**（回读 6.32361E-01，
          是峰值）。所以多阶同时输出时**必须把不用的阶次显式写 0**，否则输出
          里会多出一堆 1.26 V 的谐波；
        * 相位默认也不是 0（实测第 4 阶默认 90°），有用阶次一律显式写；
        * 阶次上限 16：``HARM:AMPL 17,x`` 回 ``-220,"Parameter error"``，
          ``HARM:AMPL? 17`` 会卡 5 s 后读超时。17 次及以上只能 load_arb 合成；
        * 写完掩码后波形重建要 2.5~4 s（``HARM_SETTLE``），期间读数还是旧的。
        """
        p = ":SOUR%d:" % int(ch)
        clean = []
        for item in tones:
            k = int(item[0])
            amp = float(item[1])
            ph = item[2] if len(item) > 2 else None
            if k < 2 or k > HARM_MAX_ORDER:
                raise DgError(
                    "DG4202 谐波次数实测只能 2..%d（要 %d 次）；17 次以上请用 "
                    "load_arb 合成基波+谐波" % (HARM_MAX_ORDER, k))
            clean.append((k, amp, ph))
        if not clean:
            raise DgError("set_harmonics 需要至少一个 (阶次, 幅度) 分量")

        order = max(k for k, _, _ in clean)
        mask = harm_mask([k for k, _, _ in clean])
        self.w(p + "FUNC SIN", 0.5)              # 先退出 HARM/USER
        self.w(p + "VOLT:UNIT VPP", 0.3)
        self.set_load(50, ch=ch)                 # 实测 LOAD 50 才等于设定值
        if use_type_user:
            self.w(p + "HARM:TYPE USER", 1.0)    # USER = 只输出掩码里点亮的阶次
        self.w(p + "HARM:ORDER %d" % order, 0.5)

        used = {k for k, _, _ in clean}
        if zero_unused:
            for k in range(2, HARM_MAX_ORDER + 1):
                if k not in used:
                    self.w(p + "HARM:AMPL %d,0" % k, 0.12)
                    self.w(p + "HARM:PHAS %d,0" % k, 0.12)

        # 实测：APPL:HARM 之后写 AMPL/PHAS 才生效，所以顺序是"先进入、再填表"
        self.w(p + "APPL:HARM", 1.0)
        self.w(p + "FREQ %.10g" % float(f1_hz), 0.3)
        self.w(p + "VOLT %.10g" % float(vpp), 0.3)
        self.w(p + "VOLT:OFFS 0", 0.2)
        for k, amp, ph in clean:
            self.w(p + "HARM:AMPL %d,%.10g" % (k, amp), 0.2)
            # 相位必须显式写：默认不是全 0（实测 h4 默认回 90°）
            self.w(p + "HARM:PHAS %d,%.10g" % (
                k, 0.0 if ph is None else float(ph)), 0.2)
        # 最后一步：写掩码 = 真正打开谐波输出，并等波形重建
        self.w(p + "HARM:USER %s" % mask, HARM_SETTLE)
        self.last_harm_mask = mask
        self.last_harm_tones = list(clean)

        # 实测偶发：仪器在重建期间会吞掉一两条 AMPL/PHAS 写入（读回还是旧值），
        # 而掩码写入本身不报错。相位被吞更隐蔽——各阶幅度全对，只有合成波形的
        # 峰峰值偏大（相位没对齐）。这里把幅度和相位都读回核对，不对就补写一轮。
        for _ in range(2):
            bad = []
            for k, amp, ph in clean:
                want_ph = 0.0 if ph is None else float(ph)
                try:
                    got_a = float(self.one(p + "HARM:AMPL? %d" % k, "nan"))
                except (DgError, TypeError, ValueError):
                    got_a = float("nan")
                try:
                    got_p = float(self.one(p + "HARM:PHAS? %d" % k, "nan"))
                except (DgError, TypeError, ValueError):
                    got_p = float("nan")
                if not (abs(got_a - amp) <= 1e-6 * max(1.0, abs(amp))):
                    bad.append((k, "AMPL", amp, got_a))
                elif not (abs((got_p - want_ph + 180.0) % 360.0 - 180.0) <= 0.05):
                    bad.append((k, "PHAS", want_ph, got_p))
            if not bad:
                break
            log.warning("谐波表被吞，补写: %s", bad)
            for k, amp, ph in clean:
                self.w(p + "HARM:AMPL %d,%.10g" % (k, amp), 0.2)
                self.w(p + "HARM:PHAS %d,%.10g" % (
                    k, 0.0 if ph is None else float(ph)), 0.2)
            self.w(p + "HARM:USER %s" % mask, HARM_SETTLE)

    def read_harmonics(self, ch=1, limit=HARM_MAX_ORDER):
        """回读各阶幅度/相位，返回 ``{k: (amp_str, phase_str)}``（2..limit）。"""
        p = ":SOUR%d:" % int(ch)
        out = {}
        for k in range(2, int(limit) + 1):
            try:
                amp = self.one(p + "HARM:AMPL? %d" % k)
            except DgError:
                amp = ""
            try:
                ph = self.one(p + "HARM:PHAS? %d" % k)
            except DgError:
                ph = ""
            out[k] = (amp, ph)
        return out

    def harmonic_off(self, ch=1):
        """退出谐波模式。DG4000 没有 ``HARM:STAT``，切回正弦即关闭。"""
        self.w(":SOUR%d:FUNC SIN" % int(ch), 1.0)

    def _apply_sweep_trig(self, ch):
        """探测并写入扫频触发源。只探一次，成功模板缓存在 ``_sweep_trig_cmd``。"""
        if self._sweep_trig_cmd:
            self.w(self._sweep_trig_cmd % ch)
            return True
        try:
            self.errors()                 # 先清空错误队列，避免旧错误误判
        except DgError:
            pass
        for tpl in self._SWEEP_TRIG_CMDS:
            cmd = tpl % ch
            try:
                self.w(cmd, 0.3)
                if not self.errors():
                    self._sweep_trig_cmd = tpl
                    return True
            except DgError:
                continue
        self._sweep_trig_cmd = ""
        log.warning("DG4202: 扫频触发源命令均未被接受（已试 %s），扫频可能无法启动",
                    [t % ch for t in self._SWEEP_TRIG_CMDS])
        return False

    def set_sweep(self, f_start, f_stop, seconds, space="LOG", ch=1):
        p = ":SOUR%d:" % int(ch)
        space = (space or "LOG").upper()
        if space not in ("LOG", "LIN"):
            raise DgError("扫频空间只能是 LOG 或 LIN，收到 %r" % (space,))
        self.w(p + "SWE:STAT OFF", 0.2)
        self.w(p + "FREQ:STAR %.10g" % float(f_start), 0.25)
        self.w(p + "FREQ:STOP %.10g" % float(f_stop), 0.25)
        self.w(p + "SWE:SPAC " + space, 0.2)
        self.w(p + "SWE:TIME %.10g" % float(seconds), 0.25)
        self._apply_sweep_trig(int(ch))
        # 实测：SWE:STAT ON 之后仪器要启动扫频引擎，立即查询偶发 3 s 无应答
        self.w(p + "SWE:STAT ON", 0.8)

    def sweep_off(self, ch=1):
        self.w(":SOUR%d:SWE:STAT OFF" % int(ch))

    def output(self, on, ch=1):
        # 与 AFG 一样只认 ON/OFF；"OUTP1 1" 在部分固件上静默无效。
        self.w(":OUTP%d %s" % (int(ch), "ON" if on else "OFF"), 0.35)

    # ---- ARB ----------------------------------------------------------
    def _download_arb(self, ch, body, settle):
        if self._arb_data_cmd:
            self.w(self._arb_data_cmd % ch + body, settle)
            return self._arb_data_cmd
        try:
            self.errors()
        except DgError:
            pass
        for tpl in self._ARB_DATA_CMDS:
            self.w(tpl % ch + body, settle)
            try:
                errs = self.errors()
            except DgError as exc:
                raise DgError("ARB 数据下发后读错误队列失败：%s" % exc)
            if not errs:
                self._arb_data_cmd = tpl
                return tpl
            log.warning("DG4202: ARB 数据命令被拒（%s）：%s", tpl % ch, errs)
        raise DgError(
            "DG4202 不接受任何已知的 ARB 数据命令（:DATA:DAC / :DATA / "
            ":TRACE:DATA:DAC），固件方言不匹配")

    def _apply_srat(self, ch, srate):
        """``:FUNC:ARB:SRAT`` 是否被接受随固件而异；探测一次后缓存。"""
        if self._arb_srat_cmd == "":
            return False
        if self._arb_srat_cmd:
            self.w(self._arb_srat_cmd % (ch, srate), 0.2)
            return True
        cmd = ":SOUR%d:FUNC:ARB:SRAT %.10g" % (ch, float(srate))
        try:
            self.w(cmd, 0.2)
            if not self.errors():
                self._arb_srat_cmd = ":SOUR%d:FUNC:ARB:SRAT %.10g"
                return True
        except DgError:
            pass
        self._arb_srat_cmd = ""
        log.warning("DG4202: :FUNC:ARB:SRAT 不被接受，采样率交给 FREQ 自动换算")
        return False

    def load_arb(self, vals, ch=1, freq_hz=100000.0, amp_vpp=0.45,
                 offset=0.0, settle=1.2):
        """下载一周期波形到 USER 内存并输出。

        DG4000 方言：2..16384 点、14 bit（-8192..8191）、采样率上限 500 MSa/s。
        顺序不能变（实测）：

            FUNC SIN -> DATA:DAC VOLATILE,<ints> -> FREQ -> VOLT -> VOLT:OFFS
            -> FUNC USER -> (FUNC:ARB:SRAT) -> OUTP ON

        先 ``FUNC SIN`` 退出 USER 才能改 ARB 数据；FREQ/幅度必须在切 USER 之前
        写好，因为 ``FUNC USER`` 会把频率复位（和 AFG-2225 的坑完全一样）。
        ``FUNC:ARB:SRAT`` 不是所有固件都认，探不到就靠 FREQ 反算采样率。

        幅度：``amp_vpp`` 是**峰峰值**，ARB 模式下 ``:VOLT`` 也就是峰峰值
        （实测写 0.25 → upp 249.8 mV），和 SIN 模式一致。

        数据格式（实测踩过的坑，务必看）：``:DATA:DAC VOLATILE`` 的十进制
        ``<value>`` 是**偏移二进制 0..16383（0x0000..0x3FFF），8192 才是零电平**，
        不是 -8192..+8191 的有符号码。早先按有符号码发，负半周被当成 8192 以上
        的高码值回卷，输出变成"全波整流"——峰峰值看着只有一半、还凭空多出
        h2/h1≈0.424、h4/h1≈0.086（纯正弦也一样），谐波验收全错。这里统一 +8192。
        """
        v = [float(x) for x in vals]
        n = len(v)
        if n < 2 or n > ARB_MAX_POINTS:
            raise DgError("ARB 点数必须在 2..%d，当前 %d" % (ARB_MAX_POINTS, n))
        peak = max(abs(x) for x in v)
        if peak <= 0:
            raise DgError("ARB 波形全零")
        data = [int(round(x / peak * ARB_FULL_SCALE)) + ARB_CODE_MID for x in v]
        data = [max(0, min(ARB_CODE_MAX, d)) for d in data]
        body = ",".join(str(d) for d in data)

        f_int = int(math.floor(float(freq_hz)))
        if f_int <= 0:
            raise DgError("ARB 重复频率必须大于 0（当前 %r）" % (freq_hz,))
        srate = float(f_int) * n
        if srate > SRATE_MAX + 0.5:
            raise DgError(
                "采样率 %.3f MHz 超过 DG4202 的 %.0f MSa/s（%d 点 × %d Hz）"
                % (srate / 1e6, SRATE_MAX / 1e6, n, f_int))
        if srate < SRATE_MIN:
            raise DgError(
                "采样率 %.3g Sa/s 低于 DG4202 下限 %.0f uSa/s（%d 点 × %d Hz）"
                % (srate, SRATE_MIN * 1e6, n, f_int))

        p = ":SOUR%d:" % int(ch)
        self.w(p + "FUNC SIN", 0.4)              # 先退出 USER 才能写 DATA
        self._download_arb(int(ch), body, settle)
        self.w(p + "VOLT:UNIT VPP", 0.2)
        self.w(p + "FREQ %.10g" % f_int, 0.4)
        # 数据按偏移二进制（0..16383，8192=零电平）下发后，ARB 的 :VOLT 就是
        # 峰峰值本身：写 0.25 实测 upp=249.8 mV。早先"ARB 的 :VOLT 是幅度、
        # 要乘 2"的结论是数据格式错误（负半周回卷成全波整流，幅度减半）的假象，
        # 已作废。
        self.w(p + "VOLT %.6f" % float(amp_vpp), 0.4)
        self.w(p + "VOLT:OFFS %.4f" % float(offset), 0.3)
        self.w(p + "FUNC USER", 0.5)             # 必须在 FREQ/幅度之后
        self._apply_srat(int(ch), srate)
        self.output(True, ch=int(ch))
        try:
            errs = self.errors()
        except DgError:
            errs = []
        return {"n": n, "rate_hz": srate, "freq_hz": float(f_int),
                "amp_vpp": float(amp_vpp), "errors": errs}

    def identify(self):
        return self.one("*IDN?", "")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _fmt(st):
    order = ["func", "freq_hz", "amp_vpp", "offset_v", "unit", "output",
             "phase", "sweep_on", "sweep_start", "sweep_stop", "sweep_time",
             "sweep_space", "sweep_src", "harm_on", "harm_order", "harm_type",
             "harm_amp", "harm_phase"]
    return "\n".join("  %-12s %s" % (k, st.get(k, "")) for k in order)


def _read_arb_file(path):
    with open(path, "r", encoding="utf-8") as fh:
        text = fh.read()
    vals = []
    for tok in text.replace(",", " ").split():
        vals.append(float(tok))
    return vals


def main(argv=None):
    # Windows 管道/重定向默认 GBK，父进程按 UTF-8 读会花屏；CLI 输出统一 UTF-8
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description="RIGOL DG4202 SCPI 驱动（USBTMC/WinUSB）")
    ap.add_argument("--serial", default=None, help="仪器序列号（默认 %s）" % SERIAL_DEFAULT)
    ap.add_argument("--ch", type=int, default=1, choices=(1, 2))
    ap.add_argument("--state", action="store_true", help="回读通道状态")
    ap.add_argument("--idn", action="store_true")
    ap.add_argument("--set", nargs=2, metavar=("HZ", "VPP"), help="正弦频率/幅度")
    ap.add_argument("--offset", type=float, default=0.0)
    ap.add_argument("--sweep", nargs=3, metavar=("START", "STOP", "SEC"))
    ap.add_argument("--space", default="LOG", choices=("LOG", "LIN"))
    ap.add_argument("--sweep-off", action="store_true")
    ap.add_argument("--on", action="store_true")
    ap.add_argument("--off", action="store_true")
    ap.add_argument("--arb", metavar="FILE", help="逗号/空白分隔的浮点文件")
    ap.add_argument("--harmonic", nargs=3, metavar=("F1", "VPP", "ORDER"),
                    help="基波频率/幅度/谐波次数（2..16）")
    ap.add_argument("--harm-amp", type=float, help="该次谐波幅度 Vpp")
    ap.add_argument("--harm-phase", type=float, help="该次谐波相位 deg")
    ap.add_argument("--harm-off", action="store_true", help="退出谐波模式")
    ap.add_argument("--list", action="store_true", help="枚举 USB 设备")
    a = ap.parse_args(argv)

    if a.list:
        try:
            list_usb()
        except DgError as exc:
            print("ERROR: %s" % exc, file=sys.stderr)
            return 1
        return 0

    try:
        d = Scpi(a.serial)
        try:
            if a.idn:
                print(d.identify())
            if a.set:
                d.set_sine(float(a.set[0]), float(a.set[1]), a.ch, a.offset)
                print("set: sine %s Hz, %s Vpp, offset %s V"
                      % (a.set[0], a.set[1], a.offset))
            if a.sweep:
                d.set_sweep(float(a.sweep[0]), float(a.sweep[1]),
                            float(a.sweep[2]), a.space, a.ch)
                print("sweep: %s -> %s Hz in %s s (%s)"
                      % (a.sweep[0], a.sweep[1], a.sweep[2], a.space))
            if a.sweep_off:
                d.sweep_off(a.ch)
                print("sweep off")
            if a.on:
                d.output(True, a.ch)
                print("output ON")
            if a.off:
                d.output(False, a.ch)
                print("output OFF")
            if a.arb:
                r = d.load_arb(_read_arb_file(a.arb), ch=a.ch, offset=a.offset)
                print("arb: %s" % (r,))
            if a.harmonic:
                d.set_harmonic(float(a.harmonic[0]), float(a.harmonic[1]),
                               int(a.harmonic[2]), a.ch,
                               phase_deg=a.harm_phase, harm_amp=a.harm_amp)
                print("harmonic: f1=%s Hz, %s Vpp, order %s"
                      % (a.harmonic[0], a.harmonic[1], a.harmonic[2]))
            if a.harm_off:
                d.harmonic_off(a.ch)
                print("harmonic off")
            if a.state or not any([a.idn, a.set, a.sweep, a.sweep_off,
                                   a.on, a.off, a.arb, a.harmonic, a.harm_off]):
                for ch in ([a.ch] if not a.state else (1, 2)):
                    print("ch%d:" % ch)
                    print(_fmt(d.state(ch)))
            errs = d.errors()
            print("errors:", errs if errs else "none")
        finally:
            d.close()
    except DgError as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        return 1
    return 0


def load_arb(vals, ch=1, freq_hz=100000.0, amp_vpp=0.45, offset=0.0,
             settle=1.2, s=None):
    """模块级便捷入口（与 afg2225.load_arb 对应）。"""
    own = s is None
    if own:
        s = Scpi()
    try:
        return s.load_arb(vals, ch=ch, freq_hz=freq_hz, amp_vpp=amp_vpp,
                          offset=offset, settle=settle)
    finally:
        if own:
            s.close()


if __name__ == "__main__":
    sys.exit(main())
