# -*- coding: utf-8 -*-
"""把上位机的参数文件打包存进板子 W25Q16（A/B 双槽 + CRC）。

为什么要有这个：树莓派重刷系统后 `eq.json/config.json/calib.json/excite.json`
全没了，只能靠 PC 上的 `pi_backup/` 手工恢复。板子自己存一份，一条命令就能读回来。

blob 格式（小端）：
    'LSPB' | ver u16 | count u16 | 然后每项：name_len u8 | name | len u32 | data

命令行：
    python3 lsa_param.py info
    python3 lsa_param.py save            # 把本地文件推到板子
    python3 lsa_param.py load            # 从板子读回来覆盖本地
    python3 lsa_param.py clear
"""
import glob
import io
import json
import os
import struct
import sys

import lsa_proto as proto

CMD_PARAM_BEGIN = 0x07
CMD_PARAM_CHUNK = 0x08
CMD_PARAM_COMMIT = 0x09
CMD_PARAM_LOAD = 0x0A
CMD_PARAM_INFO = 0x0B
CMD_PARAM_CLEAR = 0x0C

BLOB_MAGIC = b"LSPB"
BLOB_VER = 1
CHUNK = 1024
MAX_BLOB = 8192

BASE = os.path.dirname(os.path.abspath(__file__))
# 这些文件进 blob（有就带上，没有就跳过）
FILES = ("eq.json", "config.json", "calib.json", "excite.json")


class ParamError(Exception):
    pass


# ── blob 打包 / 拆包 ─────────────────────────────────────────────────
def pack(files):
    """files: {name: bytes} -> blob"""
    out = io.BytesIO()
    out.write(BLOB_MAGIC)
    out.write(struct.pack("<HH", BLOB_VER, len(files)))
    for name, data in files.items():
        nb = name.encode("utf-8")
        out.write(struct.pack("<B", len(nb)))
        out.write(nb)
        out.write(struct.pack("<I", len(data)))
        out.write(data)
    return out.getvalue()


def unpack(blob):
    if len(blob) < 8 or blob[:4] != BLOB_MAGIC:
        raise ParamError("blob 头不对（前 4 字节应为 LSPB）")
    ver, cnt = struct.unpack_from("<HH", blob, 4)
    if ver != BLOB_VER:
        raise ParamError("blob 版本 %d 不认识" % ver)
    off = 8
    out = {}
    for _ in range(cnt):
        nl = blob[off]
        off += 1
        name = blob[off:off + nl].decode("utf-8")
        off += nl
        ln = struct.unpack_from("<I", blob, off)[0]
        off += 4
        out[name] = blob[off:off + ln]
        off += ln
    return out


def read_local(base=BASE):
    files = {}
    for fn in FILES:
        p = os.path.join(base, fn)
        if os.path.exists(p):
            files[fn] = io.open(p, "rb").read()
    return files


def write_local(files, base=BASE):
    n = 0
    for name, data in files.items():
        if os.path.basename(name) != name:      # 只写当前目录，防路径穿越
            continue
        p = os.path.join(base, name)
        with io.open(p, "wb") as f:
            f.write(data)
        n += 1
    return n


# ── 板子侧接口 ──────────────────────────────────────────────────────
def _frame(c, cmd, a0=0, a1=0, a2=0, a3=0, n=0):
    tok = c.tok
    c.tok = (c.tok + 1) & 0xFFFFFFFF or 1
    return tok, proto._CMD.pack(proto.LCMD_MAGIC, cmd, a0 & 0xFF, a1 & 0xFF,
                                a2 & 0xFF, a3 & 0xFF, 0, n & 0xFFFF, tok)


def _off4(off):
    return (off & 0xFF, (off >> 8) & 0xFF, (off >> 16) & 0xFF, (off >> 24) & 0xFF)


def _check_ack(h, tok, what):
    if h["tok"] != tok:
        raise ParamError("%s: 事务号不符 %d != %d" % (what, h["tok"], tok))
    if h["type"] != proto.TYPE_ACK:
        raise ParamError("%s: 板子回了 type=%d flags=%d"
                         % (what, h["type"], h["flags"]))


def info(c):
    """返回 (已存字节数, 序号)"""
    tok, fr = _frame(c, CMD_PARAM_INFO)
    c.sock.sendall(fr)
    h = c._recv_header()
    if h["type"] != proto.TYPE_DATA or h["n_this"] < 8:
        return 0, 0
    body = c._recv_exact(h["n_this"])
    size, seq = struct.unpack_from("<II", body, 0)
    return size, seq


def push(c, blob):
    if not blob:
        raise ParamError("没有可存的内容")
    if len(blob) > MAX_BLOB:
        raise ParamError("blob %d 字节超过板子上限 %d" % (len(blob), MAX_BLOB))

    tok, fr = _frame(c, CMD_PARAM_BEGIN, len(blob) & 0xFF,
                     (len(blob) >> 8) & 0xFF, (len(blob) >> 16) & 0xFF,
                     (len(blob) >> 24) & 0xFF)
    c.sock.sendall(fr)
    _check_ack(c._recv_header(), tok, "PARAM_BEGIN")

    nchunk = 0
    for off in range(0, len(blob), CHUNK):
        part = blob[off:off + CHUNK]
        tok, fr = _frame(c, CMD_PARAM_CHUNK, *_off4(off), n=len(part))
        c.sock.sendall(fr + part)
        _check_ack(c._recv_header(), tok, "PARAM_CHUNK@%d" % off)
        nchunk += 1

    tok, fr = _frame(c, CMD_PARAM_COMMIT)
    c.sock.sendall(fr)
    _check_ack(c._recv_header(), tok, "PARAM_COMMIT")
    return len(blob), nchunk


def pull(c):
    size, _seq = info(c)
    if size <= 0:
        return b""
    buf = bytearray()
    while len(buf) < size:
        off = len(buf)
        want = min(CHUNK, size - off)
        tok, fr = _frame(c, CMD_PARAM_LOAD, *_off4(off), n=want)
        c.sock.sendall(fr)
        h = c._recv_header()
        if h["type"] != proto.TYPE_DATA:
            raise ParamError("PARAM_LOAD@%d: type=%d flags=%d"
                             % (off, h["type"], h["flags"]))
        buf += c._recv_exact(h["n_this"])
    return bytes(buf)


def clear(c):
    tok, fr = _frame(c, CMD_PARAM_CLEAR)
    c.sock.sendall(fr)
    _check_ack(c._recv_header(), tok, "PARAM_CLEAR")


# ── 给界面用的两个快捷函数 ──────────────────────────────────────────
def save_from_dir(c, base=BASE):
    files = read_local(base)
    if not files:
        return 0, 0
    return push(c, pack(files))


def missing_local(base=BASE):
    """本地缺哪些参数文件"""
    return [fn for fn in FILES if not os.path.exists(os.path.join(base, fn))]


def restore_to_dir(c, base=BASE, only_missing=True):
    blob = pull(c)
    if not blob:
        return 0
    files = unpack(blob)
    if only_missing:
        files = {k: v for k, v in files.items()
                 if not os.path.exists(os.path.join(base, k))}
    return write_local(files, base)


def main(argv):
    host = "192.168.57.200"
    for i, a in enumerate(argv):
        if a == "--host" and i + 1 < len(argv):
            host = argv[i + 1]
    cmd = "info"
    for a in argv[1:]:
        if a in ("info", "save", "load", "clear"):
            cmd = a
    c = proto.LsaClient(host, 5000)
    c.connect(wait=4.0)
    try:
        if cmd == "info":
            size, seq = info(c)
            print("板子参数区: %d 字节, seq=%d" % (size, seq))
        elif cmd == "save":
            files = read_local()
            print("本地文件:", ", ".join(sorted(files)) or "(无)")
            size, nchunk = push(c, pack(files))
            print("已存 %d 字节 / %d 块" % (size, nchunk))
        elif cmd == "load":
            blob = pull(c)
            files = unpack(blob) if blob else {}
            n = write_local(files)
            print("已从板子恢复 %d 个文件: %s" % (n, ", ".join(sorted(files))))
        elif cmd == "clear":
            clear(c)
            print("已清空")
        else:
            print("用法: lsa_param.py [info|save|load|clear] [--host IP]")
    finally:
        c.close()


if __name__ == "__main__":
    main(sys.argv)
