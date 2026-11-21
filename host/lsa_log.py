# -*- coding: utf-8 -*-
"""板载掉电日志（W25Q16 32KB 环形区）。

为什么要有它：树莓派上的监测程序一旦系统卡死/重刷，/var/log 里的东西就读不出来
了。这里把日志实时推到 STM32 自己 Flash 上，板子同时也从调试串口打一遍，所以
哪怕 Pi 彻底不启动，把板子接到串口仍能看到最后一次报错。

用法（树莓派）::

    python3 lsa_log.py info            # 板子上现在有多少条、多少扇区
    python3 lsa_log.py dump            # 全部导出（按序号，从旧到新）
    python3 lsa_log.py dump --from 120 # 只导出序号 >= 120 的
    python3 lsa_log.py tail            # 用 1 Hz 轮询持续打印新日志
    python3 lsa_log.py put ERR "XXX 挂了"
    python3 lsa_log.py clear
    python3 lsa_log.py watch /home/frozen-laffer/lsa/monitor.log   # 转发某个日志文件

监测程序里也可以直接 import::

    import lsa_log
    log = lsa_log.Logger()
    log.info("启动完成")
    log.err("AFG 打开失败：%s" % e)
"""
import os
import re
import sys
import time

import lsa_proto as proto

HOST = os.environ.get("LSA_HOST", "192.168.57.200")
PORT = int(os.environ.get("LSA_PORT", "5000"))

LEVEL_DEBUG = 0
LEVEL_INFO = 1
LEVEL_WARN = 2
LEVEL_ERR = 3

LEVEL_NAME = {0: "DEBUG", 1: "INFO", 2: "WARN", 3: "ERR"}

# 板子单条正文上限（固件里 LOG_TEXT_MAX = 120）
TEXT_MAX = 120


def guess_level(text):
    """粗判级别：够用就行，宁可当 INFO 也别把正常行标成错误。"""
    low = text.lower()
    if any(k in low for k in ("error", "err:", "traceback", "exception",
                              "失败", "错误", "异常", "超时", "timeout",
                              "refused", "cannot", "unable")):
        return LEVEL_ERR
    if any(k in low for k in ("warn", "警告", "重试", "retry", "丢包")):
        return LEVEL_WARN
    return LEVEL_INFO


class Logger(object):
    """日志发送器。

    默认**短连接**（每条连一次、写完就关）：板子只允许一条 TCP 连接，长期占着
    会让主界面连不上。本地链路一次握手 0.1 ms，可以忽略。要高频连续灌日志
    （比如 watch 转发）时传 persistent=True。
    """

    def __init__(self, host=HOST, port=PORT, tag="", persistent=False):
        self.host = host
        self.port = port
        self.tag = tag
        self.persistent = persistent
        self.c = None

    def _conn(self):
        if self.c is None:
            self.c = proto.LsaClient(self.host, self.port)
            self.c.connect(wait=5.0)
        return self.c

    def close(self):
        if self.c is not None:
            try:
                self.c.close()
            except Exception:
                pass
            self.c = None

    def put(self, level, text):
        text = "" if text is None else str(text)
        if self.tag:
            text = "[%s] %s" % (self.tag, text)
        text = text.rstrip("\r\n").replace("\r", " ")
        if len(text) > TEXT_MAX:
            text = text[:TEXT_MAX - 1] + "~"
        for attempt in (0, 1):
            try:
                self._conn().log_put(level, text)
                if not self.persistent:
                    self.close()
                return True
            except Exception:
                self.close()
                if attempt:
                    return False
                time.sleep(0.2)
        return False

    def debug(self, text):
        return self.put(LEVEL_DEBUG, text)

    def info(self, text):
        return self.put(LEVEL_INFO, text)

    def warn(self, text):
        return self.put(LEVEL_WARN, text)

    def err(self, text):
        return self.put(LEVEL_ERR, text)


# ── 一次性操作（每次新建连接，适合命令行） ─────────────────────────────

def _once(fn):
    c = proto.LsaClient(HOST, PORT)
    c.connect(wait=6.0)
    try:
        return fn(c)
    finally:
        c.close()


def info():
    return _once(lambda c: c.log_info())


def dump(from_seq=1, chunk=800):
    """把 >= from_seq 的日志全部取回来，返回 [(seq, level, text), ...]。"""
    out = []
    cur = from_seq
    for _ in range(200):                      # 防御性上限，正常两三趟就完
        txt = _once(lambda c: c.log_read(cur, chunk))
        if not txt:
            break
        got = 0
        for line in txt.splitlines():
            line = line.strip("\r")
            if not line:
                continue
            parts = line.split("\t", 2)
            if len(parts) < 3:
                continue
            try:
                seq = int(parts[0])
            except ValueError:
                continue
            lvl = parts[1][1:] if parts[1].startswith("L") else parts[1]
            out.append((seq, int(lvl), parts[2]))
            cur = max(cur, seq + 1)
            got += 1
        if got == 0:
            break
    return out


def clear():
    return _once(lambda c: c.log_clear())


def tail(interval=1.0):
    try:
        seq, _sec = info()
        cur = max(1, seq - 20)                # 先给最后 20 条上下文
        print("从 #%d 开始跟随（Ctrl-C 退出）" % cur, flush=True)
        while True:
            for s, lvl, txt in dump(cur):
                print("%6d %-5s %s" % (s, LEVEL_NAME.get(lvl, lvl), txt),
                      flush=True)
                cur = s + 1
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n停止")


def watch(path, from_end=True, interval=0.5, tag="monitor"):
    """盯住一个日志文件，把新增的行转发到板子（任何程序的日志都能这么接）。"""
    log = Logger(tag=tag, persistent=True)
    log.info("开始转发 %s" % path)
    pos = 0
    if from_end and os.path.exists(path):
        pos = os.path.getsize(path)
    while True:
        try:
            if os.path.exists(path):
                size = os.path.getsize(path)
                if size < pos:                # 文件被截断/轮转
                    pos = 0
                if size > pos:
                    with open(path, "r", errors="replace") as f:
                        f.seek(pos)
                        data = f.read()
                        pos = f.tell()
                    for line in data.splitlines():
                        line = line.strip()
                        if line:
                            log.put(guess_level(line), line)
            time.sleep(interval)
        except KeyboardInterrupt:
            break
        except Exception as e:                # 监控本身不能因为异常退出
            log.close()
            time.sleep(1.0)
    log.close()


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 0
    cmd = argv[1]
    if cmd == "info":
        seq, sec = info()
        print("板载日志: %d 条, 已用 %d/8 个扇区 (32KB 环形)" % (seq, sec))
        return 0
    if cmd == "dump":
        frm = 1
        if "--from" in argv:
            frm = int(argv[argv.index("--from") + 1])
        rows = dump(frm)
        for s, lvl, txt in rows:
            print("%6d %-5s %s" % (s, LEVEL_NAME.get(lvl, lvl), txt))
        print("--- 共 %d 条 ---" % len(rows))
        return 0
    if cmd == "tail":
        tail()
        return 0
    if cmd == "put":
        if len(argv) < 4:
            print("用法: lsa_log.py put LEVEL TEXT")
            return 2
        lvl = {"debug": 0, "info": 1, "warn": 2, "err": 3}.get(argv[2].lower(), 1)
        Logger().put(lvl, " ".join(argv[3:]))
        print("已写入板载日志")
        return 0
    if cmd == "clear":
        clear()
        print("已清空")
        return 0
    if cmd == "watch":
        if len(argv) < 3:
            print("用法: lsa_log.py watch /path/to/monitor.log")
            return 2
        watch(argv[2])
        return 0
    print("未知命令:", cmd)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
