# -*- coding: utf-8 -*-
"""DG4202 驱动自测：不需要仪器，也不需要 pyusb 真的连上设备。

用假 USBTMC 传输层（记录每条写命令、按内部状态回查询）驱动真正的
``dg4202.Scpi``，断言下发命令的**原文与顺序**。重点盯住几个只有实测才会
发现的方言坑：

  * 幅度节点是 ``:VOLT`` 而不是 AFG 的 ``:AMP``；
  * 扫频起止是 ``:FREQ:STAR/STOP``，不是 ``:SWE:STAR/STOP``；
  * 扫频触发源只认 ``INT``（``IMM`` 回 -220），且只探测一次；
  * ARB 必须 ``FUNC SIN`` 先退出 USER、``FUNC USER`` 最后写（否则频率复位）；
  * 谐波 >16 次固件会静默钳位，驱动必须直接拒绝。

运行：``& python -u _dg_selftest.py``，全过退出码 0，否则 1。
"""
from __future__ import annotations

import sys

import dg4202

# 管道/重定向时 Windows 默认用 GBK 编码，父进程按 UTF-8 读会花屏；统一成 UTF-8
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

FAILED = []
TOTAL = [0]


def check(name, ok, detail=""):
    TOTAL[0] += 1
    if ok:
        print("PASS  %s" % name)
    else:
        print("FAIL  %s%s" % (name, ("  <- %r" % (detail,)) if detail else ""))
        FAILED.append(name)


def sci(v):
    return "%.6E" % float(v)


class FakeUsbtmc(object):
    """假传输层：只实现 Scpi 用到的 q/w/one/close。"""

    def __init__(self):
        self.cmds = []       # 所有写过/查过的命令，按时间顺序
        self.writes = []     # 只记写命令
        self.queries = []    # 只记查询
        self.func = "SIN"
        self.freq = "1.000000E+03"
        self.volt = "5.000000E-01"
        self.offs = "0.000000E+00"
        self.unit = "VPP"
        self.outp = "OFF"
        self.phase = "0.000000E+00"
        self.swe_stat = "OFF"
        self.fstart = "1.000000E+02"
        self.fstop = "1.000000E+03"
        self.swe_time = "1.000000E+00"
        self.swe_spac = "LIN"
        self.swe_trig = "INT"
        self.harm_order = "3.000000E+00"
        self.harm_type = "USER"
        self.harm_amp = "2.000000E-01"
        self.harm_phase = "4.500000E+01"
        self.high = "2.500000E-01"
        self.low = "-2.500000E-01"
        self.arb_data = ""
        self.srat = ""
        self.mask = "X000000000000000"     # :HARM:USER 总开关
        self.load = "50"

    # ---- 写 -----------------------------------------------------------
    def w(self, cmd, settle=0.0):
        self.cmds.append(cmd)
        self.writes.append(cmd)
        c = cmd.strip()
        u = c.upper()
        if u.startswith(":SOUR1:FUNC "):
            v = u.split(None, 1)[1].strip()
            self.func = "HARMONIC" if v == "HARM" else v
        elif u.startswith(":SOUR1:FREQ:STAR "):
            self.fstart = sci(c.split(None, 1)[1])
        elif u.startswith(":SOUR1:FREQ:STOP "):
            self.fstop = sci(c.split(None, 1)[1])
        elif u.startswith(":SOUR1:FREQ "):
            self.freq = sci(c.split(None, 1)[1])
        elif u.startswith(":SOUR1:VOLT:OFFS "):
            self.offs = sci(c.split(None, 1)[1])
        elif u.startswith(":SOUR1:VOLT:UNIT "):
            self.unit = c.split(None, 1)[1].upper()
        elif u.startswith(":SOUR1:VOLT "):
            self.volt = sci(c.split(None, 1)[1])
        elif u.startswith(":OUTP1 "):
            self.outp = c.split(None, 1)[1].upper()
        elif u.startswith(":SOUR1:SWE:STAT "):
            self.swe_stat = c.split(None, 1)[1].upper()
        elif u.startswith(":SOUR1:SWE:TIME "):
            self.swe_time = sci(c.split(None, 1)[1])
        elif u.startswith(":SOUR1:SWE:SPAC "):
            self.swe_spac = c.split(None, 1)[1].upper()
        elif u.startswith(":SOUR1:SWE:TRIG:SOUR "):
            self.swe_trig = c.split(None, 1)[1].upper()
        elif u.startswith(":SOUR1:HARM:TYPE "):
            self.harm_type = c.split(None, 1)[1].upper()
        elif u.startswith(":SOUR1:HARM:ORDER "):
            self.harm_order = sci(c.split(None, 1)[1])
        elif u.startswith(":SOUR1:HARM:AMPL "):
            pair = c.split(None, 1)[1].split(",")
            self.harm_amp = sci(pair[1])
        elif u.startswith(":SOUR1:HARM:PHAS "):
            pair = c.split(None, 1)[1].split(",")
            self.harm_phase = sci(pair[1])
        elif u.startswith(":SOUR1:HARM:USER "):
            self.mask = c.split(None, 1)[1].strip()
        elif u.startswith(":OUTP1:LOAD "):
            self.load = c.split(None, 1)[1].strip().upper()
        elif u.startswith(":SOUR1:APPL:HARM"):
            self.func = "HARMONIC"
        elif u.startswith(":SOUR1:DATA:DAC VOLATILE,"):
            self.arb_data = c.split(",", 1)[1]
        elif u.startswith(":SOUR1:FUNC:ARB:SRAT "):
            self.srat = c.split(None, 1)[1]
        # 其余（:SOUR1:VOLT:UNIT 之外的杂项）忽略

    # ---- 读 -----------------------------------------------------------
    def q(self, cmd, maxlines=8):
        self.cmds.append(cmd)
        self.queries.append(cmd)
        c = cmd.strip()
        u = c.upper()
        if u == "*IDN?":
            return ["Rigol Technologies,DG4202,DG4E202401525,00.01.14"]
        if u == "SYST:ERR?":
            return ['0,"No error"']
        if u.startswith(":SOUR1:HARM:AMPL? "):
            return [self.harm_amp]
        if u.startswith(":SOUR1:HARM:PHAS? "):
            return [self.harm_phase]
        table = {
            ":SOUR1:FUNC?": self.func,
            ":SOUR1:FREQ?": self.freq,
            ":SOUR1:VOLT?": self.volt,
            ":SOUR1:VOLT:OFFS?": self.offs,
            ":SOUR1:VOLT:UNIT?": self.unit,
            ":SOUR1:VOLT:HIGH?": self.high,
            ":SOUR1:VOLT:LOW?": self.low,
            ":OUTP1?": self.outp,
            ":OUTP1:LOAD?": self.load,
            ":SOUR1:PHAS?": self.phase,
            ":SOUR1:SWE:STAT?": self.swe_stat,
            ":SOUR1:FREQ:STAR?": self.fstart,
            ":SOUR1:FREQ:STOP?": self.fstop,
            ":SOUR1:SWE:TIME?": self.swe_time,
            ":SOUR1:SWE:SPAC?": self.swe_spac,
            ":SOUR1:SWE:TRIG:SOUR?": self.swe_trig,
            ":SOUR1:HARM:ORDER?": self.harm_order,
            ":SOUR1:HARM:TYPE?": self.harm_type,
        }
        return [table[u]] if u in table else []

    def one(self, cmd, default=""):
        r = self.q(cmd)
        return r[0] if r else default

    def close(self):
        pass


def new_scpi():
    return dg4202.Scpi(dev=FakeUsbtmc())


# ---------------------------------------------------------------------------
# 1. 正弦
# ---------------------------------------------------------------------------
s = new_scpi()
s.set_sine(1000.0, 0.5)
want = [":SOUR1:FUNC SIN", ":SOUR1:VOLT:UNIT VPP", ":SOUR1:FREQ 1000",
        ":SOUR1:VOLT 0.5", ":SOUR1:VOLT:OFFS 0"]
check("set_sine 命令序列", s.dev.writes == want, s.dev.writes)

st = s.state()
check("state 回读 func", st["func"] == "SIN", st["func"])
check("state 回读 freq", st["freq_hz"] == "1.000000E+03", st["freq_hz"])
check("state 回读 amp", st["amp_vpp"] == "5.000000E-01", st["amp_vpp"])
check("state 用 :FREQ:STAR?/:FREQ:STOP?",
      ":SOUR1:FREQ:STAR?" in s.dev.queries and ":SOUR1:FREQ:STOP?" in s.dev.queries,
      s.dev.queries)
check("state 不用 :SWE:STAR?",
      ":SOUR1:SWE:STAR?" not in s.dev.queries and ":SOUR1:SWE:STOP?" not in s.dev.queries,
      s.dev.queries)
check("state 非谐波时 harm_on=OFF", st["harm_on"] == "OFF", st["harm_on"])

s2 = new_scpi()
s2.dev.unit = ""
s2.dev.volt = ""
s2.dev.high = "4.000000E-01"
s2.dev.low = "-1.000000E-01"
st2 = s2.state()
check("state 单位空时用 HIGH-LOW 反算 Vpp",
      st2["amp_vpp"] == "5.000000E-01", st2["amp_vpp"])
check("state 单位空时仍查询 HIGH/LOW",
      ":SOUR1:VOLT:HIGH?" in s2.dev.queries and ":SOUR1:VOLT:LOW?" in s2.dev.queries,
      s2.dev.queries)

# ---------------------------------------------------------------------------
# 2. 扫频
# ---------------------------------------------------------------------------
s = new_scpi()
s.set_sweep(100.0, 10000.0, 1.5, "LIN")
want = [":SOUR1:SWE:STAT OFF", ":SOUR1:FREQ:STAR 100", ":SOUR1:FREQ:STOP 10000",
        ":SOUR1:SWE:SPAC LIN", ":SOUR1:SWE:TIME 1.5",
        ":SOUR1:SWE:TRIG:SOUR INT", ":SOUR1:SWE:STAT ON"]
check("set_sweep 命令序列", s.dev.writes == want, s.dev.writes)
check("set_sweep 缓存 INT 触发源",
      s._sweep_trig_cmd == ":SOUR%d:SWE:TRIG:SOUR INT", s._sweep_trig_cmd)
nq = len(s.dev.queries)
s.set_sweep(100.0, 20000.0, 2.0, "LOG")
check("触发源只探测一次（第二次无 SYST:ERR?）",
      len(s.dev.queries) == nq, s.dev.queries[nq:])

st = s.state()
check("state 扫频起止", (st["sweep_start"], st["sweep_stop"]) ==
      ("1.000000E+02", "2.000000E+04"), (st["sweep_start"], st["sweep_stop"]))
check("state 触发源 INT", st["sweep_src"] == "INT", st["sweep_src"])

# ---------------------------------------------------------------------------
# 3. 输出
# ---------------------------------------------------------------------------
s = new_scpi()
s.output(True)
check("output ON 用 ON 助记符", s.dev.writes == [":OUTP1 ON"], s.dev.writes)
check("output 回读 ON", s.one(":OUTP1?") == "ON", s.dev.outp)
s.output(False)
check("output OFF 用 OFF 助记符", s.dev.writes[-1] == ":OUTP1 OFF", s.dev.writes)

# ---------------------------------------------------------------------------
# 4. ARB：顺序 FUNC SIN -> DATA -> FREQ/VOLT -> FUNC USER
# ---------------------------------------------------------------------------
s = new_scpi()
vals = [0.0, 0.5, 0.0, -0.5]          # 峰值 0.5 -> 8192,16383,8192,1（偏移二进制）
r = s.load_arb(vals, freq_hz=100000.0, amp_vpp=0.5)
w = s.dev.writes
check("ARB 数据体归一化（偏移二进制 0..16383）",
      s.dev.arb_data == "8192,16383,8192,1", s.dev.arb_data)
check("ARB 下载前先 FUNC SIN",
      w.index(":SOUR1:FUNC SIN") < next(i for i, c in enumerate(w) if c.startswith(":SOUR1:DATA:DAC")),
      w)
check("ARB DATA 在 FREQ 之前",
      next(i for i, c in enumerate(w) if c.startswith(":SOUR1:DATA:DAC"))
      < w.index(":SOUR1:FREQ 100000"), w)
check("ARB FUNC USER 在 FREQ/VOLT 之后",
      w.index(":SOUR1:FUNC USER") > w.index(":SOUR1:FREQ 100000")
      and w.index(":SOUR1:FUNC USER") > max(
          i for i, c in enumerate(w) if c.startswith(":SOUR1:VOLT ")), w)
check("ARB 设置采样率", s.dev.srat == "400000", s.dev.srat)
check("ARB 最后开输出", w[-1] == ":OUTP1 ON", w)
check("ARB 返回值", r["n"] == 4 and r["rate_hz"] == 400000.0
      and r["errors"] == [], r)

# ---------------------------------------------------------------------------
# 5. 谐波
# ---------------------------------------------------------------------------
s = new_scpi()
s.set_harmonic(25000.0, 0.5, 3, harm_amp=0.2, phase_deg=45.0)
write = s.dev.writes
# 关键顺序：先退出谐波 -> 高阻负载 -> 进 HARM -> 填表 -> 最后写 USER 掩码
check("谐波第一条是 FUNC SIN（退出 HARM）", write[0] == ":SOUR1:FUNC SIN", write[0])
check("谐波设 50 欧负载", ":OUTP1:LOAD 50" in write, write[:4])
check("谐波用 APPL:HARM 进入", ":SOUR1:APPL:HARM" in write, write)
check("谐波掩码点亮第 3 阶", s.dev.mask == "X010000000000000", s.dev.mask)
check("掩码是最后一条命令（写完才生效）",
      write[-1] == ":SOUR1:HARM:USER X010000000000000", write[-1])
check("HARM:ORDER 在 AMPL 之前",
      write.index(":SOUR1:HARM:ORDER 3")
      < write.index(":SOUR1:HARM:AMPL 3,0.2"), write)
check("未用阶次被清零（第 2 阶）",
      ":SOUR1:HARM:AMPL 2,0" in write and ":SOUR1:HARM:AMPL 4,0" in write, write)
check("阶次 3 的幅度/相位都对",
      ":SOUR1:HARM:AMPL 3,0.2" in write and ":SOUR1:HARM:PHAS 3,45" in write, write)
check("基波频率/幅度写在 APPL 之后",
      write.index(":SOUR1:FREQ 25000") > write.index(":SOUR1:APPL:HARM")
      and write.index(":SOUR1:VOLT 0.5") > write.index(":SOUR1:APPL:HARM"), write)

nw = len(s.dev.writes)
try:
    s.set_harmonic(25000.0, 0.5, 17)
    rejected = False
except dg4202.DgError:
    rejected = True
check("谐波 17 次被驱动拒绝（不静默钳位）", rejected)
check("拒绝时不发任何命令", len(s.dev.writes) == nw, s.dev.writes[nw:])

st = s.state()
check("state 谐波 func", st["func"] == "HARMONIC", st["func"])
check("state 谐波阶数", st["harm_order"] == "3.000000E+00", st["harm_order"])
check("state 谐波类型", st["harm_type"] == "USER", st["harm_type"])
check("state 谐波幅度", float(st["harm_amp"]) == 0.2, st["harm_amp"])
check("state 谐波相位", float(st["harm_phase"]) == 45.0, st["harm_phase"])
check("state harm_on=ON", st["harm_on"] == "ON", st["harm_on"])
check("state 带回掩码", st["harm_mask"] == "X010000000000000", st["harm_mask"])
check("state 带负载", st["load"] == "50", st["load"])

# 多阶：掩码同时点亮 3/4，且未用阶次清零
s2 = new_scpi()
s2.set_harmonics(29296.875, 0.125, [(3, 0.075, None), (4, 0.05, 90.0)])
check("多阶掩码", s2.dev.mask == "X011000000000000", s2.dev.mask)
check("多阶阶数取最大", ":SOUR1:HARM:ORDER 4" in s2.dev.writes, s2.dev.writes)
check("多阶未指定相位默认写 0",
      ":SOUR1:HARM:PHAS 3,0" in s2.dev.writes, s2.dev.writes)

# ---------------------------------------------------------------------------
# 6. USBTMC 帧头（不碰硬件）
# ---------------------------------------------------------------------------
h = dg4202.Usbtmc._header(1, 7, 13, 1)
check("OUT 头 12 字节", list(h) == [1, 7, 248, 0, 13, 0, 0, 0, 1, 0, 0, 0], list(h))
h = dg4202.Usbtmc._header(2, 255, 1024, 0)
check("IN 头 MsgID=2/大小 LE32", list(h) == [2, 255, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0], list(h))
u = object.__new__(dg4202.Usbtmc)
u._tag = 254
check("bTag 255 后回绕到 1", (u._next_tag(), u._next_tag()) == (255, 1))

# ---------------------------------------------------------------------------
# 7. WinUSB 提示
# ---------------------------------------------------------------------------
check("WinUSB 提示含 Zadig", "Zadig" in dg4202.WINUSB_HINT
      and "WinUSB" in dg4202.WINUSB_HINT, dg4202.WINUSB_HINT)

# ---------------------------------------------------------------------------
print("-" * 60)
if FAILED:
    print("FAILED %d/%d 项：" % (len(FAILED), TOTAL[0]))
    for name in FAILED:
        print("  - %s" % name)
    sys.exit(1)
print("ALL PASS (%d 项)" % TOTAL[0])
