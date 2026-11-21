# -*- coding: utf-8 -*-
"""GW Instek AFG-2225 driver (USB CDC-ACM, SCPI over serial).

Why this file exists
--------------------
The AFG-2225 enumerates as a USB CDC virtual COM port (VID 0x2184 / PID 0x001C)
and speaks SCPI as plain ASCII lines.  Two things bit us while reverse
engineering it, so they are handled here explicitly:

1.  **This firmware only accepts the SHORT mnemonic form and no leading colon.**
    ``SOUR1:FREQ?`` answers, ``:SOURce1:FREQuency?`` does not.

2.  **Some queries answer with two lines** (``:SYST:ERR?`` returns a code and a
    message).  Reading a single line leaves the tail in the buffer and every
    later reply shifts by one -- which looks exactly like "the instrument
    ignores the command".  ``Scpi.q()`` therefore drains until the line is
    quiet.

3.  The amplitude node is ``:AMPlitude`` (``SOUR1:AMP``), *not* ``:VOLTage``.

Verified against: GW INSTEK,AFG-2225,SN:GEV833579,V2.14
"""
from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None

PORT_DEFAULT = "COM17"


class Scpi:
    def __init__(self, port=PORT_DEFAULT, baud=115200, timeout=0.25, quiet=0.8):
        self.port = port
        self.quiet = quiet
        self.s = serial.Serial(port, baud, timeout=timeout, write_timeout=2.0)
        time.sleep(0.25)
        self.s.reset_input_buffer()

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass

    def q(self, cmd, maxlines=8):
        """Query, returning every line the instrument sends back."""
        self.s.reset_input_buffer()
        self.s.write((cmd + "\n").encode("ascii"))
        self.s.flush()
        lines, t0 = [], time.time()
        while time.time() - t0 < self.quiet:
            line = self.s.readline()
            if not line:
                if lines:
                    break
                continue
            txt = line.decode("ascii", "replace").strip()
            if txt:
                lines.append(txt)
            if len(lines) >= maxlines:
                break
        return lines

    def one(self, cmd, default=""):
        r = self.q(cmd)
        return r[0] if r else default

    def w(self, cmd, settle=0.25):
        # 0.12 s was not enough right after a sweep-mode change: the writes were
        # silently dropped while the instrument digested the previous command.
        self.s.write((cmd + "\n").encode("ascii"))
        self.s.flush()
        time.sleep(settle)

    # ---- errors -------------------------------------------------------
    def remote(self):
        """进入远程状态（RWlock）。**不先进远程，FREQ/AMP 这类写命令会被静默丢弃** ——
        面板被人碰一下就会退回本地，之后所有写都不生效、也不报错，是最坑的一点。"""
        self.w("SYST:REM")
        time.sleep(0.2)
        return self

    def local(self):
        """交还面板控制。用界面手动改幅度时必须先 local，否则面板按键无效。"""
        self.w("SYST:LOC")
        return self
    def clear_status(self):
        self.w("*CLS", 0.2)

    def errors(self, limit=12):
        out = []
        for _ in range(limit):
            r = self.q("SYST:ERR?")
            if not r:
                break
            txt = " ".join(r)
            if r[0].startswith("+0") or txt.lower().startswith("no error"):
                break
            out.append(txt)
        return out

    # ---- state --------------------------------------------------------
    def state(self, ch=1):
        p = "SOUR%d:" % ch
        return {
            "func": self.one(p + "FUNC?"),
            "freq_hz": self.one(p + "FREQ?"),
            "amp_vpp": self.one(p + "AMP?"),
            "offset_v": self.one(p + "DCO?"),
            "unit": self.one(p + "VOLT:UNIT?"),
            "output": self.one("OUTP%d?" % ch),
            "sweep_on": self.one(p + "SWE:STAT?"),
            "sweep_start": self.one(p + "FREQ:STAR?"),
            "sweep_stop": self.one(p + "FREQ:STOP?"),
            "sweep_time": self.one(p + "SWE:TIME?"),
            "sweep_space": self.one(p + "SWE:SPAC?"),
            "sweep_src": self.one(p + "SWE:SOUR?"),
            "phase": self.one(p + "PHAS?"),
        }

    # ---- setters ------------------------------------------------------
    def set_sine(self, hz, vpp, ch=1, offset=0.0):
        p = "SOUR%d:" % ch
        self.w(p + "FUNC SIN")
        self.w(p + "VOLT:UNIT VPP")
        self.w(p + "FREQ %.10g" % hz)
        self.w(p + "AMP %.10g" % vpp)
        self.w(p + "DCO %.10g" % offset)

    def set_sweep(self, f_start, f_stop, seconds, space="LOG", ch=1):
        p = "SOUR%d:" % ch
        self.w(p + "SWE:STAT OFF")
        self.w(p + "FREQ:STAR %.10g" % f_start)
        self.w(p + "FREQ:STOP %.10g" % f_stop)
        self.w(p + "SWE:SPAC " + space)
        self.w(p + "SWE:TIME %.6g" % seconds)
        self.w(p + "SWE:SOUR IMM")
        self.w(p + "SWE:STAT ON")

    def sweep_off(self, ch=1):
        self.w("SOUR%d:SWE:STAT OFF" % ch)

    def output(self, on, ch=1):
        # This firmware wants the mnemonic ON/OFF: "OUTP1 1" is accepted without
        # an error but silently does nothing, and "OUTP1 0" will not turn it off.
        self.w("OUTP%d %s" % (ch, "ON" if on else "OFF"), 0.35)


def _fmt(st):
    order = ["func", "freq_hz", "amp_vpp", "offset_v", "unit", "output",
             "sweep_on", "sweep_start", "sweep_stop", "sweep_time",
             "sweep_space", "sweep_src", "phase"]
    return "\n".join("  %-12s %s" % (k, st.get(k, "")) for k in order)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=PORT_DEFAULT)
    ap.add_argument("--ch", type=int, default=1, choices=(1, 2))
    ap.add_argument("--state", action="store_true", help="read back channel state")
    ap.add_argument("--idn", action="store_true")
    ap.add_argument("--set", nargs=2, metavar=("HZ", "VPP"), help="sine freq/amplitude")
    ap.add_argument("--offset", type=float, default=0.0)
    ap.add_argument("--sweep", nargs=3, metavar=("START", "STOP", "SEC"))
    ap.add_argument("--space", default="LOG", choices=("LOG", "LIN"))
    ap.add_argument("--sweep-off", action="store_true")
    ap.add_argument("--on", action="store_true")
    ap.add_argument("--off", action="store_true")
    a = ap.parse_args()

    d = Scpi(a.port)
    try:
        if a.idn:
            print(d.one("*IDN?"))
        if a.set:
            d.set_sine(float(a.set[0]), float(a.set[1]), a.ch, a.offset)
            print("set: sine %s Hz, %s Vpp, offset %s V" % (a.set[0], a.set[1], a.offset))
        if a.sweep:
            d.set_sweep(float(a.sweep[0]), float(a.sweep[1]), float(a.sweep[2]),
                        a.space, a.ch)
            print("sweep: %s -> %s Hz in %s s (%s)" % (a.sweep[0], a.sweep[1],
                                                       a.sweep[2], a.space))
        if a.sweep_off:
            d.sweep_off(a.ch)
            print("sweep off")
        if a.on:
            d.output(True, a.ch)
            print("output ON")
        if a.off:
            d.output(False, a.ch)
            print("output OFF")
        if a.state or not any([a.idn, a.set, a.sweep, a.sweep_off, a.on, a.off]):
            for ch in ([a.ch] if not a.state else (1, 2)):
                print("ch%d:" % ch)
                print(_fmt(d.state(ch)))
        errs = d.errors()
        print("errors:", errs if errs else "none")
    finally:
        d.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

def load_arb(vals, ch=1, freq_hz=100000.0, amp_vpp=0.45, offset=0.0,
             settle=1.2, s=None):
    """
    把一个周期的波形下载到 ARB 内存并输出。

    vals    : 一周期采样点（任意浮点，内部归一到 ±511）
    freq_hz : 波形重复频率；采样率 = freq_hz * len(vals)，必须 <= 120 MHz
    amp_vpp : 输出峰峰值（±511 对应 ±amp_vpp/2）

    手册 P260：Rate = Hz × #points，Rate 上限 120 MHz，点数 1~4096。
    """
    import numpy as _np
    v = _np.asarray(vals, dtype=float)
    n = int(v.size)
    if n < 2 or n > 4096:
        raise ValueError("ARB 点数必须在 2..4096，当前 %d" % n)
    rate = float(freq_hz) * n
    if rate > 120e6 + 1.0:
        raise ValueError("采样率 %.3f MHz 超过 120 MHz（点数 %d × %.1f kHz）"
                         % (rate / 1e6, n, freq_hz / 1e3))
    peak = float(_np.max(_np.abs(v)))
    if peak <= 0:
        raise ValueError("波形全零")
    q = _np.round(v / peak * 511.0).astype(int).clip(-511, 511)
    body = ",".join(str(int(x)) for x in q)
    if s is None:
        s = Scpi()
    s.remote()
    s.w("SOUR%d:FUNC SIN" % ch, settle=0.5)          # 先退出 USER 再下载
    s.w("DATA:DAC VOLATILE, 0, " + body, settle=settle)
    # 手册 P261: USER 波形"按当前频率/幅度/偏置输出",
    # 所以 FREQ/AMP 必须在切到 USER 之前写好(反过来会被复位成 16.67 Hz)。
    s.w("SOUR%d:VOLT:UNIT VPP" % ch, settle=0.3)
    s.w("SOUR%d:FREQ %d" % (ch, int(round(freq_hz))), settle=0.6)
    s.w("SOUR%d:AMP %.4f" % (ch, float(amp_vpp)), settle=0.6)
    s.w("SOUR%d:DCO %.4f" % (ch, float(offset)), settle=0.4)
    s.w("SOUR%d:FUNC USER" % ch, settle=0.8)
    s.output(True, ch=ch)
    return dict(n=n, rate_hz=rate, peak=peak, amp_vpp=amp_vpp,
                errors=s.errors())
