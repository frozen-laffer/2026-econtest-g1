# -*- coding: utf-8 -*-
"""谐波模式的相位到底执行不执行？—— 固定各阶幅度、只改 PHAS k，看实测峰峰值跟不跟

为什么这个实验必须做：
  lsa_touch 的判据里 ub_th = max(ub)-min(ub)，而 ub 是按 ub_vpp/span 归一化出来的，
  所以 ub_th 恒等于 ub_vpp —— **Upp/U_rms/各阶幅度这三项都对相位不敏感**。
  也就是说"10 例全过"只能证明幅度/频率/比例对，证明不了波形形状（相位）对。
  真正能验相位的是：固定幅度向量、只改 PHAS k，看实测 Upp 是否按理论变化。
"""
import math
import threading
import time

import numpy as np

import dg4202
import _pi_harm_cases as H

F1 = 29296.875
A1, A3, A4 = 0.20, 0.12, 0.08          # 各阶 Vpp
CASES = [(0.0, 0.0), (180.0, 0.0), (0.0, 180.0), (90.0, 90.0)]


def expect_pp_mv(ph3, ph4):
    """按"相位被严格执行"算理论峰峰值（mV）。"""
    x = np.linspace(0, 2 * np.pi, 4096, endpoint=False)
    w = ((A1 / 2) * np.sin(x)
         + (A3 / 2) * np.sin(3 * x + math.radians(ph3))
         + (A4 / 2) * np.sin(4 * x + math.radians(ph4)))
    return float(w.max() - w.min()) * 1000.0


def main():
    import os
    os.environ["LSA_EQ_FHI_HZ"] = "520000"
    os.environ["LSA_EQ_LP_HZ"] = "520000"
    os.environ["LSA_FIT_RESID_MAX"] = "0.15"
    threading.Thread(target=H.keep_board_free, daemon=True).start()
    time.sleep(2.0)
    s = dg4202.Scpi()
    s.resync()
    print("idn:", s.one("*IDN?"))
    print("%-14s %12s %12s %10s" % ("PHAS3,PHAS4", "理论pp(mV)", "实测pp(mV)", "误差"))
    for ph3, ph4 in CASES:
        s.set_harmonics(F1, A1, [(3, A3, ph3), (4, A4, ph4)])
        s.output(True, 1)
        time.sleep(1.4)
        amps = s.read_harmonics(limit=6)
        d = H.measure()
        got = float(d["upp_mv"])
        exp = expect_pp_mv(ph3, ph4)
        err = got - exp
        flag = "OK" if abs(err) <= 5.0 else "**相位可能没被执行**"
        print("%-14s %12.2f %12.2f %+10.2f  %s"
              % ("%.0f,%.0f" % (ph3, ph4), exp, got, err, flag))
        print("     AMPL 读回: %s" % amps)
    s.set_wave("sine", 1000.0, 0.1)
    s.output(False, 1)
    print("errs:", s.errors())


if __name__ == "__main__":
    main()
