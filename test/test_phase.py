# -*- coding: utf-8 -*-
"""DG4202 相位专项：和 _phase_check.py（AFG 时代）同一套波形与判据。

基波峰值 90 mV、2/3/4 次谐波比例 0.60/0.60/0.40，最高次谐波相位取
0/90/180/270，比"装置 Upp"与解析真值（判据 5 mV）。

相位是非零的，所以这些用例一定走 ARB（谐波模式的 HARM:PHAS 只被部分执行）。
如果这个脚本过，说明 DG 在"要相位"的场合和 AFG 一样可用。
"""
import json
import math
import os
import subprocess
import sys
import threading
import time

import numpy as np

import lsa_src

os.environ.setdefault("LSA_EQ_FHI_HZ", "520000")
os.environ.setdefault("LSA_EQ_LP_HZ", "520000")
os.environ.setdefault("LSA_FIT_RESID_MAX", "0.15")

FREQS = [50e3, 100e3, 150e3, 200e3, 250e3]
PHASES = [0.0, 90.0, 180.0, 270.0]
AMP1 = 90.0                       # 基波峰值 mV
R = {2: 0.60, 3: 0.60, 4: 0.40}


def keep_board_free(interval=1.5):
    while True:
        try:
            subprocess.run(["pkill", "-f", "[l]sa_touch.py"],
                           capture_output=True, timeout=5)
        except Exception:
            pass
        time.sleep(interval)


def harmonics(f1):
    return [k for k in (2, 3, 4) if k * f1 <= 500e3 + 1.0]


def build(f1, ks, phase_deg):
    """与 _phase_check.build 逐行一致（含"先垂直居中再归一化"那一步）。"""
    n = int(min(4096, math.floor(120e6 / f1)))
    t = np.arange(n, dtype=np.float64)
    last = ks[-1]
    ph = {last: math.radians(phase_deg)}
    w = np.zeros(n)
    w += np.sin(2.0 * np.pi * 1.0 * t / n)
    for k in ks:
        w += R[k] * np.sin(2.0 * np.pi * k * t / n + ph.get(k, 0.0))
    ub = w * AMP1
    # 相位会让波形上下不对称，而 ARB 归一化按峰值来；先垂直居中再套
    # 「vals = ub/(2*peak)、amp_vpp = 峰峰值」这套口径才精确
    ub = ub - 0.5 * (float(np.max(ub)) + float(np.min(ub)))
    upp_th = float(np.max(ub) - np.min(ub))
    peak = float(np.max(np.abs(ub)))
    vals = ub / (2.0 * peak)
    comps = [(f1, AMP1)] + [(k * f1, AMP1 * R[k]) for k in ks]
    tones = [(1, AMP1 * 2.0 / 1000.0, 0.0)]
    for k in ks:
        tones.append((k, AMP1 * R[k] * 2.0 / 1000.0,
                      phase_deg if k == last else 0.0))
    return vals, comps, upp_th, tones


def measure():
    out = subprocess.run([sys.executable, "_contest_one.py", "--gear", "0",
                          "--n", "16384", "--harm", "20", "--apply"],
                         capture_output=True, text=True, timeout=180,
                         env=dict(os.environ))
    lines = [ln for ln in out.stdout.splitlines() if ln.strip().startswith("{")]
    if not lines:
        raise RuntimeError((out.stderr or out.stdout).strip()[-200:])
    return json.loads(lines[-1])


def main():
    threading.Thread(target=keep_board_free, daemon=True).start()
    time.sleep(2.0)
    src = lsa_src.open_source("dg")
    print("src:", src.name, src.identify(), flush=True)
    rows = []
    for f1 in FREQS:
        ks = harmonics(f1)
        for p in PHASES:
            vals, comps, upp_th, tones = build(f1, ks, p)
            name = "%.0fk %s %+.0f" % (f1 / 1e3, "+".join(str(k) for k in ks), p)
            tot = upp_th
            mode, note = src.load_wave(vals, f1, tot / 1000.0, ch=1,
                                       tones=tones)
            time.sleep(1.4)
            d = measure()
            e = float(d["upp_mv"]) - upp_th
            ok = abs(e) <= 5.0
            print("%-14s %-4s 真值 %7.2f  测 %7.2f  误差 %+6.2f  %s"
                  % (name, mode, upp_th, float(d["upp_mv"]), e,
                     "PASS" if ok else "FAIL"), flush=True)
            rows.append({"name": name, "mode": mode, "phase": p,
                         "upp_th": upp_th, "upp": float(d["upp_mv"]),
                         "e": e, "ok": ok})
    npass = sum(1 for r in rows if r["ok"])
    worst = max(rows, key=lambda r: abs(r["e"]))
    modes = sorted(set(r["mode"] for r in rows))
    print("\n相位专项：%d / %d 通过（路数 %s），最坏 %+0.2f mV（%s）"
          % (npass, len(rows), "/".join(modes), worst["e"], worst["name"]),
          flush=True)
    with open("/home/frozen-laffer/lsa/_phase_dg.json", "w",
              encoding="utf-8") as f:
        json.dump(rows, f, ensure_ascii=False, indent=1)
    src.set_point(1000.0, 0.1)
    src.output(False, 1)
    src.close()
    return 0 if npass == len(rows) else 1


if __name__ == "__main__":
    sys.exit(main())
