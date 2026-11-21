# -*- coding: utf-8 -*-
"""新表下的相位复扫：同幅度各分量、最高次谐波相位取 0/90/180/270，
比较装置 Upp 与输入端解析真值（判据 5 mV）。

波形构造与 _gap_test.build 完全同一套口径（vals = ub/(2*peak)、amp_vpp = tot），
只是给最高次谐波加了相位。

两种源
------
    python _phase_check.py --src afg     # 源在 PC 的 COM17，采集走树莓派 SSH
    python _phase_check.py --src dg      # 源插在树莓派上，脚本要在树莓派上跑

任意波统一走 lsa_src.Source.load_wave()：AFG 直接转 load_arb（与改造前逐条
一致）；DG 能表示成硬件谐波时用谐波模式，不能（比如本脚本的相位非 0）就
打印原因并退回 ARB。
"""
import argparse
import io
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cal_src                                        # noqa: E402

FREQS = [50e3, 100e3, 150e3, 200e3, 250e3]
PHASES = [0.0, 90.0, 180.0, 270.0]
AMP1 = 90.0          # 基波峰值 mV
R = {2: 0.60, 3: 0.60, 4: 0.40}


def log(*a):
    print(*a, flush=True)


def harmonics(f1):
    return [k for k in (2, 3, 4) if k * f1 <= 500e3 + 1.0]


def build(f1, ks, phase_deg):
    n = int(min(4096, math.floor(120e6 / f1)))
    t = np.arange(n, dtype=np.float64)
    last = ks[-1]
    ph = {last: math.radians(phase_deg)}
    w = np.zeros(n)
    w += np.sin(2.0 * np.pi * 1.0 * t / n)
    for k in ks:
        w += R[k] * np.sin(2.0 * np.pi * k * t / n + ph.get(k, 0.0))
    ub = w * AMP1                      # 基波峰值 = 90 mV
    # 相位会波形做成上下不对称（max≠|min|），而 load_arb 的归一化按峰值来，
    # 不对称时会压幅度。先把波形垂直居中，使它上下对称，再套 _gap_test 那套
    # 「vals = ub/(2*peak)、amp_vpp = 峰峰值」的口径就精确了。
    ub = ub - 0.5 * (float(np.max(ub)) + float(np.min(ub)))
    upp_th = float(np.max(ub) - np.min(ub))
    peak = float(np.max(np.abs(ub)))
    vals = ub / (2.0 * peak)
    comps = [(f1, AMP1)] + [(k * f1, AMP1 * R[k]) for k in ks]
    return vals, comps, upp_th, upp_th, n


def tones_of(ks, phase_deg):
    """comps（峰值 mV）→ load_wave 的 tones [(绝对次数, Vpp, 相位°), ...]。"""
    last = ks[-1]
    ts = [(1, 2.0 * AMP1 / 1000.0, 0.0)]
    for k in ks:
        ts.append((k, 2.0 * AMP1 * R[k] / 1000.0,
                   float(phase_deg) if k == last else 0.0))
    return ts


def main(argv=None):
    ap = argparse.ArgumentParser(description="新标定表下的相位复扫")
    ap.add_argument("--src", choices=("afg", "dg"), default="afg",
                    help="信号源类型：afg（默认，PC COM17）或 dg（树莓派）")
    ap.add_argument("--port", default=None, help="AFG 串口，默认 COM17")
    ap.add_argument("--host", default=cal_src.PI_HOST, help="树莓派地址（AFG 模式用）")
    a = ap.parse_args(argv)

    try:
        ad = cal_src.open_adapter(a.src, port=a.port, pi_host=a.host)
    except RuntimeError as exc:
        log("[错误] %s" % exc)
        return 2
    try:
        log("src:", ad.idn())
        rows = []
        for f1 in FREQS:
            ks = harmonics(f1)
            for p in PHASES:
                vals, comps, upp_th, tot, n = build(f1, ks, p)
                name = "%.0fk %s %+.0f" % (f1 / 1e3, "+".join(str(k) for k in ks), p)
                why = ("最高次谐波相位 %+.0f°，硬件谐波模式没有相位控制" % p) if p else None
                ad.load_wave(vals, f1, tot / 1000.0, ch=1,
                             tones=tones_of(ks, p), reason=why)
                time.sleep(1.2)
                d = ad.cap(gear=0, n=16384, harm=20, apply_=True)
                e = float(d["upp_mv"]) - upp_th
                ok = abs(e) <= 5.0
                log("%-16s 真值 %7.2f  测 %7.2f  误差 %+6.2f  %s"
                    % (name, upp_th, float(d["upp_mv"]), e, "PASS" if ok else "FAIL"))
                rows.append({"name": name, "f1": f1, "phase": p,
                             "upp_th": upp_th, "upp": float(d["upp_mv"]),
                             "e": e, "ok": ok})
        npass = sum(1 for r in rows if r["ok"])
        worst = max(rows, key=lambda r: abs(r["e"]))
        log("\n通过 %d / %d   最坏 %+0.2f mV (%s)"
            % (npass, len(rows), worst["e"], worst["name"]))
        out = os.path.join(HERE, "_phase2.json")
        with io.open(out, "w", encoding="utf-8") as f:
            json.dump(rows, f, ensure_ascii=False, indent=1)
        log("已写入 %s" % out)
    finally:
        ad.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
