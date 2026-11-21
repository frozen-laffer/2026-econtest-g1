# -*- coding: utf-8 -*-
"""最后一轮：用新标定表跑完整的四项验收矩阵 + 50 mVpp 边界扫描。

直接复用 _gap_test.py 里的 build/cap/run_case（同一套 ARB 下发与判定口径），
所以这里只列用例。

两种源
------
    python _matrix_run.py --src afg     # 源在 PC 的 COM17，采集走树莓派 SSH
    python _matrix_run.py --src dg      # 源插在树莓派上，脚本要在树莓派上跑
"""
import argparse
import io
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)          # _gap_test.py 在 host_app 的上一级
sys.path.insert(0, HERE)
sys.path.insert(0, ROOT)

import cal_src                        # noqa: E402
import _gap_test as G                 # noqa: E402

log = G.log
F0 = 29296.875          # 4096 点 @120 MSa/s 的基础频率（整周期重放要求）

# (名字, ARB 基础频率, tones, u_b 峰峰值 mV, 干扰频率 Hz, 干扰峰值 mV)
CASES = [
    ("题1-A 29.3k+3+4",      F0,      ((1, 1.00), (3, 0.60), (4, 0.40)), 250.0, 0.0, 0.0),
    ("题1-B 同上 100mVpp",   F0,      ((1, 1.00), (3, 0.60), (4, 0.40)), 100.0, 0.0, 0.0),
    ("题1-C 58.6k+6",        F0,      ((2, 1.00), (6, 0.55)),           175.0, 0.0, 0.0),
    ("题1-D 同上 120mVpp",   F0,      ((2, 1.00), (6, 0.55)),           120.0, 0.0, 0.0),
    ("题2-A 117k+3+4",       F0,      ((4, 1.00), (12, 0.60), (16, 0.40)), 250.0, 0.0, 0.0),
    ("题2-B 同上 50mVpp",    F0,      ((4, 1.00), (12, 0.60), (16, 0.40)), 50.0, 0.0, 0.0),
    ("题2-C 234k+2",         F0,      ((8, 1.00), (16, 0.70)),         250.0, 0.0, 0.0),
    ("题2-D 29.3k+10+17",    F0,      ((1, 1.00), (10, 0.50), (17, 0.30)), 150.0, 0.0, 0.0),
    ("题3-A +1.201M 干扰",   F0,      ((4, 1.00), (12, 0.60), (16, 0.40)), 250.0, 41 * F0, 100.0),
    ("题3-B +1.025M 干扰",   F0,      ((2, 1.00), (6, 0.55)),           250.0, 35 * F0, 100.0),
]

# 题2 最小幅度（50 mVpp）单音扫频，查 508.4 kHz 杂散+高频衰减那段
BOUND = [10e3, 50e3, 100e3, 200e3, 300e3, 400e3, 500e3]


def main(argv=None):
    ap = argparse.ArgumentParser(description="四项验收矩阵 + 50 mVpp 边界扫描")
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

        log("\n== 一、四项验收矩阵（新表 + 相位补偿 + 三判据）==")
        for name, base, tones, vpp, jf, jm in CASES:
            log(name)
            rows.append(G.run_case(ad.src, name, base, tones, vpp,
                                   jam_hz=jf, jam_mv=jm, ad=ad))

        log("\n== 二、题2 最小幅度 50 mVpp 单音扫频 ==")
        for f in BOUND:
            rows.append(G.run_case(ad.src, "50mVpp %.3gk" % (f / 1e3), f,
                                   ((1, 1.00),), 50.0, ad=ad))

        log("\n" + "=" * 78)
        log("%-24s %8s %8s %10s %7s %5s %6s" %
            ("用例", "Upp", "Urms", "f1/kHz", "分量", "方法", "耗时"))
        log("-" * 78)
        npass = 0
        worst = {"upp": 0.0, "urms": 0.0, "f1": 0.0, "comp": 0.0, "t": 0.0}
        for r in rows:
            npass += 1 if r["ok"] else 0
            worst["upp"] = max(worst["upp"], abs(r["e_upp"]))
            worst["urms"] = max(worst["urms"], abs(r["e_urms"]))
            worst["f1"] = max(worst["f1"], abs(r["e_f1_khz"]))
            worst["comp"] = max(worst["comp"], abs(r["e_comp"]))
            worst["t"] = max(worst["t"], float(r.get("t_ms") or 0.0))
            log("%-24s %+8.2f %+8.2f %+10.3f %7.2f %5s %5.0fms %s"
                % (r["name"], r["e_upp"], r["e_urms"], r["e_f1_khz"], r["e_comp"],
                   str(r.get("method"))[:5], float(r.get("t_ms") or 0.0),
                   "PASS" if r["ok"] else "FAIL"))
        log("-" * 78)
        log("通过 %d / %d   最坏：Upp %.2f  Urms %.2f  f1 %.3f kHz  分量 %.2f  用时 %.0f ms"
            % (npass, len(rows), worst["upp"], worst["urms"], worst["f1"],
               worst["comp"], worst["t"]))
        out = os.path.join(HERE, "_final.json")
        with io.open(out, "w", encoding="utf-8") as f:
            json.dump(rows, f, ensure_ascii=False, indent=1)
        log("已写入 %s" % out)
    finally:
        ad.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
