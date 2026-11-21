# -*- coding: utf-8 -*-
"""树莓派侧全测试（DG4202 直连板子）：波形 × 频率 × 幅度 逐点闭环。

    python _pi_put.py run _pi_fulltest.py --quick
    python _pi_put.py run _pi_fulltest.py            # 全矩阵

为什么在派上跑：DG4202 现在插在派的 USB 上（/dev/usbtmc0），PC 侧的
full_test.py 拿不到源；而且在派上采集不用再走一层 SSH。
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

import numpy as np

import dg4202

# DG 专用测量口径（与上位机 _measure_env() 一致）：
# DG 的输出在 ~600 kHz 有一根固定约 6 mV 的残留，eq.json 在 550 kHz 处要补
# ×7.12，会把这根残留放大到跟 50 mV 的小信号同一个量级、把 f1 认到 600 kHz。
# 带限 + 放宽拟合残差门限后，50 mV 档也能得到正确的主频与峰峰值。
os.environ.setdefault("LSA_EQ_FHI_HZ", "520000")
os.environ.setdefault("LSA_EQ_LP_HZ", "520000")
os.environ.setdefault("LSA_FIT_RESID_MAX", "0.15")

WAVES = ["sine", "square", "triangle", "half"]
FREQS = [10000.0, 20000.0, 50000.0, 100000.0, 150000.0, 200000.0, 300000.0, 500000.0]
AMPS_MV = [50.0, 100.0, 150.0, 200.0, 250.0]
FREQS_Q = [10000.0, 100000.0, 500000.0]
AMPS_Q = [50.0, 250.0]


def cap(wave):
    out = subprocess.run([sys.executable, "cap_one.py", "--gear", "0",
                          "--n", "16384", "--apply", "--wave", wave],
                         capture_output=True, text=True, timeout=180,
                         env=dict(os.environ))
    lines = [ln for ln in out.stdout.splitlines() if ln.strip().startswith("{")]
    if not lines:
        raise RuntimeError((out.stderr or out.stdout).strip()[-200:])
    return json.loads(lines[-1])


def half_wave(n=2048):
    """半波整流的正弦：正半周 sin，负半周 0（与 full_test 的 half 定义一致）。"""
    t = np.arange(n) / float(n)
    return np.where(np.sin(2 * math.pi * t) > 0, np.sin(2 * math.pi * t), 0.0)


def theory(wave, vpp):
    """返回 (期望 p-p, 期望 urms)，单位 V。"""
    if wave == "half":
        w = half_wave()
        w = w / np.max(np.abs(w))            # 驱动内部按峰值归一
        v = w * vpp
        return float(v.max() - v.min()), float(np.sqrt(np.mean(v ** 2)))
    if wave in ("sine", "triangle"):
        return vpp, vpp / (2.0 * math.sqrt(2.0)) if wave == "sine" else vpp / (2.0 * math.sqrt(3.0))
    if wave == "square":
        return vpp, vpp / 2.0
    raise ValueError(wave)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--settle", type=float, default=1.0)
    a = ap.parse_args()
    freqs = FREQS_Q if a.quick else FREQS
    amps = AMPS_Q if a.quick else AMPS_MV
    waves = WAVES if not a.quick else ["sine", "square", "triangle"]

    s = dg4202.Scpi()
    print("源:", s.identify())
    print("resync 丢弃错误:", s.resync())
    s.output(False, 1)

    rows = []
    total = len(waves) * len(freqs) * len(amps)
    idx = 0
    t0 = time.time()
    for wave in waves:
        for hz in freqs:
            for mv in amps:
                idx += 1
                vpp = mv / 1000.0
                tag = "%-8s %7.0f Hz %5.0f mV" % (wave, hz, mv)
                try:
                    if wave == "half":
                        # DG 没有"半波"这个内置波形，只能下载 2048 点 ARB
                        s.load_arb(half_wave(2048), ch=1, freq_hz=hz,
                                   amp_vpp=vpp)
                    else:
                        s.set_wave(wave, hz, vpp, ch=1)
                    s.output(True, 1)
                    st = s.state(1)
                    if not st["output"].upper().startswith("ON"):
                        raise RuntimeError("输出没开")
                    time.sleep(a.settle)
                    d = cap(wave)
                    e_pp, e_rms = theory(wave, vpp)
                    d_pp = d["upp_mv"] / 1000.0
                    d_rms = d["urms_mv"] / 1000.0
                    d_f1 = d["f1_hz"]
                    e_f1 = hz if wave != "half" else hz
                    ok = (abs(d_pp - e_pp) <= 0.005 + 0.03 * e_pp
                          and abs(d_rms - e_rms) <= 0.005 + 0.03 * e_rms
                          and abs(d_f1 - e_f1) <= max(1000.0, 0.02 * hz))
                    errs = s.errors()
                    rows.append((wave, hz, mv, d_pp * 1e3, e_pp * 1e3,
                                 d_rms * 1e3, e_rms * 1e3, d_f1, ok, errs))
                    print("[%3d/%3d] %s  测得 pp=%7.1f (期望 %7.1f) rms=%6.1f "
                          "(期望 %6.1f) f1=%8.1f  %s%s"
                          % (idx, total, tag, d_pp * 1e3, e_pp * 1e3,
                             d_rms * 1e3, e_rms * 1e3, d_f1,
                             "PASS" if ok else "FAIL",
                             ("  errs=%s" % errs) if errs else ""), flush=True)
                except Exception as exc:
                    rows.append((wave, hz, mv, 0, 0, 0, 0, 0, False, str(exc)))
                    print("[%3d/%3d] %s  EXC %s" % (idx, total, tag, exc), flush=True)
                finally:
                    s.output(False, 1)

    npass = sum(1 for r in rows if r[8])
    print("\n== 结果 %d/%d 通过，用时 %.0f s ==" % (npass, len(rows), time.time() - t0))
    bad = [r for r in rows if not r[8]]
    for r in bad[:40]:
        print("  FAIL %-8s %7.0f Hz %5.0f mV  pp=%7.1f/%7.1f rms=%6.1f/%6.1f f1=%.0f %s"
              % (r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[9] or ""))
    with open("_pi_fulltest.csv", "w", encoding="utf-8") as f:
        f.write("wave,freq_hz,amp_mv,meas_pp_mv,exp_pp_mv,meas_rms_mv,exp_rms_mv,"
                "f1_hz,ok,note\n")
        for r in rows:
            f.write("%s,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%s\n"
                    % (r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7],
                       1 if r[8] else 0, str(r[9]).replace(",", ";")))
    print("明细: ~/lsa/_pi_fulltest.csv")

    s.resync()
    s.set_sine(1000.0, 0.1, ch=1)
    s.output(False, 1)
    print("收尾: func=%s out=%s errs=%s"
          % (s.one(":SOUR1:FUNC?"), s.one(":OUTP1?"), s.errors() or "无"))
    s.close()
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
