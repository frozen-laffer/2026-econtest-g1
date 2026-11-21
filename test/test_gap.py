# -*- coding: utf-8 -*-
"""补测四项验收的缺口：干扰频率扫描 / 弱基波 / 题3 最小信号 / 10 kHz 基频。

在树莓派上跑，直接驱动 AFG + 板子（不经界面）：
    python3 _gap_test.py
"""
import json
import math
import subprocess
import sys
import time

import numpy as np

import lsa_src

FS_ARB = 120e6
MAX_N = 4096
HERE = "/home/frozen-laffer/lsa"


def log(*a):
    print(*a, flush=True)


def build(f1, tones, ub_vpp, jam_hz=0.0, jam_mv=0.0):
    """合成一周期波形。返回 (vals, comps, upp_th, tot, n, f_arb)。

    tones: ((m, ratio), ...)，第 m 次谐波、相对幅度；f1 = 基波频率。
    干扰 jam_hz 必须是 f1 的整数倍（整周期重放）。
    """
    n = int(min(MAX_N, math.floor(FS_ARB / f1)))
    if n < knot_need(tones):
        raise ValueError("点数不够")
    t = np.arange(n, dtype=np.float64)
    w = np.zeros(n)
    for m, r in tones:
        w += float(r) * np.sin(2.0 * np.pi * float(m) * t / n)
    span = float(np.max(w) - np.min(w))
    ub = w * (ub_vpp / span)
    upp_th = float(np.max(ub) - np.min(ub))
    if jam_hz:
        jm = float(jam_hz) / float(f1)
        if abs(jm - round(jm)) > 1e-9:
            raise ValueError("干扰 %.6f kHz 不是基频 %.6f kHz 的整数倍"
                             % (jam_hz / 1e3, f1 / 1e3))
        ub = ub + jam_mv * np.sin(2.0 * np.pi * jm * t / n)
    tot = float(np.max(ub) - np.min(ub))
    peak = float(np.max(np.abs(ub)))
    vals = ub / (2.0 * peak)
    comps = [(float(m) * f1, float(r) * ub_vpp / span) for m, r in tones]
    return vals, comps, upp_th, tot, n


def knot_need(tones):
    return max(8 * int(m) for m, _ in tones)


def cap(f1_exp, n_pts=16384, harm=20, ad=None):
    """采一帧。

    ad=None 保持老行为：在本机（树莓派）直接跑 _contest_one.py。
    给了适配器（PC 侧脚本）则由适配器决定走 SSH（AFG）还是本机（DG）。
    """
    if ad is not None:
        return ad.cap(gear=0, n=n_pts, harm=harm, apply_=True)
    out = subprocess.run(
        [sys.executable, "_contest_one.py", "--gear", "0", "--n", str(n_pts),
         "--harm", str(harm), "--apply"],
        cwd=HERE, capture_output=True, text=True, timeout=180)
    lines = [ln for ln in (out.stdout or "").splitlines()
             if ln.strip().startswith("{")]
    if not lines:
        raise RuntimeError((out.stderr or out.stdout).strip()[-200:])
    return json.loads(lines[-1])


def tones_for_src(tones, comps, jam_hz=0.0):
    """用例表 → load_wave 的 tones。

    tones: ((m, ratio), ...) 相对幅度；comps: [(f, 峰值 mV), ...]。
    与 lsa_touch.tones_for_src 同一口径（Vpp = 2×峰值）。带干扰时硬件谐波
    模式表示不了，返回 (None, 原因) 让 load_wave 退回 ARB。
    """
    if jam_hz:
        return None, ("带干扰 u_J（%g Hz），不是基波整数倍，谐波模式放不出来"
                      % float(jam_hz))
    return ([(int(round(float(m))), 2.0 * float(peak_mv) / 1000.0, 0.0)
             for (m, _r), (_f, peak_mv) in zip(tones, comps)], None)


def run_case(src, name, f1, tones, ub_vpp, jam_hz=0.0, jam_mv=100.0, ad=None):
    vals, comps, upp_th, tot, n = build(f1, tones, ub_vpp, jam_hz, jam_mv)
    rate = f1 * n
    log("  n=%d  rate=%.3f MHz  总峰峰 %.1f mV" % (n, rate / 1e6, tot))
    if ad is None:
        src.load_arb(vals, f1, tot / 1000.0, ch=1)
    else:
        ts, why = tones_for_src(tones, comps, jam_hz)
        ad.load_wave(vals, f1, tot / 1000.0, ch=1, tones=ts, reason=why)
    time.sleep(1.2)
    d = cap(f1, ad=ad)
    hm = d.get("harm_mv") or []
    f_meas = float(d["f1_hz"])
    e_upp = float(d["upp_mv"]) - upp_th
    urms_th = math.sqrt(sum((a / math.sqrt(2.0)) ** 2 for _, a in comps))
    e_urms = float(d["urms_mv"]) - urms_th
    e_f1 = (f_meas - comps[0][0]) / 1000.0
    ec = 0.0
    det = []
    for f, a in comps:
        k = int(round(f / f_meas)) if f_meas > 0 else 0
        meas = hm[k - 1] if 0 < k <= len(hm) else float("nan")
        if meas == meas:
            ec = max(ec, abs(meas - a))
        det.append("%.2fk:%.1f/%.1f" % (f / 1e3, a, meas))
    ok = (abs(e_upp) <= 5.0 and abs(e_urms) <= 5.0
          and abs(e_f1) <= 1.0 and ec <= 5.0)
    log("  %-26s Upp %+6.2f  Urms %+6.2f  f1 %+7.3f kHz  分量 %.2f  %s"
        % (name, e_upp, e_urms, e_f1, ec, "PASS" if ok else "FAIL"))
    log("  " + "  ".join(det))
    return {"name": name, "e_upp": e_upp, "e_urms": e_urms,
            "e_f1_khz": e_f1, "e_comp": ec, "ok": bool(ok),
            "upp_th": upp_th, "upp": float(d["upp_mv"]),
            "f1_th": comps[0][0], "f1": f_meas,
            "method": d.get("method"), "t_ms": d.get("t_ms")}


def main():
    src = lsa_src.open_source("afg")
    log("src:", src.identify())
    rows = []
    F0 = 29296.875       # 4096 点 @120 MSa/s 的基础频率

    log("\n== A. 干扰频率扫描（u_b 117.19k+3次+4次，250 mVpp；u_J 200 mVpp）==")
    a_tones = ((4, 1.00), (12, 0.60), (16, 0.40))
    for m_j in (34, 35, 68, 109, 119, 137):
        f_j = m_j * F0
        log(" 干扰 %.3f MHz" % (f_j / 1e6))
        rows.append(run_case(src, "干扰 %.3fM" % (f_j / 1e6), F0, a_tones,
                             250.0, jam_hz=f_j, jam_mv=100.0))

    log("\n== B. 弱基波（基波 5 mV 峰值 + 3 次 120 mV 峰值）==")
    log(" 理想：基波 5.0 mV、3 次 120 mV")
    rows.append(run_case(src, "弱基波 5mV+120mV", F0, ((1, 1.0), (3, 24.0)),
                         250.0))

    log("\n== C. 题3 最小信号（u_b 117.19k 50 mVpp + u_J 1.172 MHz 200 mVpp）==")
    rows.append(run_case(src, "u_b 50mVpp + u_J", 117187.5, ((1, 1.00),),
                         50.0, jam_hz=10 * 117187.5, jam_mv=100.0))

    log("\n== D. 基频 10 kHz（10k + 3次 30k + 4次 40k，250 mVpp；u_J 1 MHz）==")
    rows.append(run_case(src, "10 kHz 无干扰", 10000.0, ((1, 1.00), (3, 0.60),
                                                       (4, 0.40)), 250.0))
    rows.append(run_case(src, "10 kHz + 1MHz 干扰", 10000.0,
                         ((1, 1.00), (3, 0.60), (4, 0.40)), 250.0,
                         jam_hz=1e6, jam_mv=100.0))

    log("\n" + "=" * 74)
    log("%-26s %8s %8s %9s %7s %s" % ("用例", "Upp", "Urms", "f1/kHz", "分量", "判定"))
    log("-" * 74)
    npass = 0
    for r in rows:
        npass += 1 if r["ok"] else 0
        log("%-26s %+8.2f %+8.2f %+9.3f %7.2f %s"
            % (r["name"], r["e_upp"], r["e_urms"], r["e_f1_khz"], r["e_comp"],
               "PASS" if r["ok"] else "FAIL"))
    log("-" * 74)
    log("通过 %d / %d" % (npass, len(rows)))
    with open("/home/frozen-laffer/lsa/_gap.json", "w") as f:
        json.dump(rows, f, ensure_ascii=False, indent=1)
    src.close()


if __name__ == "__main__":
    main()
