#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在树莓派上采一帧并输出赛题验收所需的全部量（JSON）。

给 PC 侧 `_contest_check.py` 当从机用。与 `cap_one.py` 的区别：
除了 Upp/Urms/f1，还给出各次谐波幅值、以及 1/3 周期显示窗的
「周期数误差」和「首尾不连续度」——对应赛题第 1 项。

    python3 _contest_one.py --gear 0 --n 16384 --apply
"""
import argparse
import json
import os
import sys
import time

import numpy as np

# 目录名从 frozen_laffer 改成 frozen-laffer 后，写死的老路径就不存在了；
# 直接用本文件所在目录，脚本放哪都能 import 同目录模块。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ".")

import lsa_proto as proto           # noqa: E402
import lsa_dsp as dsp               # noqa: E402
import lsa_cfg as cfg_mod           # noqa: E402
import lsa_eq as eq_mod             # noqa: E402


def wrap_stats(v, fs, f1, count):
    """取 count 个完整周期，量「周期数误差」和「首尾不连续度」。

    首尾不连续度：用基波拟合 a·cos+b·sin 在外推的 wrap 点与起点之差，
    窗口内含整数个周期时应为 0。
    """
    t_seg, seg = dsp.slice_periods(v, fs, f1, count=count)
    if seg is None or len(seg) < 8:
        return None
    t = np.asarray(t_seg, dtype=float)          # 注意 slice_periods 返回 (t, y)
    seg = np.asarray(seg, dtype=float)
    w = 2.0 * np.pi * f1
    A = np.column_stack([np.cos(w * t), np.sin(w * t), np.ones(len(t))])
    coef, *_ = np.linalg.lstsq(A, seg, rcond=None)
    a, b, c0 = float(coef[0]), float(coef[1]), float(coef[2])
    T = float(len(seg)) / fs
    p0 = a * np.cos(w * t[0]) + b * np.sin(w * t[0]) + c0
    p1 = a * np.cos(w * (t[0] + T)) + b * np.sin(w * (t[0] + T)) + c0
    wrap_v = float(p1 - p0)                     # 首尾同一相位处的跳变
    upp = 2.0 * float(np.hypot(a, b))
    return dict(cycles=float(f1 * T), cycles_err=float(f1 * T - count),
                wrap_mv=float(abs(wrap_v) * 1e3),
                wrap_pct=float(abs(wrap_v) / upp * 100.0) if upp > 1e-9 else 0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.57.200")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--gear", type=int, default=0)
    ap.add_argument("--ch", type=int, default=0)
    ap.add_argument("--n", type=int, default=16384)
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--wave", default="auto")
    ap.add_argument("--harm", type=int, default=5)
    ap.add_argument("--phase", action="store_true",
                    help="额外输出各次谐波的相位（度）")
    ap.add_argument("--f1", type=float, default=0.0,
                    help="强制基频（Hz）；双音测相时必须指定")
    ap.add_argument("--phi-list", default="",
                    help="逗号分隔的补偿角(度)，用同一帧数据逐个算 Upp")
    a = ap.parse_args()

    c = proto.LsaClient(a.host, a.port, timeout=6.0)
    t_conn = time.time()
    c.connect()
    try:
        hdr, raw = c.capture(gear=a.gear, inter3=False, ch=a.ch,
                             n=a.n, timeout=8.0)
    finally:
        c.close()
    t_ms = (time.time() - t_conn) * 1000.0        # 连接+采集的墙钟时间

    codes = np.asarray(proto.raw_to_int16(raw), dtype=np.int16)
    gain = float(hdr.get("gain_milli") or 3925)
    fs = float(hdr["fs_hz"])
    v = dsp.codes_to_volts(codes, gain, offset_code=0.0)

    margin = 0
    if a.apply:
        eqm = eq_mod.load()
        v = eq_mod.apply(v, fs, eqm)
        margin = eq_mod.trim_samples(fs)
        v = v * cfg_mod.get_in_scale()

    m = dsp.measure(v, fs, margin=margin,
                    force_wave=(None if a.wave == "auto" else a.wave))
    f1 = float(a.f1) if a.f1 > 0 else float(m["f1"])
    ha = dsp.harmonic_amplitudes(v, fs, f1, count=a.harm, tol=0.03)
    out = {
        "fs_hz": fs, "n_this": int(hdr["n_this"]),
        "t_ms": t_ms,
        "n_used": int(m["n"]),
        "upp_mv": m["upp"] * 1e3, "urms_mv": m["urms"] * 1e3,
        "udc_mv": m["udc"] * 1e3, "f1_hz": f1,
        "a1_mv": m["a1"] * 1e3,
        "harm_mv": [float(x) * 1e3 for x in np.atleast_1d(ha)],
        "wave_kind": m.get("wave_kind"),
        "method": m.get("method"),
        "fit_k": m.get("fit_k"),
        "fit_resid": m.get("fit_resid"),
        "fit_diag": m.get("fit_diag"),
        "fit_upp_mv": (None if m.get("fit_upp") is None
                       else float(m["fit_upp"]) * 1e3),
        "clip_pct": float(np.mean(np.abs(codes.astype(np.int32)) >= 2046)) * 100.0,
        "p1": wrap_stats(v, fs, f1, 1),
        "p3": wrap_stats(v, fs, f1, 3),
    }
    if a.phase:
        # 各次谐波的相位（度），用 a·cos + b·sin 最小二乘，phase = atan2(b, a)。
        # 同一次采集里各分量的相对相位是可信的（共同的起始时刻），
        # 绝对相位没有意义（采集起点是任意的）。
        n = v.size
        t = np.arange(n) / fs
        tt = t - t[0]
        cols = [np.ones_like(tt)]
        for k in range(1, a.harm + 1):
            cols.append(np.cos(2 * np.pi * k * f1 * tt))
            cols.append(np.sin(2 * np.pi * k * f1 * tt))
        A = np.column_stack(cols)
        cc, *_ = np.linalg.lstsq(A, v, rcond=None)
        out["harm_deg"] = [
            float(np.degrees(np.arctan2(cc[2 * k], cc[2 * k - 1])))
            for k in range(1, a.harm + 1)]
    if a.phi_list:
        # 同一帧数据、不同补偿角 → 一次采集扫完整条误差曲线
        dsp.PHI_ENABLE = True
        seq = []
        for s in a.phi_list.split(","):
            s = s.strip()
            if not s:
                continue
            try:
                ang = float(s)
            except ValueError:
                continue
            dsp.PHI_OVERRIDE = ang
            mm = dsp.measure(v, fs, margin=margin,
                            force_wave=(None if a.wave == "auto" else a.wave))
            seq.append([ang, float(mm["upp"]) * 1e3])
        dsp.PHI_OVERRIDE = None
        out["phi_upp"] = seq
    print(json.dumps(out))


if __name__ == "__main__":
    main()
