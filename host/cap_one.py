#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在树莓派上单点采集一次，打印 JSON。

    python3 cap_one.py --gear 0 --n 16384 --ch 0          # 原始（不补偿）
    python3 cap_one.py --gear 0 --n 16384 --apply         # 用 eq.json + config.json 补偿后

给 cal_step.py（PC 侧阶梯校准）当从机用；也可以单独拿来读数。
"""
import argparse
import json
import os
import sys

import numpy as np

# 目录名从 frozen_laffer 改成 frozen-laffer 后，写死的老路径就不存在了；
# 直接用本文件所在目录，脚本放哪都能 import 同目录模块。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ".")

import lsa_proto as proto           # noqa: E402
import lsa_dsp as dsp               # noqa: E402
import lsa_cfg as cfg_mod           # noqa: E402
import lsa_eq as eq_mod             # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.57.200")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--gear", type=int, default=0)
    ap.add_argument("--ch", type=int, default=0)
    ap.add_argument("--n", type=int, default=16384)
    ap.add_argument("--apply", action="store_true", help="用 eq.json + config.json 补偿")
    ap.add_argument("--inter3", action="store_true")
    ap.add_argument("--dump-raw", action="store_true", help="附带前 2048 个原始码（调试用）")
    ap.add_argument("--wave", default="auto", choices=["auto", "sine", "square", "triangle", "half"],
                    help="波形类型；auto=自动判型，高频非正弦请手动指定")
    a = ap.parse_args()

    c = proto.LsaClient(a.host, a.port, timeout=6.0)
    c.connect()
    try:
        hdr, raw = c.capture(gear=a.gear, inter3=a.inter3, ch=a.ch,
                             n=a.n, timeout=8.0)
    finally:
        c.close()

    codes = np.asarray(proto.raw_to_int16(raw), dtype=np.int16)
    gain = float(hdr.get("gain_milli") or 3925)
    fs = float(hdr["fs_hz"])
    v = dsp.codes_to_volts(codes, gain, offset_code=0.0)

    margin = 0
    scale = 1.0
    if a.apply:
        eqm = eq_mod.load()
        v = eq_mod.apply(v, fs, eqm)
        margin = eq_mod.trim_samples(fs)
        scale = cfg_mod.get_in_scale()
        v = v * scale

    m = dsp.measure(v, fs, margin=margin, force_wave=(None if a.wave == "auto" else a.wave))
    out = {
        "fs_hz": fs, "n_this": int(hdr["n_this"]), "gain_milli": gain,
        "n_used": int(m["n"]),
        "upp_mv": m["upp"] * 1e3, "urms_mv": m["urms"] * 1e3,
        "udc_mv": m["udc"] * 1e3, "f1_hz": m["f1"], "a1_mv": m["a1"] * 1e3,
        "wave_kind": m.get("wave_kind"), "fit_k": m.get("fit_k"),
        "fit_resid": m.get("fit_resid"),
        "clip_pct": float(np.mean(np.abs(codes.astype(np.int32)) >= 2046)) * 100.0,
        "applied": bool(a.apply), "in_scale": scale,
    }
    if a.dump_raw:
        out["raw"] = ",".join(str(int(x)) for x in codes[:2048])
    print(json.dumps(out))


if __name__ == "__main__":
    main()
