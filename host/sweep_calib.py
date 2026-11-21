# -*- coding: utf-8 -*-
"""Headless sweep calibration runner (runs on the Raspberry Pi).

Usage
  python3 sweep_calib.py cap  --secs 75 --vpp 50  --out seg50.csv
  python3 sweep_calib.py cap  --secs 75 --vpp 250 --out seg250.csv
  python3 sweep_calib.py fit  seg50.csv seg250.csv

`cap` just records: for every frame it finds the dominant spectral line and
stores (t, f, amplitude, upp, urms, dc).  `fit` does what the touch dialog
does: fit the shape from the 250 mVpp run, then set in_scale = 250 / plateau.
"""
import argparse
import csv
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import lsa_proto as proto          # noqa: E402
import lsa_dsp as dsp              # noqa: E402
import lsa_eq as eq_mod            # noqa: E402
import lsa_cfg as cfg_mod          # noqa: E402

FMIN, FMAX = 8e3, 560e3


def _client(args):
    c = proto.LsaClient(args.host, args.port, timeout=8.0)
    c.connect()
    return c


def cmd_cap(args):
    c = _client(args)
    hdr = c.ping()
    fs = float(hdr["fs_hz"])
    print("connected: ver=%s fs=%.0f Hz" % (hdr["ver"], fs))
    rows = []
    t0 = time.time()
    errs = 0
    while time.time() - t0 < args.secs:
        try:
            h, raw = c.capture(gear=args.gear, inter3=False, ch=args.ch,
                               n=args.n, timeout=8.0)
            fs = float(h["fs_hz"])
            codes = np.frombuffer(raw, dtype="<i2").astype(np.float64)
            v = dsp.codes_to_volts(codes, h["gain_milli"], scale=1.0)
            freqs, amp = dsp.spectrum(v, fs, "bh", pad=4)
            lo = int(FMIN / (freqs[1] - freqs[0]))
            hi = min(int(FMAX / (freqs[1] - freqs[0])), freqs.size - 1)
            if hi <= lo:
                continue
            k = lo + int(np.argmax(amp[lo:hi]))
            fpk, apk = dsp.parabolic_peak(freqs, amp, k)
            m = dsp.measure(v, fs)
            rows.append((time.time() - t0, fpk, apk, m["upp"], m["urms"], m["udc"]))
            errs = 0
        except Exception as ex:
            errs += 1
            print("  ! %s" % ex)
            try:
                c.drain()
            except Exception:
                pass
            if errs >= 6:
                print("too many errors, abort")
                break
            time.sleep(0.2)
            continue
        if len(rows) % 40 == 0:
            print("  %.1f s  %3d frames  f=%.1f kHz  up=%.1f mV" %
                  (time.time() - t0, len(rows), rows[-1][1] / 1e3, rows[-1][3] * 1e3))
    c.close()
    with open(args.out, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t_s", "f_hz", "amp_v", "upp_v", "urms_v", "dc_v"])
        w.writerows(rows)
    print("wrote %s  (%d frames)" % (args.out, len(rows)))


def _load(path):
    f, u = [], []
    with open(path, encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            f.append(float(r["f_hz"]))
            u.append(float(r["upp_v"]))
    return np.array(f), np.array(u)


def cmd_fit(args):
    ref_of = {}
    bins_of = {}
    for path in args.csv:
        f, u = _load(path)
        bf, bu = eq_mod.sweep_bins(f, u)
        tag = os.path.basename(path)
        if bf.size < 8:
            print("%s: only %d bins, skipped" % (tag, bf.size))
            continue
        ref = float(np.median(bu[:max(3, bf.size // 10)]))
        ref_of[tag] = ref
        bins_of[tag] = (bf, bu)
        print("%-14s bins=%3d  plateau=%.2f mV" % (tag, bf.size, ref * 1e3))

    # 形状用幅度最大的那一段（信噪比最好），这里取 plateau 最大的
    if not bins_of:
        print("no usable segment")
        return 1
    best = max(ref_of, key=lambda k: ref_of[k])
    bf, bu = bins_of[best]
    fn, Q, rms, ref = eq_mod.fit_points(bf, bu, fmin=FMIN, fmax=FMAX)
    print("fit(%s): fn=%.1f kHz  Q=%.3f  rms=%.3f dB" % (best, fn / 1e3, Q, rms))

    target = 250.0e-3
    scale = target / ref
    scale = float(min(4.0, max(0.05, scale)))
    print("gain: 250 mVpp / %.2f mV = x%.4f" % (ref * 1e3, scale))

    if not args.dry:
        eq_mod.save_model(fn, Q)
        cfg_mod.set_in_scale(scale)
        print("written eq.json / config.json")
    # 线性度
    names = sorted(ref_of)
    if len(names) >= 2:
        a, b = names[0], names[-1]
        print("linearity: %s=%.2f mV  %s=%.2f mV  ratio=%.3f" %
              (a, ref_of[a] * 1e3, b, ref_of[b] * 1e3, ref_of[b] / ref_of[a]))
    return 0


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)

    a = sub.add_parser("cap")
    a.add_argument("--host", default="192.168.57.200")
    a.add_argument("--port", type=int, default=5000)
    a.add_argument("--secs", type=float, default=75.0)
    a.add_argument("--vpp", type=float, default=50.0)
    a.add_argument("--gear", type=int, default=0)
    a.add_argument("--ch", type=int, default=0)
    a.add_argument("--n", type=int, default=8192)
    a.add_argument("--out", required=True)
    a.set_defaults(func=cmd_cap)

    b = sub.add_parser("fit")
    b.add_argument("csv", nargs="+")
    b.add_argument("--dry", action="store_true")
    b.set_defaults(func=cmd_fit)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())