#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""扫频记录器：连续采集，每次用量到的谱峰频率自标注，输出 (t, f, 幅度) CSV。

不依赖信号源的扫频时序——频率直接从数据里测出来，所以信号源扫得快慢、
是否线性、是否重复触发都不影响结果。

用法（树莓派）：
    python3 sweep_capture.py --secs 130 --out sweep.csv
    python3 sweep_capture.py --secs 130 --n 8192 --gear 0 --ch 0

注意：
  * 界面里「去 u3」「补偿」都要关掉，否则测到的是被 DSP 拉平后的曲线。
  * 用单路 ADC1（三路交替有 fs/3 假信号，且幅频用的是同一路）。
"""
import argparse
import csv
import sys
import time

import numpy as np

import lsa_dsp as dsp
import lsa_proto as proto


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.57.200")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--secs", type=float, default=130.0, help="记录时长")
    ap.add_argument("--n", type=int, default=8192, help="每次采集点数")
    ap.add_argument("--gear", type=int, default=0, help="采样档位 0..3")
    ap.add_argument("--ch", type=int, default=0, help="通道 0/1/2 (ADC1/ADC2/ADC5)，单路模式用")
    ap.add_argument("--inter3", action="store_true", help="三路交替（fs×3，三路同时采）")
    ap.add_argument("--fmin", type=float, default=5000.0, help="搜峰下限 Hz")
    ap.add_argument("--fmax", type=float, default=600e3, help="搜峰上限 Hz")
    ap.add_argument("--out", default="sweep.csv")
    a = ap.parse_args()
    if a.inter3 and (a.n % 3) != 0:
        a.n = (a.n // 3) * 3
        print("inter3: n 取 3 的倍数 -> %d" % a.n)

    c = proto.LsaClient(host=a.host, port=a.port, timeout=6.0)
    c.connect()
    print("connected %s:%d" % (a.host, a.port))

    t0 = time.time()
    n_ok = 0
    n_err = 0
    fh = open(a.out, "w", newline="")
    w = csv.writer(fh)
    w.writerow(["t_s", "f_hz", "amp_mv", "upp_mv", "urms_mv", "dc_mv", "fs_hz", "gain_milli"])

    try:
        while True:
            t = time.time() - t0
            if t > a.secs:
                break
            try:
                h, raw = c.capture(gear=a.gear, inter3=a.inter3, ch=a.ch,
                                   n=a.n, timeout=6.0)
            except Exception as e:                       # 网络抖动/超时
                n_err += 1
                print("  capture error: %s" % e)
                try:
                    c.close()
                except Exception:
                    pass
                time.sleep(0.3)
                try:
                    c = proto.LsaClient(host=a.host, port=a.port, timeout=6.0)
                    c.connect()
                except Exception:
                    pass
                continue

            codes = np.frombuffer(raw, dtype="<i2").astype(np.float64)
            fs = float(h["fs_hz"])
            v = dsp.codes_to_volts(codes, h["gain_milli"])
            if a.inter3:
                v = dsp.align_channels(v, 3)

            freqs, amp = dsp.spectrum(v, fs, "bh", pad=4)
            df = float(freqs[1] - freqs[0])
            lo = max(1, int(a.fmin / df))
            hi = min(len(amp) - 1, int(a.fmax / df))
            k = lo + int(np.argmax(amp[lo:hi]))
            fp, ap = dsp.parabolic_peak(freqs, amp, k)

            upp = float(np.ptp(v))
            urms = float(np.sqrt(np.mean(v * v)))
            dc = float(np.mean(v))
            w.writerow(["%.4f" % t, "%.1f" % fp, "%.4f" % (ap * 1e3),
                        "%.4f" % (upp * 1e3), "%.4f" % (urms * 1e3),
                        "%.4f" % (dc * 1e3), "%.0f" % fs, h["gain_milli"]])
            fh.flush()
            n_ok += 1
            if n_ok % 20 == 0:
                print("  t=%6.1fs  f=%9.1f Hz  amp=%8.3f mV  upp=%8.3f mV  (%d ok, %d err)"
                      % (t, fp, ap * 1e3, upp * 1e3, n_ok, n_err))
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        fh.close()
        try:
            c.close()
        except Exception:
            pass

    print("done: %d samples in %.1f s -> %s" % (n_ok, time.time() - t0, a.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
