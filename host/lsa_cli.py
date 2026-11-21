# -*- coding: utf-8 -*-
"""
lsa_cli.py —— 命令行联调工具（不依赖图形界面，树莓派上可直接跑）

用法
    python3 lsa_cli.py ping                      # 只探测连接
    python3 lsa_cli.py selftest                  # 三路交替自检
    python3 lsa_cli.py cap --gear 0 --inter3 --n 24576
    python3 lsa_cli.py auto                      # 自动选档 + 全套测量（推荐）
    python3 lsa_cli.py calib                     # 零输入标定（先短路输入！）
    python3 lsa_cli.py demo                      # 无硬件自检（合成信号）

    --host 192.168.57.200  --port 5000
"""
import argparse
import sys
import time

import numpy as np

import lsa_calib as calib_mod
import lsa_cfg as cfg_mod
import lsa_dsp as dsp
import lsa_eq as eq_mod
import lsa_proto as proto


def synth(fs, n, comps, noise_mv=0.0, seed=1):
    """comps: [(freq, amp_peak)] -> 合成波形"""
    rng = np.random.RandomState(seed)
    t = np.arange(n) / fs
    v = np.zeros(n)
    for f, a in comps:
        v += a * np.sin(2 * np.pi * f * t + rng.uniform(0, 2 * np.pi))
    if noise_mv:
        v += rng.normal(0, noise_mv * 1e-3, n)
    return v


def report(v, fs, title="", margin=0, codes=None, gain_milli=None):
    m = dsp.measure(v, fs, margin=margin)
    print("── %s ────────────────────────────────" % (title or "measure"))
    print("  采样: fs=%.3f MSPS  N=%d  Δf=%.0f Hz" % (fs / 1e6, m["n"], m["df"]))
    print("  Upp = %.2f mV    Urms = %.2f mV    Udc = %.2f mV"
          % (m["upp"] * 1e3, m["urms"] * 1e3, m["udc"] * 1e3))
    print("  f1  = %.1f Hz    基波幅度 = %.2f mV   波峰因数 = %.3f"
          % (m["f1"], m["a1"] * 1e3, m["crest"]))
    if m.get("wave_kind") and m["wave_kind"] != "mixed":
        print("  波形: %s（Upp 由基波解析换算）" % m["wave_kind"])
    # 有原始码时用「整链 G(f)」直接算分量（不做反补偿、不裁边），
    # 这是赛题第三问"各分量幅值"最干净的一条路。
    if codes is not None:
        comp = dsp.components_direct(codes, fs, gain_total=(gain_milli or 3925) / 1000.0)
    else:
        comp = dsp.components(v, fs)
    if comp:
        print("  分量（输入端峰值）:")
        for f, a in comp:
            print("    %10.1f Hz   %8.2f mV" % (f, a * 1e3))
    harm = dsp.harmonic_amplitudes(v, fs, m["f1"])
    if harm:
        thd = np.sqrt(sum(x * x for x in harm[1:])) / harm[0] if harm[0] else 0.0
        print("  谐波: " + "  ".join("%.2f" % (x * 1e3) for x in harm) + " mV")
        print("  THD  = %.2f %%" % (thd * 100))
    return m


def cmd_demo(args):
    fs = 7083000.0
    n = 24576
    # 题2 典型输入: 200 mVpp @ 137 kHz，含 2/3 次谐波
    comps = [(137000.0, 0.100), (274000.0, 0.020), (411000.0, 0.010)]
    v = synth(fs, n, comps, noise_mv=0.3)
    report(v, fs, "demo: 137k 200mVpp + 谐波")
    t, y = dsp.slice_periods(v, fs, 137000.0, 3)
    print("  3 周期截取: %d 点 = %.3f ms" % (len(y), (t[-1] - t[0]) * 1e3))
    # 题3: u2 + f3=3.1 MHz 强干扰，先低通再测
    comps3 = [(100000.0, 0.100), (3100000.0, 0.300)]
    v3 = synth(fs, n, comps3, noise_mv=0.3)
    print("\n  加入 3.1MHz / 600mVpp 干扰后:")
    report(v3, fs, "before LP")
    v3f = dsp.lowpass_fir(v3, fs, dsp.LP_FC_HZ)
    report(v3f, fs, "after LP %.0fk" % (dsp.LP_FC_HZ/1e3), margin=dsp.FIR_MARGIN)
    return 0


def cmd_auto(args):
    cal = calib_mod.load(args.calib)
    with proto.LsaClient(args.host, args.port, args.timeout) as c:
        print("PING -> %s" % c.ping())
        n = args.n or proto.MAX_POINTS_TOTAL
        best = None
        for gear in range(proto.GEAR_COUNT):
            try:
                h, raw = c.capture(gear=gear, inter3=True, n=n, timeout=6.0)
            except proto.LsaError as e:
                print("  gear%d 失败: %s" % (gear, e))
                continue
            a = np.asarray(proto.raw_to_int16(raw), dtype=np.float64)
            fs = float(h["fs_hz"]) or proto.GEAR_RATE_INTER3[gear]
            v = dsp.codes_to_volts(a, h["gain_milli"] or 3925)
            v = _fix(cal, v, h, inter3=True)
            ratio = dsp.out_of_band_ratio(v, fs)
            print("  gear%d fs=%.3fMSPS n=%d gain=%.3f 带外能量=%.2f%%"
                  % (gear, fs / 1e6, h["n_this"], h["gain_milli"] / 1000.0, ratio * 100))
            if best is None or ratio < best[0]:
                best = (ratio, gear, v, fs, h)
        if best is None:
            print("全部档位采集失败")
            return 1
        ratio, gear, v, fs, h = best
        print("\n选用 gear%d（带外能量 %.2f%%）" % (gear, ratio * 100))
        v = _eq(v, fs, eq_mod.load(args.eq), not args.no_eq)
        report(v, fs, "实测", codes=a, gain_milli=h["gain_milli"])
    return 0


def _fix(cal, v, h, inter3):
    """按标定表扣偏置；没标定时退回通道均值对齐。"""
    gain = h.get("gain_milli") or 3925
    mode = "inter3" if inter3 else "single"
    if any(calib_mod.offset_of(cal, c, gain) for c in range(3)):
        return calib_mod.apply(v, cal, mode, 0, gain)
    return dsp.align_channels(v, 3) if inter3 else v


def _eq(v, fs, eq, on):
    """模拟链路幅频补偿（板子 500 kHz 掉 ~19 dB，不补偿 Upp 会明显偏低）。"""
    if not on:
        return v
    return eq_mod.apply(v, fs, eq)


def cmd_calib(args):
    """零输入标定：采一批无信号数据，把每路直流偏置写进 calib.json。"""
    inter3 = not args.single
    n = args.n or (proto.MAX_POINTS_TOTAL if inter3 else proto.MAX_POINTS_PER_CH)
    with proto.LsaClient(args.host, args.port, args.timeout) as c:
        h, raw = c.capture(gear=args.gear, inter3=inter3, ch=args.ch, n=n, timeout=6.0)
        a = np.asarray(proto.raw_to_int16(raw), dtype=np.float64)
        gain = h.get("gain_milli") or 3925
        v = dsp.codes_to_volts(a, gain)
        offs = calib_mod.offsets_from(v, "inter3" if inter3 else "single", gain)
        cal = calib_mod.build(calib_mod.load(args.calib), inter3, gain, offs,
                              source="shorted_or_floating")
        path = calib_mod.save(cal, args.calib)
    print("写出 %s" % path)
    print(calib_mod.summary(cal))
    return 0


def cmd_cap(args):
    cal = calib_mod.load(args.calib)
    with proto.LsaClient(args.host, args.port, args.timeout) as c:
        h, raw = c.capture(gear=args.gear, inter3=not args.single,
                           ch=args.ch, n=args.n, timeout=6.0)
        a = np.asarray(proto.raw_to_int16(raw), dtype=np.float64)
        fs = float(h["fs_hz"])
        v = dsp.codes_to_volts(a, h["gain_milli"] or 3925)
        v = _fix(cal, v, h, inter3=not args.single)
        v = _eq(v, fs, eq_mod.load(args.eq), not args.no_eq)
        print("header:", h)
        report(v, fs, "实测 gear%d" % h["gear"], codes=a, gain_milli=h["gain_milli"])
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="周期信号谐波分析装置 上位机联调工具")
    ap.add_argument("--host", default="192.168.57.200")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--timeout", type=float, default=3.0)
    ap.add_argument("--calib", default=None, help="标定文件路径（默认同目录 calib.json）")
    ap.add_argument("--eq", default=None, help="幅频补偿模型路径（默认同目录 eq.json）")
    ap.add_argument("--no-eq", action="store_true", help="关闭幅频补偿（看原始幅度时用）")
    ap.add_argument("--in-scale", type=float, default=None,
                    help="输入比例：显示 = 实测×该值（50Ω 等效填 0.5；缺省读 config.json）")
    sub = ap.add_subparsers(dest="cmd")

    p = sub.add_parser("ping")
    sub.add_parser("selftest")
    sub.add_parser("demo")
    sub.add_parser("auto")
    p = sub.add_parser("calib")
    p.add_argument("--gear", type=int, default=0)
    p.add_argument("--single", action="store_true")
    p.add_argument("--ch", type=int, default=0)
    p.add_argument("--n", type=int, default=0)
    p = sub.add_parser("cap")
    p.add_argument("--gear", type=int, default=0)
    p.add_argument("--single", action="store_true")
    p.add_argument("--ch", type=int, default=0)
    p.add_argument("--n", type=int, default=0)

    args = ap.parse_args(argv)
    args.n = getattr(args, "n", 0) or 0

    # 输入比例：命令行优先，否则读 config.json（与触摸界面共用同一份）
    if args.in_scale is not None:
        dsp.IN_SCALE = float(args.in_scale)
    else:
        dsp.IN_SCALE = cfg_mod.get_in_scale()

    if args.cmd == "demo":
        return cmd_demo(args)
    if args.cmd == "auto":
        return cmd_auto(args)
    if args.cmd == "calib":
        return cmd_calib(args)
    if args.cmd == "cap":
        if not args.n:
            args.n = proto.MAX_POINTS_PER_CH if args.single else proto.MAX_POINTS_TOTAL
        return cmd_cap(args)
    if args.cmd == "selftest":
        with proto.LsaClient(args.host, args.port, args.timeout) as c:
            h = c.selftest()
            print("selftest flags=0x%04X inter3_ok=%s" % (h["flags"], h.get("inter3_ok")))
        return 0
    if args.cmd == "ping":
        with proto.LsaClient(args.host, args.port, args.timeout) as c:
            print(c.ping())
        return 0
    ap.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
