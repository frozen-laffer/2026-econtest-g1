#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
现场一键自检（比赛当天开机跑一次）

    python3 check_field.py                # 默认连 192.168.57.200:5000
    python3 check_field.py --host 192.168.57.200 --quick
    python3 check_field.py --no-net       # 只跑离线部分（没接板子时用）

检查项：
    1  配置文件     eq.json 实测表 / config.json 的 in_scale / calib.json
    2  网络         TCP 连接 + PING 应答 + CRC/事务号
    3  采集链路     采一帧 16384 点，看 fs / N / Δf / 耗时
    4  读数合理性   Upp 量级、是否削顶、f1 是否落在合法范围
    5  离线回归     四种波形（正弦/方波/三角/半波）的 Upp 识别

退出码 0 = 全部通过；1 = 有 FAIL。
"""
from __future__ import print_function

import argparse
import os
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import lsa_proto
import lsa_dsp as dsp
import lsa_eq as eqm
import lsa_cfg as cfgm
import lsa_calib as calibm

OK = "OK  "
BAD = "FAIL"
WARN = "WARN"

rows = []


def add(tag, ok, msg):
    """ok: True / False / None(=提示)"""
    mark = OK if ok is True else (BAD if ok is False else WARN)
    rows.append((mark, tag, msg))
    print("  [%s] %-12s %s" % (mark, tag, msg))
    return ok is not False


def head(title):
    print("\n" + "=" * 74)
    print(title)
    print("=" * 74)


# ────────────────────────────── 1. 配置文件 ──────────────────────────────
def check_files():
    head("1  配置文件")
    here = os.path.dirname(os.path.abspath(__file__))

    eqpath = eqm.default_path()
    if os.path.exists(eqpath):
        eq = eqm.load(eqpath)
        n = len(eq.get("table") or [])
        if n:
            add("eq.json", True, "实测表 %d 点   %s" % (n, eqm.summary(eq)))
        else:
            add("eq.json", None, "只有解析模型，没有实测表   %s" % eqm.summary(eq))
    else:
        add("eq.json", False, "不存在：%s（补偿会退回内置模型，精度不可信）" % eqpath)

    isc = cfgm.get_in_scale()
    if 0.2 <= isc <= 2.0:
        add("in_scale", True, "%.6f（输入比例，×%.3f）" % (isc, isc))
    else:
        add("in_scale", False, "%.6f 超出合理范围 [0.2, 2.0]" % isc)

    cal = calibm.load()
    if cal:
        offs = (cal.get("offsets_v") or {}).values()
        worst = max([abs(float(o)) for o in offs] or [0.0])
        if worst > 0.30:
            add("calib.json", False, "偏置最大 %.1f mV，明显是接着信号标定的，请删除后重标"
                % (worst * 1e3))
        else:
            add("calib.json", True, calibm.summary(cal))
    else:
        add("calib.json", None, "未标定（单路测量可以不用）")

    print("  工作目录 %s" % here)


# ────────────────────────────── 2. 网络 ──────────────────────────────
def check_net(host, port):
    head("2  网络与协议")
    t0 = time.time()
    try:
        c = lsa_proto.LsaClient(host=host, port=port, timeout=2.5)
        c.connect()
    except Exception as e:
        add("连接", False, "%s:%d  %s" % (host, port, e))
        print("      排查：网线是否插在树莓派 eth0？板子是否上电？")
        print("           sudo nmcli con up 'Wired connection 1'; ping %s" % host)
        # 板子只允许一条 TCP 连接：界面在跑时本工具必然被拒
        try:
            out = subprocess.run(["bash", "-lc", "ss -tnp 2>/dev/null | grep :%d" % port],
                                 capture_output=True, text=True, timeout=5).stdout.strip()
        except Exception:
            out = ""
        if out:
            print("      本机已有别的客户端占着这条连接（板子只允许一条）：")
            for ln in out.splitlines():
                print("        " + ln.strip())
            print("      先退出界面（pkill -f '[l]sa_touch.py'）再跑本自检。")
        return None
    dt = (time.time() - t0) * 1000.0
    add("连接", True, "%s:%d  用时 %.0f ms" % (host, port, dt))

    try:
        h = c.ping()
    except Exception as e:
        add("PING", False, "无应答：%s" % e)
        c.close()
        return None
    if not h.get("crc_ok"):
        add("PING", False, "CRC 校验失败（回包被改坏）")
        c.close()
        return None
    if not h.get("tok_ok"):
        add("PING", False, "事务号不回显（tok=%s）" % h.get("tok"))
        c.close()
        return None
    add("PING", True, "ver=%s crc=OK tok=OK" % h.get("ver"))
    return c


# ────────────────────────────── 3. 采集链路 ──────────────────────────────
def check_capture(c, gear=0, n=16384):
    head("3  采集链路")
    t0 = time.time()
    try:
        hdr, raw = c.capture(gear=gear, inter3=False, ch=0, n=n, timeout=6.0)
    except Exception as e:
        add("采集", False, "%s" % e)
        return None
    dt = (time.time() - t0) * 1000.0
    codes = np.asarray(lsa_proto.raw_to_int16(raw), dtype=np.int16)
    if codes.size != hdr["n_this"]:
        add("采集", False, "点数不符：收到 %d，头里 %d" % (codes.size, hdr["n_this"]))
        return None
    fs = float(hdr["fs_hz"])
    add("采集", True, "n=%d  fs=%.3f MSPS  用时 %.0f ms"
        % (codes.size, fs / 1e6, dt))

    clip = float(np.mean(np.abs(codes.astype(np.int32)) >= 2046))
    v_raw = dsp.codes_to_volts(codes, hdr.get("gain_milli") or 3925, offset_code=0)
    v = eqm.apply(v_raw, fs, eqm.load(), trim=True)
    # 与界面同一条链路：先幅频补偿，再乘输入比例（顺序反了会把补偿量一起打对折）
    isc = float(cfgm.get_in_scale() or 1.0)
    v = v * isc
    margin = eqm.trim_samples(fs)
    m = dsp.measure(v, fs, margin=margin)
    m["upp_raw"] = float(np.ptp(v_raw)) * isc
    n_used = int(m.get("n") or 0)
    df = fs / max(1, n_used)
    add("分辨率", df <= 500.0, "N=%d  df=%.0f Hz（赛题要求 ≤500 Hz）" % (n_used, df))
    return hdr, codes, v, m, clip, dt


# ────────────────────────────── 4. 读数合理性 ──────────────────────────────
def check_reading(ctx):
    head("4  读数合理性")
    hdr, codes, v, m, clip, dt = ctx
    if clip > 2e-4:
        add("削顶", False, "ADC 削顶 %.2f%%（输入太大或直流偏置太高）" % (clip * 100))
    else:
        add("削顶", True, "0（未削顶）")

    upp_mv = m["upp"] * 1e3
    fs_pp_mv = 2.0 * (3.0 / 3.925) * float(cfgm.get_in_scale() or 1.0) * 1e3
    if upp_mv < 0.5:
        add("Upp", None, "%.2f mV —— 输入似乎没接信号" % upp_mv)
    elif upp_mv > fs_pp_mv * 0.98:
        add("Upp", False, "%.1f mV 超过输入满量程 %.0f mVpp" % (upp_mv, fs_pp_mv))
    else:
        add("Upp", True, "%.2f mV（输入满量程 %.0f mVpp，占 %.0f%%）"
            % (upp_mv, fs_pp_mv, 100.0 * upp_mv / fs_pp_mv))

    f1 = m["f1"]
    add("f1", 1000.0 <= f1 <= fs0(ctx) * 0.5,
        "%.1f Hz（Nyquist %.0f kHz）" % (f1, fs0(ctx) * 0.5 / 1e3))
    add("Udc", abs(m["udc"]) < 0.9,
        "%.1f mV（EQ 后应接近 0）" % (m["udc"] * 1e3))
    add("耗时", dt < 2000.0, "本次 %.0f ms（赛题上限 2 s）" % dt)
    print("      波形类型 %s    Upp=%.2f mV  Urms=%.2f mV  基波=%.2f mV"
          % (m.get("wave_kind"), upp_mv, m["urms"] * 1e3, m["a1"] * 1e3))
    return m


def fs0(ctx):
    return float(ctx[0]["fs_hz"])


# ────────────────────────────── 5. 离线回归 ──────────────────────────────
def check_offline():
    head("5  离线回归（四种波形的 Upp 识别，真值 250 mVpp）")
    fs = 3541666.0
    n = 16384
    t = np.arange(n) / fs
    ph = 2 * np.pi * 100e3 * t
    waves = [
        ("正弦", np.sin(ph), "mixed"),
        ("方波", np.sign(np.sin(ph)), "square"),
        ("三角波", 2.0 / np.pi * np.arcsin(np.sin(ph)), "triangle"),
        ("半波整流", np.maximum(np.sin(ph), 0.0), "half"),
    ]
    worst = 0.0
    for name, x, want in waves:
        x = x / (np.max(x) - np.min(x)) * 0.250          # 归一到 250 mVpp
        kind, _fac = dsp.waveform_kind(x, fs, 100e3)
        m = dsp.measure(x, fs)
        err = abs(m["upp"] * 1e3 - 250.0)
        worst = max(worst, err)
        ok = err <= 5.0
        if want != "mixed" and kind != want:
            ok = False
            add(name, False, "判型 %s（应为 %s）  Upp=%.2f mV" % (kind, want, m["upp"] * 1e3))
        else:
            add(name, ok, "判型 %-9s Upp=%.3f mV  误差 %+.3f mV"
                % (kind, m["upp"] * 1e3, m["upp"] * 1e3 - 250.0))
    add("最坏", worst <= 5.0, "%.3f mV（判据 ≤5 mV）" % worst)


# ────────────────────────────── main ──────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.57.200")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--gear", type=int, default=0)
    ap.add_argument("--n", type=int, default=16384)
    ap.add_argument("--no-net", action="store_true", help="跳过板子相关检查")
    ap.add_argument("--quick", action="store_true", help="跳过离线回归")
    args = ap.parse_args()

    print("周期信号谐波分析装置  现场自检")
    print("时间 %s   Python %s" % (time.strftime("%Y-%m-%d %H:%M:%S"),
                                   sys.version.split()[0]))
    check_files()

    c = None
    ctx = None
    if not args.no_net:
        c = check_net(args.host, args.port)
        if c is not None:
            try:
                ctx = check_capture(c, gear=args.gear, n=args.n)
            except Exception as e:
                add("采集", False, "%s" % e)
            if ctx is not None:
                check_reading(ctx)
    else:
        head("2/3/4  网络与采集（已跳过）")

    if not args.quick:
        try:
            check_offline()
        except Exception as e:
            add("离线回归", False, "%s" % e)

    if c is not None:
        try:
            c.close()
        except Exception:
            pass

    head("汇总")
    nbad = sum(1 for r in rows if r[0] == BAD)
    nwarn = sum(1 for r in rows if r[0] == WARN)
    nok = sum(1 for r in rows if r[0] == OK)
    print("  通过 %d   提示 %d   失败 %d" % (nok, nwarn, nbad))
    if nbad:
        print("\n  失败项：")
        for mark, tag, msg in rows:
            if mark == BAD:
                print("    - %-12s %s" % (tag, msg))
        print("\n  结论：不可上场，先按上面提示排查。")
        return 1
    print("\n  结论：可以上场。" + ("（有提示项，留意即可）" if nwarn else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
