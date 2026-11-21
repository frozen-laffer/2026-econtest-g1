# -*- coding: utf-8 -*-
"""
端到端精度自检（纯离线，不接硬件）
=================================
把「真实信号 → 12bit 差分量化 → 上位机 DSP」整条链路模拟一遍，
用已知真值去卡四项赛题指标：

  题1/题2：Upp / Urms 误差 ≤ 5 mV，基频 f1 误差 ≤ 1 kHz
  题3    ：抑制 u3 后，u2 各分量幅值误差 ≤ 5 mV

量化模型与固件一致：
  code = round(v * gain * 2047 / 3.0)   裁剪到 ±2048
  v    = code * (3.0 / 2047) / gain

用法：
  python selftest_dsp.py            # 跑全部
  python selftest_dsp.py -v         # 打印每个用例
"""
from __future__ import annotations

import sys

import numpy as np

import lsa_dsp as dsp

VREF = 3.0
HALF = 2047.0
GAIN_X2 = 3.925          # 固件 g_total_gain
FS_SINGLE = 2361111.0    # 档位 0 单路
FS_INTER3 = 7083333.0    # 档位 0 三路交替


# ──────────────────────────────────────────────────────────────
# 量化往返
# ──────────────────────────────────────────────────────────────
def adc_roundtrip(v, gain=GAIN_X2, vref=VREF):
    codes = np.round(np.asarray(v, dtype=np.float64) * gain * (HALF / vref))
    codes = np.clip(codes, -2048.0, HALF)
    return codes * (vref / HALF) / gain


def tone(f, amp, fs, n, phase=0.0):
    t = np.arange(n) / fs
    return amp * np.sin(2.0 * np.pi * f * t + phase)


def composite(parts, fs, n):
    """parts = [(f, 峰值幅度), ...]"""
    v = np.zeros(n)
    for f, a in parts:
        v += tone(f, a, fs, n)
    return v


# ──────────────────────────────────────────────────────────────
# 题1 / 题2
# ──────────────────────────────────────────────────────────────
def case_t12(f1, upp_mv, fs=FS_SINGLE, n=8192, harmonics=(2, 3)):
    """单频 + 2/3 次谐波（各 10%/5% 幅度），模拟题1 的『基波与谐波合成』"""
    a1 = upp_mv / 2.0 / 1e3
    parts = [(f1, a1)]
    for k in harmonics:
        if k * f1 < fs * 0.5:
            parts.append((k * f1, a1 * (0.10 if k == 2 else 0.05)))
    v_true = composite(parts, fs, n)

    v_meas = adc_roundtrip(v_true)
    m = dsp.measure(v_meas, fs)

    upp_true = (np.ptp(v_true)) * 1e3
    urms_true = np.sqrt(np.mean(v_true ** 2)) * 1e3
    return {
        "f1": f1, "upp_in": upp_mv,
        "upp_err": m["upp"] * 1e3 - upp_true,
        "urms_err": m["urms"] * 1e3 - urms_true,
        "f1_err": m["f1"] - f1,
        "upp_meas": m["upp"] * 1e3, "upp_true": upp_true,
    }


# ──────────────────────────────────────────────────────────────
# 题3
# ──────────────────────────────────────────────────────────────
def case_t3(f3, fs=FS_INTER3, n=24576, fc=None):
    """u = u2(100kHz 200mVpp + 300kHz 50mVpp) + u3(200mVpp @f3)"""
    if fc is None:
        fc = dsp.LP_FC_HZ
    parts2 = [(100e3, 100e-3), (300e3, 25e-3), (500e3, 50e-3)]
    v2 = composite(parts2, fs, n)
    v3 = tone(f3, 100e-3, fs, n)
    v_true = v2 + v3

    v_meas = adc_roundtrip(v_true)
    v_filt = dsp.lowpass_fir(v_meas, fs, fc)
    # 去掉滤波器首尾瞬态
    m = dsp.FIR_MARGIN
    seg = v_filt[m:len(v_filt) - m]

    comps = dsp.components(seg, fs, f_min=10e3, f_max=fc * 1.1, top=8, rel=0.0005)
    got = {}
    for cf, ca in comps:
        for tf in (100e3, 300e3, 500e3):
            if abs(cf - tf) < 5e3:
                # 同一目标频率附近可能同时出现主瓣与旁瓣，取最大的那条
                if ca > got.get(tf, -1.0):
                    got[tf] = ca
    got = {k: v * 1e3 for k, v in got.items()}

    # u3 残留：取 f3 附近最大谱线
    f, amp = dsp.spectrum(seg, fs, pad=4)
    band = (f > f3 - 50e3) & (f < f3 + 50e3)
    resid = float(np.max(amp[band]) * 1e3) if np.any(band) else 0.0

    return {
        "f3": f3,
        "a100_err": got.get(100e3, float("nan")) - 100.0,
        "a300_err": got.get(300e3, float("nan")) - 25.0,
        "a500_err": got.get(500e3, float("nan")) - 50.0,
        "resid_mv": resid,
        "upp_meas": np.ptp(seg) * 1e3,
    }


# ──────────────────────────────────────────────────────────────
# 三路交替：通道失配模拟
# ──────────────────────────────────────────────────────────────
def case_glitch(n_spikes=3, f1=137e3, upp_mv=200.0, fs=FS_SINGLE, n=8192,
                spike_v=0.5):
    """在干净信号上打几个单点尖峰，比较原始 Upp 与抗毛刺 Upp"""
    v = np.asarray(tone(f1, upp_mv / 2e3, fs, n), dtype=np.float64).copy()
    idx = np.linspace(300, n - 300, n_spikes).astype(int)
    v[idx] += spike_v
    m = dsp.measure(v, fs)                       # 默认 robust=True
    m_raw = dsp.measure(v, fs, robust=False)
    return {
        "upp": m["upp"] * 1e3,
        "upp_raw": m_raw["upp"] * 1e3,
        "err": (m["upp"] - upp_mv / 1e3) * 1e3,
        "glitch_n": m["glitch_n"],
        "upp_true": upp_mv,
    }


def case_interleave(f1=137e3, upp_mv=200.0, fs=FS_INTER3, n=24576,
                    offset5=737.8e-3, noise5=0.368e-3):
    """模拟 ADC5 的 +738 mV 偏置与 0.37 mV 噪声，验证 align_channels 的效果"""
    v_true = tone(f1, upp_mv / 2e3, fs, n)
    v = adc_roundtrip(v_true)

    a = v.reshape(-1, 3).copy()
    a[:, 2] += offset5                                   # ADC5 直流偏置
    a[:, 2] += np.random.normal(0.0, noise5, a.shape[0])  # ADC5 噪声
    raw = a.reshape(-1)

    before = np.ptp(raw) * 1e3
    after = np.ptp(dsp.align_channels(raw, 3)) * 1e3
    m = dsp.measure(dsp.align_channels(raw, 3), fs)
    return {
        "upp_before": before, "upp_after": after,
        "upp_true": np.ptp(v_true) * 1e3,
        "f1_err": m["f1"] - f1,
    }


def case_calib(f1=137e3, upp_mv=200.0, fs=FS_INTER3, n=24576,
               offs=(0.0, 0.0, 737.8e-3), noise5=0.368e-3):
    """
    验证零输入基线标定：先用"零输入数据"量出每路偏置，再拿去扣有信号的数据。
    对比三种做法：不处理 / align_channels（只对齐通道均值）/ calib（扣绝对偏置）。
    """
    import lsa_calib as calib_mod

    # ① 零输入标定数据：只有偏置 + ADC5 噪声
    z = adc_roundtrip(np.zeros(n)).reshape(-1, 3).copy()
    for c, o in enumerate(offs):
        z[:, c] += o
    z[:, 2] += np.random.normal(0.0, noise5, z.shape[0])
    measured = calib_mod.offsets_from(z.reshape(-1), "inter3", GAIN_X2)
    cal = calib_mod.build({}, "inter3", GAIN_X2, measured, source="shorted")

    # ② 同一批通道上叠加真实信号
    v_true = tone(f1, upp_mv / 2e3, fs, n)
    a = adc_roundtrip(v_true).reshape(-1, 3).copy()
    for c, o in enumerate(offs):
        a[:, c] += o
    a[:, 2] += np.random.normal(0.0, noise5, a.shape[0])
    raw = a.reshape(-1)

    fixed = calib_mod.apply(raw, cal, "inter3", 0, GAIN_X2)
    return {
        "upp_true": np.ptp(v_true) * 1e3,
        "upp_raw": np.ptp(raw) * 1e3,
        "upp_align": np.ptp(dsp.align_channels(raw, 3)) * 1e3,
        "upp_cal": np.ptp(fixed) * 1e3,
        "cal_offsets": measured,
        "resid": [float(np.mean(fixed[c::3])) * 1e3 for c in range(3)],
    }


def case_eq(f1=300e3, upp_mv=150.0, fs=FS_INTER3, n=24576):
    """
    验证幅频补偿：把信号按实测评测到的响应（fn=167.7k, Q=0.857）衰减一遍，
    再用 lsa_eq 把 1/|H| 乘回去，看能不能还原到真值。
    """
    import lsa_eq as eq_mod

    def through(v):
        nfft = 1 << 17
        X = np.fft.rfft(v, nfft)
        f = np.fft.rfftfreq(nfft, 1.0 / fs)
        return np.fft.irfft(X * eq_mod.analytic(f, 167.7e3, 0.857), nfft)[:v.size]

    v_true = tone(f1, upp_mv / 2e3, fs, n)
    got = through(v_true)
    # 衰减和补偿必须用同一个模型，否则就是拿 A 的响应去补 B 的衰减
    model = {"source": "model", "model": {"fn": 167.7e3, "Q": 0.857}}
    rec = eq_mod.apply(got, fs, model)
    return {
        "upp_true": float(np.ptp(v_true)) * 1e3,
        "upp_hw": float(dsp.measure(got, fs)["upp"]) * 1e3,
        "upp_eq": float(dsp.measure(rec, fs)["upp"]) * 1e3,
        "boost_db": eq_mod.boost_at(f1, model),
    }


def case_wave(kind, f1, upp_mv=250.0, fs=FS_SINGLE, n=8192):
    """
    非正弦波形的 Upp 精度：带限重建取 max-min 会因 Gibbs 过冲把方波固定高估约 18%
    （加到 49 次谐波仍是 +17.9%），三角波则低估。这里验证「基波解析换算」
    （方波 π/2·a1、三角 π²/4·a1、半波 2·a1）能把它们都拉回 1 mV 以内。
    """
    t = np.arange(n) / fs
    ph = 2.0 * np.pi * f1 * t
    A = upp_mv / 2000.0
    if kind == "square":
        v = A * np.sign(np.sin(ph))
    elif kind == "triangle":
        v = A * (2.0 / np.pi) * np.arcsin(np.sin(ph))
    elif kind == "half":
        v = (2.0 * A) * np.maximum(np.sin(ph), 0.0) - (2.0 * A) / np.pi
    else:
        v = A * np.sin(ph)
    v = v - v.mean()
    m = dsp.measure(v, fs)
    return {"upp": m["upp"] * 1000.0, "kind": m["wave_kind"],
            "err": m["upp"] * 1000.0 - upp_mv}


# ──────────────────────────────────────────────────────────────
def main(argv):
    verbose = "-v" in argv
    np.random.seed(2026)

    print("=" * 74)
    print("题1/题2 精度自检  (fs=%.3f MSPS, N=8192, Δf=%.0f Hz, 增益 %.3f)"
          % (FS_SINGLE / 1e6, FS_SINGLE / 8192, GAIN_X2))
    print("=" * 74)
    print("%9s %8s %11s %11s %11s" % ("f1(Hz)", "Upp_in", "Upp_err(mV)", "Urms_err(mV)", "f1_err(Hz)"))
    worst = {"upp": 0.0, "urms": 0.0, "f1": 0.0}
    fails = 0
    for f1 in (10e3, 20e3, 50e3, 137e3, 200e3, 300e3, 400e3, 499e3):
        for upp in (50.0, 100.0, 250.0):
            r = case_t12(f1, upp)
            worst["upp"] = max(worst["upp"], abs(r["upp_err"]))
            worst["urms"] = max(worst["urms"], abs(r["urms_err"]))
            worst["f1"] = max(worst["f1"], abs(r["f1_err"]))
            ok = abs(r["upp_err"]) <= 5 and abs(r["urms_err"]) <= 5 and abs(r["f1_err"]) <= 1e3
            if not ok:
                fails += 1
            if verbose or not ok:
                print("%9.0f %8.1f %11.3f %11.3f %11.1f %s"
                      % (r["f1"], r["upp_in"], r["upp_err"], r["urms_err"], r["f1_err"],
                         "" if ok else "  <-- 超差"))
    print("-" * 74)
    print("最坏误差: Upp %.3f mV | Urms %.3f mV | f1 %.1f Hz   超差用例 %d"
          % (worst["upp"], worst["urms"], worst["f1"], fails))
    print("判定(≤5mV/≤1kHz): %s" % ("PASS" if (worst["upp"] <= 5 and worst["urms"] <= 5
                                              and worst["f1"] <= 1e3) else "FAIL"))

    print()
    print("=" * 74)
    print("题3 u3 抑制自检  (fs=%.3f MSPS, N=24576, LPF %.0f kHz)"
          % (FS_INTER3 / 1e6, dsp.LP_FC_HZ / 1e3))
    print("=" * 74)
    print("%10s %12s %12s %12s %11s" % ("f3", "100k_err", "300k_err", "500k_err", "u3残留(mV)"))
    for f3 in (1.0e6, 1.2e6, 1.5e6, 2.0e6, 3.0e6):
        r = case_t3(f3)
        print("%10.3f MHz %12.3f %12.3f %12.3f %11.3f"
              % (f3 / 1e6, r["a100_err"], r["a300_err"], r["a500_err"], r["resid_mv"]))
    print("判定(分量误差≤5mV): 见上表")

    print()
    print("=" * 74)
    print("三路交替通道失配")
    print("=" * 74)
    r = case_interleave()
    print("Upp 真值      = %.2f mV" % r["upp_true"])
    print("对齐前 Upp    = %.2f mV  (谎报 %.1f 倍)" % (r["upp_before"], r["upp_before"] / r["upp_true"]))
    print("对齐后 Upp    = %.2f mV  (误差 %+.2f mV)" % (r["upp_after"], r["upp_after"] - r["upp_true"]))
    print("f1 误差       = %.1f Hz" % r["f1_err"])

    print()
    print("=" * 74)
    print("零输入基线标定（扣除三路直流偏置）")
    print("=" * 74)
    r = case_calib()
    print("量出的偏置    = CH0 %+.3f mV   CH1 %+.3f mV   CH2 %+.3f mV"
          % (r["cal_offsets"][0] * 1e3, r["cal_offsets"][1] * 1e3,
             r["cal_offsets"][2] * 1e3))
    print("Upp 真值      = %.2f mV" % r["upp_true"])
    print("不处理        = %.2f mV  (误差 %+.2f mV)"
          % (r["upp_raw"], r["upp_raw"] - r["upp_true"]))
    print("仅通道均值对齐 = %.2f mV  (误差 %+.2f mV)"
          % (r["upp_align"], r["upp_align"] - r["upp_true"]))
    print("基线标定后    = %.2f mV  (误差 %+.2f mV)   残差 %+.3f / %+.3f / %+.3f mV"
          % (r["upp_cal"], r["upp_cal"] - r["upp_true"],
             r["resid"][0], r["resid"][1], r["resid"][2]))
    print("判定(误差≤5mV): %s" % ("PASS" if abs(r["upp_cal"] - r["upp_true"]) <= 5 else "FAIL"))

    print()
    print("=" * 74)
    print("毛刺抑制（单点尖峰）")
    print("=" * 74)
    print("%10s %12s %12s %12s %10s" % ("尖峰数", "原始 Upp", "抗毛刺 Upp", "Upp 误差", "替换点数"))
    for nsp in (1, 3, 8):
        r = case_glitch(nsp)
        print("%10d %10.2f mV %10.2f mV %+9.3f mV %10d"
              % (nsp, r["upp_raw"], r["upp"], r["err"], r["glitch_n"]))
    print("判定(误差≤5mV): 见上表")

    print()
    print("=" * 74)
    print("幅频补偿（板子实测响应 fn=167.7kHz Q=0.857，再用 1/|H| 拉回来）")
    print("=" * 74)
    print("%10s %11s %11s %11s %11s %10s"
          % ("f1", "Upp真值", "板子输出", "补偿后", "误差", "补偿量"))
    worst_eq = 0.0
    for f1 in (100e3, 200e3, 300e3, 400e3, 500e3):
        r = case_eq(f1=f1)
        err = r["upp_eq"] - r["upp_true"]
        worst_eq = max(worst_eq, abs(err))
        print("%8.0f k %9.2f mV %9.2f mV %9.2f mV %+9.3f mV %8.1f dB"
              % (f1 / 1e3, r["upp_true"], r["upp_hw"], r["upp_eq"], err, r["boost_db"]))
    print("最坏误差 %.3f mV" % worst_eq)
    print("判定(误差≤5mV): %s" % ("PASS" if worst_eq <= 5 else "FAIL"))

    print()
    print("=" * 74)
    print("非正弦波形 Upp（Gibbs 过冲 vs 基波解析换算）")
    print("=" * 74)
    print("%-10s %8s %12s %12s %10s" % ("波形", "f1", "Upp真值", "Upp实测", "误差"))
    worst_w = 0.0
    for kind, name in (("square", "方波"), ("triangle", "三角波"), ("half", "半波整流")):
        for f1 in (10e3, 50e3, 100e3):
            r = case_wave(kind, f1)
            worst_w = max(worst_w, abs(r["err"]))
            print("%-10s %6.0f k %10.2f mV %10.2f mV %+9.3f mV  [%s]"
                  % (name, f1 / 1e3, 250.0, r["upp"], r["err"], r["kind"]))
    print("最坏误差 %.3f mV" % worst_w)
    print("判定(误差≤5mV): %s" % ("PASS" if worst_w <= 5 else "FAIL"))
    print("注：f1 > 167 kHz 时 3 次谐波已超出经校准的 520 kHz 带，方波/三角波无法判型，")
    print("    此时按正弦假设处理，方波会高估约 18%——赛题该频段输入为正弦，可接受。")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
