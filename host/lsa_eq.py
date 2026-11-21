"""
lsa_eq —— 模拟前端幅频响应的软件补偿（均衡）

为什么需要：
    实测这块板子的模拟链路在 500 kHz 已衰减约 18.7 dB（正常设计应当平直 ±1 dB），
    硬件一时改不动，就在数字域把幅频响应拉回来。

同一趟 FFT 里做三件事：
    1) 带内均衡：每根谱线乘 1/|H(f)|，零相位（只改幅度，不改相位）；
    2) 低端高通：压掉 f_lo 以下的漂移 / 1/f / 直流残差；
    3) 高端低通：压掉 f_hi 以上（与"去 u3"的 700 kHz 一致）。

实测精度（合成信号按实测响应衰减后再补偿）：
    f = 100k/200k/300k/400k/500k 时，150.00 mVpp 输入 -> 150.00 mVpp 输出（<0.05%），
    前提是裁掉两端各 TRIM_S 秒的过渡带（本模块自动裁）。

默认模型来自实测拟合：单级二阶 fn=167.7 kHz、Q=0.857。eq.json 可覆盖：
    {"model": {"fn": 167700.0, "Q": 0.857}}
    {"table": [[10000, 0.998], [20000, 0.995], ...]}      # 频率 Hz, 线性幅度
"""

import json
import math
import os

import numpy as np

EQ_VERSION = 1
DEF_NAME = "eq.json"

#: 实测拟合（2026-11-11：100k/200k/300k/400k/500k = −0.14/−2.99/−9.04/−15.56/−18.66 dB）
# 2026-11-11 线性扫频实测（10k~560k，462 点，拟合残差 0.27 dB）
# 整条模拟链等效为「单级二阶」：fc=364.1 kHz、Q=0.755
#   500 kHz: 实测 -6.17 dB   1 MHz: -17.5 dB
# 2026-11-13 换硬件后重测（三路交替，对数扫频 10k~599k，355 点，拟合残差 0.03 dB）
#   → fc=540.6 kHz、Q=0.607；500 kHz -3.65 dB，599 kHz -5.29 dB
DEFAULT_FN = 540.6e3
DEFAULT_Q = 0.607

#: 默认工作区间
DEF_F_LO = 5e3
DEF_F_HI = 700e3
DEF_MAX_BOOST_DB = 24.0

#: 频域补偿后两端必须裁掉的过渡时间（实测 0.4 ms 即可完全消除误差，取 0.45 ms 留余量）
TRIM_S = 0.45e-3


# ──────────────────────────────────────────────────────────────
# 响应模型
# ──────────────────────────────────────────────────────────────
def analytic(f, fn=DEFAULT_FN, Q=DEFAULT_Q):
    """单级二阶低通的线性幅度 |H(f)|"""
    x = np.asarray(f, dtype=np.float64) / float(fn)
    return 1.0 / np.sqrt((1.0 - x * x) ** 2 + (x / float(Q)) ** 2)


def save_table(table, fn, Q, path=None):
    """把实测分箱表连同拟合的 (fn, Q) 一起写进 eq.json。

    表比单级二阶解析式准得多：4 阶板 + FDA 的真实曲线在末端比二阶更陡，
    用解析式会在 500 kHz 处差 1~2 dB，用表就是查表插值，误差只来自分箱。
    """
    p = path or default_path()
    doc = {"version": EQ_VERSION, "source": "table",
           "table": [[float(a), float(b)] for a, b in table]}
    if fn:
        doc["model"] = {"fn": float(fn), "Q": float(Q)}
    # 重新标定幅频表时不要丢掉相频表：相位表是单独一次闭环标定出来的，
    # 丢掉之后 90°/270° 谐波相位下的 Upp 会重新超预算（实测 +7~8 mV）。
    try:
        with open(p, encoding="utf-8") as fh:
            old = json.load(fh)
    except Exception:
        old = {}
    if isinstance(old, dict):
        for key in ("phase", "note"):
            if key in old and key not in doc:
                doc[key] = old[key]
    tmp = p + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=2)
    os.replace(tmp, p)
    return doc

def default_path():
    p = os.environ.get("LSA_EQ", "").strip()
    if p:
        return p
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), DEF_NAME)


def load(path=None):
    """读 eq.json；不存在或损坏都退回内置模型，绝不抛异常。"""
    p = path or default_path()
    try:
        with open(p, "r", encoding="utf-8") as f:
            d = json.load(f)
        if d.get("table"):
            d["table"] = [(float(a), float(b)) for a, b in d["table"]]
            d.setdefault("source", "table")
            if d.get("phase"):
                try:                     # 相移表交给 dsp，供波形重建去旋转
                    import lsa_dsp
                    lsa_dsp.set_phase_table(d["phase"])
                except Exception:
                    pass
            return d
        if "model" in d:
            d.setdefault("source", "model")
            return d
    except Exception:
        pass
    return {"model": {"fn": DEFAULT_FN, "Q": DEFAULT_Q}, "source": "builtin"}


def save_model(fn, Q, path=None):
    p = path or default_path()
    d = {"model": {"fn": float(fn), "Q": float(Q)}, "source": "fit", "version": EQ_VERSION}
    tmp = p + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(d, f, ensure_ascii=False, indent=2)
    os.replace(tmp, p)
    return d


def gain_of(f, eq=None):
    """给定频率处的线性幅度 |H(f)|"""
    eq = eq or load()
    f = np.atleast_1d(np.asarray(f, dtype=np.float64))
    if eq.get("table"):
        xs = np.array([a for a, _ in eq["table"]], dtype=np.float64)
        ys = np.array([b for _, b in eq["table"]], dtype=np.float64)
        return np.maximum(np.interp(f, xs, ys, left=ys[0], right=ys[-1]), 1e-6)
    m = eq.get("model", {})
    return np.maximum(analytic(f, m.get("fn", DEFAULT_FN), m.get("Q", DEFAULT_Q)), 1e-6)


def chain_gain(f, eq=None, in_scale=None, gain_total=3.925, vref=3.0, full=2047.0):
    """
    整链增益 G(f) = 输入端幅度(V) / ADC 码幅度(code)。

        G(f) = (vref/full)/gain_total · in_scale / |H(f)|

    前一项把「码」换成 ADC 差分电压（gain_total 是协议里约定的模拟增益），
    in_scale 把「低频平台 Upp」折算成标称幅度（≈1/2.56），
    再除以 |H(f)| 把这一频点的衰减补回来。

    统一口径的意义：分量幅值直接 a_code·G(f) 得到，不必「先换算成伏特、
    再频域补偿、再乘 in_scale」——那条路上同一个比例会被算三次，任何一处
    改了另一半就悄悄失配。实测（250 mVpp 正弦）a_code·G(f) 与整链结果
    在 10k~500k 差 <0.01 mV。
    """
    if in_scale is None:
        try:
            import lsa_cfg
            in_scale = lsa_cfg.get_in_scale()
        except Exception:
            in_scale = 1.0
    k_adc = float(vref) / float(full) / float(gain_total)      # V/code（ADC 侧）
    return k_adc * float(in_scale) / gain_of(f, eq)


def summary(eq=None):
    """一行文字：来源 + 几个关键频点的实测增益"""
    eq = eq or load()
    src = {"builtin": "内置拟合", "model": "eq.json 模型",
           "table": "eq.json 实测表", "fit": "eq.json 拟合"}.get(
        eq.get("source", ""), str(eq.get("source", "?")))
    f = np.array([10e3, 100e3, 300e3, 500e3])
    g = gain_of(f, eq)
    parts = " ".join("%.0fk:%+.1fdB" % (ff / 1e3, 20 * math.log10(gg)) for ff, gg in zip(f, g))
    return "%s  %s" % (src, parts)


def boost_at(f, eq=None, max_boost_db=DEF_MAX_BOOST_DB):
    """该频点的补偿量（dB，正数表示提升）"""
    g = float(gain_of(np.array([f]), eq)[0])
    return min(-20.0 * math.log10(g), max_boost_db)


# ──────────────────────────────────────────────────────────────
# 频域补偿
# ──────────────────────────────────────────────────────────────
def trim_samples(fs):
    """补偿后两端要裁掉的样点数"""
    return int(round(TRIM_S * float(fs)))


def _mask(f, f_lo, f_hi):
    """带通掩模：低端 0→f_lo 升余弦、高端 f_hi 起升余弦，避免硬切振铃"""
    m = np.ones_like(f)
    if f_lo > 0:
        m *= 0.5 * (1.0 - np.cos(np.pi * np.clip(f / float(f_lo), 0.0, 1.0)))
    w_hi = max(float(f_hi) * 0.08, 1.0)
    m *= 0.5 * (1.0 + np.cos(np.pi * np.clip((f - float(f_hi)) / w_hi, 0.0, 1.0)))
    return m


def next_pow2(n):
    p = 1
    while p < n:
        p <<= 1
    return p


def apply(v, fs, eq=None, f_lo=DEF_F_LO, f_hi=None,
          max_boost_db=DEF_MAX_BOOST_DB, trim=True):
    """
    对时域信号做幅频补偿，零相位。

    f_hi=None 表示不设硬低通：只补到表/模型覆盖的最高频率，
    再往上保持该点的增益（常数放大，不改变频谱形状）。
    这样 1 MHz 的 u3（题3）不会被抹掉——抑制 u3 是「去 u3」那个 FIR 的事，
    补偿只负责把通带拉平。

    返回补偿后的数组；trim=True 时已裁掉两端 TRIM_S 秒的过渡带
    （不裁的话 100 kHz 会偏高约 1.2 mV，这是高通掩模的建立时间）。
    """
    v = np.asarray(v, dtype=np.float64)
    n = v.size
    if n < 256:
        return v.copy()

    pad = int(n * 0.5)
    x = np.concatenate([v[:pad][::-1], v, v[-pad:][::-1]])
    m = x.size
    nfft = next_pow2(m)

    X = np.fft.rfft(x, nfft)
    f = np.fft.rfftfreq(nfft, 1.0 / float(fs))

    g = gain_of(f, eq)
    comp = np.minimum(1.0 / g, 10.0 ** (max_boost_db / 20.0))

    # 最高补偿频率：查表时取表的最后一点，解析模型时取 700 kHz
    if f_hi is None:
        tab = (eq or {}).get("table")
        f_hi = float(tab[-1][0]) if tab else DEF_F_HI
        f_hi = max(f_hi, DEF_F_LO * 2.0)
    # f_hi 以上不再补偿：从 f_hi 处的补偿量线性收口到 1.0（跨度=0.25*f_hi），
    # 再往上是 1.0（直通）。
    # 理由：表只测到 550 kHz；若照旧"保持 f_hi 处增益"，那里已经要补 ×7.2，
    # 1 MHz 的 u3 会被放大 7 倍，宽带噪声也同比放大，谱图和 Upp 都会虚高。
    # 现场调参开关（不设 = 默认行为一字不变）：
    #   LSA_EQ_FHI_HZ=520000  把"最高补偿频率"压到 520 kHz，末端不再补 ×7
    #   LSA_EQ_LP_HZ=520000   补偿后加一道硬低通（0/未设 = 不加）
    # 为什么需要：eq 表在 550 kHz 处要补 ×7.12，而信号源在该带的固定残留
    # （实测 DG4202 ≈5.7 mV @585 kHz，不随输出幅度变）被同比放大后，
    # 会把 Upp 抬高 60~90 mV。带限之后 Upp 回到设计值 ±7 mV 以内。
    _env_fhi = os.environ.get("LSA_EQ_FHI_HZ") or ""
    if _env_fhi:
        try:
            f_hi = float(_env_fhi)
        except ValueError:
            pass
    idx = int(np.searchsorted(f, f_hi))
    idx = min(max(idx, 1), comp.size - 1)
    c_hi = float(comp[idx])
    if c_hi > 1.0:
        taper = np.clip((f - f_hi) / (0.25 * f_hi), 0.0, 1.0)
        comp[idx + 1:] = c_hi + (1.0 - c_hi) * taper[idx + 1:]
    else:
        comp[idx + 1:] = c_hi
    comp = np.maximum(comp, 1.0 / 10.0 ** (max_boost_db / 20.0))

    _lp = 0.0
    _env_lp = os.environ.get("LSA_EQ_LP_HZ") or ""
    if _env_lp:
        try:
            _lp = float(_env_lp)
        except ValueError:
            _lp = 0.0
    y = np.fft.irfft(X * comp * _mask(f, f_lo, _lp if _lp > 0 else f_hi * 1e9),
                     nfft)[:m]
    y = y[pad:pad + n]

    if trim:
        t = trim_samples(fs)
        if n - 2 * t >= 512:
            y = y[t:n - t]
    return y


def fit_points(freqs, amps, fmin=8e3, fmax=560e3):
    """用实测 (频率, 幅度) 点集最小二乘拟合单级二阶 fc/Q。

    幅度先按低频平台归一化，然后只在 dB 域拟合形状——绝对增益由调用方另行
    标定（这样形状与增益耦合不会互相污染）。返回 (fn, Q, rms_db, ref_amp)。
    """
    import numpy as np

    f = np.asarray(freqs, dtype=np.float64)
    a = np.asarray(amps, dtype=np.float64)
    m = (f > 0) & (a > 0) & np.isfinite(f) & np.isfinite(a) & (f >= fmin) & (f <= fmax)
    f, a = f[m], a[m]
    if f.size < 8:
        raise ValueError("有效点太少（%d），扫频没采到数据？" % f.size)
    low = f < max(40e3, float(np.percentile(f, 12)))
    if low.sum() < 5:
        low = f < float(np.percentile(f, 15))
    ref = float(np.median(a[low]))
    db = 20.0 * np.log10(a / ref)

    def cost(fn, Q):
        x = f / fn
        h = 1.0 / np.sqrt((1.0 - x * x) ** 2 + (x / Q) ** 2)
        return float(np.sqrt(np.mean((db - 20.0 * np.log10(h)) ** 2)))

    best = None
    for fn in np.linspace(2.4e5, 9.5e5, 61):
        for Q in np.linspace(0.40, 1.60, 55):
            r = cost(fn, Q)
            if best is None or r < best[0]:
                best = (r, float(fn), float(Q))
    r, fn, Q = best
    for span in (0.10, 0.03, 0.01, 0.003):
        for dfn in np.linspace(-span, span, 21) * fn:
            for dq in np.linspace(-span, span, 21) * Q:
                f2, q2 = fn + dfn, Q + dq
                if f2 <= 0 or q2 <= 0:
                    continue
                rr = cost(f2, q2)
                if rr < r:
                    r, fn, Q = rr, float(f2), float(q2)
    return float(fn), float(Q), float(r), float(ref)


def sweep_bins(freqs, amps, nbin=70, fmin=10e3, fmax=520e3, min_per_bin=3):
    """把扫频记录 (f, 幅度) 变成可拟合的曲线。

    扫频数据里既有正程（频率单调上升）也有回程与静止段，回程那部分因为
    3.47 ms 窗内跨了很多频点、谱峰被抹平，幅度偏低。这里
      1) 只保留频率单调上升的点（正程）
      2) 在对数频率上分箱，箱内取中位数（抗离群）
    返回 (f_bins, amp_bins)。
    """
    import numpy as np

    f = np.asarray(freqs, dtype=np.float64)
    a = np.asarray(amps, dtype=np.float64)
    m = (f > 0) & (a > 0) & np.isfinite(f) & np.isfinite(a)
    f, a = f[m], a[m]
    if f.size < 10:
        return np.array([]), np.array([])

    up = np.zeros(f.size, dtype=bool)
    up[1:] = f[1:] > f[:-1] * 1.0002
    idx = np.nonzero(up)[0]
    if idx.size:
        groups = np.split(idx, np.nonzero(np.diff(idx) != 1)[0] + 1)
        good = [g for g in groups if g.size >= 5]
        if good:
            keep = np.concatenate(good)
            f, a = f[keep], a[keep]

    edges = np.logspace(np.log10(fmin), np.log10(fmax), nbin + 1)
    # 点少的时候每个箱里可能只有 1~2 个点，门槛要跟着降，否则整条曲线会被丢空
    need = max(1, min(int(min_per_bin), int(np.ceil(f.size / float(nbin)))))
    fb, ab = [], []
    for i in range(nbin):
        k = (f >= edges[i]) & (f < edges[i + 1])
        if k.sum() >= need:
            fb.append(float(np.sqrt(edges[i] * edges[i + 1])))
            ab.append(float(np.median(a[k])))
    return np.array(fb), np.array(ab)
