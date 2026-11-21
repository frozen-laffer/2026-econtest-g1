# -*- coding: utf-8 -*-
"""
lsa_dsp.py —— 信号分析与测量

约定
  * 输入一律是"伏特"数组（已按增益换算），采样率 fs 单位 Hz
  * ADC 为 12 位差分，满量程 ±2047 对应 ±VREF
  * 频谱幅度为"峰值幅度"（正弦波幅度 A，单边谱峰值为 A，不是 RMS）
"""
import math
import os

import numpy as np

VREF = 3.0
HALF_SCALE = 2047.0

#: 去 f3 用低通滤波器的默认阶数与边界丢弃点数
FIR_TAPS = 257
FIR_MARGIN = (FIR_TAPS - 1) // 2

#: 「去 u3」用的低通截止频率（实测，fs=7.083 MSPS，257 阶 Kaiser β=8.6）
#:   600 kHz: 500k 平直、550k −0.18 dB、600k −6 dB、1 MHz −108 dB
#:   700 kHz: 600k 平直、700k −0.2 dB、1 MHz −122 dB  ← 通带更宽、阻带反而更深
#: 题2/题3 的信号成分上限是 500 kHz，留 20% 余量。
LP_FC_HZ = 700e3

#: 手动指定波形时的基波→Upp 换算系数（Upp = k·a1）与 Upp→交流有效值系数。
#:   正弦 a1=A          → Upp=2A            Urms=A/√2 = 0.35355·Upp
#:   方波 a1=(4/π)A     → Upp=2A=(π/2)·a1    Urms=A     = 0.5·Upp
#:   三角 a1=(8/π²)A    → Upp=2A=(π²/4)·a1   Urms=A/√3  = 0.28868·Upp
#:   半波 a1=A/2        → Upp=A=2·a1         Urms_ac=A·√(1/4−1/π²)=0.38549·Upp
_WAVE_FACTOR = {"sine": 2.0, "square": math.pi / 2.0,
                "triangle": math.pi * math.pi / 4.0, "half": 2.0}
_WAVE_URMS = {"sine": 1.0 / (2.0 * math.sqrt(2.0)), "square": 0.5,
              "triangle": 1.0 / (2.0 * math.sqrt(3.0)), "half": 0.3854942}


# ──────────────────────────────────────────────────────────────
# 模拟链谐波附加相移补偿
# ──────────────────────────────────────────────────────────────
#: 模拟链对不同次谐波的相移不一样：Δφ_k(f1) = ∠H(k·f1) − k·∠H(f1)。
#: 它**不改变任何分量的幅值**，却改变合成波形的形状，从而改变峰峰值 ——
#: 实测 200 kHz 处 k=2 的附加相移约 +11°，谐波相对相位取 90° 时 Upp 偏
#: +9.4 mV（预算 5 mV）。表由 `_phase_cal2.py` 闭环实测得到（扫补偿角取
#: 极小点，不依赖相位约定），格式 `[[f1, [[k, 度], ...]], ...]`，按 f1 插值。
PHI_ROWS = []
#: 默认**开启**。表由 `_phase_cal2.py` 闭环实测得到（每点扫 21 个补偿角取
#: 极小点，不依赖相位约定）。实测：45 kHz 以下最优角 ≈0°，200~250 kHz 的
#: 二次谐波 +9~12°；补偿后 90°/270° 相位的误差从 +8~9 mV 降到约 +2 mV。
PHI_ENABLE = os.environ.get("LSA_PHI", "1") != "0"
#: 补偿方向：+1 = 按 +Δφ 旋转，−1 = 按 −Δφ 旋转（用 A/B 实验定，见 _phase_ab.py）
PHI_SIGN = float(os.environ.get("LSA_PHI_SIGN", "-1"))
#: 运行期强制补偿角（度）。标定时一次性采一帧原始数据，再离线扫这个角，
#: 省掉每个角度都重新采样的开销。None = 按表插值。
PHI_OVERRIDE = None


def set_phase_table(rows):
    """装载实测相移表（`lsa_eq.load` 读到 eq.json 的 phase 字段时调用）。"""
    global PHI_ROWS
    out = []
    for f1, row in rows or []:
        try:
            # 每行是 [k, Δφ(度), 该次谐波实测幅度]，只用前两项
            out.append((float(f1), [(int(r[0]), float(r[1])) for r in row]))
        except Exception:
            continue
    out.sort(key=lambda r: r[0])
    PHI_ROWS = out


def dphi_deg(k, f1):
    """查表插值 Δφ_k(f1)，单位度；表里没有这次谐波就返回 0（不补偿）。"""
    if k < 2:
        return 0.0
    if PHI_OVERRIDE is not None:
        try:
            return float(PHI_OVERRIDE)
        except (TypeError, ValueError):
            pass
    ov = os.environ.get("LSA_PHI_OVERRIDE", "")
    if ov != "":                     # 闭环标定用：所有谐波强制同一个角度
        try:
            return float(ov)
        except ValueError:
            pass
    if not PHI_ROWS:
        return 0.0
    xs, ys = [], []
    for f1r, row in PHI_ROWS:
        for kk, dd in row:
            if kk == k:
                xs.append(f1r)
                ys.append(dd)
                break
    if not xs:
        return 0.0
    if len(xs) == 1:
        return float(ys[0])
    return float(np.interp(float(f1), np.asarray(xs), np.asarray(ys)))


# ──────────────────────────────────────────────────────────────
# 换算
# ──────────────────────────────────────────────────────────────
#: 输入比例：显示值 = 板子实测值 × IN_SCALE
#: 信号源按 50Ω 负载设定、而板子输入是高阻时，板子看到的是设定值的 2 倍，
#: 取 0.5 即把显示值折算回"50Ω 等效"。由界面/CLI 调整，默认 1.0（不折算）。
IN_SCALE = 1.0


def codes_to_volts(raw, gain_milli, vref=VREF, offset_code=0.0, scale=None):
    """int16 原始差分码 -> 伏特（已扣除总增益）

    scale: 输入比例（None 表示用模块级 IN_SCALE）。
    """
    a = np.asarray(raw, dtype=np.float64)
    if offset_code:
        a = a - float(offset_code)
    k = (float(vref) / HALF_SCALE) / (float(gain_milli) / 1000.0)
    s = IN_SCALE if scale is None else float(scale)
    return a * k * s


def upsample_periodic(v, factor=8):
    """仅用于画图的 sinc 插值（FFT 零填充整数倍上采样）。

    传入的必须是整数个周期（slice_periods 的输出），此时循环前缀正好接上，
    FFT 插值等价于理想带限重建，无边界伪影。测量永远用原始采样点，
    本函数只改善曲线的观感。"""
    x = np.asarray(v, dtype=np.float64)
    f = max(1, int(factor))
    n = x.size
    if f <= 1 or n < 8:
        return x.copy()
    m = n * f
    spec = np.fft.rfft(x - x.mean())
    out = np.zeros(m // 2 + 1, dtype=complex)
    h = min(spec.size, out.size)
    out[:h] = spec[:h]
    y = np.fft.irfft(out, m) * float(f)
    return y + x.mean()


def align_channels(v, n_ch=3):
    """三路交替采集：三路 ADC 的直流偏置并不相同，直接交织会在 fs/n_ch 处
    造出一个巨大的方波假信号（幅度 = 偏置差），把 Upp/Urms 完全带偏。

    这里以第 0 路为基准，只把其余各路相对的直流失配拉平：
        v[:, i] -= mean(v[:, i]) - mean(v[:, 0])
    这样既消除通道间失配，又保留第 0 路的真实直流（Udc 仍然有意义）。
    逐通道的绝对偏置标定，等有标准源时再做。"""
    v = np.asarray(v, dtype=np.float64)
    m = (v.size // n_ch) * n_ch
    if m < n_ch * 8:
        return v
    a = v[:m].reshape(-1, n_ch)
    base = float(np.mean(a[:, 0]))
    a = a - (np.mean(a, axis=0) - base)
    out = v.copy()
    out[:m] = a.reshape(-1)
    return out


# ──────────────────────────────────────────────────────────────
# 窗函数
# ──────────────────────────────────────────────────────────────
def _window(n, kind="bh"):
    i = np.arange(n, dtype=np.float64)
    if kind == "rect":
        return np.ones(n)
    if kind == "hann":
        return 0.5 - 0.5 * np.cos(2 * np.pi * i / (n - 1))
    if kind == "flattop":
        a = (0.21557895, 0.41663158, 0.277263158, 0.083578947, 0.006947368)
        x = 2 * np.pi * i / (n - 1)
        return (a[0] - a[1] * np.cos(x) + a[2] * np.cos(2 * x)
                - a[3] * np.cos(3 * x) + a[4] * np.cos(4 * x))
    # blackman-harris 4-term（默认，谐波分析用）
    a = (0.35875, 0.48829, 0.14128, 0.01168)
    x = 2 * np.pi * i / (n - 1)
    return (a[0] - a[1] * np.cos(x) + a[2] * np.cos(2 * x)
            - a[3] * np.cos(3 * x))


# ──────────────────────────────────────────────────────────────
# 频谱
# ──────────────────────────────────────────────────────────────
def next_pow2(n):
    return 1 << (int(n - 1).bit_length())


def spectrum(v, fs, win="bh", pad=1):
    """
    返回 (freqs, amp)
      freqs : 0 .. fs/2 的频率轴（正频率轴，符合赛题"只画正频率轴"）
      amp   : 对应频率的峰值幅度（伏特）
    幅度已做窗函数相干增益补偿：正弦 A*sin 的谱峰 = A

    pad : 补零倍数。补零不提高真实分辨率（真实分辨率恒为 fs/N），
          但能让频率轴变密，避免窄主瓣（Blackman-Harris 主瓣很窄）
          被粗网格"采不到峰顶"而低估幅度。测量时应取 pad=8~32。
    """
    v = np.asarray(v, dtype=np.float64)
    n = v.size
    w = _window(n, win)
    sumw = float(np.sum(w))
    x = (v - np.mean(v)) * w
    nfft = next_pow2(n * max(1, int(pad)))
    spec = np.fft.rfft(x, nfft)
    amp = np.abs(spec) * (2.0 / sumw)
    freqs = np.fft.rfftfreq(nfft, d=1.0 / fs)
    return freqs, amp


def parabolic_peak(freqs, amp, k):
    """对第 k 个 bin 做对数抛物线插值，返回 (freq, amp)"""
    if k <= 0 or k >= len(amp) - 1:
        return float(freqs[k]), float(amp[k])
    y0, y1, y2 = (math.log(max(amp[k - 1], 1e-30)),
                  math.log(max(amp[k], 1e-30)),
                  math.log(max(amp[k + 1], 1e-30)))
    den = (y0 - 2 * y1 + y2)
    d = 0.5 * (y0 - y2) / den if den != 0 else 0.0
    d = max(-0.5, min(0.5, d))
    df = freqs[1] - freqs[0]
    f = float(freqs[k]) + d * df
    a = math.exp(y1 - 0.25 * (y0 - y2) * d)
    return f, a


def _peak_near(freqs, amp, f, half=2):
    """取 f 附近 ±half 个 bin 内的最大谱线幅度（用于判断某个谐波在不在）"""
    df = freqs[1] - freqs[0]
    j = int(round(f / df))
    j0 = max(0, j - half)
    j1 = min(len(amp) - 1, j + half)
    if j1 < j0:
        return 0.0
    return float(np.max(amp[j0:j1 + 1]))


def find_fundamental(freqs, amp, f_min=1000.0, f_max=None, max_k=10, rel=0.01):
    """
    找基频，返回 (f0, 该处幅度)。

    不能简单地取"最大峰"——半波整流、脉冲串的高次谐波都可能比基波大，
    那时最大峰会给出 2·f0 / 3·f0，f1 误差直接几十 kHz（赛题限 1 kHz）。

    这里用**谐波覆盖率**判据：
      1. 先找出所有显著谱峰（> rel·最大峰）；
      2. 对每个候选峰 f 算覆盖率 = 能被 f 的整数倍解释掉的那部分谱峰能量占比；
      3. 覆盖率最高者即基频；并列时取自身幅度更高、再取频率更低者。

    这样：
      * 半波整流（50k 最强，100k/200k… 都在）→ 50 kHz 覆盖率 100%，100 kHz 漏掉 50k → 选 50k；
      * 全波整流（只有 100k/200k/300k…，50k 根本没有谱峰，不进候选）→ 选 100 kHz；
      * 三次谐波比基波强（50/100/150k 都在）→ 50 kHz 覆盖率 100% → 选 50k；
      * 纯单音 → 只有一个候选，直接返回。
    """
    df = freqs[1] - freqs[0]
    nyq = float(freqs[-1])
    lo = max(1, int(math.ceil(f_min / df)))
    hi = len(freqs) - 2 if f_max is None else min(len(freqs) - 2, int(f_max / df))
    if hi <= lo:
        return 0.0, 0.0

    seg = amp[lo:hi + 1]
    amax = float(np.max(seg))
    if amax <= 0.0:
        return 0.0, 0.0

    # 1) 所有显著谱峰（去重：每个主瓣只取一个）
    peaks = []
    for i in range(1, len(seg) - 1):
        if seg[i] > seg[i - 1] and seg[i] >= seg[i + 1] and seg[i] > rel * amax:
            f, a = parabolic_peak(freqs, amp, lo + i)
            if peaks and f - peaks[-1][0] < 4.0 * df:
                if a > peaks[-1][1]:
                    peaks[-1] = (f, a)
            else:
                peaks.append((f, a))
    if not peaks:
        return parabolic_peak(freqs, amp, lo + int(np.argmax(seg)))

    # 只保留最强的若干个，避免噪声峰拖慢/带偏
    peaks.sort(key=lambda p: -p[1])
    peaks = peaks[:24]
    peaks.sort(key=lambda p: p[0])
    etot = sum(a for _, a in peaks)
    if etot <= 0.0:
        return 0.0, 0.0

    # 只让自身确实有谱峰的候选参与评比。
    # 门限用**绝对幅度**而不是最大峰的 5%：赛区问答明确「信号中各频率分量幅值
    # 均不小于 5 mV（峰值）」，所以 4 mV 的绝对门限既能把基波（哪怕它比谐波弱
    # 得多）保留成候选，又能把泄漏/杂散挡在外面——
    # 例：500 kHz 方波在 250 kHz 的泄漏是 2.6%（125 mV 峰值时仅 3.3 mV），
    # 用 5% 相对门限时它会被保留并反超真基频（覆盖率 0.5525 vs 0.5119）；
    # 而「基波 5 mV + 3 次谐波 120 mV」是合法信号（总 Upp 约 250 mVpp），
    # 相对门限 6.25 mV 会把基波挡掉，f1 直接报成 3 次谐波。
    floor = max(0.004, 0.02 * amax)
    strong = [p for p in peaks if p[1] >= floor]
    cand = strong if strong else peaks

    # 2) 覆盖率打分
    best = None                      # (cov, amp, f)
    for f0, a0 in cand:
        cov = a0
        for p, ap in peaks:
            if p is f0:
                continue
            k = int(round(p / f0))
            if 1 <= k <= max_k and abs(p - k * f0) <= max(0.004 * p, 1.5 * df):
                cov += ap
        cov /= etot
        key = (round(cov, 6), a0)
        if best is None or key > best[0]:
            best = (key, f0, a0)

    return float(best[1]), float(best[2])


# ──────────────────────────────────────────────────────────────
# 毛刺抑制
# ──────────────────────────────────────────────────────────────
def despike(v, k=6.0, max_frac=0.02):
    """
    剔除孤立的野点（单点尖峰），返回 (清理后的数组, 被替换的点数)。

    判据用**二阶差分** v[i-1]-2·v[i]+v[i+1]：
      * 对真实信号，二阶差分只反映波形本身的弯曲，幅度有界（正弦为 A·(ω/fs)²）；
      * 对单点尖峰，二阶差分约等于 2·尖峰高度，远超信号自身的弯曲。
    门限取信号自身 |二阶差分| 的 95 分位 × k，因此**自动随信号频率/幅度调整**，
    不会因为信号频率高（弯曲大）而误判，也不需要事先知道信号幅度。

    max_frac：野点占比超过这个比例就不动（说明不是毛刺，是波形本身有大跳变，
    例如削顶方波、或信号真的有大尖峰），宁可不动也不误伤。
    替换方式：用两侧好点线性插值，不引入直流偏移。
    """
    v = np.asarray(v, dtype=np.float64)
    n = v.size
    if n < 32:
        return v, 0
    d2 = np.empty(n, dtype=np.float64)
    d2[1:-1] = v[:-2] - 2.0 * v[1:-1] + v[2:]
    d2[0], d2[-1] = d2[1], d2[-2]
    a2 = np.abs(d2)
    ref = float(np.percentile(a2, 95.0))
    ref = max(ref, 8.0 * float(np.median(a2)))   # 纯噪声时不要过于灵敏
    if not np.isfinite(ref) or ref <= 0.0:
        return v, 0
    bad = a2 > k * ref
    nbad = int(np.count_nonzero(bad))
    if nbad == 0 or nbad > int(max_frac * n):
        return v, 0
    idx = np.arange(n)
    out = v.copy()
    out[bad] = np.interp(idx[bad], idx[~bad], v[~bad])
    return out, nbad


# ──────────────────────────────────────────────────────────────
# 测量
# ──────────────────────────────────────────────────────────────
def harmonic_fit(v, fs, f1, K=None, min_k=1, max_k=64, frac=0.45, band_hi=520e3):
    """对周期性信号做 1..K 次谐波的最小二乘拟合，用它算 Upp/Urms。

    为什么需要它：`Upp = max-min`（哪怕是 0.1/99.9 分位数）会被噪声抬高——
    500 kHz 处 EQ 要补 ×4.67，噪声同比放大，分位数 Upp 的重复性只有 0.06 mV、
    而且系统性偏高约 1 mV。拟合只用 f1 及其谐波这些"确定性"分量，
    噪声被平均掉：实测 σ 从 0.062 mV 降到 0.022 mV。

    K   : 谐波数；None 表示按 Nyquist 自动取 min(max_k, floor(frac*fs/f1))，
          避免把超过 Nyquist 的谐波放进设计矩阵（那部分会混叠，反而污染拟合）。
    返回 dict：upp/urms/a1/k/resid（resid=残差 RMS / (upp/2)）。
    """
    v = np.asarray(v, dtype=np.float64)
    n = v.size
    if n < 64 or f1 <= 0:
        return None
    if K is None:
        # 谐波上限取"信号带 520 kHz"与"Nyquist 45%"的较小者：
        # 只按 Nyquist 取会把 1.5 MHz 以上的分量也建进模型，既拟合噪声，
        # 又可能把信号带外的固定杂散（如信号源 508.4 kHz）当成谐波吸收掉。
        k_band = int(np.floor(band_hi / float(f1)))
        k_nyq = int(np.floor(frac * float(fs) / float(f1)))
        K = int(min(max_k, max(min_k, min(k_band, k_nyq))))
    K = int(max(1, K))
    t = np.arange(n) / float(fs)
    w = 2.0 * np.pi * float(f1)
    cols = [np.ones(n)]
    for k in range(1, K + 1):
        cols.append(np.cos(k * w * t))
        cols.append(np.sin(k * w * t))
    A = np.column_stack(cols)
    try:
        c, *_ = np.linalg.lstsq(A, v, rcond=None)
    except Exception:
        return None
    resid = float(np.sqrt(np.mean((v - A @ c) ** 2)))
    # 用一个周期的密集网格重建波形，再取峰峰值。
    # 重建前先把谐波"去旋转"回输入端的样子：模拟链把第 k 次谐波转了
    # Δφ_k(f1)，不补偿的话 Upp 会随输入谐波相位漂最多 ~9 mV。旋转只改相位、
    # 不改幅度，所以 ha/urms 不受影响；残差 rv 仍用原始 c 算。
    cr = c
    if PHI_ENABLE and (PHI_ROWS or PHI_OVERRIDE is not None):
        cr = np.array(c, dtype=np.float64, copy=True)
        for k in range(2, K + 1):
            th = math.radians(PHI_SIGN * dphi_deg(k, f1))
            if th == 0.0:
                continue
            ca, sa = float(cr[2 * k - 1]), float(cr[2 * k])
            ct, st = math.cos(th), math.sin(th)
            # 余弦约定下"给第 k 个分量增加相位 δ"的系数更新（ASTRA 给出）：
            #   a' = a·cosδ + b·sinδ ,  b' = b·cosδ − a·sinδ
            cr[2 * k - 1] = ca * ct + sa * st
            cr[2 * k] = sa * ct - ca * st
    T = 1.0 / float(f1)
    tt = np.linspace(0.0, T, 2048, endpoint=False)
    y = np.full(tt.size, c[0])
    for k in range(1, K + 1):
        y += cr[2 * k - 1] * np.cos(k * w * tt) + cr[2 * k] * np.sin(k * w * tt)
    upp = float(np.max(y) - np.min(y))
    urms = float(np.sqrt(c[0] ** 2 + 0.5 * np.sum(c[1:] ** 2)))
    a1 = float(np.hypot(c[1], c[2]))
    return {"upp": upp, "urms": urms, "a1": a1, "k": K,
            "ha": np.hypot(c[1::2], c[2::2]),
            "resid": resid / (0.5 * upp) if upp > 0 else 9.9,
            "rv": v - A @ c, "resid_rms": resid}


def fit_diag(v, rv, fs, f1, K, resid_rms, ha=None, band_hi=520e3,
             f_min=1000.0, spur_bins=8, spec=None):
    """把"谐波模型能不能信"拆成三个可解释的判据（Astra 建议的完整判据）。

    为什么不能只看相对残差：赛题的判据是 **±5 mV 绝对值**，而相对残差
    在 Upp=50 mV 和 250 mV 时对应的绝对误差差了 5 倍。所以必须换成 mV。

    v   : 原始信号（只用来算频谱，做"频率归属"判据）
    rv  : 拟合残差 v−模型（用来算误差预算与稀疏度）

    1) budget_mv —— 绝对误差预算。残差里落在信号带内的分量若同相叠加，最多能把
       Upp 改变 `2·sqrt(Σ a_i²)`。一根 1.3 mV 峰值、508.4 kHz 的杂散给出
       2.6 mV，正好对上实测"50 mVpp 档偏高 +2.9 mV"——量纲和量级都对。
    2) sparse —— 残差谱稀疏度：前 `spur_bins` 个谱峰占残差总能量的比例。
       稀疏 = 残差是几根孤立杂散（不属于被测信号，按赛题说明 1 应排除）；
       不稀疏 = 宽带噪声或模型漏掉的分量，这时不能信拟合。
    3) attr —— 频率归属：K 个谐波里有多少在**信号**频谱上"确实有一根线"
       （该 bin 是 ±2 bin 内的局部极大，且比附近中位数高 3 倍）。低了说明
       f1 估错、或 K 取得比信号实际谐波数还多。
    """
    rv = np.asarray(rv, dtype=np.float64)
    rv = rv - np.mean(rv)
    if spec is not None:
        f, a = spec
    else:
        f, a = spectrum(np.asarray(v, dtype=np.float64), fs, "bh", pad=4)
    df = max(f[1] - f[0], 1e-9)
    fr, ar = spectrum(rv, fs, "bh", pad=4)
    lo = max(1, int(f_min / df))
    hi = min(ar.size, int(band_hi / df) + 1)
    if hi <= lo:
        return {"budget_mv": 9e9, "sparse": 0.0, "attr": 0.0, "ok": False}
    ab = ar[lo:hi]
    # 误差预算不能用"残差总能量"：宽带噪声每个 bin 都有一点，求和会随噪声线性
    # 增长（实测 50 mVpp 时 budget 10.6 mV，而拟合本身只差 0.12 mV），把本来很准
    # 的拟合挡掉。真正能把 Upp 拉偏的是**孤立的未建模谱线**（一根峰值 a 的线最多
    # 贡献 2a），宽带噪声在最小二乘里是被平均掉的。所以只统计"高于噪声底"的
    # 局部极大值。
    base = float(np.median(ab)) + 1e-15
    nb = min(8, max(1, ab.size // 8))
    is_pk = np.zeros(ab.size, dtype=bool)
    if ab.size > 2 * nb + 1:
        for j in range(nb, ab.size - nb):
            if ab[j] > 3.0 * base and ab[j] >= float(np.max(ab[j - nb:j + nb + 1])):
                is_pk[j] = True
    spikes = ab[is_pk]
    if spikes.size > 24:                       # 太多就当宽带，取最大 24 根
        spikes = np.sort(spikes)[-24:]
    budget_mv = 2.0 * float(np.sqrt(np.sum(spikes * spikes))) * 1000.0

    e = ab * ab
    tot = float(np.sum(e))
    if tot > 0.0:
        top = np.sort(e)[-min(spur_bins, e.size):]
        sparse = float(np.sum(top) / tot)
    else:
        sparse = 1.0

    # 归属判据用**信号**频谱（残差里已经把这些谐波减掉了）。
    # 注意 BH 窗的主瓣在 pad=4 的谱上宽约 ±8 bin，用 ±6 bin 求中位数会被主瓣
    # 自己的裙边抬高（实测纯正弦 0.1245 vs 0.0943，比值只有 1.3），所以
    # 局部极大用 ±8 bin，基线中位数退出主瓣用 ±50 bin。
    # 归属**按能量加权**：只看"K 个谐波里有几个真的存在"对纯正弦恒为 1/K，
    # 没有意义；真正关心的是"拟合出来的能量有多少落在真实谱线上"。
    good = np.zeros(int(K), dtype=bool)
    for k in range(1, int(K) + 1):
        j = int(round(k * float(f1) / df))
        if j < 3 or j > a.size - 4:
            continue
        lo_b = max(0, j - 50)
        hi_b = min(a.size, j + 51)
        base = float(np.median(a[lo_b:hi_b])) + 1e-12
        is_peak = a[j] >= float(np.max(a[max(0, j - 8):j + 9]))
        if is_peak and a[j] > 3.0 * base:
            good[k - 1] = True
    if ha is not None and len(ha) == int(K):
        w = np.asarray(ha, dtype=np.float64) ** 2
        tot_w = float(np.sum(w))
        attr = float(np.sum(w[good]) / tot_w) if tot_w > 0 else 0.0
    else:
        attr = float(np.sum(good)) / float(max(1, int(K)))

    return {"budget_mv": budget_mv, "sparse": sparse, "attr": attr,
            "resid_rms_mv": float(resid_rms) * 1000.0, "ok": True}


def waveform_kind(v, fs, f1, spec=None, band_hi=520e3, pad=4):
    """
    判断是不是「标准波形」，并给出从基波幅度 a1 换算峰峰值的系数。

    为什么需要它：带限重建后取 max-min 对方波会固定高估约 18%（Gibbs 过冲，
    加到 49 次谐波也还是 +17.9%），三角波则低估（K=9 时 -4%，K=1 时 -18.9%）。
    改用「基波幅度 × 解析系数」就没有这个问题：

        正弦   a1 = Upp/2        →  Upp = 2·a1
        方波   a1 = 2·Upp/π      →  Upp = (π/2)·a1      = 1.5708·a1
        三角波 a1 = 4·Upp/π²     →  Upp = (π²/4)·a1     = 2.4674·a1
        半波   a1 = Upp/2        →  Upp = 2·a1（与正弦同式）

    判据只用 3/5/7 次谐波与基波之比（方波 ≈ 1/k，三角波 ≈ 1/k²），
    多音信号（例如 u2=100k+300k）的 5/7 次对不上，不会被误判。
    返回 (kind, factor)；kind 为 'square' / 'triangle' / 'mixed'，mixed 表示不做改动。
    """
    freqs, amp = spec if spec is not None else spectrum(v, fs, "bh", pad=pad)
    df = float(freqs[1] - freqs[0])

    def harm(k):
        f = k * float(f1)
        if f > band_hi:
            return 0.0
        i = int(round(f / df))
        lo, hi = max(1, i - 2), min(amp.size - 1, i + 2)
        if hi <= lo:
            return 0.0
        return float(np.max(amp[lo:hi + 1]))

    a1 = harm(1)
    if a1 <= 0:
        return "mixed", None
    r2, r3, r5, r7 = harm(2) / a1, harm(3) / a1, harm(5) / a1, harm(7) / a1
    have3 = 3.0 * float(f1) <= band_hi
    have5 = 5.0 * float(f1) <= band_hi
    have7 = 7.0 * float(f1) <= band_hi
    # 三次谐波已经在通带之外：模拟前端把方波/三角波的谐波全部滤掉了，
    # 到 ADC 就是一个正弦，物理上无法分辨输入原本是不是方波。
    # 此时不套用任何波形换算（factor=None → 保留分位数 Upp，已按正弦标定过），
    # 只置 unreliable 标志让界面提示"判型不可信"。
    if not have3:
        return "unreliable", None
    # 半波整流：偶次为主（a2/a1 ≈ 0.42），奇次几乎没有 → 与正弦同式 Upp=2·a1
    if 0.30 <= r2 <= 0.55 and r3 < 0.10 and (not have5 or r5 < 0.08):
        return "half", 2.0
    if abs(r3 - 1.0 / 3.0) < 0.09:
        ok = True
        if have5:
            ok = ok and abs(r5 - 1.0 / 5.0) < 0.07
        if have7:
            ok = ok and abs(r7 - 1.0 / 7.0) < 0.06
        if ok:
            return "square", float(np.pi / 2.0)
    if abs(r3 - 1.0 / 9.0) < 0.045 and (not have5 or abs(r5 - 1.0 / 25.0) < 0.035):
        return "triangle", float(np.pi * np.pi / 4.0)
    # 正弦：**通带内所有**谐波都很小，才按 Upp = 2·a1。
    # 只看 2/3/5/7 次是不够的：赛题说明 3 的示例就是 10.5k + 31.5k(3次) + 42.0k(4次)，
    # 只带 4 次谐波的信号会被当成正弦，Upp 少算 2·a4。
    kmax = int(min(64, np.floor(band_hi / float(f1))))
    rmax = 0.0
    for k in range(2, kmax + 1):
        rmax = max(rmax, harm(k) / a1)
    if rmax < 0.05:
        return "sine", 2.0
    return "mixed", None


# ──────────────────────────────────────────────────────────────
# 谐波模型的可信度门限（Astra 建议的"绝对误差预算"判据）
# ──────────────────────────────────────────────────────────────
# 赛题判据是 ±5 mV 绝对值，所以这里也必须用 mV，不能用相对残差。
FIT_BUDGET_MV = 5.0
# 残差谱里前 8 个峰至少要占这么多能量，才算"稀疏的孤立杂散"而不是宽带失配。
# **只做诊断，不参与放行**：小信号时残差以宽带噪声为主，sparse 天然很低。
FIT_SPARSE = 0.60
# 频率归属率下限：拟合出来的谐波能量里，至少这么多要落在真实谱线上。
FIT_ATTR = 0.50


def measure(v, fs, f_min=1000.0, margin=0, robust=True, p_lo=0.1, p_hi=99.9, pad=4,
            method="auto", fit_k=None, fit_resid_max=0.03, fit_resid_loose=0.12,
            fit_agree=0.20, wave=True, force_wave=None,
            fit_budget_mv=FIT_BUDGET_MV, fit_sparse=FIT_SPARSE,
            fit_attr=FIT_ATTR):
    """
    force_wave : None/"auto" 自动判型；"sine"/"square"/"triangle"/"half" 强制按该波形换算。
        为什么需要它：4 阶模拟板 fc≈230 kHz，方波/三角波的 3 次谐波（≥600 kHz）
        被压到 −16 dB 以下，到 ADC 几乎就是正弦，自动判型物理上不可能。
        但基波幅度 a1 不受影响，只要告诉系统波形类型就能用解析式反推 Upp：
            正弦 Upp=2·a1    方波 Upp=(π/2)·a1
            三角 Upp=(π²/4)·a1  半波 Upp=2·a1
    """
    v = np.asarray(v, dtype=np.float64)
    # 现场调参开关（不设 = 默认行为一字不变）：放宽"残差太大就退回分位数"的门限。
    # 为什么需要：DG4202 的输出在 ~600 kHz 有一根固定约 6 mV 的残留（不随幅度变，
    # 会在前端混叠进来），数字低通去不掉；它让小幅度波形（50~150 mVpp）的拟合残差
    # 刚好越过 0.03，measure() 退回分位数法，Upp 被抬高约 10 mV，而同一组数据的
    # 拟合 Upp 与设计值只差 1.5 mV 以内。例：LSA_FIT_RESID_MAX=0.15
    _env_resid = os.environ.get("LSA_FIT_RESID_MAX") or ""
    if _env_resid:
        try:
            fit_resid_max = float(_env_resid)
        except ValueError:
            pass
    if margin and v.size > 4 * margin:
        v = v[margin:v.size - margin]
    glitch_n = 0
    upp_raw = float(np.max(v) - np.min(v))        # 去毛刺前的原始 max-min，留作对照
    if robust:
        v, glitch_n = despike(v)
    n = v.size
    udc = float(np.mean(v))
    ac = v - udc
    if robust and n >= 100:
        upp = float(np.percentile(v, p_hi) - np.percentile(v, p_lo))
    else:
        upp = upp_raw
    urms = float(np.sqrt(np.mean(ac * ac)))
    freqs, amp = spectrum(v, fs, "bh", pad=pad)
    f1, a1 = find_fundamental(freqs, amp, f_min)

    # 谐波拟合：周期性信号用它算 Upp/Urms 比 max-min / 分位数准得多
    # （噪声被平均掉，实测 500 kHz 处 σ 从 0.062 mV 降到 0.022 mV）。
    # 拟合残差太大（波形不是干净周期信号）就退回分位数，避免模型误差。
    fit = None
    method_used = "pct"
    _last_diag = None
    if method in ("auto", "fit"):
        try:
            fit = harmonic_fit(v, fs, f1, K=fit_k)
        except Exception:
            fit = None
        if fit is not None:
            ok = (method == "fit") or (fit["resid"] <= fit_resid_max)
            diag = None
            if not ok and fit["resid"] <= fit_resid_loose and upp > 0:
                # 残差偏大时不再只看"相对比例"，改用 Astra 建议的完整判据：
                # 绝对误差预算(≤FIT_BUDGET_MV) + 残差谱稀疏度 + 与分位数法吻合。
                # 只用相对门限会在小信号上放行大误差（50 mVpp 时 12% = 6 mV）。
                try:
                    diag = fit_diag(v, fit["rv"], fs, f1, fit["k"],
                                    fit["resid_rms"], ha=fit.get("ha"),
                                    spec=(freqs, amp))
                except Exception as exc:
                    diag = {"budget_mv": 9e9, "sparse": 0.0, "attr": 0.0,
                            "ok": False, "err": repr(exc)[:60]}
                _last_diag = diag
                if diag is not None and diag["ok"]:
                    # 三条判据：绝对误差预算 + 与分位数法吻合 + 频率归属。
                    # **不再用"残差谱稀疏度"做门**：小信号（50~100 mVpp）时残差
                    # 主要是宽带噪声，sparse 天然很低，会把本来很准的拟合挡掉，
                    # 退回分位数后反而更差（实测 100 mVpp 由 +0.4 变 +5.8 mV）。
                    # 宽带噪声对拟合是无偏的，真正的风险是"漏掉一根谱线"，
                    # 那已由 budget（绝对 mV）与 attr（谱线归属）覆盖。
                    ok = (diag["budget_mv"] <= fit_budget_mv
                          and abs(float(fit["upp"]) - upp) <= fit_agree * upp
                          and diag["attr"] >= fit_attr)
                else:
                    # 诊断本身失败时退回旧规则，宁严不松
                    ok = abs(float(fit["upp"]) - upp) <= fit_agree * upp
            if ok:
                upp = float(fit["upp"])
                urms = float(fit["urms"])
                a1 = float(fit["a1"])
                method_used = "fit"

    # 标准波形（方波/三角波）用基波解析换算，避开 Gibbs 过冲
    wave_kind, wave_factor = ("mixed", None)
    upp_wave = upp
    forced = (force_wave or "").strip().lower()
    if forced in ("sine", "square", "triangle", "half"):
        # 手动指定波形：不判型，直接按解析式换算（高频非正弦必须用这条路）
        wave_kind = forced
        try:
            wave_kind, wave_factor = forced, _WAVE_FACTOR[forced]
            upp_wave = float(wave_factor * a1)
            upp = upp_wave
            urms = float(_WAVE_URMS[forced] * upp)
        except Exception:
            wave_kind, wave_factor, upp_wave = "mixed", None, upp
    elif wave:
        try:
            wave_kind, wave_factor = waveform_kind(v, fs, f1, spec=(freqs, amp))
            if wave_factor is not None:
                upp_wave = float(wave_factor * a1)
                upp = upp_wave
                if wave_kind == "square":
                    urms = upp / 2.0
                elif wave_kind == "triangle":
                    urms = upp / (2.0 * np.sqrt(3.0))
                elif wave_kind == "half":
                    # 半波（峰值 A，Upp=A）的交流有效值 = A·sqrt(1/4 − 1/π²) = 0.38549·Upp
                    urms = 0.3854942 * upp
        except Exception:
            wave_kind, wave_factor, upp_wave = "mixed", None, upp

    # 分位数法对偶发坏点很敏感：实测 250 mVpp 纯正弦会偶发 +35~40 mV 的假误差
    # （同一帧的 Urms 却完全正确，说明是十几个样本被打坏、despike 没清干净）。
    # 正弦有恒等式 Upp = 2√2·Urms，用它做一次一致性校核，偏差过大就改用 RMS 折算。
    upp_src = method_used
    # 只有"真的只有一根谱线"时才允许用 Upp=2√2·Urms 兜底：
    # k<=2 表示存在 2 次谐波（赛题的基波+谐波正是这种），那样 2√2·Urms 会低估
    # Upp —— 实测 234.4k+2 次(0.7) 被这个兜底算出 Upp 206 mV（真值 250）。
    _sine_like = (wave_kind == "sine"
                  or (fit is not None and int(fit.get("k") or 0) <= 1))
    if wave_factor is None and urms > 0 and _sine_like:
        upp_rms = 2.0 * math.sqrt(2.0) * urms
        if abs(upp - upp_rms) > 0.04 * upp_rms:
            upp = upp_rms
            upp_wave = upp_rms
            upp_src = "rms"

    # 3 次谐波已出带 → 判型不可信，按正弦报并置标志
    wave_unreliable = (wave_kind == "unreliable")
    if wave_unreliable:
        wave_kind = "sine?"

    return {
        "n": n, "fs": float(fs), "df": float(fs) / n,
        "upp": upp, "upp_raw": upp_raw, "glitch_n": glitch_n,
        "upp_wave": upp_wave, "wave_kind": wave_kind,
        "upp_src": upp_src,
        "wave_unreliable": wave_unreliable,
        "wave_forced": (forced if forced in ("sine", "square", "triangle", "half") else ""),
        "urms": urms, "udc": udc,
        "f1": f1, "a1": a1,
        "method": method_used,
        "fit_k": (fit["k"] if fit else 0),
        "fit_resid": (fit["resid"] if fit else None),
        "fit_diag": _last_diag,          # 诊断快照：budget/sparse/attr，排障用
        "fit_upp": (float(fit["upp"]) if fit else None),
        "crest": (upp / 2.0) / urms if urms > 0 else 0.0,
        # 复用同一份频谱：传给 components()/harmonic_amplitudes() 的 spec= 参数，
        # 避免一次刷新里做 3~4 次同样的 FFT（树莓派上一次 24576 点要 290 ms）
        "spec": (freqs, amp),
    }


def components_direct(v_codes, fs, f_min=1000.0, f_max=600e3, top=6, rel=0.02, pad=4,
                      gain_total=3.925, eq=None, in_scale=None, spec=None):
    """
    直接由**原始码**算分量幅值（不走「补零→反补偿→裁边」那条路）。

        A_in(f) = a_code(f) · G(f)

    好处：不需要先做频域反补偿，因此没有反补偿放大噪声、也没有两端裁边，
    高频仍按实测表折算（表外按最后一点保持，不再回落）。
    对题3 的"测 u2 各分量幅值"这是最干净的一条路。

    返回 [(freq_hz, amp_V), ...]（按幅度降序），幅度是**输入端峰值**。
    """
    import lsa_eq

    v = np.asarray(v_codes, dtype=np.float64)
    freqs, amp = spec if spec is not None else spectrum(v, fs, "bh", pad=pad)
    df = float(freqs[1] - freqs[0])
    lo = max(1, int(f_min / df))
    hi = min(amp.size - 1, int(f_max / df))
    if hi <= lo:
        return []
    seg = amp[lo:hi + 1]
    thr = rel * float(np.max(seg)) if seg.size else 0.0
    out = []
    for i in range(1, seg.size - 1):
        if seg[i] > thr and seg[i] >= seg[i - 1] and seg[i] >= seg[i + 1]:
            f, a = parabolic_peak(freqs, amp, lo + i)
            out.append((float(f), float(a) * float(lsa_eq.chain_gain(
                f, eq, in_scale, gain_total))))
    out.sort(key=lambda t: -t[1])
    return out[:top]


def components(v, fs, f_min=1000.0, f_max=600e3, top=6, rel=0.02, pad=4, spec=None):
    """
    在 f_min..f_max 内找若干谱峰，返回 [(freq, amp), ...]（幅度降序）
    rel : 相对最大峰的幅度门限
    spec: 已算好的 (freqs, amp)，传进来可省一次 FFT
    """
    freqs, amp = spec if spec is not None else spectrum(v, fs, "bh", pad=pad)
    df = freqs[1] - freqs[0]
    lo = max(1, int(math.ceil(f_min / df)))
    hi = min(len(freqs) - 2, int(f_max / df))
    if hi <= lo:
        return []
    seg = amp[lo:hi + 1]
    order = np.argsort(seg)[::-1]
    out = []
    used = []
    amax = float(seg[order[0]])
    for idx in order:
        k = lo + int(idx)
        f, a = parabolic_peak(freqs, amp, k)
        if a < rel * amax or len(out) >= top:
            break
        # 峰间隔至少 3 个真实分辨率单元，避免同一个峰的旁瓣被重复计入
        if any(abs(f - u) < 3.0 * fs / len(v) for u in used):
            continue
        used.append(f)
        out.append((f, a))
    return out


def harmonic_amplitudes(v, fs, f1, count=5, tol=0.03, pad=4, spec=None):
    """返回基频各次谐波的幅度 [A1, A2, ...]（在 ±tol 相对范围内取峰）"""
    freqs, amp = spec if spec is not None else spectrum(v, fs, "bh", pad=pad)
    res = []
    for h in range(1, count + 1):
        fc = f1 * h
        if fc >= fs / 2:
            break
        df = freqs[1] - freqs[0]
        lo = max(1, int((fc * (1 - tol)) / df))
        hi = min(len(freqs) - 2, int((fc * (1 + tol)) / df))
        if hi <= lo:
            res.append(0.0)
            continue
        k = lo + int(np.argmax(amp[lo:hi + 1]))
        res.append(parabolic_peak(freqs, amp, k)[1])
    return res


def out_of_band_ratio(v, fs, band_hi=600e3):
    """带外（band_hi..fs/2）能量占比，用于自动选档判断 f3 是否混叠进带内"""
    freqs, amp = spectrum(v, fs, "hann")
    df = freqs[1] - freqs[0]
    lo = int(band_hi / df)
    p = amp * amp
    tot = float(np.sum(p[1:]))
    if tot <= 0:
        return 0.0
    ob = float(np.sum(p[lo:]))
    return ob / tot


def out_of_band_peak(v, fs, band_hi=LP_FC_HZ, pad=4, min_amp=1e-3):
    """
    找 band_hi 以上最强的单根谱线，返回 (freq_hz, amp_v)。
    没有任何高于 min_amp 的分量时返回 (0.0, 0.0)。

    用途：显示"被低通滤掉的到底是什么"，直接区分
      2.361 MHz → 三路交织失配（fs/3 假信号，靠校准消除）
      3.667 MHz → 32 MHz 自激在 7.083 MSPS 下的折叠
      生成器/前端杂散     → 另一根未知频率
    """
    freqs, amp = spectrum(v, fs, "bh", pad=pad)
    df = freqs[1] - freqs[0]
    lo = max(1, int(band_hi / df))
    if lo >= amp.size:
        return 0.0, 0.0
    seg = amp[lo:]
    i = int(np.argmax(seg))
    if seg[i] < min_amp:
        return 0.0, 0.0
    f0, a0 = parabolic_peak(freqs[lo:], seg, i)
    return float(f0), float(a0)


# ──────────────────────────────────────────────────────────────
# 数字滤波（去 f3 用）
# ──────────────────────────────────────────────────────────────
def lowpass_fir(v, fs, fc, taps=FIR_TAPS, beta=8.6):
    """
    Kaiser 窗 sinc 低通，返回与输入等长的滤波结果（已补偿群延迟）

    两端做镜像延拓再卷积，避免零填充在记录首尾产生虚假瞬态
    （否则残余干扰会把 Upp 抬高几个 mV，直接影响 5 mV 的判据）
    """
    v = np.asarray(v, dtype=np.float64)
    n = v.size
    taps = int(taps) | 1
    m = taps - 1
    k = np.arange(taps) - m / 2.0
    h = 2.0 * fc / fs * np.sinc(2.0 * fc / fs * k) * np.kaiser(taps, beta)
    h /= np.sum(h)
    if n <= taps:
        return np.convolve(v, h, mode="same")
    pad = m // 2
    left = v[:pad][::-1]
    right = v[-pad:][::-1]
    vp = np.concatenate((left, v, right))
    y = np.convolve(vp, h, mode="same")
    return y[pad:pad + n]


# ──────────────────────────────────────────────────────────────
# 波形裁剪（题1：1 个 / 3 个完整周期）
# ──────────────────────────────────────────────────────────────
def rising_crossing(v, fs, f1, start_frac=0.05):
    """在记录前段找基频的一个上升过零点，返回亚采样插值后的下标（float）"""
    v = np.asarray(v, dtype=np.float64)
    n = v.size
    i0 = max(1, int(n * start_frac))
    dc = float(np.mean(v))
    x = v - dc
    # 只在基频周期长度内搜索，避免抓到谐波过零
    span = int(max(2.0 * fs / max(f1, 1.0), 8))
    for i in range(i0, min(n - 1, i0 + span)):
        if x[i] <= 0.0 < x[i + 1]:
            dx = x[i + 1] - x[i]
            frac = -x[i] / dx if dx != 0 else 0.0
            return i + frac
    return float(i0)


def _interp_cubic(xi, y):
    """Catmull-Rom 三次插值，用于把波形重采样到"整数个周期"。

    为什么不用 np.interp：线性插值的误差是弦高 ≈ (ωT_s)²/8。500 kHz 成分在
    3.542 MSPS 下只有 7.1 点/周期，弦高误差约 8%——"3 个完整周期"的首尾就对
    不齐（题2 实测 5.99 mV）。三次插值把误差降到 (ωT_s)⁴ 量级（<0.5%）。
    """
    xi = np.asarray(xi, dtype=np.float64)
    i = np.floor(xi).astype(np.int64)
    i = np.clip(i, 1, y.size - 3)
    t = xi - i
    y0, y1 = y[i - 1], y[i]
    y2, y3 = y[i + 1], y[i + 2]
    a = -0.5 * y0 + 1.5 * y1 - 1.5 * y2 + 0.5 * y3
    b = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3
    c = -0.5 * y0 + 0.5 * y2
    return ((a * t + b) * t + c) * t + y1


def slice_periods(v, fs, f1, count=3, start_frac=0.05):
    """
    截出 count 个完整周期，返回 (t, y)
    t 从 0 开始，单位秒
    """
    v = np.asarray(v, dtype=np.float64)
    n = v.size
    if f1 <= 0:
        return np.arange(n) / fs, v
    i0 = rising_crossing(v, fs, f1, start_frac)
    # 采样点只能落在整数位置：直接取 round(count*fs/f1) 个点，在高 f1 时会差出
    # 半个样本（400 kHz @3.542 MSPS：1 周期 8.855 点，3 周期 26.57 点 -> 取 27
    # 点其实是 3.049 个周期），首尾对不齐，屏幕上能看出接缝。这里在原始序列上
    # 线性重采样，让窗口精确覆盖 count 个周期。
    want_f = count * fs / f1
    N = int(round(want_f))
    if N < 2:
        return np.arange(n) / fs, v
    step = want_f / N
    if i0 + step * N >= n - 1:
        N = int((n - 1 - i0) / step)
        if N < 2:
            return np.arange(n) / fs, v
        step = (n - 1 - i0) / N
    xs = i0 + step * np.arange(N)
    seg = _interp_cubic(xs, v)
    t = np.arange(N) / fs
    return t, seg
