# -*- coding: utf-8 -*-
"""
lsa_app.py —— 周期信号谐波分析装置 上位机（图形界面）

树莓派 4：直接 python3 lsa_app.py，窗口默认最大化铺满 6 英寸以上屏幕
Windows ：python lsa_app.py

界面布局
  ┌───────────────────────────────────────────────┐
  │ 左：波形（1 个 / 3 个完整周期）  右：正频率轴频谱 │
  ├───────────────────────────────────────────────┤
  │ 读数条：Upp / Urms / f1 / 各分量                  │
  ├───────────────────────────────────────────────┤
  │ 按钮：采集 / 档位 / 周期 / 保存                   │
  └───────────────────────────────────────────────┘
"""
import math
import os
import sys
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np

import lsa_calib as calib_mod
import lsa_dsp as dsp
import lsa_proto as proto


def _fix_offsets(v, h, inter3):
    """按 calib.json 扣掉每路直流偏置；没标定时退回通道均值对齐。"""
    try:
        cal = calib_mod.load()
        gain = h.get("gain_milli") or 3925
        if any(calib_mod.offset_of(cal, c, gain) for c in range(3)):
            return calib_mod.apply(v, cal, "inter3" if inter3 else "single", 0, gain)
    except Exception:
        pass
    return dsp.align_channels(v, 3) if inter3 else v

try:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    HAVE_MPL = True
except Exception:  # pragma: no cover
    HAVE_MPL = False


BG = "#101418"
FG = "#e6edf3"
GRID = "#2a333d"
C_WAVE = "#4fd1c5"
C_SPEC = "#f6ad55"
C_MARK = "#fc8181"


def setup_font():
    """挑一个可用的中文字体，避免图上的中文变成方块"""
    if not HAVE_MPL:
        return
    from matplotlib import font_manager, rcParams
    have = {f.name for f in font_manager.fontManager.ttflist}
    for cand in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                 "WenQuanYi Zen Hei", "WenQuanYi Micro Hei",
                 "Source Han Sans SC", "Droid Sans Fallback", "SimSun"):
        if cand in have:
            rcParams["font.sans-serif"] = [cand] + list(rcParams["font.sans-serif"])
            rcParams["font.family"] = "sans-serif"
            break
    rcParams["axes.unicode_minus"] = False


setup_font()


class Scope(tk.Tk):
    def __init__(self):
        tk.Tk.__init__(self)
        self.title("周期信号谐波分析装置  ·  LSA Host")
        self.configure(bg=BG)
        try:
            self.state("zoomed")
        except tk.TclError:
            self.geometry("1280x800")

        self.client = None
        self.hdr = {}
        self.v = None            # 伏特数组
        self.fs = 0.0
        self.meas = {}
        self.n_periods = 3
        self.gear = 0
        self.inter3 = True
        self.ch = 0
        self.auto_gear = tk.BooleanVar(value=True)
        self.margin = 0
        self.status = tk.StringVar(value="就绪")

        self._build_ui()
        self._bind_keys()

    # ──────────────────────────────────────────────
    # 界面
    # ──────────────────────────────────────────────
    def _build_ui(self):
        top = tk.Frame(self, bg=BG)
        top.pack(side=tk.TOP, fill=tk.X, padx=8, pady=6)

        tk.Label(top, text="目标 IP", bg=BG, fg=FG).pack(side=tk.LEFT)
        self.e_host = tk.Entry(top, width=16, bg="#1b2229", fg=FG,
                               insertbackground=FG, relief=tk.FLAT)
        self.e_host.insert(0, "192.168.57.200")
        self.e_host.pack(side=tk.LEFT, padx=(4, 10))

        self.btn_conn = tk.Button(top, text="连接", width=8, command=self.on_connect,
                                  bg="#243040", fg=FG, relief=tk.FLAT, activebackground="#33465e")
        self.btn_conn.pack(side=tk.LEFT, padx=2)

        tk.Button(top, text="采集 (空格)", width=14, command=self.on_capture,
                  bg="#1f6f5c", fg="#ffffff", relief=tk.FLAT,
                  activebackground="#2a8f78").pack(side=tk.LEFT, padx=(14, 4))

        self.cb_period = ttk.Combobox(top, width=10, state="readonly",
                                      values=["1 个周期", "3 个周期"])
        self.cb_period.current(1)
        self.cb_period.pack(side=tk.LEFT, padx=(14, 4))
        self.cb_period.bind("<<ComboboxSelected>>", lambda e: self._redraw_wave())

        self.cb_gear = ttk.Combobox(top, width=22, state="readonly",
                                    values=["自动选档", "档0 7.08 MSPS", "档1 5.31 MSPS",
                                            "档2 4.25 MSPS", "档3 3.54 MSPS"])
        self.cb_gear.current(0)
        self.cb_gear.pack(side=tk.LEFT, padx=(10, 4))

        tk.Checkbutton(top, text="三路交替", variable=self.inter3, bg=BG, fg=FG,
                       selectcolor="#1b2229", activebackground=BG,
                       activeforeground=FG).pack(side=tk.LEFT, padx=8)

        self.rm_f3 = tk.BooleanVar(value=False)
        tk.Checkbutton(top, text="去 u3(>700k)", variable=self.rm_f3, bg=BG, fg=FG,
                       selectcolor="#1b2229", activebackground=BG,
                       activeforeground=FG).pack(side=tk.LEFT, padx=6)

        tk.Button(top, text="保存", width=6, command=self.on_save,
                  bg="#243040", fg=FG, relief=tk.FLAT).pack(side=tk.RIGHT, padx=2)
        tk.Button(top, text="自检", width=6, command=self.on_selftest,
                  bg="#243040", fg=FG, relief=tk.FLAT).pack(side=tk.RIGHT, padx=2)

        # 图
        if HAVE_MPL:
            self.fig = Figure(figsize=(12, 5.2), dpi=100, facecolor=BG)
            gs = self.fig.add_gridspec(1, 2, width_ratios=[1.45, 1.0],
                                       left=0.055, right=0.985, top=0.93,
                                       bottom=0.115, wspace=0.16)
            self.ax_w = self.fig.add_subplot(gs[0, 0])
            self.ax_s = self.fig.add_subplot(gs[0, 1])
            for ax in (self.ax_w, self.ax_s):
                ax.set_facecolor("#0b0f13")
                ax.tick_params(colors=FG, labelsize=9)
                ax.grid(True, color=GRID, lw=0.6)
                for s in ax.spines.values():
                    s.set_color(GRID)
            self.ax_w.set_xlabel("时间 (µs)", color=FG, fontsize=9)
            self.ax_w.set_ylabel("电压 (mV)", color=FG, fontsize=9)
            self.ax_s.set_xlabel("频率 (kHz)", color=FG, fontsize=9)
            self.ax_s.set_ylabel("幅度 (mV)", color=FG, fontsize=9)
            self.canvas = FigureCanvasTkAgg(self.fig, master=self)
            self.canvas.get_tk_widget().configure(bg=BG, highlightthickness=0)
            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=6)
        else:
            tk.Label(self, text="未安装 matplotlib：pip3 install matplotlib",
                     bg=BG, fg=C_MARK).pack(expand=True)

        # 读数条
        bar = tk.Frame(self, bg="#161c22")
        bar.pack(side=tk.TOP, fill=tk.X)
        self.lbl = {}
        for i, (key, name) in enumerate([("upp", "Upp"), ("urms", "Urms"),
                                         ("f1", "f1"), ("a1", "基波"),
                                         ("thd", "THD"), ("df", "Δf")]):
            f = tk.Frame(bar, bg="#161c22")
            f.pack(side=tk.LEFT, padx=16, pady=6)
            tk.Label(f, text=name, bg="#161c22", fg="#8b98a5",
                     font=("DejaVu Sans", 9)).pack(anchor="w")
            v = tk.Label(f, text="--", bg="#161c22", fg=FG,
                         font=("DejaVu Sans", 16, "bold"))
            v.pack(anchor="w")
            self.lbl[key] = v

        self.lbl_comp = tk.Label(self, text="", bg=BG, fg=C_SPEC, anchor="w",
                                 justify="left", font=("DejaVu Sans", 9))
        self.lbl_comp.pack(side=tk.TOP, fill=tk.X, padx=12, pady=(4, 0))

        st = tk.Label(self, textvariable=self.status, bg="#161c22", fg="#8b98a5",
                      anchor="w", font=("DejaVu Sans", 9))
        st.pack(side=tk.BOTTOM, fill=tk.X)

    def _bind_keys(self):
        self.bind("<space>", lambda e: self.on_capture())
        self.bind("<Key-1>", lambda e: self._set_periods(1))
        self.bind("<Key-3>", lambda e: self._set_periods(3))
        self.bind("<Key-s>", lambda e: self.on_save())
        self.bind("<Escape>", lambda e: self.destroy())

    def _set_periods(self, n):
        self.n_periods = n
        self.cb_period.current(0 if n == 1 else 1)
        self._redraw_wave()

    # ──────────────────────────────────────────────
    # 网络
    # ──────────────────────────────────────────────
    def on_connect(self):
        try:
            if self.client:
                self.client.close()
            self.client = proto.LsaClient(self.e_host.get().strip(), 5000, 3.0)
            self.client.connect()
            h = self.client.ping()
            self.status.set("已连接 %s:%d  (ver=%d)" % (self.e_host.get().strip(), 5000,
                                                        h.get("ver", 0)))
        except Exception as e:
            self.client = None
            self.status.set("连接失败: %s" % e)

    def _ensure(self):
        if self.client is None:
            self.on_connect()
        return self.client is not None

    def on_selftest(self):
        if not self._ensure():
            return
        try:
            h = self.client.selftest()
            ok = bool(h.get("flags", 0) & proto.LSDA_FLAG_INTER3_OK)
            self.status.set("自检：三路交替 %s，flags=0x%04X" % ("正常" if ok else "失败",
                                                                h.get("flags", 0)))
        except Exception as e:
            self.status.set("自检失败: %s" % e)

    def on_capture(self):
        if not self._ensure():
            return
        t0 = time.time()
        try:
            sel = self.cb_gear.current()
            gears = range(4) if sel == 0 else [sel - 1]
            best = None
            for g in gears:
                h, raw = self.client.capture(gear=g, inter3=self.inter3,
                                             ch=self.ch, n=None, timeout=6.0)
                a = np.asarray(proto.raw_to_int16(raw), dtype=np.float64)
                fs = float(h["fs_hz"]) or (proto.GEAR_RATE_INTER3[g] if self.inter3
                                           else proto.GEAR_RATE_SINGLE[g])
                v = dsp.codes_to_volts(a, h["gain_milli"] or 3925)
                v = _fix_offsets(v, h, self.inter3)
                ratio = dsp.out_of_band_ratio(v, fs) if self.inter3 else 0.0
                if best is None or ratio < best[0]:
                    best = (ratio, g, h, v, fs)
            ratio, g, h, v, fs = best
            self.margin = 0
            if self.rm_f3.get():
                v = dsp.lowpass_fir(v, fs, dsp.LP_FC_HZ)
                self.margin = dsp.FIR_MARGIN
            self.hdr, self.v, self.fs, self.gear = h, v, fs, g
            self.meas = dsp.measure(v, fs, margin=self.margin)
            self._refresh()
            self.status.set("采集完成 档%d  fs=%.3f MSPS  N=%d  用时 %.0f ms  带外 %.1f%%"
                            % (g, fs / 1e6, len(v), (time.time() - t0) * 1e3, ratio * 100))
        except Exception as e:
            self.status.set("采集失败: %s" % e)

    # ──────────────────────────────────────────────
    # 显示
    # ──────────────────────────────────────────────
    def _refresh(self):
        m = self.meas
        if not m:
            return
        self.lbl["upp"].config(text="%.1f mV" % (m["upp"] * 1e3))
        self.lbl["urms"].config(text="%.1f mV" % (m["urms"] * 1e3))
        f1 = m["f1"]
        self.lbl["f1"].config(text=("%.2f kHz" % (f1 / 1e3)) if f1 >= 1e4
                              else ("%.0f Hz" % f1))
        self.lbl["a1"].config(text="%.1f mV" % (m["a1"] * 1e3))
        h = dsp.harmonic_amplitudes(self.v, self.fs, f1)
        if h and h[0] > 0:
            thd = np.sqrt(sum(x * x for x in h[1:])) / h[0]
            self.lbl["thd"].config(text="%.1f %%" % (thd * 100))
        else:
            self.lbl["thd"].config(text="--")
        self.lbl["df"].config(text="%.0f Hz" % m["df"])

        comp = dsp.components(self.v, self.fs, f_min=max(500.0, f1 * 0.5))
        txt = "  ".join("%.1f kHz / %.1f mV" % (f / 1e3, a * 1e3) for f, a in comp[:6])
        self.lbl_comp.config(text="分量:  " + (txt if txt else "--"))

        self._redraw_wave()
        self._redraw_spec()

    @staticmethod
    def _nice_span(peak):
        """把半量程吸附到 1/1.5/2/2.5/3/5/7.5×10^k，量程稳定不跳。"""
        if peak <= 0:
            return 1.0
        e = math.floor(math.log10(peak))
        base = 10.0 ** e
        for m in (1.0, 1.5, 2.0, 2.5, 3.0, 5.0, 7.5, 10.0):
            if peak <= m * base:
                return m * base
        return 10.0 * base

    def _redraw_wave(self):
        if not HAVE_MPL or self.v is None:
            return
        ax = self.ax_w
        ax.clear()
        ax.set_facecolor("#0b0f13")
        ax.grid(True, color=GRID, lw=0.6)
        f1 = self.meas.get("f1", 0.0)
        if f1 <= 0:
            return
        n = 1 if self.cb_period.current() == 0 else 3
        t, y = dsp.slice_periods(self.v, self.fs, f1, n)
        udc = float(np.mean(y)) * 1e3
        yac = (y - float(np.mean(y))) * 1e3          # 自动量程只看交流
        peak = float(np.max(np.abs(yac))) if yac.size else 0.0
        lim = 1.0 if peak < 0.05 else self._nice_span(peak * 1.12)
        ax.plot(t * 1e6, yac, color=C_WAVE, lw=1.2)
        ax.set_ylim(-lim, lim)
        ax.set_xlabel("时间 (µs)", color=FG, fontsize=9)
        ax.set_ylabel("电压 (mV)", color=FG, fontsize=9)
        ax.tick_params(colors=FG, labelsize=9)
        for s in ax.spines.values():
            s.set_color(GRID)
        ax.set_title("%d 个完整周期  (f1 = %.2f kHz, Udc = %.1f mV)" % (n, f1 / 1e3, udc),
                     color=FG, fontsize=10)
        self.canvas.draw_idle()

    def _redraw_spec(self):
        if not HAVE_MPL or self.v is None:
            return
        ax = self.ax_s
        ax.clear()
        ax.set_facecolor("#0b0f13")
        ax.grid(True, color=GRID, lw=0.6)
        f, a = dsp.spectrum(self.v, self.fs, "bh", pad=1)
        hi = min(len(f), int(600e3 / (f[1] - f[0])) + 2)
        ax.plot(f[:hi] / 1e3, a[:hi] * 1e3, color=C_SPEC, lw=1.0)
        for ff, aa in dsp.components(self.v, self.fs, f_min=max(500.0,
                                                                self.meas.get("f1", 1e3) * 0.5)):
            ax.plot([ff / 1e3], [aa * 1e3], "o", ms=3.5, color=C_MARK)
            # 每根谱线都要标出频率和幅度（题(4) 判分要读幅度，只标频率不够）
            ax.annotate("%.0f k\n%.1f mV" % (ff / 1e3, aa * 1e3),
                        (ff / 1e3, aa * 1e3),
                        textcoords="offset points", xytext=(2, 3),
                        color=C_MARK, fontsize=6.5, linespacing=1.15)
        ax.set_xlabel("频率 (kHz)  正频率轴", color=FG, fontsize=9)
        ax.set_ylabel("幅度 (mV)", color=FG, fontsize=9)
        ax.tick_params(colors=FG, labelsize=9)
        for s in ax.spines.values():
            s.set_color(GRID)
        self.canvas.draw_idle()

    # ──────────────────────────────────────────────
    def on_save(self):
        if self.v is None:
            return
        p = filedialog.asksaveasfilename(defaultextension=".npz",
                                         filetypes=[("NumPy", "*.npz"), ("CSV", "*.csv")])
        if not p:
            return
        try:
            if p.lower().endswith(".csv"):
                t = np.arange(len(self.v)) / self.fs
                np.savetxt(p, np.column_stack((t, self.v)),
                           delimiter=",", header="t_s,v", comments="")
            else:
                np.savez(p, v=self.v, fs=self.fs, hdr=str(self.hdr))
            self.status.set("已保存 %s" % p)
        except Exception as e:
            messagebox.showerror("保存失败", str(e))


def main():
    app = Scope()
    app.mainloop()


if __name__ == "__main__":
    main()
