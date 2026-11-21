#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lsa_touch.py —— 全触摸上位机（树莓派 / 6 英寸以上触摸屏）

设计约束（针对触摸屏，无键盘无鼠标）：
  * 界面上没有任何输入框（Entry / Text），不需要敲字符
  * 所有操作都是单块大按钮，最小触摸高度 >= 56 px
  * 启动即自动连接 192.168.57.200:5000，连不上就在顶栏提示并可点「连接」重试
  * 默认全屏（--no-fullscreen 可关），退出按钮二次确认
  * 分区固定、不滚动：顶栏 / 波形+频谱 / 大字号读数 / 两行控制条

用法：
    python3 lsa_touch.py                      # 全屏，自动连接
    python3 lsa_touch.py --no-fullscreen      # 窗口模式（调试用）
    python3 lsa_touch.py --host 192.168.57.200 --port 5000
"""

import argparse
import io
import json
import os
import math
import queue
import sys
import threading
import time
import traceback

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
import tkinter.font as tkfont
from tkinter import messagebox

import lsa_proto
import lsa_calib as calib_mod
import lsa_cfg as cfg_mod
import lsa_dsp as dsp
import lsa_eq as eq_mod


# ────────────────────────────── 外观 ──────────────────────────────
BG      = "#0d1117"   # 背景
PANEL   = "#161b22"   # 面板
FG      = "#e6edf3"   # 主文字
MUTED   = "#8b949e"   # 次要文字
BTN     = "#21262d"   # 按钮常态
BTN_ON  = "#0f766e"   # 按钮选中（青）
BTN_RED = "#b91c1c"   # 危险
C_WAVE  = "#2dd4bf"   # 波形线
C_SPEC  = "#f59e0b"   # 频谱线
C_MARK  = "#ef4444"   # 标注
GRID    = "#30363d"

FS_BIG   = 22   # 大字号读数
FS_VAL   = 20
FS_BTN   = 16
FS_SMALL = 11
BTN_FOCUS = "#1f6feb"   # 键盘焦点（蓝）


def setup_font():
    """西文用 DejaVu Sans，中文用系统里能找到的 CJK 字体。

    matplotlib 3.6+ 支持 font.family 给一串字体名逐字符回退，所以这里把
    西文字体排在前面（保证数字/负号/单位正常），CJK 字体排在后面兜中文。
    只把 CJK 字体放第一位会让坐标轴的 0.0/1.0 全变成方块。
    """
    from matplotlib import font_manager, rcParams

    latin = ["DejaVu Sans", "Liberation Sans", "Noto Sans", "FreeSans"]
    cjk = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "Noto Serif CJK SC",
           "WenQuanYi Zen Hei", "WenQuanYi Micro Hei", "Source Han Sans SC",
           "Droid Sans Fallback", "AR PL UMing TW MBE", "AR PL UMing CN",
           "Noto Sans CJK JP", "Noto Serif CJK JP", "SimSun"]
    have = {f.name for f in font_manager.fontManager.ttflist}
    names = [n for n in latin if n in have] + [n for n in cjk if n in have]
    if names:
        rcParams["font.family"] = names
    rcParams["axes.unicode_minus"] = False



setup_font()


# ────────────────────────── 通用控件 ──────────────────────────


def _guess_level(text):
    """按关键词猜日志级别：3=ERR 2=WARN 1=INFO（与固件里的定义一致）。"""
    t = (text or "").lower()
    if any(k in t for k in ("error", "traceback", "exception", "失败", "错误",
                            "超时", "refused", "timeout", "fail")):
        return 3
    if any(k in t for k in ("warn", "警告", "偏差", "重试")):
        return 2
    return 1


# 监测程序的日志文件；界面会 tail 它并转发到板载日志（不存在则跳过）
FWD_LOG = os.environ.get("LSA_FWD_LOG", "/var/log/pi_watch.log")
FWD_CRASH = os.environ.get("LSA_FWD_CRASH", "/var/log/crash/boots.log")

EXCITE_JSON = "excite.json"


def _excite_path():
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), EXCITE_JSON)


def load_excite():
    """读上次装载的激励记录（重启后主界面仍显示）。"""
    try:
        with io.open(_excite_path(), "r", encoding="utf-8") as f:
            d = json.load(f)
        return d if isinstance(d, dict) and d.get("f1_hz") else None
    except Exception:
        return None


def save_excite(d):
    try:
        d = dict(d)
        d["loaded_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        tmp = _excite_path() + ".tmp"
        with io.open(tmp, "w", encoding="utf-8", newline="\n") as f:
            f.write(json.dumps(d, ensure_ascii=False, indent=1))
        os.replace(tmp, _excite_path())
        return True
    except Exception:
        traceback.print_exc()
        return False


def param_save(app):
    """把 eq/config/calib/excite 打包存进板子 W25Q16（失败只提示不打断）。"""
    try:
        import lsa_param
        c = getattr(app, "client", None)
        if c is None:
            return 0
        size, _n = lsa_param.save_from_dir(c)
        if size and hasattr(app, "_status"):
            app._status("参数已存板子 %d B" % size, MUTED)
        return size
    except Exception:
        return 0


def param_restore(app):
    """本地缺参数文件时，从板子 Flash 读回来（重刷系统后自动恢复）。"""
    try:
        import lsa_param
        c = getattr(app, "client", None)
        if c is None:
            return 0
        miss = lsa_param.missing_local()
        if not miss:
            return 0
        n = lsa_param.restore_to_dir(c)
        if n:
            app.eq = eq_mod.load()
            app.cfg = cfg_mod.load()
            app.in_scale = cfg_mod.get_in_scale(app.cfg)
            app._status("已从板子恢复 %d 个参数文件" % n, C_WAVE)
        return n
    except Exception:
        return 0


def excite_text(d, stale=False):
    """把一条激励记录写成一行。stale=True 表示这是上次会话留下的。"""
    if not d:
        return ""
    _t = list(d.get("tones") or [])
    _m0 = min([int(m) for m, _r in _t]) if _t else 1
    # 预置里有些用例为了凑整数周期会让「基础频率」不是最低分量本身
    # （例如题2-C 用 29.297 kHz × 8 = 234.375 kHz 作基波），
    # 这里统一折算到**实际基波**，与主界面 f1 的口径一致，否则会被当成跑飞。
    tones = "+".join(("%gx" % (int(m) // _m0)) if float(r) == 1.0
                     else ("%gx%g" % (int(m) // _m0, r)) for m, r in _t)
    parts = ["%.4g kHz" % (float(d["f1_hz"]) * _m0 / 1e3),
             "%.1f mVpp" % float(d.get("vpp_mv") or 0.0)]
    if tones:
        parts.append(tones)
    if d.get("jam_hz"):
        parts.append("干扰 %.3f MHz" % (float(d["jam_hz"]) / 1e6))
    ts = str(d.get("loaded_at") or "")[11:16]
    if ts:
        parts.append(("%s上次 %s" % ("· ", ts)) if stale else ("· " + ts))
    return "  ".join(parts)


class BigButton(tk.Button):
    """大块触摸按钮：常态深灰，按下/选中青绿。"""

    def __init__(self, master, text, command, font_size=FS_BTN, padx=16, pady=10,
                 color=BTN, width=None, takefocus=1):
        super().__init__(
            master, text=text, command=command,
            font=("", font_size, "bold"),
            bg=color, fg=FG, activebackground=BTN_ON, activeforeground="white",
            relief="flat", bd=0, padx=padx, pady=pady,
            highlightthickness=0, takefocus=takefocus,
        )
        if width:
            self.configure(width=width)
        self._base = color
        self._pre_focus = color
        # 纯键盘操作：Tab 切到按钮 → 底色变蓝；Enter / 小键盘回车等价于点击
        self.bind("<Return>", self._kb_invoke)
        self.bind("<KP_Enter>", self._kb_invoke)
        self.bind("<FocusIn>", self._on_focus_in)
        self.bind("<FocusOut>", self._on_focus_out)

    def _kb_invoke(self, _e=None):
        if str(self.cget("state")) != "disabled":
            self.invoke()
        return "break"

    def _on_focus_in(self, _e=None):
        self._pre_focus = str(self.cget("bg"))
        self.configure(bg=BTN_FOCUS)

    def _on_focus_out(self, _e=None):
        self.configure(bg=getattr(self, "_pre_focus", self._base))

    def flash(self, color, ms=140):
        old = self.cget("bg")
        self.configure(bg=color)
        self.after(ms, lambda: self.configure(bg=old))


class SegBar(tk.Frame):
    """一排互斥按钮（选中态用青绿底色），代替下拉框。"""

    def __init__(self, master, options, on_pick, font_size=FS_BTN, padx=12, pady=10):
        super().__init__(master, bg=BG)
        self.var = options[0][0]
        self.on_pick = on_pick
        self.btns = {}
        for key, label in options:
            b = BigButton(self, label, lambda k=key: self.pick(k),
                          font_size=font_size, padx=padx, pady=pady)
            b.pack(side="left", padx=(0, 6))
            self.btns[key] = b
        self._paint()

    def pick(self, key):
        self.var = key
        self._paint()
        if self.on_pick:
            self.on_pick(key)

    def set(self, key):
        self.var = key
        self._paint()

    def _paint(self):
        for k, b in self.btns.items():
            b.configure(bg=BTN_ON if k == self.var else BTN,
                        fg="white" if k == self.var else FG)


class Readout(tk.Frame):
    """一格大字号读数：上面小标签，下面大数值。"""

    def __init__(self, master, label, text="-", color=FG):
        super().__init__(master, bg=PANEL)
        tk.Label(self, text=label, font=("", FS_SMALL), bg=PANEL, fg=MUTED).pack(anchor="w", padx=7, pady=(6, 0))
        self.val = tk.Label(self, text=text, font=("", FS_BIG, "bold"), bg=PANEL, fg=color)
        self.val.pack(anchor="w", padx=7, pady=(0, 6))

    def set(self, text, color=None):
        self.val.configure(text=text)
        if color:
            self.val.configure(fg=color)


# ────────────────── 阶梯校准（引导式，21 点） ──────────────────
def tones_for_src(tones, comps):
    """把赛题用例的「第 m 倍 / 相对比 r」+「分量峰值 mV」换成源要的绝对幅度表。

    comps 是各分量的**峰值 mV**（_build_wave 给的），源要的是 **Vpp**，所以乘 2
    再除 1000。返回 [(绝对次数 m, Vpp, 相位deg), ...] —— **保留绝对次数**，
    例如 ((2,1.0),(6,0.55)) 返回 [(2,...),(6,...)]：基波是 2 次（2·F0），
    谐波是 6 次（相对基波是 3 次）。归一化的活交给 load_wave 做，因为只有它
    知道 DG 的谐波表要的是"相对基波的次数"。
    """
    return [(int(round(float(m))), 2.0 * float(peak_mv) / 1000.0, 0.0)
            for (m, _r), (_f, peak_mv) in zip(tones, comps)]


class StepCalib(tk.Toplevel):
    """引导式阶梯校准：21 个固定频点，逐点「设频率 → 采一点」。

    为什么不扫频：AFG 的 SCPI 扫频在这台机器上不生效；即使生效，连续扫频时
    3.47 ms 的采集窗会扫过好几个频点，谱峰被抹平。逐点停住再采，频率由人眼
    确认，算法与 PC 侧 cal_step.py 完全相同（21 点归一化表 + 单级二阶拟合）。

    归一化基准取 <=20 kHz 的平台 —— 与源的负载设定、板子绝对增益都无关，
    所以信号源上用 250 mVpp 还是 100 mVpp 都不影响形状，只影响信噪比。
    """

    FREQS = (10e3, 14e3, 20e3, 28e3, 40e3, 56e3, 80e3, 110e3, 150e3, 200e3,
             250e3, 300e3, 325e3, 350e3, 375e3, 400e3, 440e3, 470e3, 500e3,
             520e3, 550e3)
    AMP_MV = 250.0          # 赛题工作幅度
    N_PTS = 16384
    TOL = 0.03              # 源频率允许偏差（判断信号源有没有真的设进去）

    # ── 全测试：校准完立刻按赛题说明 1/3 复测四项 ──
    # ARB N=4096 @120 MSa/s → 基频只能取 m·29296.875 Hz（整周期重放，否则拼接处跳变）。
    # 说明 1：u_a/u_b = 基波 + 1~2 个谐波；说明 3：谱线幅值报**峰值**。
    # u_j 固定 200 mVpp（峰值 100 mV），与被测信号幅度无关。
    ARB_N = 4096
    ARB_FS = 120e6
    F0 = ARB_FS / ARB_N
    JAM_PEAK_MV = 100.0
    TEST_CASES = (
        ("题1-A", 1, ((1, 1.00), (3, 0.60), (4, 0.40)), 250.0, None),
        ("题1-B", 1, ((1, 1.00), (3, 0.60), (4, 0.40)), 100.0, None),
        ("题1-C", 1, ((2, 1.00), (6, 0.55)), 175.0, None),
        ("题1-D", 1, ((2, 1.00), (6, 0.55)), 120.0, None),
        ("题2-A", 2, ((4, 1.00), (12, 0.60), (16, 0.40)), 250.0, None),
        ("题2-B", 2, ((4, 1.00), (12, 0.60), (16, 0.40)), 50.0, None),
        ("题2-C", 2, ((8, 1.00), (16, 0.70)), 250.0, None),
        ("题2-D", 2, ((1, 1.00), (10, 0.50), (17, 0.30)), 150.0, None),
        ("题3-A", 3, ((4, 1.00), (12, 0.60), (16, 0.40)), 250.0, 41),
        ("题3-B", 3, ((2, 1.00), (6, 0.55)), 250.0, 35),
    )

    def __init__(self, app):
        super().__init__(app.root)
        self.app = app
        self.title("阶梯校准")
        self.configure(bg=BG)
        self.resizable(False, False)
        self.geometry("980x470+40+80")
        try:
            self.transient(app.root)
            self.grab_set()
        except tk.TclError:
            pass
        self.rows = []              # [(f_req_hz, upp_mv)]
        self.i = 0                  # 下一个待采的点
        self.busy = False
        self.done = False
        self.src = None             # 已连接的信号源（lsa_src.Source），None = 手动
        self.auto_run = False       # 「全自动」：采完一点自动设下一点
        self._q = queue.Queue()
        self._build()
        self._drain()
        self._show_step()

    # ---------- 界面 ----------
    def _build(self):
        tk.Label(self, text="阶梯校准（%d 点）" % len(self.FREQS), font=("", FS_VAL, "bold"),
                 bg=BG, fg=FG).pack(anchor="w", padx=18, pady=(12, 2))
        self.lbl_step = tk.Label(self, text="", font=("", FS_BTN, "bold"),
                                 bg=BG, fg=C_WAVE, anchor="w")
        self.lbl_step.pack(fill="x", padx=18)
        self.lbl_hint = tk.Label(
            self,
            text="正弦 · 偏置 0 V · 负载不限（按低频平台自动归一）· 每点约 1 秒\n"
                 "把信号源设到上面这个频率，再按「采一点」。频率对不上会提示重来。",
            font=("", FS_SMALL), bg=BG, fg=MUTED, anchor="w", justify="left")
        self.lbl_hint.pack(fill="x", padx=18, pady=(4, 10))

        self.cv = tk.Canvas(self, width=940, height=26, bg=PANEL, highlightthickness=0)
        self.cv.pack(padx=18)
        self.bar = self.cv.create_rectangle(0, 0, 0, 26, fill=BTN_ON, width=0)

        self.lbl_prog = tk.Label(self, text="", font=("", FS_BTN), bg=BG, fg=FG,
                                 anchor="w")
        self.lbl_prog.pack(fill="x", padx=18, pady=(6, 2))
        self.lbl_live = tk.Label(self, text="", font=("", FS_SMALL), bg=BG, fg=MUTED,
                                 anchor="w")
        self.lbl_live.pack(fill="x", padx=18)

        row = tk.Frame(self, bg=BG)
        row.pack(fill="x", padx=18, pady=(12, 6))
        self.btn_cap = BigButton(row, "采一点", self._on_cap, padx=26)
        self.btn_cap.pack(side="left")
        self.btn_skip = BigButton(row, "跳过", self._on_skip, padx=16,
                                  font_size=FS_BTN - 2)
        self.btn_skip.pack(side="left", padx=8)
        BigButton(row, "关闭", self._on_close, color=BTN_RED, padx=18,
                  font_size=FS_BTN - 2).pack(side="right")
        # 信号源：手动 = 人来改频率再点「采一点」；AFG = 由本机 SCPI 发指令
        self.btn_src = BigButton(row, "源: 未连接", self._on_toggle_src, padx=14,
                                 font_size=FS_BTN - 2)
        self.btn_src.pack(side="right", padx=8)
        # ↑/↓ 在候选信号源之间切换（AFG / DG / FAKE / 未连接）
        self.btn_src.bind("<Down>", lambda _e: self._on_toggle_src(+1))
        self.btn_src.bind("<Up>", lambda _e: self._on_toggle_src(-1))
        self.btn_auto = BigButton(row, "全自动", self._on_auto, padx=14,
                                  font_size=FS_BTN - 2)
        self.btn_auto.pack(side="right")
        # 校准完可以立刻跑一遍 10 个赛题用例，验证补偿表是否有效
        self.btn_test = BigButton(row, "全测试", self._on_fulltest, padx=14,
                                  font_size=FS_BTN - 2)
        self.btn_test.pack(side="right", padx=8)
        self.btn_test.configure(state="disabled")

        self.lbl_res = tk.Label(self, text="", font=("", FS_SMALL), bg=BG, fg=FG,
                                anchor="w", justify="left")
        self.lbl_res.pack(fill="x", padx=18, pady=(6, 0))

        # 全测试结果表（默认不显示，进测试阶段才 pack）
        self.txt_res = tk.Text(self, height=13, bg=PANEL, fg=FG, bd=0,
                               font=("mono", FS_SMALL), wrap="none",
                               highlightthickness=0, takefocus=0)
        self.txt_res.tag_configure("ok", foreground=C_WAVE)
        self.txt_res.tag_configure("bad", foreground=C_MARK)

    def _ui(self, fn):
        """从采集线程往主线程投递界面更新（Tk 只能在主线程动）。"""
        self._q.put(fn)

    def _drain(self):
        try:
            while True:
                fn = self._q.get_nowait()
                try:
                    fn()
                except tk.TclError:
                    return
                except Exception:
                    traceback.print_exc()
        except queue.Empty:
            pass
        try:
            self.after(80, self._drain)
        except tk.TclError:
            pass

    def _paint(self):
        w = int(940.0 * len(self.rows) / len(self.FREQS))
        self.cv.coords(self.bar, 0, 0, max(0, w), 26)

    def _tbl_text(self):
        if not self.rows:
            return "—"
        return "　".join("%.4g kHz → %.1f mV" % (f / 1e3, u)
                         for f, u in self.rows[-2:])

    def _show_step(self):
        if self.done:
            return
        f = self.FREQS[self.i]
        self.lbl_step.configure(
            text="第 %d / %d 点：把信号源设为 %.4g kHz / %.0f mVpp"
                 % (self.i + 1, len(self.FREQS), f / 1e3, self.AMP_MV),
            fg=C_WAVE)
        self.lbl_prog.configure(
            text="已采 %d 点 / %d    待采 %.4g kHz"
                 % (len(self.rows), len(self.FREQS), f / 1e3))
        self._paint()

    # ---------- 按钮 ----------
    def _on_cap(self):
        if self.busy or self.done:
            return
        self.busy = True
        self.btn_cap.configure(state="disabled", text="采集中…")
        threading.Thread(target=self._worker, args=(self.FREQS[self.i],),
                         daemon=True).start()

    # ---------- 信号源 ----------
    def _on_toggle_src(self, step=1):
        if self.busy or self.done:
            return
        cycle_src(self, step)

    def _src_note(self, text, bad=False):
        """cycle_src 的回调：把信号源状态写到提示区。"""
        try:
            self.lbl_hint.configure(
                text=("%s  本机会按提示自动设置频率/幅度。" % text) if not bad else text,
                fg=(C_MARK if bad else MUTED))
        except tk.TclError:
            pass

    def _on_auto(self):
        """从当前点开始自动跑完剩下所有点（需要已连上信号源）。"""
        if self.busy or self.done:
            return
        if self.src is None:
            self.lbl_step.configure(text="请先点「源: 手动」切换成自动（连信号源）",
                                    fg=C_MARK)
            return
        self.auto_run = True
        self._on_cap()

    def _on_skip(self):
        if self.busy or self.done:
            return
        self.i += 1
        if self.i >= len(self.FREQS):
            self._finish()
        else:
            self._show_step()

    def _on_close(self):
        if self.src is not None:
            try:
                self.src.close()
            except Exception:
                pass
            self.src = None
        try:
            self.grab_release()
        except tk.TclError:
            pass
        self.destroy()

    # ---------- 信号源回读 ----------
    def _src_check(self, f_expect, vpp_expect):
        """采集前回读信号源设定，确认真的写进去了（AFG 会静默丢写）。

        返回 (ok, 说明)。silent 模式（没有回读能力）直接放行，
        那时只能靠采集到的频率做二次校验。
        """
        if self.src is None:
            return True, "无信号源"
        if getattr(self.src, "silent", False):
            return True, "无回读（silent）"
        try:
            rb = self.src.readback(ch=1)
        except Exception as e:
            return False, "回读失败：%s" % e
        f = rb.get("freq_hz")
        a = rb.get("amp_vpp")
        fn = str(rb.get("func") or "")
        if f is None or a is None:
            return False, "回读为空（%s）" % (fn or "?")
        if abs(f - f_expect) > max(1.0, 0.005 * abs(f_expect)):
            return False, "频率回读 %.3f kHz，应为 %.4g kHz" % (f / 1e3, f_expect / 1e3)
        if abs(a - vpp_expect) > max(0.005, 0.08 * abs(vpp_expect)):
            return False, "幅度回读 %.4f Vpp，应为 %.4f Vpp" % (a, vpp_expect)
        return True, "%s %.3f kHz %.4f Vpp" % (fn or "?", f / 1e3, a)

    def _src_apply(self, f_expect, vpp_expect, again=None):
        """设置并回读确认；不对就重下一次，仍不对返回错误说明。"""
        ok, why = self._src_check(f_expect, vpp_expect)
        if ok:
            return None
        try:
            if again is not None:
                again()
            else:
                self.src.set_point(f_expect, vpp_expect)
        except Exception as e:
            return "重设失败：%s（%s）" % (e, why)
        ok, why = self._src_check(f_expect, vpp_expect)
        return None if ok else why

    # ---------- 采集 ----------
    def _worker(self, f_req):
        try:
            if self.src is not None:
                self.src.set_point(f_req, self.AMP_MV / 1000.0)
                bad = self._src_apply(f_req, self.AMP_MV / 1000.0)
                if bad:
                    raise RuntimeError("信号源没设上：%s" % bad)
            hdr, raw = self.app.client.capture(gear=0, inter3=False, ch=0,
                                               n=self.N_PTS, timeout=10.0)
            codes = np.frombuffer(raw, dtype="<i2").astype(np.float64)
            fs = float(hdr["fs_hz"])
            v = dsp.codes_to_volts(codes, hdr["gain_milli"], scale=1.0)
            # 用 measure()（谐波最小二乘 + 强制正弦）而不是 np.ptp：
            # ptp 会把噪声的极值算进去，高频段信号小（550 kHz 只有 ~35 mVpp）
            # 时噪声占 ±1~2 码，ptp 会把表抬高 2~7%，补偿就欠了。
            # 校准源一定是正弦，force_wave="sine" 使 Upp = 2*a1，与噪声无关。
            m = dsp.measure(v, fs, force_wave="sine")
            upp = float(m["upp"]) * 1000.0         # mV，原始未补偿
            fp = float(m["f1"])
            self._ui(lambda fq=float(fp), u=upp: self._on_point(f_req, fq, u, None))
        except Exception as exc:
            msg = str(exc)
            self._ui(lambda m=msg: self._on_point(f_req, 0.0, 0.0, m))

    def _on_point(self, f_req, fp, upp, err):
        self.busy = False
        self.btn_cap.configure(state="normal", text="采一点")
        if self.done:
            return
        if err:
            self.auto_run = False
            self.lbl_step.configure(text="采集失败：%s" % err, fg=C_MARK)
            return
        if fp > 0 and abs(fp - f_req) > self.TOL * f_req:
            self.auto_run = False
            self.lbl_step.configure(
                text="频率不对：读到 %.2f kHz，应为 %.4g kHz —— 检查信号源"
                     % (fp / 1e3, f_req / 1e3), fg=C_MARK)
            return
        self.rows.append((f_req, upp))
        self.lbl_live.configure(text="已采 %d 点　最近：%s"
                                     % (len(self.rows), self._tbl_text()))
        self.i += 1
        if self.i >= len(self.FREQS):
            self._finish()
        else:
            self._show_step()
            if self.auto_run and self.src is not None:
                self.after(180, self._on_cap)

    # ---------- 拟合与存盘 ----------
    def _finish(self):
        self.done = True
        self.btn_cap.configure(state="disabled", text="已完成")
        self.btn_skip.configure(state="disabled")
        rows = self.rows
        if len(rows) < 6:
            self.lbl_step.configure(text="校准失败", fg=C_MARK)
            self.app._log_board(3, "阶梯校准失败，已采 %d 点" % len(self.rows))
            self.lbl_res.configure(
                text="有效点只有 %d 个（至少 6 个才能建表）。\n"
                     "检查信号源后关掉重来。" % len(rows), fg=C_MARK)
            return
        freqs = np.array([r[0] for r in rows], dtype=np.float64)
        amps = np.array([r[1] for r in rows], dtype=np.float64)
        low = amps[freqs <= 20e3]
        ref = float(np.median(low)) if low.size else float(amps[0])
        if ref <= 1.0:
            self.lbl_step.configure(text="校准失败", fg=C_MARK)
            self.lbl_res.configure(
                text="低频平台只有 %.2f mV，信号太小（应 >=50 mVpp）。" % ref,
                fg=C_MARK)
            return
        table = [(float(f), float(u / ref)) for f, u in rows]
        try:
            fn, Q, rms, _r = eq_mod.fit_points(freqs, amps)
        except Exception:
            fn = Q = rms = 0.0
        in_scale = self.AMP_MV / ref
        try:
            eq_mod.save_table(table, fn, Q)
            cfg_mod.set_in_scale(in_scale)
            self.app.eq = eq_mod.load()
            self.app.cfg = cfg_mod.load()
            self.app.in_scale = cfg_mod.get_in_scale(self.app.cfg)
            self.app._refresh()
        except Exception:
            traceback.print_exc()
            self.lbl_res.configure(text="写盘失败（见控制台）", fg=C_MARK)
            return
        self._paint()
        self.lbl_step.configure(text="校准完成：已写入 eq.json / config.json",
                                fg=C_WAVE)
        self.lbl_prog.configure(text="共 %d 点   低频平台 %.2f mVpp（源 %.0f mVpp）"
                                     % (len(rows), ref, self.AMP_MV))
        self.lbl_res.configure(
            text="模拟链路  fc = %.1f kHz   Q = %.3f   （拟合 RMS %.2f dB）\n"
                 "增益系数  in_scale = %.0f / %.2f = %.7f\n"
                 "界面已立即生效。建议再挑几个频点复测确认。"
                 % (fn / 1e3, Q, rms, self.AMP_MV, ref, in_scale), fg=C_WAVE)
        # 校准结果顺手备份到板子 Flash（树莓派重刷也不丢）
        param_save(self.app)
        # 有信号源就直接把 10 个赛题用例跑一遍，用实测验证这张表
        try:
            self.btn_test.configure(state="normal")
            if self.src is not None:
                self.after(1500, self._on_fulltest)
        except tk.TclError:
            pass

    # ---------- 全测试：校准后按赛题说明 1/3 复测四项 ----------
    def _measure_env(self):
        """DG4202 专用的测量口径（不在场时一个字都不改）。

        为什么要这三个开关：DG 的输出在 ~600 kHz 有一根固定的、不随幅度变化的
        约 6 mV 残留（会经由前端混叠进来），而 eq.json 在 550 kHz 要补 ×7.12，
        这根残留被放大成 Upp 虚高 60~90 mV；同时它让小幅度波形的拟合残差越过
        0.03，measure() 退回分位数法，Upp 再被抬高约 10 mV。
        实测同一组数据：分位数法 261 mV，拟合 250.06 mV，设计值 250 mV。
        """
        env = dict(os.environ)
        if getattr(self.src, "name", "") == "DG4202":
            env.setdefault("LSA_EQ_FHI_HZ", "520000")
            env.setdefault("LSA_EQ_LP_HZ", "520000")
            env.setdefault("LSA_FIT_RESID_MAX", "0.15")
        return env

    def _build_wave(self, tones, ub_vpp, jam_m):
        """合成「基波 + 1~2 个谐波」的一周期波形（N=4096，整周期重放）。

        tones: ((m, ratio), ...)，第 m 次谐波、相对幅度 ratio；初相全 0。
        返回 (vals, comps, ub_th, tot)：comps 是 [(f_hz, 峰值mV)]，
        ub_th 是 u_b **整体合成波形**的峰峰值（赛题 Q1：U_pp 指总峰峰值）。
        """
        n = self.ARB_N
        t = np.arange(n, dtype=np.float64)
        w = np.zeros(n)
        for m, r in tones:
            w += float(r) * np.sin(2.0 * np.pi * float(m) * t / n)
        span = float(np.max(w) - np.min(w))
        if span <= 0:
            raise ValueError("合成波形退化")
        ub = w * (ub_vpp / span)                 # 缩放到 u_b 的目标峰峰值
        ub_th = float(np.max(ub) - np.min(ub))
        if jam_m:
            ub = ub + self.JAM_PEAK_MV * np.sin(2.0 * np.pi * float(jam_m) * t / n)
        tot = float(np.max(ub) - np.min(ub))
        peak = float(np.max(np.abs(ub)))
        vals = ub / (2.0 * peak)                 # ARB 一律归一化到 ±0.5
        # 各分量峰值：ub = w·(ub_vpp/span)，所以第 m 个分量的峰值就是 r·ub_vpp/span，
        # 单位已经是 mV（ub_vpp 是 mV）。这里再乘 1000 或除 2 都是错的。
        # 注意：**干扰 u_j 不进 comps**。赛题四项都只针对 u_b，u_j 是要求被抑制掉的，
        # 拿它当"应测到的分量"会让误差恒等于 100 mV。
        comps = [(float(m) * self.F0, float(r) * ub_vpp / span)
                 for m, r in tones]
        return vals, comps, ub_th, tot

    def _enter_test_ui(self):
        for w in (self.lbl_hint, self.cv, self.lbl_prog, self.lbl_live):
            try:
                w.pack_forget()
            except tk.TclError:
                pass
        try:
            if not self.txt_res.winfo_ismapped():
                self.txt_res.pack(fill="both", expand=True, padx=18, pady=(8, 4))
        except tk.TclError:
            pass

    def _on_fulltest(self):
        if self.busy or not self.done:
            return
        if self.src is None:
            self.lbl_res.configure(
                text="全测试需要信号源：先点「源: 手动」连上 AFG。", fg=C_MARK)
            return
        self.busy = True
        try:
            self.btn_test.configure(state="disabled", text="测试中…")
            self.btn_cap.configure(state="disabled")
            self.btn_auto.configure(state="disabled")
            self._enter_test_ui()
            self.txt_res.delete("1.0", "end")
            self.txt_res.insert(
                "end", "%-7s %8s %9s %8s %8s  %s\n"
                       % ("用例", "Upp误差", "Urms误差", "f1误差", "分量最大", "判定"))
            self.txt_res.insert("end", "-" * 58 + "\n")
        except tk.TclError:
            pass
        threading.Thread(target=self._test_worker, daemon=True).start()

    def _conn_release(self):
        """板子只允许一条 TCP 连接：跑子进程前必须先让出。

        固件同一时刻只服务一个 socket，界面自己占着的话 `_contest_one.py`
        连上去会直接被拒（ConnectionRefused）。关掉后要等它看到 FIN 再放行。
        """
        try:
            if self.app.client is not None:
                self.app.client.close()
        except Exception:
            pass
        self.app.client = None
        self.app.connected = False
        time.sleep(0.5)

    def _conn_reacquire(self):
        for _ in range(4):
            try:
                c = lsa_proto.LsaClient(host=self.app.host, port=self.app.port,
                                        timeout=2.0)
                c.connect()
                c.ping()
                self.app.client = c
                self.app.connected = True
                return True
            except Exception:
                time.sleep(0.8)
        return False

    def _test_worker(self):
        import json
        import os as _os
        import subprocess
        worst = {"upp": 0.0, "urms": 0.0, "f1": 0.0, "comp": 0.0}
        npass = 0
        try:
            for label, _item, tones, ub_vpp, jam in self.TEST_CASES:
                vals, comps, ub_th, tot = self._build_wave(tones, ub_vpp, jam)
                # tot 是 mV，load_arb 要 V —— 传错单位会被固件钳到 10 Vpp（满量程削顶）
                vpp = tot / 1000.0
                # 谐波优先：把「第几次 / 多大幅度」一并交给源。DG4202 走硬件谐波模式；
                # 表示不了（最高 >16 次、非整数倍、带干扰）时它自己打印原因并退回 ARB。
                # 带干扰 u_j 时直接给 None：干扰不是基波的整数次谐波，谐波模式放不出来。
                ts = None if jam else tones_for_src(tones, comps)
                why = ("带干扰 u_j（%g×F0 = %.4f MHz），不是基波整数倍，"
                       "谐波模式放不出来" % (jam, jam * self.F0 / 1e6)) if jam else None
                mode, note = self.src.load_wave(vals, self.F0, vpp, ch=1,
                                                tones=ts, reason=why)
                # 谐波模式下 :VOLT 是**基波** Vpp，回读校验要用基波幅度，
                # 不能用 tot（tot 是整个合成波形的峰峰值）
                fund = (float(comps[0][1]) * 2.0 / 1000.0
                        if mode == "harmonic" else vpp)
                # 谐波模式下基波频率是「最小次数 × F0」，不是 F0 本身
                f_chk = (self.F0 * min(float(m) for m, _ in tones)
                         if mode == "harmonic" else self.F0)
                # 采 ADC 之前先回读源：AFG 会静默丢写，确认不对就重下一次
                bad = self._src_apply(
                    f_chk, fund,
                    again=lambda: self.src.load_wave(vals, self.F0, vpp,
                                                     ch=1, tones=ts,
                                                     reason=why))
                self._ui(lambda t=(label, note): self._test_note(
                    "  %s 路数：%s" % t))
                if bad:
                    self._ui(lambda a=(label, 0.0, 0.0, 0.0, 0.0, False):
                             self._test_row(a, note="源未设上：%s" % bad))
                    npass += 0
                    continue
                # 主界面「激励」行改成当前测试波形：否则它还显示上次手动装载的
                # 参数，而源里是测试波形，两边对不上就像"跑飞"
                self._ui(lambda s = ("%s  %.0f mVpp" % (label, tot)):
                         self.app.set_excite("测试波形 " + s))
                time.sleep(1.4)
                self._conn_release()
                try:
                    out = subprocess.run(
                        [sys.executable, "_contest_one.py", "--gear", "0",
                         "--n", "16384", "--harm", "20", "--apply"],
                        cwd=os.path.dirname(os.path.abspath(__file__)),
                        capture_output=True, text=True, timeout=120,
                        env=self._measure_env())
                finally:
                    self._conn_reacquire()
                lines = [ln for ln in out.stdout.splitlines()
                         if ln.strip().startswith("{")]
                if not lines:
                    raise RuntimeError((out.stderr or out.stdout).strip()[-160:])
                d = json.loads(lines[-1])
                hm = d.get("harm_mv") or []
                f1 = float(d["f1_hz"])
                urms_th = math.sqrt(sum((a / math.sqrt(2.0)) ** 2
                                        for _, a in comps))
                e_upp = float(d["upp_mv"]) - ub_th
                e_urms = float(d["urms_mv"]) - urms_th
                e_f1 = f1 - comps[0][0]
                ec = 0.0
                for f, a in comps:
                    k = int(round(f / f1)) if f1 > 0 else 0
                    meas = hm[k - 1] if 0 < k <= len(hm) else float("nan")
                    if meas == meas:
                        ec = max(ec, abs(meas - a))
                ok = (abs(e_upp) <= 5.0 and abs(e_urms) <= 5.0
                      and abs(e_f1) <= 1000.0 and ec <= 5.0)
                npass += 1 if ok else 0
                worst["upp"] = max(worst["upp"], abs(e_upp))
                worst["urms"] = max(worst["urms"], abs(e_urms))
                worst["f1"] = max(worst["f1"], abs(e_f1))
                worst["comp"] = max(worst["comp"], ec)
                self._ui(lambda a=(label, e_upp, e_urms, e_f1, ec, ok):
                         self._test_row(a))
        except Exception as e:
            traceback.print_exc()
            self._ui(lambda e=e: self._test_done(0, {}, str(e)))
            return
        # 测完把源恢复成最后一次手动装载的波形，否则主界面量到的是测试波形
        try:
            d = load_excite()
            print("[restore] excite =", d, flush=True)
            if d and not d.get("jam_hz"):
                _tt = tuple((int(m), float(r)) for m, r in d["tones"])
                vals, _c, _ub, tot = self._build_wave(_tt, float(d["vpp_mv"]), 0)
                # 恢复也走「谐波优先」：DG 上能谐波就谐波，这样恢复出来的波形
                # 和测试时走的是同一条路径
                self.src.load_wave(vals, float(d["f1_hz"]), tot / 1000.0,
                                   ch=1, tones=tones_for_src(_tt, _c))
                self._ui(lambda dd=d: self.app.set_excite(excite_text(dd)))
            elif d:
                self._ui(lambda dd=d: self.app.set_excite(
                    excite_text(dd) + "  (源内是测试波形)"))
            else:
                self._ui(lambda: self.app.set_excite("（源内是测试波形）"))
        except Exception:
            traceback.print_exc()      # 恢复失败要看得见，别悄悄吞掉
        self._ui(lambda: self._test_done(npass, worst, None))

    def _test_note(self, text):
        """在全测试结果区插一行说明（用来显示每个用例走了谐波还是 ARB）。"""
        try:
            self.txt_res.insert("end", text + "\n")
            self.txt_res.see("end")
        except tk.TclError:
            pass

    def _test_row(self, a, note=None):
        label, e_upp, e_urms, e_f1, ec, ok = a
        try:
            if note:
                self.txt_res.insert(
                    "end", "%-7s %8s %9s %8s %8s  %s\n"
                           % (label, "-", "-", "-", "-", note), ("bad",))
            else:
                self.txt_res.insert(
                    "end", "%-7s %+8.2f %+9.2f %+8.1f %8.2f  %s\n"
                           % (label, e_upp, e_urms, e_f1, ec,
                              "PASS" if ok else "FAIL"),
                    ("ok" if ok else "bad",))
            self.txt_res.see("end")
        except tk.TclError:
            pass

    def _test_done(self, npass, worst, err):
        try:
            self.btn_test.configure(state="normal", text="全测试")
            self.btn_cap.configure(state="normal")
            self.btn_auto.configure(state="normal")
            if err:
                self.txt_res.insert("end", "\n中断：%s\n" % err)
                self.lbl_step.configure(text="全测试中断", fg=C_MARK)
            else:
                n = len(self.TEST_CASES)
                self.txt_res.insert(
                    "end", "-" * 58 + "\n通过的 %d / %d    最坏：Upp %.2f  "
                           "Urms %.2f  f1 %.1f Hz  分量 %.2f mV\n"
                           % (npass, n, worst.get("upp", 0.0),
                              worst.get("urms", 0.0), worst.get("f1", 0.0),
                              worst.get("comp", 0.0)), ())
                self.lbl_step.configure(
                    text="全测试完成：%d / %d 通过" % (npass, n),
                    fg=C_WAVE if npass == n else C_MARK)
        except tk.TclError:
            pass
        self.busy = False


# ────────────────────────── 主程序 ──────────────────────────
class TouchApp:
    def __init__(self, root, host, port):
        self.root = root
        self.host = host
        self.port = port

        self.client = None
        self.connected = False
        self.inter3 = False          # 三路交替
        self.gear_mode = 0            # 0..3 或 "auto"；默认档位 0（最高采样率）
        self.periods = 3             # 波形显示 1 / 3 个完整周期
        self.remove_u3 = False       # 数字低通去 u3
        self.use_eq = True           # 模拟链路幅频补偿（板子在高频掉增益，默认开）
        self.wave_mode = "auto"      # 波形类型：auto 自动判型，其余强制解析换算
        self.eq = eq_mod.load()
        self.busy = False
        self._worst_ms = 0.0         # 「手指按下 -> 读数出完」最坏耗时（赛题限 2 s）
        self._last_ms = 0.0          # 最近一次的耗时
        self._codes = None           # 最近一帧的原始码（保存/直接分量法要用）

        # 输入比例：生成器按 50Ω 设定而板子高阻时，板子看到的是设定值的 2 倍
        self.cfg = cfg_mod.load()
        self.in_scale = cfg_mod.get_in_scale(self.cfg)
        dsp.IN_SCALE = 1.0          # 比例在 _refresh 里于补偿之后统一施加

        self.v = None
        self.fs = 0.0
        self.hdr = None
        self.meas = None
        self.calib = calib_mod.load()   # 零输入基线（没有文件就是空表，不做修正）

        self._build_ui()
        self._restore_excite()      # 上次装载的激励参数，重启后仍显示
        self.root.after(400, self.btn_cap.focus_set)   # 纯键盘：默认焦点在「采集」
        self.root.title("周期信号谐波分析装置")
        self.root.protocol("WM_DELETE_WINDOW", self.on_quit)
        self.root.bind("<Destroy>", self._on_destroy)
        self.root.bind("<Escape>", self._on_escape)
        # 启动后自动连一次；失败不弹窗，只在顶栏显示
        self.root.after(200, self.on_connect)
        self.root.after(8000, self._keepalive)

    # ---------------- 界面 ----------------
    def _build_ui(self):
        r = self.root
        r.configure(bg=BG)
        r.rowconfigure(1, weight=1)
        r.columnconfigure(0, weight=1)

        # ① 顶栏
        top = tk.Frame(r, bg=BG)
        top.grid(row=0, column=0, sticky="ew", padx=8, pady=(8, 4))

        # 先摆右边的按钮，再摆左边的；中间标签可压缩，避免连接字符串把右侧按钮挤出屏幕
        self.btn_quit = BigButton(top, "退出", self.on_quit, color=BTN_RED)
        self.btn_quit.pack(side="right")
        self.btn_test = BigButton(top, "自检", self.on_selftest)
        self.btn_test.pack(side="right", padx=6)
        self.btn_save = BigButton(top, "保存", self.on_save)
        self.btn_save.pack(side="right", padx=6)

        # 幅频补偿（板子模拟链路在 500 kHz 掉了 ~19 dB，只有把它拉平 Upp 才是真的）
        self.btn_eq = BigButton(top, "补偿 开", self.on_toggle_eq,
                                font_size=FS_BTN - 2, padx=12)
        self.btn_eq.pack(side="right", padx=(0, 6))

        self.btn_conn = BigButton(top, "连接", self.on_connect)
        self.btn_conn.pack(side="left")

        # 中间这段放「连接状态 + 状态消息」，可随窗口伸缩而压缩
        self.top_mid = tk.Frame(top, bg=BG)
        self.top_mid.pack(side="left", fill="x", expand=True)

        self.lbl_conn = tk.Label(self.top_mid, text="未连接",
                                 font=("", FS_BTN, "bold"),
                                 bg=BG, fg=MUTED, anchor="w")
        self.lbl_conn.pack(side="left", padx=(12, 8))

        # 赛题限「每项测量 ≤2 s」：把耗时单独做成一个变色徽标，
        # 不要混在状态栏那串字里（现场一眼就能看出还剩多少余量）。
        self.lbl_time = tk.Label(self.top_mid, text="", font=("", FS_BTN, "bold"),
                                 bg=BG, fg=MUTED, anchor="w")
        self.lbl_time.pack(side="left", padx=(0, 10))

        # ② 波形 + 频谱
        mid = tk.Frame(r, bg=BG)
        mid.grid(row=1, column=0, sticky="nsew", padx=8, pady=4)
        mid.rowconfigure(0, weight=1)
        mid.columnconfigure(0, weight=1)

        self.fig = Figure(figsize=(10, 3.4), dpi=100, facecolor=BG)
        self.ax_w, self.ax_s = self.fig.subplots(
            1, 2, gridspec_kw={"width_ratios": [1.4, 1.0]})
        self.fig.subplots_adjust(left=0.055, right=0.985, top=0.93, bottom=0.16, wspace=0.16)
        self._style_axes(self.ax_w, "波形")
        self._style_axes(self.ax_s, "频谱")
        self.canvas = FigureCanvasTkAgg(self.fig, master=mid)
        self.canvas.get_tk_widget().configure(bg=BG, highlightthickness=0,
                                             takefocus=0)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        self.canvas.draw()

        # ③ 当前激励（信号源）参数：装载后主界面要一直看得到
        info = tk.Frame(r, bg=BG)
        info.grid(row=2, column=0, sticky="ew", padx=10)
        self.lbl_excite = tk.Label(info, text="激励：未装载", bg=BG, fg=MUTED,
                                   anchor="w", font=("", FS_SMALL))
        self.lbl_excite.pack(side="left")
        self.lbl_comp = tk.Label(info, text="", bg=BG, fg=C_SPEC, anchor="e",
                                 font=("", FS_SMALL))
        self.lbl_comp.pack(side="right")

        # ④ 大字号读数
        strip = tk.Frame(r, bg=BG)
        strip.grid(row=3, column=0, sticky="ew", padx=8, pady=4)
        for i in range(5):
            strip.columnconfigure(i, weight=1, uniform="ro")
        self.ro_upp = Readout(strip, "Upp", "-", C_WAVE)
        self.ro_upp.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        self.ro_urms = Readout(strip, "Urms", "-", C_WAVE)
        self.ro_urms.grid(row=0, column=1, sticky="ew", padx=(0, 6))
        self.ro_f1 = Readout(strip, "基频 f1", "-", C_SPEC)
        self.ro_f1.grid(row=0, column=2, sticky="ew", padx=(0, 6))
        self.ro_a1 = Readout(strip, "基波幅值", "-", C_SPEC)
        self.ro_a1.grid(row=0, column=3, sticky="ew", padx=(0, 6))
        self.ro_thd = Readout(strip, "THD", "-", FG)
        self.ro_thd.grid(row=0, column=4, sticky="ew")

        # ⑤ 控制条 1：采集 + 周期 + 档位
        ctl1 = tk.Frame(r, bg=BG)
        ctl1.grid(row=4, column=0, sticky="ew", padx=8, pady=(4, 2))

        self.btn_cap = BigButton(ctl1, "采  集", self.on_capture, font_size=18, padx=22, pady=12)
        self.btn_cap.pack(side="left")
        tk.Frame(ctl1, bg=BG, width=10).pack(side="left")

        tk.Label(ctl1, text="周期", font=("", FS_SMALL), bg=BG, fg=MUTED).pack(side="left", padx=(4, 4))
        self.seg_per = SegBar(ctl1, [(1, "1 周期"), (3, "3 周期")],
                              self.on_periods, padx=10, pady=10)
        self.seg_per.set(3)
        self.seg_per.pack(side="left")
        tk.Frame(ctl1, bg=BG, width=10).pack(side="left")

        tk.Label(ctl1, text="档位", font=("", FS_SMALL), bg=BG, fg=MUTED).pack(side="left", padx=(4, 4))
        self.seg_gear = SegBar(ctl1, [("auto", "自动"), (0, "0"), (1, "1"), (2, "2"), (3, "3")],
                               self.on_gear, padx=10, pady=10)
        self.seg_gear.set(0)
        self.seg_gear.pack(side="left")

        # ⑥ 控制条 2：三路交替 / 去 u3 / 状态
        ctl2 = tk.Frame(r, bg=BG)
        ctl2.grid(row=5, column=0, sticky="ew", padx=8, pady=(2, 8))

        self.btn_inter3 = BigButton(ctl2, "三路交替 关", self.on_toggle_inter3)
        self.btn_inter3.pack(side="left")
        self.btn_u3 = BigButton(ctl2, "去 u3 关", self.on_toggle_u3)
        self.btn_u3.pack(side="left", padx=6)

        # 谱宽：题1 的成分都在 200 kHz 以内、题2/3 在 500 kHz 以内。
        # 判分时把横轴卡到对应带宽，谱线才不会被压成一根；默认 500k。
        self.btn_span = BigButton(ctl2, "谱宽 500k", self.on_toggle_span, padx=10)
        self.btn_span.pack(side="left")

        # 输入比例固定 ×0.5（信号源按 50Ω 设定、板子高阻时看到 2 倍），不再做开关
        self.btn_smooth = BigButton(ctl2, "平滑 开", self.on_toggle_smooth,
                                    padx=10, font_size=FS_BTN - 2)
        self.btn_smooth.pack(side="left", padx=(6, 0))

        # 波形类型：自动判型在 f1 > 173 kHz 时不可靠（3 次谐波被 4 阶板滤掉），
        # 这时手动指定类型，用基波幅度解析换算 Upp，精度恢复。
        self.btn_wave = BigButton(ctl2, self._wave_label(), self.on_toggle_wave,
                                  padx=8, font_size=FS_BTN - 2)
        self.btn_wave.pack(side="left", padx=(6, 0))

        # 阶梯校准：21 个频点逐点采，建幅频表 + 标定增益，写 eq.json / config.json
        self.btn_sweep = BigButton(ctl2, "阶梯校准", self.on_sweep,
                                   padx=10, font_size=FS_BTN - 2)
        self.btn_sweep.pack(side="left", padx=(6, 0))

        # 激励：由本机把「基波 + 1~2 个谐波」合成波灌进信号源，
        # 同时显示「预设 / 源回读 / 板子实测」三列——老师不用碰信号源面板。
        self.btn_excite = BigButton(ctl2, "激励", self.on_excite,
                                    padx=10, font_size=FS_BTN - 2)
        self.btn_excite.pack(side="left", padx=(6, 0))

        # 状态文字移到顶栏中段（底部这行 7 个按钮已经占满）
        self.lbl_status = tk.Label(self.top_mid, text="就绪", font=("", FS_SMALL),
                                   bg=BG, fg=MUTED, anchor="w", justify="left")
        self.lbl_status.pack(side="left", fill="x", expand=True, padx=(0, 8))
        # 状态文字经常比剩余宽度长，Tk 的 Label 不会自动省略、会直接画到窗口外，
        # 所以按实际像素宽截断（<Configure> 时重算，窗口尺寸变了也跟得上）。
        try:
            self._f_status = tkfont.Font(font=self.lbl_status.cget("font"))
        except Exception:
            self._f_status = None
        self._status_full = "就绪"
        self.lbl_status.bind("<Configure>",
                             lambda e: self.lbl_status.configure(
                                 text=self._fit_status(self._status_full)))

    def _style_axes(self, ax, title):
        ax.set_facecolor(PANEL)
        ax.set_title(title, color=FG, fontsize=12, pad=6)
        ax.tick_params(colors=MUTED, labelsize=9)
        for s in ax.spines.values():
            s.set_color(GRID)
        ax.grid(True, color=GRID, lw=0.5, alpha=0.7)

    # ---------------- 状态 ----------------
    def _status(self, text, color=MUTED):
        self._status_full = text
        self.lbl_status.configure(text=self._fit_status(text), fg=color)

    def _fit_status(self, text):
        """按当前可用像素宽度截断状态文字，超出部分用省略号（Tk 自己不会省略）。"""
        avail = self.lbl_status.winfo_width()
        f = getattr(self, "_f_status", None)
        if f is None or avail <= 40:
            return text
        try:
            if f.measure(text) <= avail - 6:
                return text
            s = text
            while s and f.measure(s + "…") > avail - 6:
                s = s[:-1]
            return (s + "…") if s else ""
        except Exception:
            return text

    def _need_conn(self):
        if self.connected and self.client is not None:
            return True
        self._status("未连接，先点左上角「连接」", C_MARK)
        self.btn_conn.flash(C_MARK)
        return False

    # ---------------- 按钮回调 ----------------
    def _start_logfwd(self):
        """后台线程：把监测程序的心跳/启动记录转发到板载掉电日志。

        心跳每 10 秒一行，全转发两个多小时就把 256 个槽位撑满，所以只在
        **异常或状态变化**时才写板子；另外每 30 分钟补一条「仍在运行」的活体
        标记 —— 这样板子上最后一条记录的"年龄"本身就是判据：很新说明刚死，
        几小时前说明死之前一直正常。
        """
        if getattr(self, "_logfwd_t", None) is not None:
            return

        def loop():
            st = {}
            while not getattr(self, "_closing", False):
                for path, key in ((FWD_LOG, "hb"), (FWD_CRASH, "crash")):
                    try:
                        self._fwd_tail(path, key, st)
                    except Exception:
                        pass
                time.sleep(2.0)

        self._logfwd_t = threading.Thread(target=loop, daemon=True)
        self._logfwd_t.start()

    def _fwd_tail(self, path, key, st):
        """tail 一个文件，把该转发的新行写进板子；位置存在 st[key]。"""
        if not os.path.exists(path):
            return
        pos = st.get(key)
        size = os.path.getsize(path)
        if pos is None or size < pos:          # 首次 / 被轮转截断
            st[key] = size
            return
        if size == pos:
            return
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            f.seek(pos)
            for line in f:
                line = line.rstrip("\r\n")
                got = self._fwd_reason(line, st, key)
                if got is not None:
                    self._log_board(got[0], got[1][:110])
            st[key] = f.tell()

    @staticmethod
    def _fwd_reason(line, st, key):
        """判断一行要不要进板子，要就返回 (level, text)，否则 None。"""
        line = (line or "").strip()
        if not line:
            return None
        if key == "crash":                      # 开机记录，一行一个 boot
            return (1, "开机 " + line)

        if line.startswith("==="):              # pi-watch 的启动横幅
            return (1, line)
        if "|" not in line:                     # 非心跳行（错误堆栈等）
            return (_guess_level(line), line)

        f = [x.strip() for x in line.split("|")]
        bad = []
        thr = f[1] if len(f) > 1 else "0x0"
        if thr not in ("0x0", "n/a"):
            bad.append("节流" + thr)
        for x in f:
            if x.startswith("lcrit=") and x[6:7] not in ("0", "", "n"):
                bad.append("欠压锁存")
            elif x.startswith("disk="):
                ms = x[5:].replace("ms", "")
                if "STALL" in x:
                    bad.append("根盘卡死")
                elif ms.isdigit() and int(ms) > 500:
                    bad.append("根盘慢%sms" % ms)
            elif x.startswith("usb_err=") and "(+" in x:
                d = x.split("(+")[1].rstrip(")")
                if d.isdigit() and int(d) > 0:
                    bad.append("USB异常+%s" % d)
            elif x.startswith("fserr=") and "(+" in x:
                d = x.split("(+")[1].rstrip(")")
                if d.isdigit() and int(d) > 0:
                    bad.append("文件系统错+%s" % d)
            elif x.endswith("C") and x[:-1].replace(".", "", 1).isdigit():
                if float(x[:-1]) > 78.0:
                    bad.append("温度" + x)
            elif x.startswith("core="):
                try:
                    v = float(x[5:].rstrip("V"))
                    if v < 0.80 or v > 1.06:
                        bad.append("核心电压%.2fV" % v)
                except ValueError:
                    pass
            elif x.startswith("load1="):
                try:
                    if float(x[6:]) > 4.0:
                        bad.append("负载" + x[6:])
                except ValueError:
                    pass
            elif x.startswith("wlan=") and x[5:] not in ("up", "unknown", "n/a"):
                bad.append("WiFi" + x[5:])
            elif x.startswith("eth=") and x[4:] not in ("up", "unknown", "none", "n/a"):
                bad.append("网口" + x[4:])

        if bad:
            # 原因必须放最前面：固件只存 120 字节，整行有 190+，写在末尾会被
            # 截掉，那就等于什么都没说。
            head = f[0] if f else ""
            rest = " ".join(x.replace(" ", "") for x in f[2:])
            return (2, "%s <%s> %s" % (head, "、".join(bad), rest))

        now = time.time()
        if now - float(st.get("alive", 0.0)) > 1800.0:
            st["alive"] = now
            return (1, "心跳正常 " + line)
        return None

    def _log_board(self, level, text):
        """把一条日志写进板载掉电日志（复用当前连接，不另开 socket）。

        级别：0=DEBUG 1=INFO 2=WARN 3=ERR。没连上就静默跳过 —— 记录日志
        本身不能变成新的故障源。
        """
        c = getattr(self, "client", None)
        if c is None:
            return
        try:
            txt = str(text).replace("\r", " ").replace("\n", " ")
            if len(txt) > 120:
                txt = txt[:119] + "~"
            c.log_put(int(level), txt)
        except Exception:
            pass

    def on_connect(self):
        if self.busy:
            return
        self.busy = True
        self._status("正在连接 %s:%d …" % (self.host, self.port), MUTED)
        self.root.update_idletasks()
        try:
            if self.client is not None:
                try:
                    self.client.close()
                except Exception:
                    pass
            c = lsa_proto.LsaClient(host=self.host, port=self.port, timeout=2.0)
            c.connect()
            h = c.ping()                      # 用 PING 确认协议与事务号
            self.client = c
            self.connected = True
            self._start_logfwd()
            self.lbl_conn.configure(text="已连接", fg=C_WAVE)
            param_restore(self)                  # 本地缺文件就从板子读回来
            # 底部状态栏只剩 ~20 个字符的宽度（左边挤了 6 个按钮），
            # 所以这里只报最关键的三项；host:port 放在顶栏的连接提示里。
            self._status("已连接 ver=%d %.2fM crc=%s" % (
                h.get("ver", 0), float(h.get("fs_hz", 0)) / 1e6,
                "OK" if h.get("crc_ok") else "BAD"), C_WAVE)
            self.btn_conn.flash(BTN_ON, 200)
        except Exception as e:
            self.client = None
            self.connected = False
            self.lbl_conn.configure(text="未连接", fg=C_MARK)
        try:
            self.lbl_time.configure(text="", fg=MUTED)
        except (tk.TclError, AttributeError):
            pass
            self._status("连接失败：%s" % e, C_MARK)
            self._log_board(3, "连接失败 %s:%s %s" % (host, port, e))
        finally:
            self.busy = False

    def _reconnect(self):
        """丢弃旧连接并重建（板子复位后会留下半开连接，必须显式重建）。"""
        if self.client is not None:
            try:
                self.client.close()
            except Exception:
                pass
        self.client = None
        self.connected = False
        c = lsa_proto.LsaClient(host=self.host, port=self.port, timeout=2.0)
        c.connect()
        c.ping()
        self.client = c
        self.connected = True
        self.lbl_conn.configure(text="已连接", fg=C_WAVE)
        return c

    def on_selftest(self):
        if not self._need_conn() or self.busy:
            return
        self.busy = True
        try:
            h = self.client.selftest()
            ok = bool(h.get("flags", 0) & lsa_proto.LSDA_FLAG_INTER3_OK)
            if ok:
                self._status("自检通过：三路交替可用（7.083 MSPS）", C_WAVE)
                self.btn_test.flash(BTN_ON, 200)
            else:
                self._status("自检失败：三路交替不可用，请用单路模式", C_MARK)
                self.btn_test.flash(C_MARK, 200)
        except Exception as e:
            self._status("自检出错：%s" % e, C_MARK)
        finally:
            self.busy = False

    def on_periods(self, key):
        self.periods = int(key)
        if self.v is not None:
            self._redraw_wave()

    def on_gear(self, key):
        self.gear_mode = key

    def on_toggle_inter3(self):
        # 打开前先提示一次：三路交替的通道时序还没校准，镜像可能比基波还强。
        # 保留这个功能（板子支持、也允许用户自己判断），但必须让操作者知道风险。
        if not self.inter3:
            try:
                ok = messagebox.askyesno(
                    "三路交替",
                    "三路交替用 ADC1/ADC2/ADC5 以 3 倍速率轮采。\n\n"
                    "当前通道时序未校准：实测镜像分量可能比基波还强，\n"
                    "Upp/分量的读数不可信（仅供观察）。\n\n"
                    "正式测量请用单路。仍要打开吗？",
                    icon="warning", default="no")
            except Exception:
                ok = True
            if not ok:
                return
        self.inter3 = not self.inter3
        self.btn_inter3.configure(
            text="三路交替 开" if self.inter3 else "三路交替 关",
            bg=BTN_ON if self.inter3 else BTN,
            fg="white" if self.inter3 else FG)
        if self.inter3:
            self._status("三路交替已开（时序未校准，读数仅供观察）", C_MARK)
        else:
            self._status("已切回单路采集", MUTED)

    def on_toggle_u3(self):
        self.remove_u3 = not self.remove_u3
        self.btn_u3.configure(
            text="去 u3 开" if self.remove_u3 else "去 u3 关",
            bg=BTN_ON if self.remove_u3 else BTN,
            fg="white" if self.remove_u3 else FG)
        if self.v is not None:
            self._refresh()

    def on_toggle_eq(self):
        self.use_eq = not self.use_eq
        self.btn_eq.configure(
            text="补偿 开" if self.use_eq else "补偿 关",
            bg=BTN_ON if self.use_eq else BTN,
            fg="white" if self.use_eq else FG)
        if self.v is not None:
            self._refresh()

    def _gear_now(self):
        """当前档位（自动模式按 0 档标定）。"""
        return 0 if self.gear_mode == "auto" else int(self.gear_mode)

    # ---- 阶梯校准 ----
    def on_sweep(self):
        """打开阶梯校准：21 个频点逐点采 → 建幅频表 + 标定增益。"""
        if not self._need_conn() or self.busy:
            return
        self._sweep_active = True
        try:
            dlg = StepCalib(self)

            def _clear(_e=None):
                self._sweep_active = False
            dlg.bind("<Destroy>", _clear, add="+")
        except Exception as e:
            self._sweep_active = False
            self._status("打不开阶梯校准：%s" % e, C_MARK)
            traceback.print_exc()

    def _on_escape(self, _e=None):
        """Esc 不直接退出（键盘误触代价太大），只提示；退出走「退出」按钮。"""
        try:
            self._status("按「退出」按钮关闭程序（Esc 不退出）", MUTED)
        except tk.TclError:
            pass

    def _restore_excite(self):
        d = load_excite()
        if d:
            self.set_excite(excite_text(d, stale=True))

    def set_excite(self, text):
        """主界面那行「激励」：装载后一直显示当前信号源参数。"""
        try:
            self.lbl_excite.configure(
                text=("激励 %s" % text) if text else "激励：未装载",
                fg=(FG if text else MUTED))
        except tk.TclError:
            pass

    def set_comp(self, text):
        """主界面右侧那行「分量」：每次采集后更新实测的各次谐波峰值。"""
        try:
            self.lbl_comp.configure(text=text or "")
        except tk.TclError:
            pass

    def on_excite(self):
        """打开「激励」面板：填参数 → 灌进信号源 → 对照预设/回读/量化。"""
        if self.busy:
            return
        try:
            SourcePanel(self)
        except Exception as e:
            self._status("打不开激励面板：%s" % e, C_MARK)
            traceback.print_exc()

    # ---- 波形平滑（只影响画图）----
    # ---- 谱宽（200k / 500k / 600k）----
    _SPAN_CYCLE = (500e3, 200e3, 600e3)

    def on_toggle_span(self):
        """频谱横轴上限。题1 用 200k、题2/3 用 500k，看带外杂散用 600k。"""
        cur = getattr(self, "spec_span", 500e3)
        i = 0
        for j, s in enumerate(self._SPAN_CYCLE):
            if abs(s - cur) < 1.0:
                i = j
                break
        self.spec_span = self._SPAN_CYCLE[(i + 1) % len(self._SPAN_CYCLE)]
        self.btn_span.configure(text="谱宽 %.0fk" % (self.spec_span / 1000.0))
        if self.v is not None:
            self._redraw_spec()

    def on_toggle_smooth(self):
        """开：整数倍带限插值，曲线顺滑；关：只把真实采样点两两直连。

        采样点本身不会变，Upp/Urms/频谱一律用原始点算。"""
        self.smooth = not getattr(self, "smooth", True)
        self.btn_smooth.configure(
            text="平滑 开" if self.smooth else "平滑 关",
            bg=BTN_ON if self.smooth else BTN,
            fg="white" if self.smooth else FG)
        if self.v is not None:
            self._redraw_wave()

    # ---- 波形类型（自动 / 正弦 / 方波 / 三角 / 半波）----
    _WAVE_CYCLE = ("auto", "sine", "square", "triangle", "half")
    _WAVE_TEXT = {"auto": "波形 自动", "sine": "波形 正弦", "square": "波形 方波",
                  "triangle": "波形 三角", "half": "波形 半波"}

    def _wave_label(self):
        return self._WAVE_TEXT.get(getattr(self, "wave_mode", "auto"), "波形 自动")

    def on_toggle_wave(self):
        """自动判型在 f1 超过 ~173 kHz 时失效（3 次谐波 3·f1 已出带），
        这时手动指定类型：Upp 用基波幅度解析换算，不受谐波被滤掉的影响。"""
        cur = getattr(self, "wave_mode", "auto")
        i = self._WAVE_CYCLE.index(cur) if cur in self._WAVE_CYCLE else 0
        self.wave_mode = self._WAVE_CYCLE[(i + 1) % len(self._WAVE_CYCLE)]
        manual = self.wave_mode != "auto"
        self.btn_wave.configure(
            text=self._wave_label(),
            bg=BTN_ON if manual else BTN,
            fg="white" if manual else FG)
        if self.v is not None:
            self._refresh()

    def _keepalive(self):
        try:
            if (self.connected and self.client is not None and not self.busy
                    and not getattr(self, "_sweep_active", False)):
                self.client.ping()
        except Exception:
            self.connected = False
            self.lbl_conn.configure(text="已断开", fg=C_MARK)
            try:
                self._reconnect()
                self.lbl_conn.configure(text="已连接", fg=C_WAVE)
            except Exception:
                pass
        finally:
            self.root.after(8000, self._keepalive)

    def _update_calib_btn(self):
        """「校准」按钮已下线（直流偏置不影响 Upp/Urms/频谱，赛题也不要求测直流），
        保留本方法只为兼容旧调用：没有按钮时什么都不做。"""
        btn = getattr(self, "btn_calib", None)
        if btn is None:
            return
        ok = bool(self.calib.get("offsets_v")) if self.calib else False
        btn.configure(text="校准 ✓" if ok else "校准",
                      bg=BTN_ON if ok else BTN,
                      fg="white" if ok else FG)

    def _fix_offsets(self, v, hdr):
        """
        按标定表扣掉每路直流偏置。
        没标定时退回原来的"通道均值对齐"兜底（只能消掉通道间差，消不掉共同偏置）。
        """
        gain = int(hdr.get("gain_milli", 3925) or 3925)
        if self.inter3:
            if any(calib_mod.offset_of(self.calib, c, gain) for c in range(3)):
                return calib_mod.apply(v, self.calib, "inter3", 0, gain)
            return dsp.align_channels(v, 3)
        if calib_mod.offset_of(self.calib, 0, gain):
            return calib_mod.apply(v, self.calib, "single", 0, gain)
        return v

    def on_calib(self):
        """零输入标定（已从界面下线，保留入口便于命令行/将来恢复）。

        为什么下线：直流偏置只影响 Udc 显示，Upp 用重建波形的 max−min、Urms 用交流分量、
        频谱不看直流，三者都与偏置无关；而它曾经在"输入端还接着信号"时被误点，
        存下一份 +0.764 V 的假标定（等于满量程），把读数整体拉偏 700 mV。
        """
        if not self._need_conn() or self.busy:
            return
        if not messagebox.askyesno(
                "零输入标定",
                "标定前请确认：\n\n"
                "① 输入端短路（最好是短路；空载也可以，但偏置会略有差别）\n"
                "② 全系统上电、预热 1 分钟以上\n"
                "③ 当前「三路交替」开关与比赛时要一致\n\n"
                "点「是」开始采集并写入 calib.json。"):
            return
        self.busy = True
        self.btn_calib.flash(BTN_ON, 200)
        self.root.update_idletasks()
        try:
            g = self._gear_now()
            hdr, raw = self.client.capture(gear=g, inter3=self.inter3, ch=0,
                                           n=None if self.inter3 else 8192, timeout=4.0)
            codes = np.asarray(lsa_proto.raw_to_int16(raw), dtype=np.int16)
            gain = int(hdr.get("gain_milli", 3925) or 3925)
            v = dsp.codes_to_volts(codes, gain, offset_code=0)
            offs = calib_mod.offsets_from(v, "inter3" if self.inter3 else "single", gain)
            # 防呆：偏置不可能超过满量程的一半（±764 mV 的 40% 已经很离谱）。
            # 输入端还接着信号源、或 ADC 已经饱和时点"校准"，会存下一份把读数
            # 整体偏掉的假标定（曾经出现过三路全是 +0.764331 V 的情况）。
            bad = [o for o in (offs.values() if isinstance(offs, dict) else offs)
                   if abs(float(o)) > 0.30]
            if bad:
                self.busy = False
                self._status("标定被拒绝：偏置 %.0f mV 明显异常，请先断开输入信号再标定"
                             % (max(abs(float(o)) for o in bad) * 1e3), C_MARK)
                messagebox.showerror(
                    "标定被拒绝",
                    "测到的直流偏置是 %.0f mV，远超正常范围（正常应在几十 mV 内）。\n\n"
                    "最常见原因：输入端还连着信号源，或 ADC 已经饱和。\n"
                    "请把输入短路（或拔掉信号源）后重新标定。"
                    % (max(abs(float(o)) for o in bad) * 1e3))
                return
            self.calib = calib_mod.build(self.calib, self.inter3, gain, offs,
                                         source="shorted_or_floating")
            path = calib_mod.save(self.calib)
            self._update_calib_btn()
            self._status("已标定并写入 %s：%s" % (path, calib_mod.summary(self.calib)),
                         C_WAVE)
            if self.v is not None:
                # 立刻用新基线重画一次，方便判断效果
                self.v = self._fix_offsets(
                    dsp.codes_to_volts(np.asarray(lsa_proto.raw_to_int16(raw), dtype=np.int16),
                                       gain, offset_code=0), hdr)
                self.hdr, self.fs = hdr, float(hdr["fs_hz"])
                self._refresh()
        except Exception as e:
            self._status("标定失败：%s" % e, C_MARK)
            traceback.print_exc()
        finally:
            self.busy = False

    def on_capture(self):
        if not self._need_conn() or self.busy:
            return
        t_click = time.time()        # 赛题计时起点：手指按下的瞬间
        self.busy = True
        self.btn_cap.configure(bg=BTN_ON)
        self.root.update_idletasks()
        try:
            # 单路用 16384 点：EQ 两端各裁 0.45 ms 后仍有 ~13.2k 点，
            # Δf≈268 Hz，满足赛题「频率分辨率 500 Hz」的要求。
            n = None if self.inter3 else 16384
            if self.gear_mode == "auto":
                best = None
                for g in range(4):
                    try:
                        hdr, raw = self.client.capture(gear=g, inter3=self.inter3, ch=0,
                                                       n=n, timeout=4.0)
                    except Exception:
                        continue
                    if hdr["fs_hz"] * 0.5 < 700e3:
                        continue        # Nyquist 不够，f3 会折进带内，直接排除
                    codes = np.asarray(lsa_proto.raw_to_int16(raw), dtype=np.int16)
                    v = dsp.codes_to_volts(codes, hdr.get("gain_milli", 3925), offset_code=0)
                    v = self._fix_offsets(v, hdr)
                    score = dsp.out_of_band_ratio(v, hdr["fs_hz"])
                    cl = float(np.mean(np.abs(codes.astype(np.int32)) >= 2046))
                    if best is None or score < best[0]:
                        best = (score, g, hdr, v, cl)
                if best is None:
                    raise RuntimeError("四个档位都没有采到数据")
                _, g, hdr, v, clip_frac = best
            else:
                g = int(self.gear_mode)
                hdr, raw = self.client.capture(gear=g, inter3=self.inter3, ch=0,
                                               n=n, timeout=4.0)
                codes = np.asarray(lsa_proto.raw_to_int16(raw), dtype=np.int16)
                v = dsp.codes_to_volts(codes, hdr.get("gain_milli", 3925), offset_code=0)
                v = self._fix_offsets(v, hdr)
                clip_frac = float(np.mean(np.abs(codes.astype(np.int32)) >= 2046))

            self.hdr, self.v, self.fs = hdr, v, float(hdr["fs_hz"])
            self._codes = codes          # 保存与「直接分量法」都要用原始码
            self._refresh()

            # ── 稳定性复采 ────────────────────────────────────────────
            # 4 阶板第二级 Q=2.94，幅频特性随温度漂 1~2%：刚上电或刚换频点时
            # 第一帧会偏低。这里用同一条链路再采 1~2 帧，相邻两帧差 ≤0.3% 就停。
            # 每帧约 230 ms，最多 2 帧（+0.46 s），仍远低于赛题 2 s。
            stab_rel = 0.0
            try:
                u_prev = float((self.meas or {}).get("upp") or 0.0)
                adopted = False
                for _k in range(2):
                    if u_prev <= 0.0:
                        break
                    hdr2, raw2 = self.client.capture(gear=g, inter3=self.inter3, ch=0,
                                                     n=n, timeout=4.0)
                    fs2 = float(hdr2["fs_hz"])
                    c2 = np.asarray(lsa_proto.raw_to_int16(raw2), dtype=np.int16)
                    v_raw2 = dsp.codes_to_volts(c2, hdr2.get("gain_milli", 3925), offset_code=0)
                    v_raw2 = self._fix_offsets(v_raw2, hdr2)
                    # 注意：self.v 必须存**未经补偿/未乘比例**的原始电压，
                    # _refresh() 里才做 EQ 与 in_scale；这里只把处理后的副本用于比较。
                    v2 = v_raw2
                    if self.use_eq:
                        v2 = eq_mod.apply(v2, fs2, self.eq)
                        mg2 = eq_mod.trim_samples(fs2)
                    else:
                        mg2 = 0
                    v2 = v2 * float(getattr(self, "in_scale", 1.0))
                    m2 = dsp.measure(
                        v2, fs2, f_min=1000.0, margin=mg2,
                        force_wave=(None if getattr(self, "wave_mode", "auto") == "auto"
                                    else self.wave_mode))
                    u2 = float(m2["upp"])
                    stab_rel = abs(u2 - u_prev) / max(u_prev, 1e-12)
                    u_prev = u2
                    if stab_rel <= 0.005:
                        break        # 已经稳了（≤0.5%，250 mVpp 上 1.2 mV），保留第一帧即可
                    self.hdr, self.v, self.fs = hdr2, v_raw2, fs2
                    self._codes = c2
                    adopted = True
                if adopted:
                    self._refresh()
            except Exception:
                pass             # 复采失败不影响主流程，第一帧照样出数
            tips = ""
            m = self.meas or {}
            if clip_frac > 2e-4:
                tips += "  ⚠削顶%.1f%%(读数不可信)" % (clip_frac * 100.0)
            if m.get("glitch_n", 0) > 0:
                tips = "  毛刺%d点(原%.0fmV)" % (
                    m["glitch_n"], m["upp_raw"] * 1000.0)
            # 只在"确实识别/指定了非正弦"时提示；正弦是默认假设，不占状态栏
            wk = m.get("wave_kind", "mixed")
            if wk in ("square", "triangle", "half") or m.get("wave_forced"):
                _nm = {"square": "方波", "triangle": "三角波", "half": "半波",
                       "sine": "正弦", "sine?": "正弦"}.get(wk, wk)
                tips += ("  已按[%s]换算" if m.get("wave_forced") else "  识别%s") % _nm
            oob_f, oob_a = getattr(self, "_oob", (0.0, 0.0))
            if oob_a > 1e-3:
                tips += "  带外%.3fMHz/%.0fmV" % (oob_f / 1e6, oob_a * 2000.0)
                if not self.remove_u3:
                    tips += "（建议开「去 u3」）"
            # 三路交替的 fs/3 镜像：只要镜像幅度接近基波，说明通道时序没对齐，
            # 这时 Upp/Urms 都不可信，必须退回单路。
            if self.inter3 and m.get("a1"):
                if oob_a > 0.5 * m["a1"]:
                    tips += ("  ⚠ 交织镜像 %.2f MHz / %.0f mVpp 过大（三路时序未校准），"
                             "读数不可信，请关「三路交替」用单路" % (oob_f / 1e6, oob_a * 2000.0))
            if self.fs * 0.5 < 700e3:
                tips += "  ⚠Nyquist仅%.0fkHz" % (self.fs * 0.5 / 1e3)
            if self.use_eq and m.get("f1"):
                tips += "  补偿%+.1fdB" % eq_mod.boost_at(m["f1"], self.eq)
            if stab_rel > 0.005:
                # 高频段的跳变来自模拟板 Stage2（Q=2.94，用掉 99% 的 GBW），
                # 软件改不了，只能提示用户重采或降低对高频端的期望。
                if float(m.get("f1") or 0.0) >= 350e3:
                    tips += "  ⚠在变%.1f%%(模拟板500k谐振点跳变)" % (stab_rel * 100.0)
                else:
                    tips += "  ⚠仍在变%.1f%%" % (stab_rel * 100.0)
            elif stab_rel > 0.0:
                tips += "  稳定%.2f%%" % (stab_rel * 100.0)
            # 显示"实际参与测量"的点数（EQ 会裁掉两端，Δf 以它为准）
            n_used = int(m.get("n") or hdr["n_this"])
            # 赛题限 2 s：记录"手指按下 → 读数出完"的耗时，并保留最坏值
            dt_ms = (time.time() - t_click) * 1000.0
            self._last_ms = dt_ms
            self._worst_ms = max(self._worst_ms, dt_ms)
            # 状态栏窄，基信息尽量短，把位置留给提示
            # 赛题限每项 ≤2 s：接近上限就变色，现场一眼能发现（含复采最多 ~1 s）
            over = dt_ms > 1800.0
            if over:
                tips += "  ⚠超时%.1fs" % (dt_ms / 1000.0)
            try:
                self.lbl_time.configure(
                    text="%.2f s" % (dt_ms / 1000.0),
                    fg=(C_MARK if dt_ms > 2000.0
                        else (C_SPEC if dt_ms > 1000.0 else C_WAVE)))
            except tk.TclError:
                pass
            self._status("%d档 %.2fM %.0fΔf %.0fms%s" % (
                g, self.fs / 1e6, self.fs / max(1, n_used), dt_ms, tips),
                C_MARK if over else MUTED)
            self.btn_cap.flash(BTN_ON, 160)
        except Exception as e:
            # 板子复位/断线后 Pi 侧可能残留半开连接，导致后续全部超时。
            # 顺序：清残留字节 -> 重连 -> 自动重试一次；仍失败才让用户看错误。
            try:
                self.client.drain()
            except Exception:
                pass
            again = not getattr(self, "_retry_once", False)
            if again:
                self._retry_once = True
                try:
                    self._reconnect()
                    self._status("采集失败（%s），已重连，自动重试…" % e, C_SPEC)
                    self.root.update_idletasks()
                except Exception as e2:
                    again = False
                    self._status("采集失败：%s  |  重连也失败：%s" % (e, e2), C_MARK)
                    self._log_board(3, "采集失败 %s" % e)
            else:
                self._status("采集失败：%s（已重试过一次）" % e, C_MARK)
                self._log_board(3, "采集失败 %s" % e)
            if not again:
                traceback.print_exc()
        finally:
            self.btn_cap.configure(bg=BTN)
            self.busy = False
            if getattr(self, "_retry_once", False):
                self._retry_once = False
                self.root.after(60, self.on_capture)

    def on_save(self):
        """一键打包：波形 CSV + 截图 PNG + 结果摘要 TXT。

        裁判直接记录数据时要用这个，所以这里把「当时到底测的是什么」
        全部落盘：原始码、换算电压、测量结果、补偿/标定版本、耗时。
        """
        if self.v is None:
            self._status("还没有数据可保存", C_SPEC)
            return
        import datetime
        import os
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "caps")
        d = os.path.join(base, "cap_%s" % ts)
        try:
            os.makedirs(d, exist_ok=True)
            m = self.meas or {}
            fs = float(self.fs or 0.0)
            codes = getattr(self, "_codes", None)
            # ① 波形 CSV：时间 / 原始码 / 电压(mV)
            wav = os.path.join(d, "wave.csv")
            with open(wav, "w", encoding="utf-8") as fp:
                fp.write("t_s,code,v_mv\n")
                if codes is not None and len(codes) == self.v.size:
                    for i in range(self.v.size):
                        fp.write("%.9f,%d,%.4f\n" % (i / fs, int(codes[i]), self.v[i] * 1000.0))
                else:
                    for i in range(self.v.size):
                        fp.write("%.9f,,%.4f\n" % (i / fs, self.v[i] * 1000.0))
            # ② 图形 PNG
            png = os.path.join(d, "screen.png")
            try:
                self.fig.savefig(png, dpi=100, facecolor=BG)
            except Exception:
                pass
            # ③ 结果摘要
            txt = os.path.join(d, "result.txt")
            hdr = self.hdr or {}
            with open(txt, "w", encoding="utf-8") as fp:
                fp.write("周期信号谐波分析装置  测量记录\n")
                fp.write("时间      %s\n" % datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
                fp.write("板卡      %s:%d\n" % (self.host, self.port))
                fp.write("---- 采集 ----\n")
                fp.write("档位      %s\n" % str(self.gear_mode))
                fp.write("模式      %s\n" % ("三路交替" if self.inter3 else "单路"))
                fp.write("采样率    %.6f MSPS    实际 fs=%d Hz\n" % (fs / 1e6, int(hdr.get("fs_hz", fs))))
                fp.write("点数      n_this=%s  n_used=%s\n" % (hdr.get("n_this"), m.get("n")))
                fp.write("分辨率    df=%.1f Hz\n" % (m.get("df", 0.0)))
                fp.write("增益      gain_milli=%s\n" % hdr.get("gain_milli"))
                fp.write("---- 结果 ----\n")
                fp.write("Upp       %.3f mV\n" % (m.get("upp", 0.0) * 1000.0))
                fp.write("Urms      %.3f mV\n" % (m.get("urms", 0.0) * 1000.0))
                fp.write("Udc       %.3f mV\n" % (m.get("udc", 0.0) * 1000.0))
                fp.write("f1        %.2f Hz\n" % m.get("f1", 0.0))
                fp.write("基波幅度  %.3f mV\n" % (m.get("a1", 0.0) * 1000.0))
                fp.write("波形类型  %s\n" % m.get("wave_kind", "mixed"))
                fp.write("估计方法  %s (K=%s, 残差=%s)\n" % (
                    m.get("method"), m.get("fit_k"), m.get("fit_resid")))
                fp.write("毛刺抑制  %s 点\n" % m.get("glitch_n"))
                fp.write("---- 各分量（输入端峰值）----\n")
                try:
                    if codes is not None:
                        comp = dsp.components_direct(
                            codes, fs, gain_total=(hdr.get("gain_milli", 3925) or 3925) / 1000.0)
                    else:
                        comp = dsp.components(self.v, fs)
                    for cf, ca in comp:
                        fp.write("%12.1f Hz   %9.3f mV\n" % (cf, ca * 1000.0))
                except Exception as e:
                    fp.write("（分量计算失败：%s）\n" % e)
                fp.write("---- 修正设置 ----\n")
                fp.write("幅频补偿  %s\n" % ("开" if self.use_eq else "关"))
                fp.write("补偿模型  %s\n" % eq_mod.summary(self.eq))
                fp.write("输入比例  in_scale=%.6f\n" % float(getattr(self, "in_scale", 1.0)))
                fp.write("去 u3     %s (LP_FC=%.0f kHz)\n" % (
                    "开" if self.remove_u3 else "关", dsp.LP_FC_HZ / 1000.0))
                fp.write("标定      %s\n" % (calib_mod.summary(self.calib) or "未标定"))
                fp.write("耗时      本次 %.0f ms  最坏 %.0f ms\n" % (
                    getattr(self, "_last_ms", 0.0), getattr(self, "_worst_ms", 0.0)))
            self._status("已保存到 caps/cap_%s/（wave.csv / screen.png / result.txt）" % ts, C_WAVE)
        except Exception as e:
            self._status("保存失败：%s" % e, C_MARK)
            traceback.print_exc()

    def _on_destroy(self, ev):
        """窗口销毁时务必关掉 TCP，否则板子会卡在 ESTABLISHED。"""
        if ev.widget is not self.root:
            return
        try:
            if self.client is not None:
                self.client.close()
                self.client = None
                self.connected = False
        except Exception:
            pass

    def on_quit(self):
        if messagebox.askyesno("退出", "确定退出上位机？"):
            try:
                if self.client is not None:
                    self.client.close()
            except Exception:
                pass
            self.root.destroy()

    # ---------------- 处理与重绘 ----------------
    def _refresh(self):
        if self.v is None:
            return
        v = self.v
        margin = 0
        if self.remove_u3:
            v = dsp.lowpass_fir(v, self.fs, dsp.LP_FC_HZ)
            margin = dsp.FIR_MARGIN
        if self.use_eq:
            # 板子模拟链路高频掉增益：把 1/|H(f)| 乘回去，顺带 5 kHz 高通压掉低频漂移
            v = eq_mod.apply(v, self.fs, self.eq)
            margin = 0                      # apply() 内部已经把过渡带裁掉了
        # 输入比例必须放在补偿**之后**：先把滤波器掉的增益补回来（得到 BNC 处真实电压），
        # 再折算成 50Ω 等效值。顺序反了会把补偿量也一起打对折。
        _s = float(getattr(self, "in_scale", 1.0))
        if _s != 1.0:
            v = v * _s

        m = dsp.measure(v, self.fs, f_min=1000.0, margin=margin,
                        force_wave=(None if getattr(self, "wave_mode", "auto") == "auto"
                                    else self.wave_mode))
        self.meas = m
        self._vcalc = v                     # 图上显示的必须是"参与测量的那一路"（去 u3 后）
        self._spec = m.get("spec")          # 复用 measure() 已经算好的频谱，省两次 FFT
        # 诊断：原始数据里带外最强的那根谱线是什么（交织失配 / 自激 / 杂散）
        try:
            self._oob = dsp.out_of_band_peak(self.v, self.fs)
        except Exception:
            self._oob = (0.0, 0.0)
        self.ro_upp.set("%.1f mV" % (m["upp"] * 1000))
        self.ro_urms.set("%.1f mV" % (m["urms"] * 1000))
        f1 = m["f1"]
        self.ro_f1.set(("%.1f kHz" % (f1 / 1000)) if f1 >= 1000 else ("%.0f Hz" % f1))
        self.ro_a1.set("%.1f mV" % (m["a1"] * 1000))
        self.ro_thd.set("%.2f %%" % self._thd(v, m["f1"]))
        self._update_comp(v, f1)

        self._redraw_wave()
        self._redraw_spec()

    def _update_comp(self, v, f1):
        """把实测的各次谐波峰值写进主界面右上角那行（按频率对齐，最多 5 根）。"""
        try:
            if not f1 or f1 <= 0:
                self.set_comp("")
                return
            kmax = int(min(20, max(1, (self.fs * 0.5) / f1)))
            hs = dsp.harmonic_amplitudes(v, self.fs, f1, count=kmax,
                                         spec=getattr(self, "_spec", None))
            a1 = (float(hs[0]) * 1000.0) if len(hs) else 0.0
            thr = max(0.5, 0.02 * a1)       # 噪声级谐波不占地方：0.5 mV 且 ≥基波 2%
            items = []
            for k in range(1, len(hs) + 1):
                a = float(hs[k - 1]) * 1000.0
                if k > 1 and a < thr:
                    continue
                items.append("%d×%.1f" % (k, a))
                if len(items) >= 4:
                    break
            self.set_comp(("分量 " + " ".join(items) + " mV") if items else "")
        except Exception:
            self.set_comp("")

    def _thd(self, v, f1):
        """总谐波失真（%），取 2..5 次谐波 / 基波。"""
        if not f1 or f1 <= 0:
            return 0.0
        try:
            hs = dsp.harmonic_amplitudes(v, self.fs, f1, count=5,
                                         spec=getattr(self, "_spec", None))
            if len(hs) < 2 or hs[0] <= 0:
                return 0.0
            return float(np.sqrt(sum(h * h for h in hs[1:])) / hs[0] * 100.0)
        except Exception:
            return 0.0

    @staticmethod
    def _nice_span(peak):
        """把半量程吸附到 1/2/5×10^k，保证量程稳定不跳。"""
        if peak <= 0:
            return 1.0
        e = math.floor(math.log10(peak))
        base = 10.0 ** e
        for m in (1.0, 1.5, 2.0, 2.5, 3.0, 5.0, 7.5, 10.0):
            if peak <= m * base:
                return m * base
        return 10.0 * base

    @staticmethod
    def _nice_top(top):
        """把 0..top 的上限吸附到 1/2/2.5/5×10^k。"""
        if top <= 0:
            return 1.0
        e = math.floor(math.log10(top))
        base = 10.0 ** e
        for m in (1.0, 2.0, 2.5, 5.0, 10.0):
            if top <= m * base:
                return m * base
        return 10.0 * base

    def _redraw_wave(self):
        ax = self.ax_w
        ax.clear()
        v, fs = self._cur_v(), self.fs
        f1 = self.meas["f1"] if self.meas else 0.0
        udc = float(np.mean(v)) * 1000.0
        interp_note = ""
        try:
            if f1 > 0:
                t, seg = dsp.slice_periods(v, fs, f1, count=self.periods, start_frac=0.1)
                t = t * 1000.0
                # 采样点不够密时（例如 500 kHz 只落到 8 点/周期），可以选画图用整数倍
                # 带限插值补点；关掉平滑则只把真实采样点两两直连。
                # 测量始终用原始采样点，曲线怎么画都不影响读数。
                if seg.size >= 6 and getattr(self, "smooth", True):
                    seg8 = dsp.upsample_periodic(seg, 8)
                    if seg8.size > seg.size:
                        # 时间轴必须按"每原始样本间隔 / 倍数"重建，
                        # 用总跨度平分会让末端漂移近一个采样点，插值曲线就和真实点错开。
                        dt = float(t[-1] - t[0]) / max(1, seg.size - 1)
                        t = t[0] + np.arange(seg8.size) * (dt / (seg8.size / float(seg.size)))
                        seg = seg8
                        interp_note = "  · 8×插值显示"
                y = seg * 1000.0
            else:
                t = np.arange(v.size) / fs * 1000.0
                y = v * 1000.0
        except Exception:
            t = np.arange(v.size) / fs * 1000.0
            y = v * 1000.0
        if y is None or y.size < 2:
            self._style_axes(ax, "波形")
            self.canvas.draw_idle()
            return

        yac = y - float(np.mean(y))           # 自动量程只看交流，直流偏置不占视窗
        peak = float(np.max(np.abs(yac)))
        if peak < 0.05:
            lim, note = 0.5, "无信号"
        else:
            lim, note = self._nice_span(peak * 1.12), ""

        self._style_axes(ax, "波形（%d 个完整周期）%s    Udc=%.1f mV%s"
                         % (self.periods, ("  " + note) if note else "", udc,
                            interp_note))
        ax.plot(t, yac, color=C_WAVE, lw=1.6)
        if interp_note:
            # 原始采样点也画出来，便于判断"真的有这么多点"还是被插值糊过
            try:
                t0, seg0 = dsp.slice_periods(v, fs, f1, count=self.periods, start_frac=0.1)
                ax.plot(t0 * 1000.0, (seg0 - np.mean(seg0)) * 1000.0, "o",
                        color=C_SPEC, ms=3.2, mfc="none", mew=1.0, alpha=0.85)
            except Exception:
                pass
        ax.set_xlim(float(t[0]), float(t[-1]))
        ax.set_ylim(-lim, lim)
        ax.set_xlabel("时间 (ms)", color=MUTED, fontsize=10)
        ax.set_ylabel("mV", color=MUTED, fontsize=10)
        self.canvas.draw_idle()

    def _cur_v(self):
        """当前参与测量的信号（去掉 u3 后与读数一致）。"""
        v = getattr(self, "_vcalc", None)
        return self.v if v is None else v

    def _redraw_spec(self):
        ax = self.ax_s
        ax.clear()
        v = self._cur_v()
        spec = getattr(self, "_spec", None)
        if spec is not None:
            f, amp = spec
        else:
            f, amp = dsp.spectrum(v, self.fs, win="bh", pad=4)
        df = float(f[1] - f[0]) if f.size > 1 else 0.0
        if df <= 0:
            self._style_axes(ax, "频谱（正频率轴）")
            self.canvas.draw_idle()
            return
        f1 = self.meas["f1"] if self.meas else 0.0
        # 横轴：按「谱宽」按钮，超出谱宽的基波才被动扩展（否则小信号的谱线挤在左边）
        span = float(getattr(self, "spec_span", 500e3))
        xhi = min(self.fs * 0.5, max(span, f1 * 1.3))
        # 带外有明显分量时把坐标轴延伸到那里，否则用户根本看不见它是谁。
        # 门限取 1.5 mV（峰值）：题3 的 u3 经模拟链后只剩 1~3 mVpp（0.5~1.5 mV 峰值），
        # 原来 5 mV 的门限会让它在图上彻底消失；上限仍卡在 Nyquist。
        oob_f, oob_a = getattr(self, "_oob", (0.0, 0.0))
        if oob_a > 1e-3:
            xhi = min(self.fs * 0.5, max(xhi, oob_f * 1.15))
        hi = min(int(xhi / df), f.size - 1)
        lo = max(1, int(1000.0 / df))
        if hi <= lo:
            self._style_axes(ax, "频谱（正频率轴）")
            self.canvas.draw_idle()
            return

        ypk = amp[lo:hi] * 1000.0
        top = float(np.max(ypk)) if ypk.size else 0.0
        yhi = 1.0 if top < 0.5 else self._nice_top(top * 1.15)

        self._style_axes(ax, "频谱（正频率轴）")
        # 底噪只画成一条淡线作参考；真正的分量用粗竖线（离散谱线）画出来，
        # 免得判分老师把补偿带来的基底隆起误认成第四、第五根分量。
        ax.plot(f[lo:hi] / 1000.0, ypk, color=C_SPEC, lw=0.8, alpha=0.40)
        ax.set_xlim(0.0, xhi / 1000.0)
        ax.set_ylim(0.0, yhi)
        try:
            comps = dsp.components(v, self.fs, f_min=1000.0, f_max=xhi, top=5, spec=spec)
            prev_f, dy = -1e9, 7
            for cf, ca in comps:
                if ca * 1000.0 > yhi * 0.05:
                    ax.vlines(cf / 1000.0, 0.0, ca * 1000.0, color=C_SPEC, lw=2.6)
                    # 题1(4)/题2(4)/题3(4) 要求"显示频谱各分量的幅值，误差 ≤5 mV"，
                    # 所以每根线必须同时标出**频率和幅度**，只标频率是判不了分的。
                    # 幅度按赛题说明 3 用正弦峰值（mV），不是峰峰值。
                    if cf - prev_f < 0.12 * xhi:
                        dy = 20 if dy == 7 else 7      # 挤在一起时上下错开
                    else:
                        dy = 7
                    prev_f = cf
                    ha = "right" if cf > 0.92 * xhi else "center"
                    ax.annotate("%.1f k\n%.1f mV" % (cf / 1000.0, ca * 1000.0),
                                (cf / 1000.0, ca * 1000.0),
                                color=C_MARK, fontsize=7.5, ha=ha,
                                va="bottom", linespacing=1.2,
                                xytext=(0, dy), textcoords="offset points")
        except Exception:
            pass
        ax.set_xlabel("频率 (kHz)", color=MUTED, fontsize=10)
        ax.set_ylabel("幅度 (mV)", color=MUTED, fontsize=10)
        self.canvas.draw_idle()


# ────────────────────────── 激励（自定义信号源） ──────────────────────────
# 信号源候选：按「源」按钮或 ↑/↓ 依次切换，不再自动探测
SRC_CHOICES = ((None, "未连接"), ("afg", "AFG-2225"),
               ("dg", "DG4202"), ("fake", "FAKE"))


def cycle_src(dlg, step=1):
    """在候选信号源之间循环切换并尝试打开。

    现场有两个源（AFG / DG），自动探测既慢又可能选错，所以改成显式切换：
    点一下按钮或按 ↓ 换下一个，切到「未连接」就断开。打开失败停在失败项上，
    再按一下就跳过它。
    """
    if getattr(dlg, "src", None) is not None:
        try:
            dlg.src.close()
        except Exception:
            pass
        dlg.src = None
    idx = (int(getattr(dlg, "_src_idx", 0)) + step) % len(SRC_CHOICES)
    dlg._src_idx = idx
    kind, label = SRC_CHOICES[idx]
    if kind is None:
        dlg.btn_src.configure(text="源: 未连接")
        dlg._src_note("未连接信号源。")
        return
    try:
        import lsa_src
        src = lsa_src.open_source(kind)
        idn = src.identify()
        try:
            src.output(True, ch=1)   # 输出没开的话采出来全是噪声
        except Exception:
            pass
        dlg.src = src
        dlg.btn_src.configure(text="源: %s" % label)
        dlg._src_note(idn)
    except Exception as exc:
        dlg.btn_src.configure(text="源: %s（失败）" % label)
        dlg._src_note("连不上 %s：%s" % (label, exc), bad=True)


class SourcePanel(tk.Toplevel):
    """把「基波 + 1~2 个谐波」的合成波灌进信号源，并三列对照。

    左：预设用例一键填表，或手动填参数；
    右：预设 / 源回读（含下发后的量化） / 板子实测 三列，底下给一句结论。

    存在的理由：现场老师不会调 AFG 面板。这里填几个数就能出题，
    而且"我要的"和"源里实际的"分开显示，误差来源一眼可分。
    """

    FS = FS_SMALL

    def __init__(self, app):
        tk.Toplevel.__init__(self, app.root)
        self.app = app
        self.src = None
        self.busy = False
        self.title("激励 · 自定义信号源")
        self.configure(bg=BG)
        self.geometry("1010x578+7+12")
        self.transient(app.root)
        self._q = queue.Queue()
        self._build()
        self.after(120, self._drain)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------- 界面 ----------
    def _build(self):
        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=14, pady=(10, 6))

        left = tk.Frame(body, bg=BG, width=300)
        left.pack(side="left", fill="y")
        left.pack_propagate(False)

        tk.Label(left, text="① 选一个预置", font=("", FS_SMALL, "bold"),
                 bg=BG, fg=FG, anchor="w").pack(fill="x")
        self.lb = tk.Listbox(left, height=6, bg=PANEL, fg=FG, bd=0,
                             font=("mono", FS_SMALL), selectbackground=BTN_ON,
                             highlightthickness=0, activestyle="none")
        for c in StepCalib.TEST_CASES:
            self.lb.insert("end", "%-6s %s" % (c[0], self._case_text(c)))
        self.lb.pack(fill="x", pady=(2, 8))
        self.lb.bind("<<ListboxSelect>>", self._on_pick)

        self.lbl_plan = tk.Label(left, text="", bg=BG, fg=MUTED, anchor="w",
                                 justify="left", font=("", FS_SMALL))
        self.lbl_plan.pack(fill="x", pady=(0, 6))

        tk.Label(left, text="② 或者自己填", font=("", FS_SMALL, "bold"),
                 bg=BG, fg=FG, anchor="w").pack(fill="x")

        grid = tk.Frame(left, bg=BG)
        grid.pack(fill="x", pady=(4, 0))

        def row(r, label, default, unit=""):
            tk.Label(grid, text=label, bg=BG, fg=MUTED, width=11, anchor="w",
                     font=("", FS_SMALL)).grid(row=r, column=0, sticky="w",
                                               pady=1)
            e = tk.Entry(grid, bg=PANEL, fg=FG, insertbackground=FG, bd=0,
                         font=("", FS_SMALL), width=9, justify="right")
            e.insert(0, default)
            e.grid(row=r, column=1, sticky="w", padx=(2, 4))
            if unit:
                tk.Label(grid, text=unit, bg=BG, fg=MUTED,
                         font=("", FS_SMALL)).grid(row=r, column=2, sticky="w")
            return e

        self.e_f1 = row(0, "基频", "100", "kHz")
        self.e_k1 = row(1, "1 次谐波", "1", "次")
        self.e_r1 = row(2, "  相对幅度", "1.00", "")
        self.e_k2 = row(3, "2 次谐波", "3", "次")
        self.e_r2 = row(4, "  相对幅度", "0.60", "")
        self.e_vpp = row(5, "总峰峰值", "250", "mV")
        self.e_jf = row(6, "干扰频率", "0", "MHz(0=无)")
        self.e_jv = row(7, "干扰幅度", "200", "mV")


        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="both", expand=True, padx=(12, 0))

        # 数字键盘（触摸屏用；键盘用户可以直接敲，所以不进 Tab 链）
        self._target = None
        kb = tk.Frame(right, bg=BG)
        kb.pack(side="bottom", fill="x", pady=(6, 0))
        for _c in range(4):
            kb.columnconfigure(_c, weight=1)
        keys = (("7", "8", "9", "⌫"), ("4", "5", "6", "清空"),
                ("1", "2", "3", "→"), (".", "0", "-", "←"))
        for _r, _row in enumerate(keys):
            for _c, _k in enumerate(_row):
                BigButton(kb, _k, lambda _kk=_k: self._key(_kk),
                          padx=4, pady=6, font_size=FS_BTN - 1,
                          color=(BTN if _k not in ("⌫", "清空") else "#30363d"),
                          takefocus=0).grid(row=_r, column=_c, sticky="nsew",
                                            padx=2, pady=2)

        self.txt = tk.Text(right, bg=PANEL, fg=FG, bd=0, font=("mono", FS_SMALL),
                           wrap="none", highlightthickness=0, takefocus=0)
        self.txt.pack(fill="both", expand=True)
        for tag, col in (("ok", C_WAVE), ("bad", C_MARK), ("dim", MUTED),
                         ("hdr", C_SPEC)):
            self.txt.tag_configure(tag, foreground=col)

        bar = tk.Frame(self, bg=BG)
        bar.pack(side="bottom", fill="x", padx=14, pady=(0, 10))
        self.btn_go = BigButton(bar, "装载", self._on_go, padx=22)
        self.btn_go.pack(side="left")
        self.btn_src = BigButton(bar, "源: 未连接", self._on_toggle_src,
                                 padx=14, font_size=FS_BTN - 2)
        self.btn_src.pack(side="left", padx=8)
        self.btn_src.bind("<Down>", lambda _e: self._on_toggle_src(+1))
        self.btn_src.bind("<Up>", lambda _e: self._on_toggle_src(-1))
        BigButton(bar, "关闭", self._on_close, color=BTN_RED, padx=16,
                  font_size=FS_BTN - 2).pack(side="right")
        self.lbl_msg = tk.Label(bar, text="", bg=BG, fg=MUTED, anchor="w",
                                font=("", FS_SMALL))
        self.lbl_msg.pack(side="left", fill="x", expand=True, padx=10)

        body.pack_forget()
        body.pack(fill="both", expand=True, padx=14, pady=(10, 6))

        self._render_hint()
        # 纯键盘：Esc 关闭；任意输入框里按 Enter 直接装载；↑/↓ 换信号源
        # 初始焦点放在预置列表（↑↓ 选预置，Tab 进输入框）
        self.bind("<Escape>", lambda _e: self._on_close())
        self._entries = (self.e_f1, self.e_k1, self.e_r1, self.e_k2, self.e_r2,
                         self.e_vpp, self.e_jf, self.e_jv)
        for _e in self._entries:
            _e.bind("<FocusIn>",
                    lambda _ev: setattr(self, "_target", _ev.widget))
            _e.bind("<Return>", lambda _ev: self._on_go())
            _e.bind("<KP_Enter>", lambda _ev: self._on_go())
            _e.bind("<Down>", self._focus_next)
            _e.bind("<Up>", self._focus_prev)
        self.lb.bind("<Return>", lambda _ev: self._on_pick())
        self.lb.bind("<KP_Enter>", lambda _ev: self._on_pick())
        self.after(80, self.lb.focus_set)

    def _key(self, k):
        """数字键盘：写进当前编辑框（默认基频），⌫ 退格，清空，←/→ 换框。"""
        e = self._target if self._target is not None else self.e_f1
        self._target = e
        try:
            e.focus_set()
        except Exception:
            pass
        if k in ("→", "←"):
            self._shift_target(+1 if k == "→" else -1)
            return
        cur = e.get()
        if k == "⌫":
            e.delete(0, "end")
            e.insert(0, cur[:-1])
        elif k == "清空":
            e.delete(0, "end")
        else:
            if k == "-" and cur.startswith("-"):
                e.delete(0, "end")
                e.insert(0, cur[1:])
            elif k == "-":
                e.delete(0, "end")
                e.insert(0, "-" + cur)
                return self._update_plan()
            else:
                e.delete(0, "end")
                e.insert(0, cur + k)
        self._update_plan()

    def _shift_target(self, step):
        """屏幕键盘的 ←/→：换编辑框并记住它（不能只靠 FocusIn，事件是异步的）。"""
        order = list(self._entries)
        e = self._target if self._target in order else order[0]
        n = order[(order.index(e) + step) % len(order)]
        self._target = n
        try:
            n.focus_set()
            n.icursor("end")
        except Exception:
            pass
        self._update_plan()

    def _focus_next(self, ev):
        """↓ 跳到下一个输入框（最后一个包回第一个）。"""
        return self._hop(ev.widget, +1)

    def _focus_prev(self, ev):
        return self._hop(ev.widget, -1)

    def _hop(self, w, step):
        order = [self.e_f1, self.e_k1, self.e_r1, self.e_k2, self.e_r2,
                 self.e_vpp, self.e_jf, self.e_jv]
        try:
            i = order.index(w)
        except ValueError:
            return "break"
        order[(i + step) % len(order)].focus_set()
        return "break"

    @staticmethod
    def _case_text(c):
        tones = "+".join("%dx%g" % (m, r) for m, r in c[2])
        jam = (" J%.2fM" % (c[4] * StepCalib.F0 / 1e6)) if c[4] else ""
        return "%s %gmV" % (tones + jam, c[3])

    def _render_hint(self):
        self.txt.delete("1.0", "end")
        self.txt.insert("end", "填好左边参数，或点上面一个预置，然后按「装载」。\n\n", "dim")
        self.txt.insert("end", "右边会给出三列：\n", "dim")
        self.txt.insert("end", "  预设 —— 你要的（按输入框算出来）\n", "dim")
        self.txt.insert("end", "  回读 —— 信号源里实际的（FREQ/AMP 回读；分量按量化后的缓冲反算）\n", "dim")
        self.txt.insert("end", "  这里只装载、不测量。要测板子请回主界面按「采集」。\n", "dim")

    # ---------- 表单 ----------
    def _val(self, entry, name, lo=None, hi=None, allow_blank=False):
        s = entry.get().strip()
        if not s:
            if allow_blank:
                return None
            raise ValueError("%s 不能为空" % name)
        try:
            v = float(s)
        except ValueError:
            raise ValueError("%s 不是数字：%s" % (name, s))
        if lo is not None and v < lo:
            raise ValueError("%s 太小（%g < %g）" % (name, v, lo))
        if hi is not None and v > hi:
            raise ValueError("%s 太大（%g > %g）" % (name, v, hi))
        return v

    def _form(self):
        f1 = self._val(self.e_f1, "基频", 9.0, 500.0) * 1e3
        vpp = self._val(self.e_vpp, "总峰峰值", 5.0, 300.0)
        k1 = int(self._val(self.e_k1, "1 次谐波次数", 1, 60))
        r1 = self._val(self.e_r1, "1 次相对幅度", 0.0, 1.0)
        k2 = self._val(self.e_k2, "2 次谐波次数", 0, 60, allow_blank=True)
        r2 = self._val(self.e_r2, "2 次相对幅度", 0.0, 1.0, allow_blank=True)
        jf = self._val(self.e_jf, "干扰频率", 0.0, 5.0, allow_blank=True) or 0.0
        jv = self._val(self.e_jv, "干扰幅度", 0.0, 500.0, allow_blank=True) or 0.0
        tones = [(k1, r1 if r1 > 0 else 1.0)]
        if k2 and r2 and int(k2) > 0 and r2 > 0:
            tones.append((int(k2), float(r2)))
        tones.sort()
        for k, _r in tones:
            if k * f1 > 500e3 + 1.0:
                raise ValueError("%d 次谐波 %.2f kHz 超过 500 kHz"
                                 % (k, k * f1 / 1e3))
        return dict(f1=f1, vpp=vpp, tones=tuple(tones),
                    jam_hz=jf * 1e6 if jf > 0 else 0.0, jam_mv=jv if jf > 0 else 0.0)

    def _plan(self, f1, tones):
        n = int(min(StepCalib.ARB_N, 120e6 / f1))
        n = max(64, n)
        if n * f1 > 120e6 + 1.0:
            raise ValueError("采样率 %.1f MSa/s 超过 120 MHz" % (n * f1 / 1e6))
        kmax = max(k for k, _ in tones)
        if n / float(kmax) < 8.0:
            raise ValueError("点数太少：%d 点装不下 %d 次谐波（每周期只剩 %.1f 点）"
                             % (n, kmax, n / float(kmax)))
        return n

    def _build_wave(self, f1, tones, vpp, jam_hz, jam_mv, n):
        t = np.arange(n, dtype=np.float64)
        w = np.zeros(n)
        for m, r in tones:
            w += float(r) * np.sin(2.0 * np.pi * float(m) * t / n)
        span = float(np.max(w) - np.min(w))
        ub = w * (vpp / span)
        ub_th = float(np.max(ub) - np.min(ub))
        if jam_hz:
            jam_m = float(jam_hz) / float(f1)
            if abs(jam_m - round(jam_m)) > 1e-6:
                raise ValueError("干扰 %.4f MHz 不是基频 %.4f kHz 的整数倍，"
                                 "整周期重放装不下" % (jam_hz / 1e6, f1 / 1e3))
            ub = ub + jam_mv * np.sin(2.0 * np.pi * jam_m * t / n)
        tot = float(np.max(ub) - np.min(ub))
        peak = float(np.max(np.abs(ub)))
        vals = ub / (2.0 * peak)
        comps = [(float(m) * f1, float(r) * vpp / span) for m, r in tones]
        return vals, comps, ub_th, tot

    # ---------- 交互 ----------
    def _on_pick(self, _ev=None):
        sel = self.lb.curselection()
        if not sel:
            return
        label, _item, tones, vpp, jam = StepCalib.TEST_CASES[int(sel[0])]
        k1, r1 = tones[0]
        k2, r2 = (tones[1] if len(tones) > 1 else (0, 0.0))
        vals = {
            self.e_f1: "%.6g" % (StepCalib.F0 / 1e3),
            self.e_k1: str(k1), self.e_r1: "%.2f" % r1,
            self.e_k2: (str(k2) if k2 else ""), self.e_r2: ("%.2f" % r2 if k2 else ""),
            self.e_vpp: "%.0f" % vpp,
            self.e_jf: ("%.4f" % (jam * StepCalib.F0 / 1e6)) if jam else "0",
            self.e_jv: "200",
        }
        for e, s in vals.items():
            e.delete(0, "end")
            e.insert(0, s)
        self.lbl_msg.configure(text="已套用 %s" % label, fg=MUTED)
        self._update_plan()

    def _update_plan(self):
        try:
            f = self._form()
            n = self._plan(f["f1"], f["tones"])
            _m0 = min(int(m) for m, _r in f["tones"])
            _note = ""
            if _m0 != 1:
                _note = ("\n实际基波 %.4g kHz（= %d×基础频率 %.4g kHz）"
                         % (f["f1"] * _m0 / 1e3, _m0, f["f1"] / 1e3))
            self.lbl_plan.configure(
                text="N = %d 点  ·  %.1f MSa/s\n分量数 %d  ·  干扰 %s%s"
                     % (n, n * f["f1"] / 1e6, len(f["tones"]),
                        ("%.3f MHz / %.0f mV" % (f["jam_hz"] / 1e6, f["jam_mv"]))
                        if f["jam_hz"] else "无", _note), fg=MUTED)
        except Exception as e:
            self.lbl_plan.configure(text=str(e), fg=C_MARK)

    def _on_toggle_src(self, step=1):
        if self.busy:
            return
        cycle_src(self, step)

    def _src_note(self, text, bad=False):
        """cycle_src 的回调：把信号源状态写到左下角消息位。"""
        try:
            self.lbl_msg.configure(text=text, fg=(C_MARK if bad else MUTED))
        except tk.TclError:
            pass

    def _on_go(self):
        if self.busy:
            return
        if self.src is None:
            self.lbl_msg.configure(text="信号源未连接，先点「源」", fg=C_MARK)
            return
        try:
            f = self._form()
            n = self._plan(f["f1"], f["tones"])
        except Exception as e:
            self.lbl_msg.configure(text=str(e), fg=C_MARK)
            return
        self.busy = True
        self.btn_go.configure(state="disabled", text="装载中…")
        self.lbl_msg.configure(text="装载中…", fg=MUTED)
        threading.Thread(target=self._worker, args=(f, n), daemon=True).start()

    def _worker(self, f, n):
        try:
            vals, comps, ub_th, tot = self._build_wave(
                f["f1"], f["tones"], f["vpp"], f["jam_hz"], f["jam_mv"], n)
            vpp = tot / 1000.0
            # 谐波优先：能谐波就谐波，不能就由 Dg4202.load_wave 打印原因后退回 ARB
            ts = None if f["jam_hz"] else tones_for_src(f["tones"], comps)
            mode, note = self.src.load_wave(vals, f["f1"], vpp, ch=1, tones=ts)
            self._src_note(note)
            # 谐波模式下回读到的幅度是**基波** Vpp，不是 tot
            fund = (2.0 * float(comps[0][1]) / 1000.0
                    if mode == "harmonic" else vpp)
            if mode == "harmonic":
                f = dict(f, f1=f["f1"] * min(float(m) for m, _ in f["tones"]))
            rb = self._check(f, fund)
            if rb is None:                      # 回读不对，重下一次
                self.src.load_wave(vals, f["f1"], vpp, ch=1, tones=ts)
                rb = self._check(f, fund)
            if rb is False:
                raise RuntimeError("信号源没有接受这组参数（回读不符）")
            # 先落盘 + 写主界面：测不测成功，源的状态都看得到，重启也不丢
            rec = {"f1_hz": float(f["f1"]),
                   "tones": [[int(m), float(r)] for m, r in f["tones"]],
                   "vpp_mv": float(f["vpp"]),
                   "tot_mv": float(tot),
                   "jam_hz": float(f["jam_hz"]),
                   "jam_mv": float(f["jam_mv"])}
            save_excite(rec)
            param_save(self.app)          # 装载的参数也备份到板子
            desc = excite_text(rec)
            self._ui(lambda: self.app.set_excite(desc))
            self._ui(lambda: self._render(f, n, comps, vals, tot, rb=self._rb))
        except Exception as e:
            traceback.print_exc()
            msg = str(e)
            self._ui(lambda: self._fail(msg))
        finally:
            self._ui(self._done)

    def _check(self, f, vpp):
        """回读确认；返回 self._rb 字典（成功）或 False（不符）。"""
        try:
            self._rb = self.src.readback(1)
        except Exception:
            self._rb = {}
            return False
        ff = self._rb.get("freq_hz")
        aa = self._rb.get("amp_vpp")
        okf = ff is not None and abs(ff - f["f1"]) <= max(1.0, 0.005 * f["f1"])
        oka = aa is not None and abs(aa - vpp) <= max(0.005, 0.08 * vpp)
        return self._rb if (okf and oka) else None

    def _ui(self, fn):
        self._q.put(fn)

    def _drain(self):
        try:
            while True:
                fn = self._q.get_nowait()
                try:
                    fn()
                except Exception:
                    traceback.print_exc()
        except queue.Empty:
            pass
        try:
            self.after(120, self._drain)
        except tk.TclError:
            pass

    def _fail(self, msg):
        self.lbl_msg.configure(text=msg, fg=C_MARK)

    def _done(self):
        self.busy = False
        try:
            self.btn_go.configure(state="normal", text="装载")
        except tk.TclError:
            pass

    def _render(self, f, n, comps, vals, tot, rb):
        q = np.round(np.asarray(vals, dtype=float) * 511.0).astype(int)
        qmax = float(np.max(np.abs(q))) or 1.0
        # 下发后各分量的实际峰值：对上交的整数缓冲做同一组基拟合
        tt = np.arange(q.size, dtype=float) / q.size
        cols = []
        for k in range(1, max(k for k, _ in f["tones"]) + 1):
            cols.append(np.cos(2.0 * np.pi * k * tt))
            cols.append(np.sin(2.0 * np.pi * k * tt))
        A = np.column_stack(cols)
        cc, *_ = np.linalg.lstsq(A, q.astype(float) - np.mean(q), rcond=None)
        amp_code = np.hypot(cc[0::2], cc[1::2])
        amp_mv = amp_code * tot / (2.0 * qmax)

        t = self.txt
        t.delete("1.0", "end")
        t.insert("end", "%-14s %-22s %s\n"
                        % ("项目", "预设（你要的）", "回读（源里实际）"), "hdr")
        t.insert("end", "-" * 58 + "\n")
        t.insert("end", "%-14s %-22s %s\n"
                        % ("波形", "基波+%d 分量" % len(f["tones"]),
                           str(rb.get("func") or "?")))
        _m0 = min(int(m) for m, _r in f["tones"])
        t.insert("end", "%-14s %-22s %s\n"
                        % ("源频率", "%.4f kHz" % (f["f1"] / 1e3),
                           "%.4f kHz" % ((rb.get("freq_hz") or 0) / 1e3)))
        if _m0 != 1:
            t.insert("end", "%-14s %-22s %s\n"
                            % ("实际基波", "%.4f kHz" % (f["f1"] * _m0 / 1e3),
                               "= %.4f kHz × %d" % (float(rb.get("freq_hz") or 0) / 1e3,
                                                     _m0)))
        t.insert("end", "%-14s %-22s %s\n"
                        % ("总峰峰值", "%.1f mV" % tot,
                           "%.1f mV" % ((rb.get("amp_vpp") or 0.0) * 1e3)))
        for i, (hz, a_mv) in enumerate(comps):
            k = int(round(hz / f["f1"]))
            t.insert("end", "%-14s %-22s %s\n"
                            % ("%d 次 %.2fk" % (k, hz / 1e3),
                               "%.1f mV" % a_mv,
                               "%.1f mV" % (amp_mv[k - 1] if k - 1 < amp_mv.size
                                            else float("nan"))))
        if f["jam_hz"]:
            t.insert("end", "%-14s %-22s %s\n"
                            % ("干扰 %.3fM" % (f["jam_hz"] / 1e6),
                               "%.0f mVpp" % f["jam_mv"], "已叠加"))
        t.insert("end", "-" * 58 + "\n")
        t.insert("end", "点数 %d · 采样率 %.1f MSa/s · 幅度按 10 bit 量化下发\n"
                 % (n, n * f["f1"] / 1e6), "dim")

        # 结论：只看源是否真的接受了这组参数（不测板子）
        rf = rb.get("freq_hz")
        ra = rb.get("amp_vpp")
        e_f = abs(float(rf or 0.0) - f["f1"])
        e_a = abs(float(ra or 0.0) - tot / 1000.0)
        bad = []
        if rf is None or e_f > max(1.0, 0.005 * f["f1"]):
            bad.append("基频差 %.1f Hz" % e_f)
        if ra is None or e_a > max(0.005, 0.08 * tot / 1000.0):
            bad.append("幅度差 %.2f mV" % (e_a * 1e3))
        if bad:
            t.insert("end", "\n装载了但源没接受：%s\n" % "，".join(bad), "bad")
            self.lbl_msg.configure(text="装载异常", fg=C_MARK)
        else:
            t.insert("end", "\n已装载 ✓   源频率 %.4f kHz / %.4f Vpp，"
                            "关闭面板回主界面按「采集」\n"
                     % (float(rf) / 1e3, float(ra)), "ok")
            self.lbl_msg.configure(text="已装载到信号源", fg=C_WAVE)

    def _on_close(self):
        try:
            if self.src is not None:
                self.src.close()
        except Exception:
            pass
        self.destroy()


# ────────────────────────── 入口 ──────────────────────────
def main(argv=None):
    ap = argparse.ArgumentParser(description="周期信号谐波分析装置 —— 全触摸上位机")
    ap.add_argument("--host", default="192.168.57.200")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--no-fullscreen", action="store_true", help="窗口模式（调试用）")
    ap.add_argument("--geometry", default="1024x600", help="窗口模式下的初始尺寸")
    ap.add_argument("--autocap", type=float, default=0.0,
                    help="启动后 N 秒自动采集一次（自检/无人值守用，默认关）")
    a = ap.parse_args(argv)

    root = tk.Tk()
    root.title("周期信号谐波分析装置")
    if a.no_fullscreen:
        root.geometry(a.geometry)
    else:
        root.attributes("-fullscreen", True)

        def _fs_retry():
            # labwc/Xwayland 下第一次 -fullscreen 偶尔不生效（窗口带标题栏、面板压在
            # 上面、画布变矮把标题裁掉），再补两次。
            try:
                if not root.attributes("-fullscreen"):
                    root.attributes("-fullscreen", True)
                    root.lift()
            except tk.TclError:
                pass

        root.after(600, _fs_retry)
        root.after(1800, _fs_retry)

    app = TouchApp(root, a.host, a.port)
    if a.autocap > 0:
        root.after(int(a.autocap * 1000), app.on_capture)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
