#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""全波形 × 全频域 × 全幅度 验收测试（PC 侧驱动信号源 + 树莓派采数）。

    python full_test.py                       # 正弦/方波/三角/半波 × 7 频点 × 5 幅度
    python full_test.py --waves sine,square   # 只测两种波形
    python full_test.py --quick               # 3 频点 × 3 幅度
    python full_test.py --src dg              # 换成 RIGOL DG4202（USB/USBTMC）

前置条件：
  · --src afg（默认）：AFG-2225 接在本机 COM17，输出接板子输入端
  · --src dg：DG4202 走 USB（Windows 需 WinUSB，树莓派是 /dev/usbtmc0），忽略 --port
  · 树莓派可达，板子 192.168.57.200 在线
  · 触摸界面必须关掉（板子只允许一条 TCP 连接），脚本会自动 pkill
输出：屏幕表格 + F:\\STM32_Project\\_full_test.csv
"""
import argparse
import csv
import json
import sys
import time

import numpy as np

sys.path.insert(0, r"F:\STM32_Project\host_app")

import paramiko                     # noqa: E402
import afg2225                      # noqa: E402

from pi_env import pi_host, pi_pass, pi_user   # noqa: E402

# 树莓派连接参数：环境变量 LSA_PI / LSA_PI_USER / LSA_PI_PASS，或凭据文件 ~/.lsa_pi
PI_HOST = pi_host()
PI_USER = pi_user()
PI_PWD = pi_pass()
PI_DIR = "/home/%s/lsa" % PI_USER

FREQS_FULL = [10e3, 20e3, 50e3, 100e3, 200e3, 300e3, 400e3, 500e3]
AMPS_FULL = [50.0, 100.0, 150.0, 200.0, 250.0]
FREQS_QUICK = [20e3, 100e3, 400e3]
AMPS_QUICK = [50.0, 150.0, 250.0]
WAVES = ["sine", "square", "triangle", "half"]

AFG_FUNC = {"sine": "SIN", "square": "SQU", "triangle": "RAMP"}

# ── 信号源模式（--src）：afg = 历史 AFG-2225 路径，dg = RIGOL DG4202 ──
# SRC 由 main() 按 --src 覆盖；DG 只在 --src dg 时才 import（afg 路径不碰 pyusb）
SRC = "afg"
DG = None


# ────────────────────────────── 树莓派 ──────────────────────────────
def pi_connect():
    from pi_env import pi_pass_or_die
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(PI_HOST, 22, PI_USER, PI_PWD or pi_pass_or_die(), timeout=15,
              allow_agent=False, look_for_keys=False)
    return c


def pi_run(c, cmd, timeout=90):
    _, o, e = c.exec_command(cmd, timeout=timeout)
    out = o.read().decode("utf-8", "replace")
    err = e.read().decode("utf-8", "replace")
    return out, err


def cap_once(c, gear, n, apply_=True, wave="auto"):
    """在树莓派上采一帧，返回 JSON dict。"""
    cmd = ("cd %s && python3 cap_one.py --gear %d --n %d --wave %s %s"
           % (PI_DIR, gear, n, wave, "--apply" if apply_ else ""))
    for attempt in range(3):
        out, err = pi_run(c, cmd, timeout=60)
        line = out.strip().splitlines()[-1] if out.strip() else ""
        if line.startswith("{"):
            return json.loads(line)
        time.sleep(0.6)
    raise RuntimeError("采集失败：%s %s" % (out[-200:], err[-200:]))


# ────────────────────────────── AFG ──────────────────────────────
def set_wave(g, kind, f_hz, vpp):
    """把信号源设成指定波形/频率/幅度（Vpp），并回读校验。"""
    if SRC == "dg":
        set_wave_dg(g, kind, f_hz, vpp)
        return
    if kind == "half":
        n = int(min(4096, max(64, 120e6 / float(f_hz))))
        n = max(64, min(4096, n))
        t = np.arange(n) / float(n)
        # 半波整流后居中：min=-0.5·A, max=+0.5·A → Vpp=A
        w = np.maximum(0.0, np.sin(2 * np.pi * t)) - 0.5
        afg2225.load_arb(list(w), ch=1, freq_hz=float(f_hz),
                         amp_vpp=float(vpp) / 1000.0, offset=0.0, s=g)
        time.sleep(0.9)
        return

    g.w("SOUR1:FUNC %s" % AFG_FUNC[kind], settle=0.5)
    if kind == "triangle":
        g.w("SOUR1:RAMP:SYMM 50", settle=0.35)
    g.w("SOUR1:VOLT:UNIT VPP", settle=0.3)
    g.w("SOUR1:FREQ %.6f" % float(f_hz), settle=0.35)
    g.w("SOUR1:AMP %.6f" % (float(vpp) / 1000.0), settle=0.35)
    g.w("SOUR1:DCO 0", settle=0.3)
    g.w("OUTP1 ON", settle=0.35)
    # 回读校验（AFG 会静默丢命令）
    f_rd = g.one("SOUR1:FREQ?")
    a_rd = g.one("SOUR1:AMP?")
    try:
        f_ok = abs(float(f_rd) - f_hz) / max(1.0, f_hz) < 1e-4
        a_ok = abs(float(a_rd) - vpp / 1000.0) / max(1e-9, vpp / 1000.0) < 0.02
    except Exception:
        f_ok = a_ok = False
    if not (f_ok and a_ok):
        # 再写一遍（远程模式下偶发第一次不生效）
        g.w("SOUR1:FREQ %.6f" % float(f_hz), settle=0.4)
        g.w("SOUR1:AMP %.6f" % (float(vpp) / 1000.0), settle=0.4)


def set_wave_dg(g, kind, f_hz, vpp):
    """DG4202 分支：sine/square/triangle 走 set_wave，half 走 load_arb。

    与 AFG 分支的差别：DG 的 :VOLT 收**伏特值**（脚本里 vpp 是 mV）；RAMP:SYMM
    已由驱动内部的 set_wave 写 50%，这里不再补；收尾用 output() 而不是 OUTP1。
    """
    v_vpp = float(vpp) / 1000.0
    if kind == "half":
        # n 是**波形**点数，和 --n（采集点数，默认 16384）无关；DG ARB 上限 16384 点
        n = int(min(DG.ARB_MAX_POINTS, max(64, 120e6 / float(f_hz))))
        t = np.arange(n) / float(n)
        # 半波整流后居中：min=-0.5·A, max=+0.5·A → Vpp=A
        w = np.maximum(0.0, np.sin(2 * np.pi * t)) - 0.5
        DG.load_arb(list(w), ch=1, freq_hz=float(f_hz),
                    amp_vpp=v_vpp, offset=0.0, s=g)
        time.sleep(0.9)
        return

    g.set_wave(kind, float(f_hz), v_vpp, ch=1, offset=0.0)
    g.output(True, ch=1)
    # 回读校验：DG 不再发裸 :SOUR1:xxx?，统一读 state() 的 func / freq_hz / amp_vpp
    st = g.state(ch=1)
    want = {"sine": ("SIN", "SINE"), "square": ("SQU", "SQUARE"),
            "triangle": ("RAMP", "TRI", "TRIANGLE")}[kind]
    try:
        f_ok = abs(float(st["freq_hz"]) - f_hz) / max(1.0, f_hz) < 1e-4
        a_ok = abs(float(st["amp_vpp"]) - v_vpp) / max(1e-9, v_vpp) < 0.02
        func_ok = str(st.get("func", "")).strip().upper().startswith(want)
    except (TypeError, ValueError):
        f_ok = a_ok = func_ok = False
    if not (f_ok and a_ok and func_ok):
        # DG 在 -410 错位后也会静默丢命令，重写一遍
        g.set_wave(kind, float(f_hz), v_vpp, ch=1, offset=0.0)
        g.output(True, ch=1)


def main():
    global SRC, DG
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM17",
                    help="AFG 串口（--src dg 时忽略）")
    ap.add_argument("--src", choices=("afg", "dg"), default="afg",
                    help="信号源：afg=GW Instek AFG-2225（COM 口，默认），"
                         "dg=RIGOL DG4202（USB/USBTMC）")
    ap.add_argument("--gear", type=int, default=0)
    ap.add_argument("--n", type=int, default=16384)
    ap.add_argument("--waves", default=",".join(WAVES))
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--out", default=r"F:\STM32_Project\_full_test.csv")
    ap.add_argument("--force-wave", action="store_true",
                    help="采集时按当前波形类型强制换算 Upp（高频非正弦必需）")
    a = ap.parse_args()

    waves = [w.strip() for w in a.waves.split(",") if w.strip()]
    SRC = a.src
    if a.src == "dg":
        import dg4202                      # 只在 dg 模式才拉 pyusb/libusb
        DG = dg4202
    freqs = FREQS_QUICK if a.quick else FREQS_FULL
    amps = AMPS_QUICK if a.quick else AMPS_FULL
    total = len(waves) * len(freqs) * len(amps)
    print("波形 %s  频点 %d  幅度 %d  共 %d 点"
          % ("/".join(waves), len(freqs), len(amps), total))

    if a.src == "dg":
        # DG 走 USB/USBTMC，没有 COM 口，--port 在这里没有意义
        print("提示：--src dg 走 USB 连接，--port=%s 已忽略" % a.port)
        g = dg4202.Scpi()                  # 不传 port；构造不碰硬件
        g.remote()                         # DG 上是 no-op，保留只为对齐 AFG 流程
        dropped = g.resync()               # 清 -410 Query INTERRUPTED 之类的错位残留
        if dropped:
            print("DG4202 构造后 resync：丢弃 %d 条残留错误" % dropped)
    else:
        g = afg2225.Scpi(port=a.port)
        g.remote()
    c = pi_connect()
    pi_run(c, "pkill -f '[l]sa_touch.py'")      # 板子只允许一条连接
    time.sleep(1.0)

    rows = []
    dg_errs = []                           # dg 模式下收尾错误队列，最后落到旁路日志
    t_all = time.time()
    try:
        for wi, kind in enumerate(waves):
            print("\n【%s】" % kind)
            print("  %8s %6s | %9s %8s | %8s %8s | %7s %s"
                  % ("f(Hz)", "Vpp", "Upp mV", "err mV", "Urms mV", "err mV",
                     "f1 err", "判型"))
            for f in freqs:
                for amp in amps:
                    set_wave(g, kind, f, amp)
                    time.sleep(0.15)
                    d = cap_once(c, a.gear, a.n, wave=(kind if a.force_wave else "auto"))
                    upp = float(d["upp_mv"])
                    urms = float(d["urms_mv"])
                    f1 = float(d["f1_hz"])
                    err = upp - amp
                    urms_th = amp / (2 * np.sqrt(2))     # 正弦理论值
                    if kind == "square":
                        urms_th = amp / 2.0
                    elif kind == "triangle":
                        urms_th = amp / (2 * np.sqrt(3))
                    elif kind == "half":
                        # 居中半波（Upp=A）的交流有效值 = 0.3854942·A
                        urms_th = 0.3854942 * amp
                    uerr = urms - urms_th
                    ferr = f1 - f
                    rows.append(dict(wave=kind, f_nom=f, vpp_nom=amp,
                                     upp_mv=upp, err_mv=err,
                                     urms_mv=urms, urms_th=urms_th,
                                     urms_err=uerr, f1_hz=f1, f1_err=ferr,
                                     kind_det=d.get("wave_kind"),
                                     unrel=d.get("wave_unreliable"),
                                     clip=d.get("clip_pct"),
                                     n_used=d.get("n_used")))
                    print("  %8.0f %6.0f | %9.2f %+8.2f | %8.2f %+8.2f | %+7.1f %s"
                          % (f, amp, upp, err, urms, uerr, ferr,
                             d.get("wave_kind")))
                    sys.stdout.flush()
    finally:
        try:
            if SRC == "dg":
                g.output(False, ch=1)      # DG 没有远程态，OUTP1 OFF → output(False)
                errs = g.errors()          # 跑完再抽一次错误队列
                if errs:
                    dg_errs.extend(errs)
                    print("\nDG4202 收尾错误队列（%d 条）：" % len(errs))
                    for e in errs:
                        print("  %s" % e)
            else:
                g.w("OUTP1 OFF", settle=0.3)
                g.local()
        except Exception:
            pass
        c.close()

    # 汇总
    print("\n================ 汇总 ================")
    allerr = np.array([abs(r["err_mv"]) for r in rows])
    print("点数 %d   总耗时 %.0f s" % (len(rows), time.time() - t_all))
    print("Upp 绝对误差：最大 %.2f mV   均值 %.2f mV   RMS %.2f mV"
          % (allerr.max(), allerr.mean(), float(np.sqrt((allerr ** 2).mean()))))
    for kind in waves:
        sub = [r for r in rows if r["wave"] == kind]
        if not sub:
            continue
        e = np.array([abs(r["err_mv"]) for r in sub])
        fe = np.array([abs(r["f1_err"]) for r in sub])
        bad = sum(1 for r in sub if abs(r["err_mv"]) > 5.0)
        print("  %-9s 最大 Upp 误差 %6.2f mV   RMS %5.2f   f1 最大误差 %6.1f Hz  超差 %d/%d"
              % (kind, e.max(), float(np.sqrt((e ** 2).mean())), fe.max(), bad, len(sub)))

    with open(a.out, "w", newline="", encoding="utf-8-sig") as fh:
        wr = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)
    print("\n已写入 %s" % a.out)

    if dg_errs:                            # DG 的错误队列旁路落盘，CSV 列结构不动
        err_path = a.out + ".dg_err.txt"
        with open(err_path, "w", encoding="utf-8") as fh:
            fh.write("\n".join(dg_errs) + "\n")
        print("DG4202 错误已写入 %s" % err_path)


if __name__ == "__main__":
    main()
