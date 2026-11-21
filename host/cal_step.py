#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶梯扫频校准（PC 侧一键）：驱动 AFG-2225 / DG4202 + 经树莓派采集 + 生成 eq.json / config.json

背景
----
AFG-2225 的 SCPI 扫频模式在这台机器上不生效（`SOUR:SWE:STAT ON` 后输出频率不扫），
所以改成"逐点 CW"：每个频点单独设频、单独采一次，比扫频更准也更好复现。

流程
----
1) 在树莓派上杀掉触摸界面（板子只允许一条 TCP 连接）
2) 逐点设源（每点回读校验）→ 在树莓派上采一帧 → 收 Upp
3) 归一化成幅频表，拟合 fc/Q，写 eq.json + config.json 并上传
4) 再跑一遍"补偿后"的实采验证，按实测均值微调 in_scale 后重新上传
5) 打印最终表 + 误差

用法
----
    python cal_step.py                 # 完整流程（约 90 秒）
    python cal_step.py --dry-run       # 只采表、不写文件
    python cal_step.py --skip-verify    # 不跑第二遍微调
    python cal_step.py --src dg        # 源是插在树莓派上的 DG4202，要在树莓派上跑

注意：源输出必须接在板子输入端，且幅度用 250 mVpp（和赛题工作点一致）。
"""
import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cal_src                      # noqa: E402
import lsa_eq                       # noqa: E402

# 旧接口兼容用（临时脚本还在 import 这几个名字）
PI_HOST = cal_src.PI_HOST
PI_USER = cal_src.PI_USER
PI_PASS = cal_src.PI_PASS
PI_DIR = cal_src.PI_DIR

# 19 个频点：低端密、400k 以后加密（那里斜率变化最快）
# 325k / 375k 是为了压掉 300~400 kHz 那段的插值凹陷（实测 −1.3~−1.4 mV）
FREQS = [10e3, 14e3, 20e3, 28e3, 40e3, 56e3, 80e3, 110e3, 150e3, 200e3,
         250e3, 300e3, 325e3, 350e3, 375e3, 400e3, 440e3, 470e3, 500e3,
         520e3, 550e3]


def ssh():
    import paramiko
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(PI_HOST, username=PI_USER, password=PI_PASS, timeout=20,
              allow_agent=False, look_for_keys=False)
    return c


def pi_run(c, cmd, timeout=90):
    _i, o, e = c.exec_command(cmd, timeout=timeout)
    out = o.read().decode("utf-8", "replace")
    err = e.read().decode("utf-8", "replace")
    return out, err


def pi_put(c, local, remote):
    s = c.open_sftp()
    try:
        s.put(local, remote)
    finally:
        s.close()


def cap_json(c, gear, n, apply_=False, ch=0):
    cmd = ("cd %s && python3 cap_one.py --gear %d --n %d --ch %d%s 2>&1"
           % (PI_DIR, gear, n, ch, " --apply" if apply_ else ""))
    last = ""
    for attempt in range(4):
        try:
            out, err = pi_run(c, cmd, timeout=120)
        except Exception as exc:            # SSH 通道断了（树莓派欠压时会掉）
            last = "SSH: %s" % exc
            time.sleep(3.0)
            continue
        txt = [ln for ln in out.strip().splitlines() if ln.strip().startswith("{")]
        if txt:
            return json.loads(txt[-1])
        last = (out[-300:] + err[-150:]).strip()
        # 树莓派欠压/文件系统 I/O 错误时，等一下再试
        time.sleep(2.5)
    raise RuntimeError("采集失败（已重试 4 次）：%s" % last)


def set_afg(g, f_hz, amp_vpp):
    """只改频率和幅度；FUNC/UNIT/DCO 在开头统一设过。

    保留这个旧名字（临时脚本还在用），实现搬到 cal_src.afg_set_sine，
    保证与 cal_src 适配器走的是**同一条 SCPI 序列**。
    """
    return cal_src.afg_set_sine(g, f_hz, amp_vpp)


def main():
    ap = argparse.ArgumentParser(description="阶梯扫频校准：写 eq.json / config.json")
    ap.add_argument("--src", choices=("afg", "dg"), default="afg",
                    help="信号源类型：afg（默认，PC COM17）或 dg（树莓派）")
    ap.add_argument("--port", default=None, help="AFG 串口，默认 COM17")
    ap.add_argument("--host", default=cal_src.PI_HOST, help="树莓派地址（AFG 模式用）")
    ap.add_argument("--gear", type=int, default=0)
    ap.add_argument("--n", type=int, default=16384)
    ap.add_argument("--amp", type=float, default=0.25, help="源幅度 Vpp，默认 250 mVpp")
    ap.add_argument("--nominal", type=float, default=250.0, help="该幅度下界面应显示的值 mVpp")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--skip-verify", action="store_true")
    a = ap.parse_args()

    try:
        ad = cal_src.open_adapter(a.src, port=a.port, pi_host=a.host)
    except RuntimeError as exc:
        print("[错误] %s" % exc)
        return 2

    print("== 1/4 连接仪器与树莓派 ==")
    ad.remote()
    print("   %s:" % ("AFG" if ad.is_afg else "DG"), ad.idn())
    # 开头这四条命令与改造前逐条一致（AFG：SOUR1:FUNC SIN / VOLT:UNIT / DCO / OUTP1 ON）
    out_rb = ad.setup_sine(readback=True)
    print("   输出已开：", out_rb, " 幅度单位 VPP")

    ad.pi_ping()
    if ad.is_afg:
        print("   树莓派已连上")
        for helper in ("cap_one.py", "lsa_eq.py", "lsa_cfg.py"):
            p = os.path.join(HERE, helper)
            if os.path.exists(p):
                ad.pi_put(p, "%s/%s" % (ad.pi_dir, helper))
    else:
        print("   DG 直连模式：本机即树莓派，采集直接跑本机 cap_one.py")
    # 与改造前逐字一致（含 "; sleep 1"：等板子的 TCP 连接释放）
    ad.pi_exec("pkill -f '[l]sa_touch.py' 2>/dev/null; sleep 1; echo ok")
    print("   已停掉触摸界面（板子只允许一条 TCP 连接）")

    print("\n== 2/4 逐点采集（%d 点，源 %.0f mVpp） ==" % (len(FREQS), a.amp * 1e3))
    rows = []
    t0 = time.time()
    for f in FREQS:
        fq, am = ad.set_sine(f, a.amp)
        time.sleep(0.25)
        d = ad.cap(gear=a.gear, n=a.n, ch=0, retries=4)
        rows.append((f, d["upp_mv"]))
        print("   %7.1f kHz  源 %6.1f mVpp -> 采到 Upp %8.2f mV   (%.0f s)"
              % (f / 1e3, am * 1e3, d["upp_mv"], time.time() - t0))

    freqs = np.array([r[0] for r in rows], dtype=np.float64)
    amps = np.array([r[1] for r in rows], dtype=np.float64)
    ref = float(np.median(amps[freqs <= 20e3]))
    table = [(float(f), float(u / ref)) for f, u in rows]
    fn, Q, rms, _ref2 = lsa_eq.fit_points(freqs, amps)
    print("\n   低频平台 = %.2f mVpp（源 %.0f mVpp）" % (ref, a.amp * 1e3))
    print("   拟合：fc = %.1f kHz  Q = %.3f  RMS = %.3f dB" % (fn / 1e3, Q, rms))

    in_scale = a.nominal / ref
    print("   in_scale = %.0f / %.2f = %.7f" % (a.nominal, ref, in_scale))

    if a.dry_run:
        print("\n--dry-run：不写文件、不上传。")
        if ad.is_afg:
            ad.w("OUTP1 OFF", settle=0.3)     # 与改造前逐条一致
            ad.local()
        else:
            ad.output(False)
        ad.close(restore=False)
        return 0

    eq_doc = {"version": lsa_eq.EQ_VERSION, "source": "table",
              "table": table,
              "model": {"fn": fn, "Q": Q},
              "note": "阶梯实测幅频表（%.0f mVpp，%d 点，含滤波器+FDA+AFE）"
                      % (a.amp * 1e3, len(table))}
    local_eq = os.path.join(HERE, "eq.json")
    local_cfg = os.path.join(HERE, "config.json")
    with open(local_eq, "w", encoding="utf-8") as fh:
        json.dump(eq_doc, fh, ensure_ascii=False, indent=2)
    with open(local_cfg, "w", encoding="utf-8") as fh:
        json.dump({"in_scale": in_scale, "version": 1}, fh, ensure_ascii=False, indent=2)
    ad.pi_put(local_eq, "%s/eq.json" % ad.pi_dir)
    ad.pi_put(local_cfg, "%s/config.json" % ad.pi_dir)
    print("\n   已写入并上传 eq.json / config.json")

    if not a.skip_verify:
        print("\n== 3/4 补偿后验证（100/300/500 kHz） ==")
        got = []
        for f in (100e3, 300e3, 500e3):
            ad.set_sine(f, a.amp)
            time.sleep(0.25)
            d = ad.cap(gear=a.gear, n=a.n, ch=0, apply_=True, retries=4)
            got.append(d["upp_mv"])
            print("   %7.1f kHz -> 显示 %8.2f mV  (目标 %.1f)" % (f / 1e3, d["upp_mv"], a.nominal))
        mean_v = float(np.mean(got))
        corr = a.nominal / mean_v if mean_v > 0 else 1.0
        # 三个频点的偏差如果"随频率单调变大"，说明是响应形状变了（模拟板状态漂移），
        # 不是一个整体比例错误——这时乘一个平坦系数只会把低频端拉坏。
        spread = (max(got) - min(got)) / max(1e-9, mean_v)
        if spread > 0.02:
            print("   [!] 三点偏差离散度 %.1f%%（100k %.2f / 300k %.2f / 500k %.2f），"
                  "属响应形状变化，不是整体比例错误" % (spread * 100.0, got[0], got[1], got[2]))
            print("   → 不改 in_scale（乘平坦系数会把低频端带偏）。请先稳定模拟板再校准。")
        elif abs(corr - 1.0) > 0.002:
            in_scale *= corr
            print("   实测均值 %.2f mV，整体修正 ×%.5f -> in_scale = %.7f"
                  % (mean_v, corr, in_scale))
            with open(local_cfg, "w", encoding="utf-8") as fh:
                json.dump({"in_scale": in_scale, "version": 1}, fh,
                          ensure_ascii=False, indent=2)
            ad.pi_put(local_cfg, "%s/config.json" % ad.pi_dir)
            print("   已重新上传 config.json")
        else:
            print("   实测均值 %.2f mV，偏差 %.2f%% 在容差内，不改 in_scale"
                  % (mean_v, (mean_v / a.nominal - 1) * 100))

    print("\n== 4/4 结果 ==")
    print("   %-10s %-12s %-10s %-8s" % ("f(kHz)", "源(mVpp)", "采到(mVpp)", "|H|"))
    for (f, u), (tf, th) in zip(rows, table):
        print("   %-10.1f %-12.1f %-10.2f %-8.4f" % (f / 1e3, a.amp * 1e3, u, th))
    print("   in_scale = %.7f   拟合 fc=%.1f kHz Q=%.3f RMS=%.3f dB"
          % (in_scale, fn / 1e3, Q, rms))
    print("\n   收尾：源停在 500 kHz / %.0f mVpp / 输出开，可直接回界面按「采集」。"
          % (a.amp * 1e3))
    ad.set_sine(500e3, a.amp)
    ad.close(restore=False)      # 不额外发 SYST:LOC（与改造前一致）
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
