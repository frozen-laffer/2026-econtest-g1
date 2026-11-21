# -*- coding: utf-8 -*-
"""10 秒快校：不重建整张表，只用 3~4 个点量出"当前状态 vs 标定状态"的偏差，
折算成一条随频率变化的修正曲线乘到现有 eq.json 表上。

为什么需要：4 阶板第二级 Q=2.94（THS4551 用掉 99% 的 GBW），500 kHz 附近的增益
会随温度/工作点漂 2~3%。每项测量前跑一次本脚本（约 10 s）即可把读数拉回 250.0。

用法：
    python cal_quick.py [--src afg] [--amp 0.25]
    python cal_quick.py --src dg        # 源是插在树莓派上的 DG4202，要在树莓派上跑
"""
import sys, json, time, argparse
import numpy as np

sys.path.insert(0, r"F:\STM32_Project\host_app")
import cal_src                        # noqa: E402
import lsa_eq as eq_mod               # noqa: E402

PTS = [10000.0, 300000.0, 500000.0, 550000.0]


def main():
    ap = argparse.ArgumentParser(description="10 秒快校：修 eq.json 表")
    ap.add_argument("--src", choices=("afg", "dg"), default="afg",
                    help="信号源类型：afg（默认，PC COM17）或 dg（树莓派）")
    ap.add_argument("--port", default=None, help="AFG 串口，默认 COM17")
    ap.add_argument("--amp", type=float, default=0.25, help="源幅度 Vpp")
    ap.add_argument("--host", default=cal_src.PI_HOST, help="树莓派地址（AFG 模式用）")
    a = ap.parse_args()

    try:
        cal_src.ensure_supported(a.src)      # DG 在 Windows 上先拦掉，别等读到表
    except RuntimeError as exc:
        print("[错误] %s" % exc)
        return 2

    eqd = eq_mod.load()
    tbl = eqd.get("table") or []
    if not tbl:
        print("eq.json 里没有表，请先跑 cal_step.py")
        return 2
    tf = np.array([t[0] for t in tbl], float)
    tv = np.array([t[1] for t in tbl], float)

    try:
        ad = cal_src.open_adapter(a.src, port=a.port, pi_host=a.host)
    except RuntimeError as exc:
        print("[错误] %s" % exc)
        return 2

    ad.remote()
    # FUNC/UNIT/DCO 只在开头设一次；每点只改 FREQ/AMP（反复写 FUNC 会让后续 FREQ 被丢）
    ad.setup_sine()

    ad.stop_gui()

    raw = {}
    for f in PTS:
        fq, am = ad.set_sine(f, a.amp)
        if abs(fq - f) > 1.0:
            print("  [!] %.0f Hz 未生效（回读 %.0f），跳过" % (f, fq))
            continue
        d = ad.cap(gear=0, n=16384, wave="sine")
        raw[f] = float(d["upp_mv"]) / 1000.0
        print("  %7.1f kHz  raw %.2f mVpp" % (f / 1e3, raw[f] * 1000))

    if PTS[0] not in raw:
        print("基准点（10 kHz）没采到，放弃")
        return 1
    base = raw[PTS[0]]
    ks, kf = [], []
    for f in PTS[1:]:
        if f not in raw:
            continue
        t = float(np.interp(f, tf, tv))
        ks.append((raw[f] / base) / t)
        kf.append(f)
        print("  %7.1f kHz  修正比 %.4f（当前/标定）" % (f / 1e3, ks[-1]))
    if not ks:
        return 1

    kfx = np.array([200e3] + kf, float)
    kvy = np.array([1.0] + ks, float)
    corr = np.interp(tf, kfx, kvy)
    new = np.column_stack([tf, tv * corr])
    eq_mod.save_table([(float(f), float(v)) for f, v in new], 540.6e3, 0.607)
    print("  已写回 eq.json（%d 点）" % len(new))
    ad.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
