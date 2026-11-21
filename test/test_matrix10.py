# -*- coding: utf-8 -*-
"""集成路径验收：走「上位机现在真正会走的那条路」。

和 _pi_harm_cases.py（我自己的探针）不同，这个脚本调用的是：
  lsa_src.open_source("dg")  ->  lsa_src.Dg4202.load_wave()  ->  谐波优先 / 退回 ARB
  lsa_touch.StepCalib._build_wave + TEST_CASES（判据用的同一份波形）
  lsa_touch.tones_for_src（新加的"次数+绝对 Vpp"换算）
  _contest_one.py（判据同一个测量程序，带 DG 专用测量口径环境变量）
所以它通过，就等于上位机点「全测试」能通过。
"""
import json
import math
import os
import subprocess
import sys
import threading
import time

import lsa_src
import lsa_touch

os.environ["LSA_EQ_FHI_HZ"] = "520000"
os.environ["LSA_EQ_LP_HZ"] = "520000"
os.environ["LSA_FIT_RESID_MAX"] = "0.15"


def keep_board_free(interval=1.5):
    """板子固件只服务一个连接：界面的 lsa_touch.py 会抢占，测量前要一直踢它。"""
    while True:
        try:
            subprocess.run(["pkill", "-f", "[l]sa_touch.py"],
                           capture_output=True, timeout=5)
        except Exception:
            pass
        time.sleep(interval)


class Shim(object):
    """只提供 _build_wave 需要的三个常量（真身是 StepCalib 实例）。"""
    ARB_N = lsa_touch.StepCalib.ARB_N
    F0 = lsa_touch.StepCalib.F0
    JAM_PEAK_MV = lsa_touch.StepCalib.JAM_PEAK_MV


def measure():
    out = subprocess.run([sys.executable, "_contest_one.py", "--gear", "0",
                          "--n", "16384", "--harm", "20", "--apply"],
                         capture_output=True, text=True, timeout=180,
                         env=dict(os.environ))
    lines = [ln for ln in out.stdout.splitlines() if ln.strip().startswith("{")]
    if not lines:
        raise RuntimeError((out.stderr or out.stdout).strip()[-200:])
    return json.loads(lines[-1])


def main():
    threading.Thread(target=keep_board_free, daemon=True).start()
    time.sleep(2.0)
    shim = Shim()
    src = lsa_src.open_source("dg")
    print("源:", src.name, "silent=", src.silent)
    print("idn:", src.identify())
    npass = 0
    rows = []
    for label, _item, tones, ub_vpp, jam in lsa_touch.StepCalib.TEST_CASES:
        vals, comps, ub_th, tot = lsa_touch.StepCalib._build_wave(
            shim, tones, ub_vpp, jam)
        vpp = tot / 1000.0
        ts = None if jam else lsa_touch.tones_for_src(tones, comps)
        why = ("带干扰 u_j（%g×F0 = %.4f MHz），不是基波整数倍，谐波模式放不出来"
               % (jam, jam * shim.F0 / 1e6)) if jam else None
        mode, note = src.load_wave(vals, shim.F0, vpp, ch=1, tones=ts,
                                   reason=why)
        fund = (2.0 * comps[0][1] / 1000.0) if mode == "harmonic" else vpp
        rb = src.readback(1)
        time.sleep(1.4)
        d = measure()
        hm = d.get("harm_mv") or []
        f1 = float(d["f1_hz"])
        urms_th = math.sqrt(sum((a / math.sqrt(2.0)) ** 2 for _, a in comps))
        e_pp = float(d["upp_mv"]) - ub_th
        e_rms = float(d["urms_mv"]) - urms_th
        e_f1 = f1 - comps[0][0]
        ec = 0.0
        for f, a in comps:
            k = int(round(f / f1)) if f1 > 0 else 0
            meas = hm[k - 1] if 0 < k <= len(hm) else float("nan")
            if meas == meas:
                ec = max(ec, abs(meas - a))
        ok = (abs(e_pp) <= 5.0 and abs(e_rms) <= 5.0
              and abs(e_f1) <= 1000.0 and ec <= 5.0)
        npass += 1 if ok else 0
        print("[%s] %-7s e_pp=%+7.2f e_rms=%+6.2f e_f1=%+7.1f ec=%5.2f  %s"
              % (mode[:4], label, e_pp, e_rms, e_f1, ec, "PASS" if ok else "FAIL"))
        print("      路数: %s" % note)
        print("      回读: %s" % (rb,))
        rows.append((label, mode, e_pp))
    print("\n== 集成路径 %d/%d 通过 ==" % (npass, len(rows)))
    src.set_point(1000.0, 0.1)
    src.output(False, 1)
    src.close()
    return 0 if npass == len(rows) else 1


if __name__ == "__main__":
    sys.exit(main())
