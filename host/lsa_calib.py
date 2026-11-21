#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lsa_calib.py —— 零输入基线校准（每通道直流偏置）

为什么需要
──────────
三路 ADC 的模拟前端各自有直流偏置，实测本板 ADC5 约 +737.8 mV、ADC1/ADC2 约 0。
三路交替采样时这三路被交织进同一串数据，偏置差会在 fs/3 处造出一个假信号
（fs=7.083 MSPS 时是 2.361 MHz），时域 Upp 会被撑到 790 mV 以上，完全没法读数。

如果把每路的偏置量出来存进 JSON，测量时先减掉，三路交替就能直接用于题1/题2，
不再需要"只敢用单路 ADC1"。

量什么
──────
输入端**短路或空载**时，每路 ADC 读到的平均值（伏特，已经过增益换算、就是
ADC 引脚上的电压）。这个值同时包含了 ADC 自身失调和模拟前端的失调。

偏置随增益档变化（×2 档 / ×20 档），所以按 `"<通道>@<gain_milli>"` 建键，
换档后没有对应键就不做修正（宁可不修，也不要拿错档的常数去减）。

文件格式（calib.json）
──────────────────────
{
  "version": 1,
  "updated": "2026-11-11T14:30:00",
  "note": "零输入基线：输入端短路时各通道的直流偏置",
  "source": "shorted",              // shorted / floating / manual
  "offsets_v": {
    "0@3925": 0.0000,               // 通道0，增益 ×2 档（gain_milli=3925）
    "1@3925": 0.0000,
    "2@3925": 0.7378
  }
}

用法
──────
    import lsa_calib as calib

    c = calib.load()                          # 读 calib.json，没有就返回空表
    v = calib.apply(v, c, mode, ch, gain_milli)   # 按需减去各通道偏置

    # 标定：采集一批零输入数据，算各通道均值并存盘
    offs = calib.offsets_from(v, mode, gain_milli)
    c = calib.build(c, mode, gain_milli, offs, source="shorted")
    calib.save(c)
"""

import json
import os
import time

import numpy as np

CALIB_VERSION = 1

# 标定文件默认与脚本同目录；可用环境变量 LSA_CALIB 覆盖
DEF_NAME = "calib.json"


def default_path():
    p = os.environ.get("LSA_CALIB")
    if p:
        return p
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), DEF_NAME)


def load(path=None):
    """读标定文件；不存在或损坏都返回空表（不影响正常测量）。"""
    path = path or default_path()
    try:
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        if not isinstance(d, dict):
            return {}
        d.setdefault("offsets_v", {})
        if not isinstance(d["offsets_v"], dict):
            d["offsets_v"] = {}
        return d
    except Exception:
        return {}


def save(calib, path=None):
    """写标定文件（先写临时文件再改名，避免掉电写坏）。"""
    path = path or default_path()
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(calib, f, ensure_ascii=False, indent=2, sort_keys=True)
        f.write("\n")
    os.replace(tmp, path)
    return path


def key_of(ch, gain_milli):
    return "%d@%d" % (int(ch), int(round(gain_milli)))


def offset_of(calib, ch, gain_milli):
    """某通道在某增益档下的偏置（伏特）。没有记录就返回 0.0。"""
    if not calib:
        return 0.0
    try:
        return float(calib.get("offsets_v", {}).get(key_of(ch, gain_milli), 0.0))
    except Exception:
        return 0.0


def is_inter3(mode):
    if isinstance(mode, bool):
        return mode
    if isinstance(mode, (int, np.integer)):
        return int(mode) != 0
    return str(mode).lower() in ("inter3", "3", "interleaved", "true")


def offsets_from(v, mode, gain_milli):
    """
    从一批零输入数据算出各通道偏置（伏特）。

    返回 {0: x, 1: y, 2: z}：
      * 三路交替：把交织数据拆成三路分别求均值；
      * 单路：只有该通道有值，其余为 None（不写进文件）。
    """
    v = np.asarray(v, dtype=np.float64)
    out = {}
    if is_inter3(mode):
        m = (v.size // 3) * 3
        a = v[:m].reshape(-1, 3)
        for c in range(3):
            out[c] = float(np.mean(a[:, c]))
    else:
        out[0] = float(np.mean(v))
    return out


def build(calib, mode, gain_milli, offs, source="shorted", note=None):
    """
    把新测的偏置并进已有标定表，返回新表（不落盘）。

    offs : {通道号: 偏置}
    """
    calib = dict(calib or {})
    table = dict(calib.get("offsets_v", {}))
    for ch, val in offs.items():
        if val is None:
            continue
        table[key_of(ch, gain_milli)] = round(float(val), 6)
    calib.update({
        "version": CALIB_VERSION,
        "updated": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "note": note or "零输入基线：输入端短路（或空载）时各通道的直流偏置",
        "source": source,
        "gain_milli": int(round(gain_milli)),
        "mode": "inter3" if is_inter3(mode) else "single",
        "offsets_v": table,
    })
    return calib


def apply(v, calib, mode, ch=0, gain_milli=3925):
    """
    按标定表减去各通道偏置，返回新数组。

    mode=inter3 时按 0/1/2 三路分别扣；单路时只扣 ch 这一路。
    没有可用标定时原样返回（浅拷贝，调用方可以安全地再加工）。
    """
    v = np.asarray(v, dtype=np.float64)
    if not calib:
        return v
    if is_inter3(mode):
        offs = [offset_of(calib, c, gain_milli) for c in range(3)]
        if not any(offs):
            return v
        out = v.copy()
        m = (out.size // 3) * 3
        for c in range(3):
            if offs[c]:
                out[c:m:3] -= offs[c]
        return out
    o = offset_of(calib, int(ch), gain_milli)
    return v - o if o else v


def summary(calib):
    """一行文字，给界面/命令行显示用。"""
    if not calib or not calib.get("offsets_v"):
        return "未标定"
    items = []
    for k in sorted(calib["offsets_v"], key=lambda s: (int(s.split("@")[0]), s)):
        ch, g = k.split("@")
        items.append("CH%s/%s×%.3f mV" % (ch, g, calib["offsets_v"][k] * 1000.0))
    return "已标定（%s）：%s" % (calib.get("updated", "?"), "，".join(items))
