# -*- coding: utf-8 -*-
"""lsa_cfg.py —— 上位机的小配置文件（可持久化的界面/测量选项）

目前只存一项：in_scale（输入比例）
    信号源按 50Ω 负载设定、而板子输入是高阻时，板子实际收到的是设定值的 2 倍。
    把 in_scale 设成 0.5，显示值就换算回"50Ω 等效值"。
    界面上点「×0.5」按钮即可切换，值会写进 config.json，重启后保持。

环境变量 LSA_CFG 可指定另一个路径（测试用）。
"""
import json
import os

CFG_VERSION = 1
DEF_NAME = "config.json"
DEF_IN_SCALE = 0.5      # 默认按 50Ω 折算：显示 = 实测 / 2


def default_path():
    p = os.environ.get("LSA_CFG")
    if p:
        return p
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), DEF_NAME)


def load(path=None):
    p = path or default_path()
    try:
        with open(p, "r", encoding="utf-8") as f:
            d = json.load(f)
        if not isinstance(d, dict):
            return {}
        d.setdefault("in_scale", DEF_IN_SCALE)
        return d
    except Exception:
        return {}


def save(cfg, path=None):
    p = path or default_path()
    tmp = p + ".tmp"
    try:
        cfg = dict(cfg)
        cfg["version"] = CFG_VERSION
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2, sort_keys=True)
        os.replace(tmp, p)
    except Exception:
        pass


def get_in_scale(cfg=None):
    cfg = load() if cfg is None else cfg
    try:
        s = float(cfg.get("in_scale", DEF_IN_SCALE))
    except Exception:
        s = DEF_IN_SCALE
    return s if s > 0 else 1.0


def set_in_scale(s, cfg=None):
    cfg = {} if cfg is None else dict(cfg)
    cfg["in_scale"] = float(s)
    save(cfg)
    return cfg


def summary(cfg=None):
    s = get_in_scale(cfg)
    if abs(s - 0.5) < 1e-9:
        return "输入×0.5（50Ω 等效）"
    if abs(s - 1.0) < 1e-9:
        return "输入×1（原始值）"
    return "输入×%g" % s
