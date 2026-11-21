"""树莓派 SSH 连接参数：统一从环境变量或本机凭据文件读取，仓库里不再出现明文密码。

取值优先级（按顺序，命中即用）：

1. 环境变量
     ``LSA_PI``       树莓派地址，缺省 10.110.75.61
     ``LSA_PI_USER``  用户名，缺省 frozen-laffer（连字符，与主机名一致）
     ``LSA_PI_PASS``  密码
2. 凭据文件，路径取 ``LSA_PI_CRED``，缺省 ``~/.lsa_pi``。支持三种写法：
     - JSON：``{"host": "10.110.75.61", "user": "frozen-laffer", "pass": "…"}``
     - 两行纯文本：第一行用户名，第二行密码
     - 单行纯文本：只有密码
3. 都没有时：``pi_pass()`` 返回空串，``pi_pass_or_die()`` 抛 RuntimeError 并打印设置方法。

用法：

    from pi_env import pi_host, pi_user, pi_pass, pi_pass_or_die
    HOST = pi_host()
    PWD  = pi_pass_or_die()      # 需要立刻连接时用这个

安全说明：``~/.lsa_pi`` 不要放进任何 git 仓库（.gitignore 已忽略 ``.lsa_pi``）。
"""
from __future__ import annotations

import json
import os

DEFAULT_HOST = "10.110.75.61"
DEFAULT_USER = "frozen-laffer"
CRED_PATH = os.environ.get("LSA_PI_CRED", os.path.join(os.path.expanduser("~"), ".lsa_pi"))

_cred_cache = None


def _load_cred():
    """读凭据文件（只读一次，结果缓存）。文件不存在或解析失败时返回空 dict。"""
    global _cred_cache
    if _cred_cache is not None:
        return _cred_cache
    out = {}
    try:
        with open(CRED_PATH, "r", encoding="utf-8") as f:
            raw = f.read()
    except OSError:
        _cred_cache = out
        return out
    text = raw.strip()
    if not text:
        _cred_cache = out
        return out
    if text.startswith("{"):
        try:
            data = json.loads(text)
            for k_src, k_dst in (("host", "host"), ("user", "user"),
                                 ("pass", "pass"), ("password", "pass"),
                                 ("pwd", "pass")):
                if data.get(k_src):
                    out[k_dst] = str(data[k_src])
        except ValueError:
            pass
    else:
        lines = [x.strip() for x in text.splitlines() if x.strip()]
        if len(lines) >= 2:
            out["user"] = lines[0]
            out["pass"] = lines[1]
        elif len(lines) == 1:
            out["pass"] = lines[0]
    _cred_cache = out
    return out


def pi_host(default=DEFAULT_HOST):
    return os.environ.get("LSA_PI") or _load_cred().get("host") or default


def pi_user(default=DEFAULT_USER):
    return os.environ.get("LSA_PI_USER") or _load_cred().get("user") or default


def pi_pass():
    """返回密码字符串；没有配置时返回空串（不抛异常，方便 --help / 纯本地路径）。"""
    return os.environ.get("LSA_PI_PASS") or _load_cred().get("pass") or ""


def pi_pass_or_die():
    """需要真正建立 SSH 连接时用：没有密码就抛 RuntimeError 并给出设置方法。"""
    pwd = pi_pass()
    if pwd:
        return pwd
    raise RuntimeError(
        "缺少树莓派密码。请任选一种方式配置后重试：\n"
        "  1) 环境变量：$env:LSA_PI_PASS='<密码>'   （bash: export LSA_PI_PASS='<密码>'）\n"
        "  2) 凭据文件：" + CRED_PATH + "\n"
        '     JSON  {"host":"10.110.75.61","user":"frozen-laffer","pass":"<密码>"}\n'
        "     或两行纯文本：第一行用户名，第二行密码\n"
        "可选：LSA_PI 指定地址，LSA_PI_USER 指定用户名，LSA_PI_CRED 指定凭据文件路径。"
    )


def has_pass():
    return bool(pi_pass())


if __name__ == "__main__":
    import sys

    if "--init" in sys.argv:
        # 交互式生成凭据文件：python pi_env.py --init
        import getpass
        args = [a for a in sys.argv[1:] if not a.startswith("--")]
        host = args[0] if len(args) > 0 else input("树莓派地址 [%s]: " % DEFAULT_HOST).strip() or DEFAULT_HOST
        user = args[1] if len(args) > 1 else input("用户名 [%s]: " % DEFAULT_USER).strip() or DEFAULT_USER
        pwd = getpass.getpass("密码（不回显）: ")
        if not pwd:
            raise SystemExit("密码为空，未写入。")
        os.makedirs(os.path.dirname(CRED_PATH) or ".", exist_ok=True)
        with open(CRED_PATH, "w", encoding="utf-8") as f:
            json.dump({"host": host, "user": user, "pass": pwd}, f, ensure_ascii=False)
        try:
            os.chmod(CRED_PATH, 0o600)
        except OSError:
            pass
        print("已写入 %s（内容：host=%s user=%s pass=***）" % (CRED_PATH, host, user))
        raise SystemExit(0)

    print("host     :", pi_host())
    print("user     :", pi_user())
    print("cred file:", CRED_PATH, "(存在)" if os.path.exists(CRED_PATH) else "(不存在)")
    print("password :", ("已配置（长度 %d）" % len(pi_pass())) if has_pass() else "未配置")
    print("\n配置方法：python pi_env.py --init   （交互式写凭据文件）")
    print("     或：设置环境变量 LSA_PI_PASS（可选 LSA_PI / LSA_PI_USER）")
