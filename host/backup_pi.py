# -*- coding: utf-8 -*-
"""树莓派侧文件的整体备份 / 恢复（一个命令搞定）。

    python backup_pi.py pull      # 把 Pi 上 /home/frozen-laffer/lsa 的关键文件抓回 pi_backup/
    python backup_pi.py push      # 把 host_app 里的代码 + pi_backup 里的数据推回 Pi
    python backup_pi.py list      # 只看本地备份里有什么

备份目录 pi_backup/ 里放的是**数据文件**（校准表、偏置、上次装载参数），
代码文件本来就在 git 里，push 时直接从仓库目录取。

密码不在代码里，见 pi_env.py：环境变量 LSA_PI_PASS，或凭据文件 ~/.lsa_pi。
"""
import io
import json
import os
import sys

import paramiko

from pi_env import pi_host, pi_pass_or_die, pi_user

HOST = pi_host()
USER = pi_user()
PWD = pi_pass_or_die()
RDIR = "/home/%s/lsa" % USER
LOCAL_CAL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pi_backup")

# 数据文件：只在 Pi 上，必须单独备份
DATA = ["eq.json", "config.json", "calib.json", "excite.json"]
# 代码文件：git 里有，push 时从仓库目录取
CODE = ["lsa_proto.py", "lsa_dsp.py", "lsa_eq.py", "lsa_cfg.py", "lsa_calib.py",
        "lsa_src.py", "lsa_cli.py", "lsa_app.py", "lsa_touch.py", "dg4202.py",
        "cap_one.py", "_contest_one.py", "start_touch.sh", "boot_start.sh"]
# 自启配置
AUTOSTART = "/home/frozen-laffer/.config/autostart/lsa-touch.desktop"


def conn(tries=6):
    last = None
    for i in range(tries):
        try:
            c = paramiko.SSHClient()
            c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            c.connect(HOST, port=22, username=USER, password=PWD, timeout=20,
                      allow_agent=False, look_for_keys=False)
            return c
        except Exception as e:          # WiFi 会间歇掉包
            last = e
            import time
            time.sleep(2 + 2 * i)
    raise last


def pull():
    os.makedirs(LOCAL_CAL, exist_ok=True)
    c = conn()
    s = c.open_sftp()
    ok = []
    for fn in DATA:
        try:
            s.get(RDIR + "/" + fn, os.path.join(LOCAL_CAL, fn))
            ok.append(fn)
        except IOError:
            print("  跳过（Pi 上没有）:", fn)
    try:
        s.get(AUTOSTART, os.path.join(LOCAL_CAL, "lsa-touch.desktop"))
        ok.append("lsa-touch.desktop")
    except IOError:
        pass
    s.close()
    c.close()
    print("已备份 %d 个文件到 %s" % (len(ok), LOCAL_CAL))
    for fn in ok:
        print("  ", fn)
    # 顺手打印校准表的摘要，便于确认备份的是哪一轮
    p = os.path.join(LOCAL_CAL, "eq.json")
    if os.path.exists(p):
        d = json.load(io.open(p, encoding="utf-8"))
        print("  eq.json: %d 点表, model=%s, phase %d 行"
              % (len(d.get("table") or []), d.get("model"),
                 len(d.get("phase") or [])))


def push():
    here = os.path.dirname(os.path.abspath(__file__))
    c = conn()
    s = c.open_sftp()
    try:
        s.stat(RDIR)
    except IOError:
        s.mkdir(RDIR)
    n = 0
    for fn in CODE:
        p = os.path.join(here, fn)
        if os.path.exists(p):
            s.put(p, RDIR + "/" + fn)
            n += 1
        else:
            print("  本地缺:", fn)
    for fn in DATA:
        p = os.path.join(LOCAL_CAL, fn)
        if os.path.exists(p):
            s.put(p, RDIR + "/" + fn)
            n += 1
    d = os.path.join(LOCAL_CAL, "lsa-touch.desktop")
    if os.path.exists(d):
        c.exec_command("mkdir -p /home/frozen-laffer/.config/autostart")[1].channel.recv_exit_status()
        s.put(d, AUTOSTART)
        n += 1
    s.close()
    c.exec_command("chmod +x %s/start_touch.sh %s/boot_start.sh; sync" % (RDIR, RDIR))[1].channel.recv_exit_status()
    c.close()
    print("已推送 %d 个文件到 %s" % (n, RDIR))


def lst():
    if not os.path.isdir(LOCAL_CAL):
        print("还没有本地备份，先跑 pull")
        return
    for fn in sorted(os.listdir(LOCAL_CAL)):
        p = os.path.join(LOCAL_CAL, fn)
        print("  %-24s %7d B  %s" % (fn, os.path.getsize(p),
                                     io.open(p, encoding="utf-8",
                                             errors="replace").read(160)
                                     .replace("\n", " ")[:70]))


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "pull"
    {"pull": pull, "push": push, "list": lst}[cmd]()
