"""把本地 shell 脚本上传到树莓派并执行，避免任何引号转义问题。

用法：python pi_exec.py <local_script.sh> [--sudo]

密码不在代码里，见 pi_env.py：环境变量 LSA_PI_PASS，或凭据文件 ~/.lsa_pi。
--sudo 时用同一份密码喂 `sudo -S`（要求树莓派上该密码同时是 sudo 密码）。
"""
import sys
import os
import paramiko

from pi_env import pi_host, pi_pass_or_die, pi_user

HOST = pi_host()
USER = pi_user()
PWD = pi_pass_or_die()


def main():
    path = sys.argv[1]
    use_sudo = "--sudo" in sys.argv
    timeout = 600.0
    for a in sys.argv[2:]:
        if a.startswith("--timeout="):
            timeout = float(a.split("=", 1)[1])
    src = open(path, "r", encoding="utf-8").read()

    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, port=22, username=USER, password=PWD,
              timeout=10, banner_timeout=10, auth_timeout=10,
              allow_agent=False, look_for_keys=False)

    sftp = c.open_sftp()
    remote = "/tmp/_dsh_task.sh"
    with sftp.open(remote, "w") as f:
        f.write(src)
    sftp.close()

    cmd = ("echo '%s' | sudo -S bash %s" % (PWD, remote)) if use_sudo else ("bash " + remote)
    stdin, stdout, stderr = c.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", "replace")
    err = stderr.read().decode("utf-8", "replace")
    rc = stdout.channel.recv_exit_status()
    if out.strip():
        print(out.rstrip())
    if err.strip():
        print("[stderr]\n" + err.rstrip())
    print("[rc=%d]" % rc)
    c.close()


if __name__ == "__main__":
    main()
