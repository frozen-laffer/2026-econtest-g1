"""在树莓派上执行命令的最小封装（paramiko）。用法：python pi_run.py "cmd" ["cmd2" ...]

注意：参数按空格切分后 exec，**带引号的复杂命令会被切坏**；复杂命令请改用 pi_exec.py。
密码不在代码里，见 pi_env.py：环境变量 LSA_PI_PASS，或凭据文件 ~/.lsa_pi。
"""
import os
import sys
import paramiko

from pi_env import pi_host, pi_pass_or_die, pi_user

HOST = pi_host()
USER = pi_user()
PWD = pi_pass_or_die()


def connect():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, port=22, username=USER, password=PWD,
              timeout=10, banner_timeout=10, auth_timeout=10,
              allow_agent=False, look_for_keys=False)
    return c


def run(c, cmd, timeout=60):
    stdin, stdout, stderr = c.exec_command(cmd, timeout=timeout, get_pty=False)
    out = stdout.read().decode("utf-8", "replace")
    err = stderr.read().decode("utf-8", "replace")
    rc = stdout.channel.recv_exit_status()
    return rc, out, err


if __name__ == "__main__":
    c = connect()
    for cmd in sys.argv[1:]:
        print("$ " + cmd)
        rc, out, err = run(c, cmd)
        if out.strip():
            print(out.rstrip())
        if err.strip():
            print("[stderr] " + err.rstrip())
        print("[rc=%d]\n" % rc)
    c.close()
