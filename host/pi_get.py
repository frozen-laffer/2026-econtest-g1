"""从树莓派抓一个文件下来。用法：python pi_get.py <远端路径> <本地路径>"""
import os
import sys
import paramiko

from pi_env import pi_host, pi_pass_or_die, pi_user

HOST = pi_host()
USER = pi_user()
PWD = pi_pass_or_die()

remote, local = sys.argv[1], sys.argv[2]
c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(HOST, port=22, username=USER, password=PWD, timeout=10,
          allow_agent=False, look_for_keys=False)
sftp = c.open_sftp()
sftp.get(remote, local)
sftp.close()
c.close()
print("get %s -> %s" % (remote, local))
