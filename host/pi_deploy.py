"""把上位机文件上传到树莓派 ~/lsa。

注意：用户名是 frozen-laffer（连字符，与主机名相同），不是下划线。
地址会随 WiFi/DHCP 变化，可用环境变量 LSA_PI 覆盖，否则用下面默认值。

密码不在代码里，见 pi_env.py：环境变量 LSA_PI_PASS，或凭据文件 ~/.lsa_pi。
"""
import os
import paramiko

from pi_env import pi_host, pi_pass_or_die, pi_user

HOST = pi_host()
USER = pi_user()
PWD = pi_pass_or_die()
LOCAL = r"F:\STM32_Project\host_app"
FILES = ["lsa_proto.py", "lsa_dsp.py", "lsa_calib.py", "lsa_cfg.py", "lsa_eq.py", "lsa_src.py",
         "lsa_cli.py", "lsa_app.py", "lsa_touch.py", "lsa_log.py", "lsa_param.py",
         "cap_one.py", "selftest_dsp.py", "sweep_capture.py", "check_field.py", "README.md",
         "start_touch.sh", "boot_start.sh",
         # DG4202 接入后新增：驱动本体、全测试子进程脚本、桌面启动器与图标
         "dg4202.py", "_contest_one.py", "lsa-touch.desktop", "lsa_touch_icon.png"]
REMOTE = "/home/%s/lsa" % USER

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(HOST, port=22, username=USER, password=PWD, timeout=10,
          allow_agent=False, look_for_keys=False)
sftp = c.open_sftp()
try:
    sftp.mkdir(REMOTE)
except IOError:
    pass
for name in FILES:
    local = os.path.join(LOCAL, name)
    remote = REMOTE + "/" + name
    sftp.put(local, remote)
    print("put %-14s %8d bytes -> %s" % (name, os.path.getsize(local), remote))
sftp.close()
c.exec_command("chmod +x %s/start_touch.sh %s/boot_start.sh" % (REMOTE, REMOTE))[1].channel.recv_exit_status()
c.close()
print("done（若需开机自启，另把 lsa-touch.desktop 放到 ~/.config/autostart/）")

