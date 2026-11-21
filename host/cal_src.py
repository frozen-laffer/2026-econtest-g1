# -*- coding: utf-8 -*-
"""信号源适配层：让 PC 侧工具脚本同时支持 AFG-2225 与 DG4202。

两种走法
--------
* ``--src afg``（默认）：源是接在 **PC 的 COM17** 上的 AFG-2225，方言为 GW Instek
  （``SOUR1:FREQ`` / ``SOUR1:AMP`` / ``SOUR1:DCO`` / ``OUTP1 ON``）。采集在
  树莓派上，本模块用 paramiko SSH 到 ``PI_HOST`` 执行 ``cap_one.py`` /
  ``_contest_one.py``。**每条命令、顺序、回读校验都与改造前逐条一致**，
  所以 AFG 跑出来的数与改造前相同。
* ``--src dg``：源是插在**树莓派**上的 RIGOL DG4202（``/dev/usbtmc0``，方言
  ``:SOUR1:FREQ`` / ``:SOUR1:VOLT`` / ``:SOUR1:VOLT:OFFS`` / ``:OUTP1 ON``）。
  此时本机就是树莓派，采集直接 ``subprocess.run([sys.executable, "cap_one.py",
  ...])``。Windows 上没有 ``/dev/usbtmc0``，``open_adapter("dg")`` 会直接抛
  ``RuntimeError`` 说明原因（不会退化成 AttributeError / 一串 traceback）。

统一接口
--------
    ad = open_adapter(kind="afg", port=None, pi_host=PI_HOST, pi_user=PI_USER,
                      pi_pwd=PI_PASS, pi_dir=PI_DIR)
    ad.idn()                      # 身份字符串
    ad.set_sine(hz, vpp)          # 按源类型发对应 SCPI，回读确认
                                  # -> (实测频率 Hz, 实测幅度 Vpp)
    ad.load_wave(vals, freq_hz, amp_vpp, ch=1, tones=[(绝对次数, Vpp, 相位), ...])
                                  # AFG 转 load_arb；DG 先试硬件谐波、不行退回 ARB
    ad.cap(gear=0, n=16384, apply_=False, wave=None, harm=None)
                                  # -> cap_one.py（或 _contest_one.py）的 JSON dict
    ad.close()                    # 收尾

另有给 cal_step.py 用的 ``setup_sine()`` / ``pi_exec()`` / ``pi_put()`` /
``stop_gui()``。
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))

# 树莓派连接参数统一由 pi_env.py 提供（环境变量 LSA_PI / LSA_PI_USER / LSA_PI_PASS，
# 或凭据文件 ~/.lsa_pi）。密码为空的场景（如 --src afg 纯本地）不会报错，
# 只有真正要 SSH 连树莓派时才会抛 RuntimeError 提示怎么配。
from pi_env import pi_host, pi_pass, pi_user  # noqa: E402

PI_HOST = pi_host()
PI_USER = pi_user()
PI_PASS = pi_pass()
PI_DIR = "/home/%s/lsa" % PI_USER

AFG_PORT = "COM17"

DG_NT_MSG = (
    "DG 直连模式下请在树莓派上运行本脚本：DG4202 只挂在树莓派的 /dev/usbtmc0 "
    "上（Windows 上没有这个设备，pyusb 也打不开它）。"
    "要在 PC 上跑请改用 --src afg（AFG-2225 接 COM17）。"
)


def afg_set_sine(raw, f_hz, amp_vpp, ch=1):
    """AFG-2225 单点下发的**逐行原版**（只改频率和幅度）。

    改造前是 ``cal_step.set_afg``；这里原样搬过来做成模块级函数，所以
    ``cal_src`` 与 ``cal_step.set_afg`` 共用同一份 SCPI 序列，不会走岔。
    ``raw`` 只要有 ``w(cmd, settle=)`` / ``one(cmd)`` 即可（afg2225.Scpi 或
    lsa_src.Afg2225 都行）。
    """
    hz = float(f_hz)
    vpp = float(amp_vpp)
    p = "SOUR%d:" % int(ch)
    raw.w(p + "FREQ %.0f" % hz, settle=0.45)
    raw.w(p + "AMP %.4f" % vpp, settle=0.45)
    fq = raw.one(p + "FREQ?")
    am = raw.one(p + "AMP?")
    try:
        ok = abs(float(fq) - hz) < 1.0 and abs(float(am) - vpp) < 2e-3
    except (TypeError, ValueError):
        ok = False
    if not ok:                      # 偶发丢命令，重写一次
        raw.w(p + "FREQ %.0f" % hz, settle=0.6)
        raw.w(p + "AMP %.4f" % vpp, settle=0.6)
        fq = raw.one(p + "FREQ?")
        am = raw.one(p + "AMP?")
    return float(fq or 0), float(am or 0)


def ensure_supported(kind="afg"):
    """只做类型/平台检查，不碰硬件（脚本可以在读表之前先调用）。"""
    kind = (kind or "afg").strip().lower()
    if kind not in ("afg", "afg2225", "dg", "dg4202"):
        raise RuntimeError("未知信号源类型：%r（只支持 afg / dg）" % (kind,))
    if kind in ("dg", "dg4202") and os.name == "nt":
        raise RuntimeError(DG_NT_MSG)
    return kind


class _Adapter(object):
    """两种源共用的壳：cap / SSH / 收尾。子类只管 SCPI 方言。"""

    kind = "?"
    is_afg = False

    def __init__(self, port=None, pi_host=PI_HOST, pi_user=PI_USER,
                 pi_pwd=PI_PASS, pi_dir=PI_DIR):
        self.port = port
        self.pi_host = pi_host
        self.pi_user = pi_user
        self.pi_pwd = pi_pwd
        self.pi_dir = pi_dir
        self.ssh = None
        self.src = None

    # ---- 底层 SCPI（脚本里偶尔要发一条源专属命令时用）------------------
    def w(self, cmd, settle=0.25):
        return self.src.w(cmd, settle)

    def one(self, cmd, default=""):
        return self.src.one(cmd, default)

    def remote(self):
        return self.src.remote()

    def local(self):
        return self.src.local()

    def idn(self):
        return self.src.identify()

    def output(self, on, ch=1):
        return self.src.output(on, ch)

    # ---- 波形 ---------------------------------------------------------
    def load_wave(self, vals, freq_hz, amp_vpp, ch=1, offset=0.0, tones=None,
                  reason=None):
        """统一下发入口；返回 (mode, note)。

        AFG 的 ``lsa_src.Source.load_wave`` 默认就是转 ``load_arb``；DG 会优先
        用硬件谐波模式，表示不了（次数>16、非整数倍、带干扰、要求非零相位）
        时打印原因并自动退回 ARB。
        """
        return self.src.load_wave(vals, freq_hz, amp_vpp, ch=ch, offset=offset,
                                  tones=tones, reason=reason)

    # ---- 正弦单点（子类实现方言）--------------------------------------
    def set_sine(self, hz, vpp, ch=1):
        raise NotImplementedError

    def setup_sine(self, ch=1, readback=False):
        """开头统一次把通道切到正弦/输出开；readback=True 时回读输出开关。"""
        self.src.output(True, ch)
        return self.one(":OUTP%d?" % int(ch)) if readback else None

    # ---- 采集 ---------------------------------------------------------
    def cap(self, gear=0, n=16384, apply_=False, wave=None, harm=None,
            ch=None, retries=1, timeout=120):
        """采一帧，返回脚本打印的 JSON dict。

        harm 为 None 时跑 ``cap_one.py``；给定次数（如 20）时跑
        ``_contest_one.py --harm N``（_matrix_run / _phase_check 要各次分量）。
        """
        if os.name == "nt" and not self.is_afg:
            raise RuntimeError(DG_NT_MSG)      # 不重试、不吞成"采集失败"
        script = "_contest_one.py" if harm is not None else "cap_one.py"
        args = ["--gear", str(int(gear)), "--n", str(int(n))]
        if ch is not None:
            args += ["--ch", str(int(ch))]
        if harm is not None:
            args += ["--harm", str(int(harm))]
        if apply_:
            args += ["--apply"]
        if wave:
            args += ["--wave", str(wave)]
        tries = max(1, int(retries))
        last = ""
        for _ in range(tries):
            try:
                txt = self._run_capture(script, args, timeout)
            except Exception as exc:          # SSH 断了 / 子进程超时
                last = "%s: %s" % (type(exc).__name__, exc)
                time.sleep(3.0)
                continue
            lines = [ln for ln in (txt or "").strip().splitlines()
                     if ln.strip().startswith("{")]
            if lines:
                return json.loads(lines[-1])
            last = (txt or "").strip()[-300:]
            time.sleep(2.5)                   # 树莓派欠压 / I/O 错误时等一下再试
        raise RuntimeError("采集失败（%s，已重试 %d 次）：%s" % (script, tries, last))

    def _run_capture(self, script, args, timeout):
        raise NotImplementedError

    # ---- 树莓派侧辅助（DG 模式下就是本机）------------------------------
    def pi_exec(self, cmd, timeout=90):
        """在树莓派上执行一条 shell 命令，返回 (stdout, stderr)。"""
        raise NotImplementedError

    def pi_put(self, local, remote):
        raise NotImplementedError

    def pi_ping(self):
        """确认采集端可用（AFG 模式建 SSH；DG 模式本机无需连接）。"""
        return True

    def stop_gui(self, wait=0.8):
        """板子只允许一条 TCP 连接：采集前先停掉触摸界面。"""
        self.pi_exec("pkill -f '[l]sa_touch.py'", timeout=20)
        time.sleep(wait)

    # ---- 收尾 ---------------------------------------------------------
    def close(self, restore=True):
        """关连接。

        restore=False：只关传输，不发 ``SYST:LOC`` —— cal_step.py 正常收尾
        历史上就是这样（AFG 留在远程，进程退出时串口才释放）。
        """
        try:
            if self.ssh is not None:
                self.ssh.close()
                self.ssh = None
        except Exception:
            pass
        try:
            if self.src is not None:
                if restore:
                    self.src.close()
                else:
                    ser = getattr(self.src, "s", None)
                    if ser is not None:
                        ser.close()
        except Exception:
            pass


class _AfgAdapter(_Adapter):
    kind = "afg"
    is_afg = True

    def __init__(self, port=None, **kw):
        _Adapter.__init__(self, port=port or AFG_PORT, **kw)
        import lsa_src
        # 显式给端口：PC 上是 COM17，不再让 _pick_port 去猜 /dev/ttyACM*
        self.src = lsa_src.open_source("afg", self.port)
        self.silent = bool(getattr(self.src, "silent", False))

    # ---- SCPI 方言 -----------------------------------------------------
    def setup_sine(self, ch=1, readback=False):
        """与改造前 cal_quick / cal_step 开头那四条命令逐条一致。"""
        p = "SOUR%d:" % int(ch)
        self.w(p + "FUNC SIN", 0.5)
        self.w(p + "VOLT:UNIT VPP", 0.5)
        self.w(p + "DCO 0", 0.5)
        self.w("OUTP%d ON" % int(ch), 0.5)
        return self.one("OUTP%d?" % int(ch)) if readback else None

    def set_sine(self, hz, vpp, ch=1):
        """与改造前 ``cal_step.set_afg`` 完全同一条序列（共用 afg_set_sine）。"""
        return afg_set_sine(self.src, hz, vpp, ch=ch)

    # ---- 采集（源在 PC、采集在树莓派）--------------------------------
    def _ssh(self):
        if self.ssh is None:
            import paramiko
            from pi_env import pi_pass_or_die
            # 密码可能来自构造参数（pi_pwd），也可能来自环境/凭据文件
            pwd = self.pi_pwd or pi_pass_or_die()
            c = paramiko.SSHClient()
            c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            c.connect(self.pi_host, username=self.pi_user, password=pwd,
                      timeout=20, allow_agent=False, look_for_keys=False)
            self.ssh = c
        return self.ssh

    def pi_ping(self):
        self._ssh()
        return True

    def pi_exec(self, cmd, timeout=90):
        _i, o, e = self._ssh().exec_command(cmd, timeout=timeout)
        return (o.read().decode("utf-8", "replace"),
                e.read().decode("utf-8", "replace"))

    def pi_put(self, local, remote):
        s = self._ssh().open_sftp()
        try:
            s.put(local, remote)
        finally:
            s.close()

    def _run_capture(self, script, args, timeout):
        cmd = "cd %s && python3 %s %s 2>&1" % (self.pi_dir, script, " ".join(args))
        out, _err = self.pi_exec(cmd, timeout=timeout)
        return out


class _DgAdapter(_Adapter):
    kind = "dg"
    is_afg = False

    def __init__(self, port=None, **kw):
        _Adapter.__init__(self, port=port, **kw)
        if os.name == "nt":
            raise RuntimeError(DG_NT_MSG)
        import lsa_src
        # DG4202.open_transport() 自己选 /dev/usbtmc0
        self.src = lsa_src.open_source("dg", self.port)

    def set_sine(self, hz, vpp, ch=1):
        """DG 方言由 Dg4202.set_point 负责（FREQ / VOLT ...VPP + 回读校验）。"""
        hz = float(hz)
        vpp = float(vpp)
        r = self.src.set_point(hz, vpp, ch=int(ch)) or {}
        f, a = r.get("freq_hz"), r.get("amp_vpp")
        if f is None or a is None:
            rb = self.src.readback(int(ch)) or {}
            f = rb.get("freq_hz")
            a = rb.get("amp_vpp")
        return float(f or 0), float(a or 0)

    # ---- 采集（本机就是树莓派）----------------------------------------
    def pi_exec(self, cmd, timeout=90):
        p = subprocess.run(["bash", "-lc", cmd], capture_output=True,
                           text=True, timeout=timeout)
        return p.stdout or "", p.stderr or ""

    def pi_put(self, local, remote):
        try:
            if os.path.abspath(local) == os.path.abspath(remote):
                return
        except Exception:
            pass
        shutil.copyfile(local, remote)

    def _run_capture(self, script, args, timeout):
        if os.name == "nt":               # 兜底：open_adapter 已经拦过
            raise RuntimeError(DG_NT_MSG)
        cmd = [sys.executable, script] + list(args)
        p = subprocess.run(cmd, cwd=self.pi_dir, capture_output=True,
                           text=True, timeout=timeout)
        if p.returncode != 0 and not (p.stdout or "").strip():
            raise RuntimeError((p.stderr or "").strip()[-300:]
                               or ("%s 退出码 %d" % (script, p.returncode)))
        return (p.stdout or "") + "\n" + (p.stderr or "")

    def close(self, restore=True):
        _Adapter.close(self, restore=True)   # DG 的 local() 不发命令


def open_adapter(kind="afg", port=None, pi_host=PI_HOST, pi_user=PI_USER,
                 pi_pwd=PI_PASS, pi_dir=PI_DIR):
    """按 ``kind`` 打开适配器。kind: afg / dg（大小写不敏感）。

    DG 在 Windows 上直接抛 RuntimeError，提示"请在树莓派上运行本脚本"。
    """
    kind = ensure_supported(kind)
    if kind in ("afg", "afg2225"):
        return _AfgAdapter(port=port, pi_host=pi_host, pi_user=pi_user,
                           pi_pwd=pi_pwd, pi_dir=pi_dir)
    if kind in ("dg", "dg4202"):
        return _DgAdapter(port=port, pi_host=pi_host, pi_user=pi_user,
                          pi_pwd=pi_pwd, pi_dir=pi_dir)
    raise RuntimeError("未知信号源类型：%r（只支持 afg / dg）" % (kind,))
