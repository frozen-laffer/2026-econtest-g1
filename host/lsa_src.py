# -*- coding: utf-8 -*-
"""信号源驱动（树莓派侧，USB 直连）。

为什么放在树莓派上
------------------
阶梯校准要逐个频点改信号源。原来靠人在 PC 上敲 SCPI，树莓派只负责采集，
两边要来回切窗口；把源挂到树莓派上之后，「设频点 → 采集 → 记录」可以在
一个进程里跑完，做到一键 21 点。

支持的设备
----------
* AFG-2225 —— USB CDC 虚拟串口（VID 0x2184 / PID 0x001C），SCPI 纯 ASCII。
  三个坑（详见 afg2225.py 的注释）：只认短助记符且不能带前导冒号；幅度节点是
  ``SOUR1:AMP``;必须先 ``SYST:REM`` 否则写命令被静默丢弃。
* DG4202 —— 预留，等实际接上再补（RIGOL 的 ARB/谐波发生器方言与 GW 不同，
  且 16 kpts 下采样率要按频点重算，不能照抄这里的实现）。

用法
----
    python3 lsa_src.py idn
    python3 lsa_src.py list
    python3 lsa_src.py set 100000 0.25
    python3 lsa_src.py off
"""
from __future__ import annotations

import glob
import os
import socket
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None


class SourceError(RuntimeError):
    pass


def find_ports():
    """列出可能是信号源的串口，返回 [(device, description), ...]。"""
    out = []
    if list_ports is not None:
        for p in list_ports.comports():
            out.append((p.device, p.description or ""))
    if not out:
        for dev in sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")):
            out.append((dev, ""))
    return out


def _pick_port(port=None):
    if port:
        return port
    ports = find_ports()
    if not ports:
        raise SourceError("没有找到串口设备（/dev/ttyACM* 或 /dev/ttyUSB* 均为空）")
    # 优先 /dev/ttyACM*（CDC-ACM，AFG-2225 就是这一类），其次 ttyUSB*
    acm = [d for d, _ in ports if "ACM" in d]
    return (acm or [d for d, _ in ports])[0]


class Source(object):
    """通用接口。上层（StepCalib）只认这四个方法。"""

    name = "source"
    silent = True          # 默认假设不能回读；Afg2225 会按 *IDN? 结果覆盖

    def identify(self):
        raise NotImplementedError

    def set_point(self, hz, vpp, ch=1):
        raise NotImplementedError

    def output(self, on, ch=1):
        raise NotImplementedError

    def load_arb(self, vals, freq_hz, amp_vpp, ch=1, offset=0.0):
        """下载任意波形（全测试要用「基波 + 1~2 个谐波」的合成波）。"""
        raise SourceError("%s 不支持任意波形下载" % self.name)

    def load_wave(self, vals, freq_hz, amp_vpp, ch=1, offset=0.0, tones=None,
                  reason=None):
        """装载一个「基波 + 若干次谐波」的波形，能走谐波模式就走谐波模式。

        默认实现（AFG 等只能下载任意波的源）直接转 load_arb。
        tones: [(order, amp_vpp, phase_deg), ...]，order 是相对基波的整数次数
               （1 = 基波，其幅度由 amp_vpp 给出）。None = 调用方只知道采样点。
        返回 (mode, note)：mode 是 "arb" / "harmonic"，note 给人看的一句话。
        """
        self.load_arb(vals, freq_hz, amp_vpp, ch=ch, offset=offset)
        return "arb", "%s：任意波（无谐波模式）" % self.name

    def readback(self, ch=1):
        """回读当前设定；没有回读能力的源返回空值（调用方会跳过校验）。"""
        return {"func": "", "freq_hz": None, "amp_vpp": None}

    def close(self):
        pass


class Afg2225(Source):
    name = "AFG-2225"
    VID = 0x2184
    SETTLE = 0.45          # 设完频点后等源稳定；实测 <0.3 s 时读数还在爬

    def __init__(self, port=None, baud=115200):
        if serial is None:
            raise SourceError("pyserial 未安装（pip install pyserial）")
        self.port = _pick_port(port)
        try:
            self.s = serial.Serial(self.port, baud, timeout=0.25, write_timeout=2.0)
        except Exception as exc:
            raise SourceError("打开 %s 失败：%r" % (self.port, exc))
        time.sleep(0.25)
        self.s.reset_input_buffer()
        self._quiet = 0.8
        self._remote_done = False
        # 有些平台（实测树莓派 + 这块 CDC 实现）能收命令但**不回任何字节**：
        # 此时把回读校验降级掉，靠板上测到的频率做独立校验。
        self.silent = not bool(self.one("*IDN?"))
        # 两通道并联接进板子时，CH2 必须跟着 CH1 一起改频率，否则它一直挂在
        # 上一个频点上，会把每一个标定点都污染成「基波 + 固定干扰」。
        self.mirror = ()
        env = os.environ.get("LSA_SRC_MIRROR", "auto").strip().lower()
        try:
            if env in ("", "0", "off", "no"):
                self.mirror = ()
            elif env == "auto":
                if not self.silent and self.one("OUTP2?") == "1":
                    self.mirror = (2,)
            else:
                self.mirror = tuple(int(x) for x in env.split(",") if x.strip())
        except Exception:
            self.mirror = ()

    # ---- 底层 ---------------------------------------------------------
    def q(self, cmd, maxlines=8):
        """查询并读干净。某些查询会回两行，只读一行会让后续应答全部错位。

        AFG-2225 的 USB-CDC 有一个坑：上一条命令的应答会"迟到"，落在下一次
        查询里，导致整串应答错位一格（`*IDN?` 读到上一条 `OUTP2?` 的 "0"）。
        所以写之前先等 60 ms 让在途应答到达，再 reset 掉。
        """
        time.sleep(0.06)
        self.s.reset_input_buffer()
        self.s.write((cmd + "\n").encode("ascii"))
        self.s.flush()
        lines, t0 = [], time.time()
        while time.time() - t0 < self._quiet:
            line = self.s.readline()
            if not line:
                if lines:
                    break
                continue
            txt = line.decode("ascii", "replace").strip()
            if txt:
                lines.append(txt)
            if len(lines) >= maxlines:
                break
        return lines

    def one(self, cmd, default="", tries=2):
        for _ in range(tries):
            r = self.q(cmd)
            if r:
                return r[0]
        return default

    def w(self, cmd, settle=0.25):
        self.s.write((cmd + "\n").encode("ascii"))
        self.s.flush()
        time.sleep(settle)

    def remote(self):
        if not self._remote_done:
            self.w("SYST:REM", 0.25)
            self._remote_done = True
        return self

    def local(self):
        try:
            self.w("SYST:LOC", 0.2)
        except Exception:
            pass
        self._remote_done = False

    def errors(self, limit=8):
        out = []
        for _ in range(limit):
            r = self.q("SYST:ERR?")
            if not r:
                break
            txt = " ".join(r)
            if r[0].startswith("+0") or txt.lower().startswith("no error"):
                break
            out.append(txt)
        return out

    # ---- 接口 ---------------------------------------------------------
    def identify(self):
        if self.silent:
            return "AFG-2225 (无回读，只发命令)"
        return self.one("*IDN?", "?")

    def set_mirror(self, chans=()):
        """显式指定镜像通道（并联接法时用）。"""
        self.mirror = tuple(int(c) for c in chans)
        return self.mirror

    def set_point(self, hz, vpp, ch=1):
        """设正弦单点（频率 + 峰峰值）。只写 FREQ/AMP，别的什么都不动。

        **不要**在每点重写 FUNC/VOLT:UNIT：实测在某些固件状态下会吞掉随后的
        FREQ 写命令（四点全部读成第一点的频率），是最隐蔽的一个坑。
        镜像通道（如并联的 CH2）单独按更慢的节奏写。
        """
        self.remote()
        # 上一轮「全测试」会把通道留在 ARB 模式，那里 FREQ 被钳在
        # 120 MHz / 4096 = 29296.875 Hz，写 100 kHz 会被静默钳回，
        # 之后每个校准点都会"频率不对"。所以先确认波形类型是正弦。
        try:
            fn = (self.one("SOUR%d:FUNC?" % int(ch)) or "").upper()
        except Exception:
            fn = ""
        if fn and "SIN" not in fn:
            self.w("SOUR%d:FUNC SIN" % int(ch), 0.6)
        res = self._set_ch(int(ch), hz, vpp)
        for m in self.mirror:
            if int(m) == int(ch):
                continue
            self._set_ch(int(m), hz, vpp, settle=1.0)
        return res

    def _set_ch(self, ch, hz, vpp, settle=0.12):
        p = "SOUR%d:" % ch
        slow = 0.9 if ch != 1 else 0.2      # CH2 写慢了才不被丢
        self.w(p + "FREQ %.10g" % float(hz), settle)
        self.w(p + "AMP %.10g" % float(vpp), settle)
        time.sleep(self.SETTLE if ch == 1 else 0.9)
        if self.silent:
            # 设备不回读：多发一遍防丢写，最终靠板上实测频率验
            self.w(p + "FREQ %.10g" % float(hz), settle)
            self.w(p + "AMP %.10g" % float(vpp), settle)
            return {"freq_hz": None, "amp_vpp": None}
        # 回读校验：真读回来不对就再写（偶发丢写）
        f = self._num(self.one(p + "FREQ?"))
        for _ in range(2):
            if f is not None and abs(f - hz) <= max(1.0, 1e-4 * hz):
                break
            self.w(p + "FREQ %.10g" % float(hz), slow)
            f = self._num(self.one(p + "FREQ?"))
        a = self._num(self.one(p + "AMP?"))
        if a is not None and vpp > 0 and abs(a - vpp) > 0.08 * vpp:
            self.w(p + "AMP %.10g" % float(vpp), slow)
            a = self._num(self.one(p + "AMP?"))
        return {"freq_hz": f, "amp_vpp": a}

    def readback(self, ch=1):
        """回读当前设定（频率/幅度/波形类型），用于采集前的确认。

        AFG-2225 在 ARB 模式下也会如实回 `FUNC?`（显示 ARB），`FREQ?` 回的是
        波形重复频率，不是正弦频率 —— 比较时要用同一个基准。
        """
        p = "SOUR%d:" % ch
        return {"func": self.one(p + "FUNC?", ""),
                "freq_hz": self._num(self.one(p + "FREQ?")),
                "amp_vpp": self._num(self.one(p + "AMP?"))}

    def output(self, on, ch=1):
        self.remote()
        # 固件只认 ON/OFF 助记符："OUTP1 1" 不报错但无效
        self.w("OUTP%d %s" % (ch, "ON" if on else "OFF"), 0.35)
        return self.one("OUTP%d?" % ch)

    def load_arb(self, vals, freq_hz, amp_vpp, ch=1, offset=0.0):
        """下载一周期波形到 ARB 并输出（1~4096 点，采样率 = freq × 点数 ≤ 120 MHz）。

        顺序不能变：**先退出 USER → 下载 DATA:DAC → 写 FREQ/AMP/DCO → 最后切
        FUNC USER**。反过来（先切 USER 再写 FREQ）频率会被复位成 16.67 Hz ——
        手册 P261 说 USER 波形"按当前频率/幅度/偏置输出"，所以要先设好再切。
        """
        import numpy as _np
        v = _np.asarray(vals, dtype=float)
        n = int(v.size)
        if n < 2 or n > 4096:
            raise SourceError("ARB 点数必须在 2..4096，当前 %d" % n)
        # 注意：这里不能拿未取整的 freq_hz 去判上限。excite.json 里存的是
        # 显示用的 29296.9 Hz（0.1 Hz 分辨率），未取整算 4096×29296.9 =
        # 120.0001 MHz 会误报超限；实际下发的是 floor 后的 29296 Hz。
        peak = float(_np.max(_np.abs(v)))
        if peak <= 0:
            raise SourceError("波形全零")
        q = _np.round(v / peak * 511.0).astype(int).clip(-511, 511)
        body = ",".join(str(int(x)) for x in q)
        p = "SOUR%d:" % ch
        # 频率取整必须向下取：round(29296.875)=29297 → 29297×4096=120.0005 MHz
        # 超过 120 MHz 上限，固件会回 "-222 Data out of range: frequency" 并拒绝。
        f_int = int(_np.floor(float(freq_hz)))
        if f_int * n > 120000000:
            raise SourceError("采样率 %d Hz 超过 120 MHz（%d 点 × %d Hz）"
                              % (f_int * n, n, f_int))
        self.remote()
        self.w(p + "FUNC SIN", 0.25)                 # 先退出 USER 才能写 DATA
        self.w("DATA:DAC VOLATILE, 0, " + body, 1.9)
        self.w(p + "VOLT:UNIT VPP", 0.15)
        self.w(p + "FREQ %d" % f_int, 0.3)
        self.w(p + "AMP %.4f" % float(amp_vpp), 0.3)
        self.w(p + "DCO %.4f" % float(offset), 0.2)
        self.w(p + "FUNC USER", 0.4)
        self.output(True, ch=ch)
        return {"n": n, "rate_hz": float(f_int) * n,
                "amp_vpp": float(amp_vpp), "freq_hz": float(f_int)}

    def close(self):
        try:
            self.local()
            self.s.close()
        except Exception:
            pass

    @staticmethod
    def _num(txt):
        try:
            return float(str(txt).split(",")[0])
        except Exception:
            return None


class Dg4202(Source):
    """RIGOL DG4202（USBTMC / WinUSB，底层见 dg4202.py）。

    与 AFG-2225 的实测差异（固件 00.01.14）：
      * 命令带前导冒号，长/短助记符都收；幅度是 ``:VOLT`` 而不是 ``:AMP``；
      * 扫频起止在 ``:SOURn:FREQ:STAR/STOP``，触发源只认 ``INT``（``IMM`` 回 -220）；
      * 没有 COM 口，只能 USBTMC；本机 VISA 坏，需要 WinUSB（Zadig）；
      * ARB 2..16384 点、500 MSa/s；``FUNC USER`` 会复位频率，必须最后写；
      * ``SYST:REM``/``SYST:LOC`` 不被接受（-113），远程状态由底层兜底。
    """
    name = "DG4202"
    SETTLE = 0.35          # 设完频点后等源稳定（USBTMC 写本身有 settle）

    def __init__(self, port=None, baud=None):
        self.port = port or "DG4E202401525"
        self.err = ""
        self.s = None
        self.mirror = ()
        self.silent = True
        # 惰性导入：没装 pyusb 时 AFG 用户仍然可以正常 import/使用 lsa_src
        try:
            import dg4202 as _dg
            self._drv = _dg
            self.s = _dg.Scpi(serial=self.port)
        except Exception as exc:
            self.err = "导入/构造 dg4202 失败：%s" % (exc,)
            return
        try:
            self.silent = not bool(self.s.q("*IDN?"))
            if self.silent:
                self.err = "DG4202 无回读（*IDN? 空）"
        except Exception as exc:
            # 打不开（没绑 WinUSB / 没插）时不抛，降级成只发命令并在 err 里留原因
            self.err = str(exc)
            self.silent = True
        # 两通道并联时 CH2 要跟着 CH1 一起改频点，否则会污染每个标定点
        env = os.environ.get("LSA_SRC_MIRROR", "auto").strip().lower()
        try:
            if env in ("", "0", "off", "no"):
                self.mirror = ()
            elif env == "auto":
                if not self.silent and self.one(":OUTP2?") == "ON":
                    self.mirror = (2,)
            else:
                self.mirror = tuple(int(x) for x in env.split(",") if x.strip())
        except Exception:
            self.mirror = ()

    def _need(self):
        if self.s is None:
            raise SourceError("DG4202 不可用：%s" % (self.err or "未连接"))
        return self.s

    # ---- 底层 ---------------------------------------------------------
    def q(self, cmd, maxlines=8):
        return self._need().q(cmd, maxlines)

    def one(self, cmd, default="", tries=2):
        for _ in range(tries):
            try:
                r = self.q(cmd)
            except Exception:
                r = []
            if r:
                return r[0]
        return default

    def w(self, cmd, settle=0.25):
        self._need().w(cmd, settle)

    def remote(self):
        if self.s is not None:
            self.s.remote()
        return self

    def local(self):
        if self.s is not None:
            try:
                self.s.local()
            except Exception:
                pass

    def errors(self, limit=8):
        return self._need().errors(limit)

    # ---- 接口 ---------------------------------------------------------
    def identify(self):
        if self.s is None:
            return "DG4202 (未连接: %s)" % (self.err or "?")
        if self.silent:
            return "DG4202 (无回读: %s)" % (self.err or "只发命令")
        return self.one("*IDN?", "DG4202")

    def set_mirror(self, chans=()):
        """显式指定镜像通道（并联接法时用）。"""
        self.mirror = tuple(int(c) for c in chans)
        return self.mirror

    def set_point(self, hz, vpp, ch=1):
        """设正弦单点（频率 + 峰峰值）。只写 FREQ/VOLT，别的什么都不动。

        和 AFG 一样，先确认波形类型：上一轮全测试可能把通道留在 USER/HARM，
        那里 FREQ 会被钳位，写 100 kHz 会被静默改掉，之后每个点都“频率不对”。
        """
        self.remote()
        p = ":SOUR%d:" % int(ch)
        try:
            fn = (self.one(p + "FUNC?") or "").upper()
        except Exception:
            fn = ""
        if fn and "SIN" not in fn:
            self.w(p + "FUNC SIN", 0.6)
        res = self._set_ch(int(ch), hz, vpp)
        for m in self.mirror:
            if int(m) == int(ch):
                continue
            self._set_ch(int(m), hz, vpp, settle=0.9)
        return res

    def _set_ch(self, ch, hz, vpp, settle=0.25):
        p = ":SOUR%d:" % ch
        self.w(p + "FREQ %.10g" % float(hz), settle)
        # 带 VPP 后缀：仪器可能停在 VRMS/DBM 单位，裸数值会被按当前单位解释
        self.w(p + "VOLT %.10gVPP" % float(vpp), settle)
        time.sleep(self.SETTLE if ch == 1 else 0.6)
        if self.silent:
            # 设备不回读：多发一遍防丢写，最终靠板上实测频率验
            self.w(p + "FREQ %.10g" % float(hz), settle)
            self.w(p + "VOLT %.10gVPP" % float(vpp), settle)
            return {"freq_hz": None, "amp_vpp": None}
        f = self._num(self.one(p + "FREQ?"))
        for _ in range(2):
            if f is not None and abs(f - hz) <= max(1.0, 1e-4 * hz):
                break
            self.w(p + "FREQ %.10g" % float(hz), 0.4)
            f = self._num(self.one(p + "FREQ?"))
        a = self._vpp(ch)
        if a is not None and vpp > 0 and abs(a - vpp) > 0.08 * vpp:
            self.w(p + "VOLT %.10gVPP" % float(vpp), 0.4)
            a = self._vpp(ch)
        return {"freq_hz": f, "amp_vpp": a}

    def _vpp(self, ch):
        """回读真实 Vpp：单位是 VPP 直接读 ``:VOLT?``，否则用 HIGH-LOW 反算。"""
        p = ":SOUR%d:" % ch
        unit = (self.one(p + "VOLT:UNIT?") or "").strip().upper()
        if unit == "VPP":
            return self._num(self.one(p + "VOLT?"))
        hi = self._num(self.one(p + "VOLT:HIGH?"))
        lo = self._num(self.one(p + "VOLT:LOW?"))
        if hi is not None and lo is not None:
            return hi - lo
        return self._num(self.one(p + "VOLT?"))

    def readback(self, ch=1):
        p = ":SOUR%d:" % ch
        return {"func": self.one(p + "FUNC?", ""),
                "freq_hz": self._num(self.one(p + "FREQ?")),
                "amp_vpp": self._vpp(ch)}

    def output(self, on, ch=1):
        self.remote()
        # 固件只认 ON/OFF 助记符（OUTP1 1 不报错但无效）
        self.w(":OUTP%d %s" % (int(ch), "ON" if on else "OFF"), 0.35)
        return self.one(":OUTP%d?" % int(ch))

    def load_arb(self, vals, freq_hz, amp_vpp, ch=1, offset=0.0):
        """下载一周期波形并输出。底层负责 2..16384 点 / 500 MSa/s 的校验和
        ``FUNC SIN -> DATA:DAC -> FREQ/VOLT/OFFS -> FUNC USER`` 的固定顺序。"""
        try:
            return self._need().load_arb(
                vals, ch=int(ch), freq_hz=float(freq_hz),
                amp_vpp=float(amp_vpp), offset=float(offset))
        except SourceError:
            raise
        except Exception as exc:
            raise SourceError("DG4202 ARB 失败：%s" % (exc,))

    def set_harmonic(self, f1_hz, vpp, order, ch=1, **kw):
        """谐波模式（2..16 次；>16 固件会静默钳位，底层直接拒绝并建议用 ARB）。"""
        try:
            self._need().set_harmonic(f1_hz, vpp, order, ch=int(ch), **kw)
        except SourceError:
            raise
        except Exception as exc:
            raise SourceError("DG4202 谐波模式失败：%s" % (exc,))

    def set_harmonics(self, f1_hz, vpp, tones, ch=1, **kw):
        """多阶谐波（一次配好基波 + 若干次谐波 + 掩码开关）。"""
        try:
            return self._need().set_harmonics(f1_hz, vpp, tones, ch=int(ch), **kw)
        except SourceError:
            raise
        except Exception as exc:
            raise SourceError("DG4202 多阶谐波失败：%s" % (exc,))

    def load_wave(self, vals, freq_hz, amp_vpp, ch=1, offset=0.0, tones=None,
                  reason=None):
        """谐波能表示就走谐波模式，表示不了就提示一句再退回 ARB。

        为什么优先谐波：DG4202 的谐波模式是硬件合成，输出干净、不用上传 4096 点、
        更贴近"信号源真的在被测"的语义；ARB 是软件拼波形，要用 DATA:DAC 下载。

        什么时候表示不了（会打印原因后走 ARB）：
          * 没有谐波结构（只有基波）；
          * 相邻分量不是整数倍关系；
          * 最高次数超过 16（固件 HARM:AMPL 只收 2..16，17 次直接 -220）；
          * 带干扰 u_j（49 次那类）—— 干扰不是基波的整数次谐波，谐波模式放不出来。
        """
        HARM_MAX = 16
        orders = None
        why = ""
        if not tones:
            why = reason or "没有谐波结构（只有采样点）"
        else:
            orders = []
            for m, a, ph in tones:
                try:
                    mi = int(round(float(m)))
                    ai = float(a)
                except (TypeError, ValueError):
                    why = "次数/幅度不是数值"
                    orders = None
                    break
                if abs(float(m) - mi) > 1e-6 or mi < 1:
                    why = "分量 %s 次不是整数" % (m,)
                    orders = None
                    break
                orders.append((mi, ai, ph))
            if orders is not None:
                base_k = min(m for m, _, _ in orders)
                hi = max(m for m, _, _ in orders)
                base = [a for m, a, _ in orders if m == base_k][0]
                # 相对基波的次数必须是整数：DG 的 HARM:AMPL 收的是"相对次数"
                bad_rel = [m for m, _, _ in orders
                           if abs(float(m) / base_k - round(float(m) / base_k)) > 1e-6]
                ncomp = len([m for m, _, _ in orders if m > base_k])
                if base <= 0:
                    why = "基波幅度为 0"
                    orders = None
                elif ncomp == 0:
                    why = "只有基波、没有谐波"
                    orders = None
                elif bad_rel:
                    why = "第 %d 次与基波 %d 次不成整数倍" % (bad_rel[0], base_k)
                    orders = None
                elif any(abs(float(ph or 0.0)) > 1e-6 for _, _, ph in orders):
                    why = ("要求非零相位（有 %.1f°），谐波模式只部分执行"
                           "（实测写 180° 只走出理论一半），相位相关场合必须用 ARB"
                           % max(abs(float(ph or 0.0)) for _, _, ph in orders))
                    orders = None
                elif int(round(hi / float(base_k))) > HARM_MAX:
                    why = ("最高 %d 次（基波 %d 次）→ 相对基波 %d 次，"
                           "超过 DG4202 谐波上限 %d 次"
                           % (hi, base_k, int(round(hi / float(base_k))), HARM_MAX))
                    orders = None
        if orders is not None:
            base_k = min(m for m, _, _ in orders)
            f_play = float(freq_hz) * base_k
            harm = [(int(round(float(m) / base_k)), a, 0.0 if ph is None else float(ph))
                    for m, a, ph in orders if m > base_k]
            try:
                self.set_harmonics(f_play, float(base), harm,
                                   ch=int(ch))
                # 谐波模式配好后要显式打开输出（set_harmonics 只配波形）
                self.output(True, int(ch))
                note = ("谐波模式：基波 %.6g Hz/%.4f Vpp + 相对 %s 次"
                        % (f_play, float(base),
                           "+".join("%d(%.4f Vpp)" % (m, a) for m, a, _ in harm)))
                print("[DG4202] " + note, flush=True)
                return "harmonic", note
            except Exception as exc:
                why = "谐波模式写入失败（%s）" % (exc,)
        note = "ARB 合成（不能用谐波模式：%s）" % why
        print("[DG4202] 提示：" + note, flush=True)
        self.load_arb(vals, freq_hz, amp_vpp, ch=ch, offset=offset)
        return "arb", note


    def harmonic_off(self, ch=1):
        try:
            self._need().harmonic_off(int(ch))
        except Exception as exc:
            raise SourceError("DG4202 退出谐波模式失败：%s" % (exc,))

    def close(self):
        try:
            if self.s is not None:
                self.s.local()
                self.s.close()
        except Exception:
            pass
        self.s = None

    @staticmethod
    def _num(txt):
        try:
            return float(str(txt).split(",")[0])
        except Exception:
            return None


class TcpSource(Source):
    """把 SCPI 通过 TCP 转发给 PC 上的 afg_bridge.py（PC 持有真实串口）。

    为什么需要：AFG-2225 的 USB 是 CDC 类、子类 0x20（非标准 ACM），
    在 Linux 上会绑成 /dev/ttyACM0 但只进不出 —— 命令发过去面板毫无反应。
    Windows 用厂家驱动正常，所以让 PC 继续持有串口，树莓派走网络发命令。
    """

    name = "AFG-2225"
    EOT = "*EOT*"
    SETTLE = 0.45

    def __init__(self, host=None, port=None, timeout=6.0):
        host = host or os.environ.get("LSA_SRC_HOST", "192.168.1.100")
        port = int(port or os.environ.get("LSA_SRC_TCP", "5061"))
        self.addr = (host, port)
        self.timeout = timeout
        try:
            self.s = socket.create_connection(self.addr, timeout=timeout)
        except Exception as exc:
            raise SourceError("连不上信号源中继 %s:%d（%r）" % (host, port, exc))
        self.s.settimeout(timeout)
        self._buf = b""
        self._remote_done = False

    def q(self, cmd, maxlines=8):
        """发一条命令，读回仪器返回的若干行（以 *EOT* 行结束）。"""
        try:
            self.s.sendall((cmd + "\n").encode("ascii"))
        except Exception as exc:
            raise SourceError("中继发送失败：%r" % (exc,))
        lines = []
        t0 = time.time()
        while time.time() - t0 < self.timeout:
            while b"\n" in self._buf:
                raw, self._buf = self._buf.split(b"\n", 1)
                txt = raw.decode("ascii", "replace").strip()
                if txt == self.EOT:
                    return lines
                if txt:
                    lines.append(txt)
                    if len(lines) >= maxlines:
                        return lines
            try:
                chunk = self.s.recv(4096)
            except socket.timeout:
                break
            except Exception:
                break
            if not chunk:
                break
            self._buf += chunk
        return lines

    def one(self, cmd, default=""):
        r = self.q(cmd)
        return r[0] if r else default

    def w(self, cmd, settle=0.25):
        self.q(cmd)
        time.sleep(0.2)

    def remote(self):
        if not self._remote_done:
            self.q("SYST:REM")
            self._remote_done = True
        return self

    def identify(self):
        return self.one("*IDN?", "?")

    def set_point(self, hz, vpp, ch=1):
        self.remote()
        p = "SOUR%d:" % ch
        f = self._num(self.one(p + "FREQ %.10g" % float(hz)))
        a = self._num(self.one(p + "AMP %.10g" % float(vpp)))
        time.sleep(self.SETTLE)
        return {"freq_hz": f, "amp_vpp": a}

    def output(self, on, ch=1):
        self.remote()
        return self.one("OUTP%d %s" % (ch, "ON" if on else "OFF"))

    def close(self):
        try:
            self.q("SYST:LOC")
        except Exception:
            pass
        try:
            self.s.close()
        except Exception:
            pass

    @staticmethod
    def _num(txt):
        try:
            return float(str(txt).split(",")[0])
        except Exception:
            return None


def open_source(kind="auto", port=None):
    """按需要打开信号源。kind: auto / afg / dg / fake。"""
    kind = (kind or "auto").lower()
    if kind == "fake":
        return FakeSource()
    if kind in ("afg", "afg2225"):
        return Afg2225(port)
    if kind == "auto":
        errs = []
        try:
            return Afg2225(port)
        except SourceError as exc:
            errs.append(str(exc))
        # 本机没有可用串口时，退到 PC 上的 TCP 中继（afg_bridge.py）
        try:
            return TcpSource()
        except SourceError as exc:
            errs.append(str(exc))
        raise SourceError("；".join(errs))
    if kind in ("dg", "dg4202"):
        return Dg4202(port)
    raise SourceError("未知信号源类型：%r" % kind)


class FakeSource(Source):
    """没有硬件时的替身：只记录调用，方便离线验证界面流程。"""
    name = "FAKE"

    def __init__(self, port=None):
        self.log = []

    def identify(self):
        return "FAKE,SOURCE,0,0"

    def set_point(self, hz, vpp, ch=1):
        self.log.append(("set", float(hz), float(vpp)))
        return {"freq_hz": float(hz), "amp_vpp": float(vpp)}

    def output(self, on, ch=1):
        self.log.append(("out", bool(on)))
        return "1" if on else "0"

    def load_arb(self, vals, freq_hz, amp_vpp, ch=1, offset=0.0):
        self.log.append(("arb", float(freq_hz), float(amp_vpp), len(vals)))
        return {"n": len(vals), "rate_hz": float(freq_hz) * len(vals),
                "amp_vpp": float(amp_vpp)}

    def close(self):
        pass


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    cmd = argv[1].lower()
    if cmd == "list":
        ports = find_ports()
        if not ports:
            print("没有串口设备")
        for dev, desc in ports:
            print("%-16s %s" % (dev, desc))
        return 0
    kind = "fake" if "--fake" in argv else ("auto")
    port = None
    for i, a in enumerate(argv):
        if a == "--port" and i + 1 < len(argv):
            port = argv[i + 1]
    src = open_source(kind, port)
    try:
        if cmd == "idn":
            print("%s @ %s : %s" % (src.name, getattr(src, "port", "-"),
                                    src.identify()))
        elif cmd == "set":
            hz, vpp = float(argv[2]), float(argv[3])
            r = src.set_point(hz, vpp)
            print("set %g Hz / %g Vpp -> 回读 %s" % (hz, vpp, r))
        elif cmd == "off":
            print("output ->", src.output(False))
        elif cmd == "on":
            print("output ->", src.output(True))
        else:
            print("未知命令", cmd)
            return 2
    finally:
        src.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
