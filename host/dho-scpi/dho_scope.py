#!/usr/bin/env python3
"""DHO812 (Rigol) SCPI helper over LAN -- zero third-party dependencies.

Why this file exists
--------------------
The DHO812 USB port speaks USBTMC.  On this PC the USBTMC class driver
(IVI ``ausbtmc``, published as ``oem49.inf``) is installed and the device
reports ``Status: Started``, but there is no user-mode transport to it:
NI-VISA's core runtime is missing, pyvisa/libusb are not installed and
there is no network access to fetch them.  A raw ``CreateFile`` on the
USBTMC device interface opens and writes, but reads never complete.

So the instrument is controlled over its Ethernet port instead.  Rigol
DHO800/900 series accept SCPI either on a raw TCP socket or through
VXI-11.  This module implements the raw-socket transport (``--port 5555``,
the Rigol default) using only the Python standard library, which makes it
work on any machine that can reach the scope.

Typical use
-----------
    python tools/scope.py scan 192.168.1.0/24
    python tools/scope.py --host 192.168.1.60 idn
    python tools/scope.py --host 192.168.1.60 cmd ":CHANnel1:SCALe 2"
    python tools/scope.py --host 192.168.1.60 status
    python tools/scope.py --host 192.168.1.60 wav 1 -o out/ch1.csv
    python tools/scope.py --host 192.168.1.60 fft 1

The ``wav`` subcommand writes a CSV with time in seconds and volts, plus a
short summary (min/max/pk-pk).  ``fft`` additionally prints the fundamental
and the strongest five harmonics so SFDR/THD can be checked against the L5
gate (SFDR >= 45 dBc, THD <= -40 dBc).
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import socket
import struct
import sys
import time
from concurrent.futures import ThreadPoolExecutor

DEFAULT_PORT = 5555          # Rigol raw-socket SCPI port
DEFAULT_TIMEOUT = 5.0
WAVE_PORTS = (5555, 111, 5025, 4880, 80, 22)

# Memory depths the DHO800/900 series accepts for :ACQuire:MDEPth.  The DHO812
# in this project offers up to 25 Mpts, which is what makes a 1 s S2C frame
# (25 ms x 40 symbols) observable at all: at 25 Mpts and a 2.5 s window the
# instrument still delivers ~10 MSa/s, and the box-average decimation below
# brings that back to the ~500 kSa/s the decoder wants.
DEPTH_STEPS = (1000, 10000, 100000, 1000000, 10000000, 25000000)


def pick_depth(points_needed, max_points=None):
    """Smallest supported memory depth that can hold ``points_needed`` samples.

    Falls back to the largest step when even that is not enough (the caller then
    gets a shorter window than asked for, which it must report, not ignore).
    """
    want = max(1, int(points_needed))
    top = DEPTH_STEPS[-1] if max_points is None else int(max_points)
    for v in DEPTH_STEPS:
        if v >= want and v <= top:
            return v
    return min(top, DEPTH_STEPS[-1])



# --------------------------------------------------------------------------
# transport
# --------------------------------------------------------------------------
class SocketTransport:
    """Raw TCP SCPI (Rigol socket mode): one '\n' terminated command per write."""

    def __init__(self, host, port=DEFAULT_PORT, timeout=DEFAULT_TIMEOUT, bind=None):
        self.host = host
        self.port = port
        self.timeout = timeout
        src = (bind, 0) if bind else None
        self.sock = socket.create_connection((host, port), timeout=timeout, source_address=src)
        self.sock.settimeout(timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    # -- low level ---------------------------------------------------------
    def write_raw(self, data: bytes) -> None:
        self.sock.sendall(data)

    def read_line(self) -> bytes:
        """Read one '\n' terminated line (used for short ASCII replies)."""
        buf = bytearray()
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                break
            if not chunk:
                break
            buf += chunk
            if b"\n" in buf:
                break
        return bytes(buf)

    def read_exact(self, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise IOError("connection closed after %d/%d bytes" % (len(buf), n))
            buf += chunk
        return bytes(buf)

    # -- SCPI --------------------------------------------------------------
    def write(self, cmd: str) -> None:
        cmd = cmd.strip()
        self.write_raw(cmd.encode("ascii") + b"\n")

    def read(self) -> str:
        return self.read_line().decode("ascii", "replace").strip()

    def query(self, cmd: str) -> str:
        self.write(cmd)
        return self.read()

    def read_block_lenient(self, quiet: float = 1.5) -> bytes:
        """Read one binary block without trusting the instrument's byte count.

        The DHO800 occasionally announces a block length it never fully sends
        (typically after ``:WAVeform:STOP`` was clamped or an earlier read was
        aborted), and a strict "read exactly N bytes" loop then blocks until the
        socket timeout.  Here the reply is collected until the line goes quiet
        and the header length is only used to trim, never to wait.
        """
        old = self.sock.gettimeout()
        self.sock.settimeout(quiet)
        buf = bytearray()
        try:
            while True:
                try:
                    chunk = self.sock.recv(65536)
                except socket.timeout:
                    break
                if not chunk:
                    break
                buf += chunk
        finally:
            self.sock.settimeout(old)
        if not buf or buf[0:1] != b"#":
            raise IOError("expected binary block, got %r" % bytes(buf[:80]))
        nd = int(buf[1:2])
        hdr = 2 + nd
        try:
            want = int(bytes(buf[2:hdr]))
        except ValueError:
            raise IOError("bad block header %r" % bytes(buf[:hdr + 8]))
        data = bytes(buf[hdr:])
        if want and len(data) > want:
            data = data[:want]
        return data

    def query_binary(self, cmd: str) -> bytes:
        """Send a query whose reply is an IEEE-488.2 definite-length block."""
        self.write(cmd)
        return self.read_block_lenient()

    def drain(self, quiet: float = 0.4) -> int:
        """Discard stale replies left in the socket by an aborted read.

        A killed ``:WAVeform:DATA?`` leaves a half-read binary block in the
        output queue; every later query is then shifted by one and silently
        returns the wrong value.  Draining until the line goes quiet restores
        the request/reply pairing.
        """
        total = 0
        self.sock.settimeout(quiet)
        try:
            while True:
                try:
                    chunk = self.sock.recv(65536)
                except socket.timeout:
                    break
                if not chunk:
                    break
                total += len(chunk)
        finally:
            self.sock.settimeout(self.timeout)
        return total

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


# --------------------------------------------------------------------------
# instrument
# --------------------------------------------------------------------------
class Scope:
    def __init__(self, transport):
        self.t = transport

    # -- basics ------------------------------------------------------------
    def idn(self) -> str:
        return self.t.query("*IDN?")

    def opc(self) -> None:
        self.t.query("*OPC?")

    def reset(self) -> None:
        self.t.write("*RST")
        self.opc()

    # -- acquisition -------------------------------------------------------
    def run(self) -> None:
        self.t.write(":RUN")

    def stop(self) -> None:
        self.t.write(":STOP")

    def single(self) -> None:
        self.t.write(":SINGle")
        self.opc()

    def timebase(self, sec_per_div: float) -> None:
        self.t.write(":TIMebase:MAIN:SCALe %g" % sec_per_div)

    def channel(self, ch: int, scale=None, offset=None, coupling=None,
                probe=None, on=None) -> None:
        if on is not None:
            self.t.write(":CHANnel%d:DISPlay %d" % (ch, 1 if on else 0))
        if scale is not None:
            self.t.write(":CHANnel%d:SCALe %g" % (ch, scale))
        if offset is not None:
            self.t.write(":CHANnel%d:OFFSet %g" % (ch, offset))
        if coupling is not None:
            self.t.write(":CHANnel%d:COUPling %s" % (ch, coupling))
        if probe is not None:
            self.t.write(":CHANnel%d:PROBe %g" % (ch, probe))

    def sample_rate(self) -> float:
        try:
            return float(self.t.query(":ACQuire:SRATe?").split(",")[0])
        except (ValueError, IndexError):
            return 0.0

    def status(self) -> dict:
        out = {}
        for key, cmd in (
            ("idn", "*IDN?"),
            ("run_state", ":RUNState?"),
            ("timebase", ":TIMebase:MAIN:SCALe?"),
            ("srate", ":ACQuire:SRATe?"),
            ("acq_type", ":ACQuire:TYPE?"),
        ):
            try:
                out[key] = self.t.query(cmd)
            except Exception as exc:  # noqa: BLE001 - report, do not die
                out[key] = "<%s>" % exc
        for ch in (1, 2):
            for key, cmd in (
                ("scale", ":CHANnel%d:SCALe?" % ch),
                ("offset", ":CHANnel%d:OFFSet?" % ch),
                ("coupling", ":CHANnel%d:COUPling?" % ch),
                ("probe", ":CHANnel%d:PROBe?" % ch),
            ):
                try:
                    out["ch%d_%s" % (ch, key)] = self.t.query(cmd)
                except Exception as exc:  # noqa: BLE001
                    out["ch%d_%s" % (ch, key)] = "<%s>" % exc
        return out

    # -- waveform ----------------------------------------------------------
    def preamble(self, ch: int) -> dict:
        self.t.write(":WAVeform:SOURce CHANnel%d" % ch)
        self.t.write(":WAVeform:MODE NORMal")
        self.t.write(":WAVeform:FORMat BYTE")
        raw = self.t.query(":WAVeform:PREamble?")
        vals = [v for v in raw.replace(";", ",").split(",") if v.strip()]
        keys = ["format", "type", "points", "count", "xincrement", "xorigin",
                "xreference", "yincrement", "yorigin", "yreference"]
        out = {}
        for i, v in enumerate(vals):
            key = keys[i] if i < len(keys) else "f%d" % i
            try:
                out[key] = float(v)
            except ValueError:
                out[key] = v
        out["_raw"] = raw
        return out

    # -- state recovery ----------------------------------------------------
    def recover(self, scale=2.0) -> bool:
        """Get a fresh frame out of the instrument, whatever state it is in.

        A ``:WAVeform:POINts?`` of 0 means there is no frame to transfer: every
        ``:WAVeform:DATA?`` then returns an empty block (``#9000000000``) or
        blocks forever.  Aborted RAW reads and forcing :STOP while the scope was
        still arming both leave it there.  RUN/STOP usually fixes it; *RST always
        does (and resets the channel setup, which is re-applied here).
        """
        self.t.drain(0.3)
        for attempt in range(3):
            if attempt:
                self.t.write("*RST")
                time.sleep(3.5)
                self.channel(1, scale, 0.0, "DC", 1, True)
                self.channel(2, scale, 0.0, "DC", 1, True)
            self.t.write(":TRIGger:SWEep AUTO")
            self.t.write(":WAVeform:MODE NORMal")
            self.t.write(":RUN")
            time.sleep(0.6)
            self.t.write(":STOP")
            time.sleep(0.5)
            pts = self.t.query(":WAVeform:POINts?")
            try:
                if int(float(pts)) > 0:
                    return True
            except ValueError:
                pass
        return False

    def screen_dc(self, ch: int):
        """Return ``(mean, lo, hi, pp)`` of one freshly acquired screen frame."""
        if not self.recover():
            raise IOError("scope has no acquisition data (POINts = 0)")
        _, ys = self.waveform(ch)
        if not ys:
            raise IOError("empty waveform for channel %d" % ch)
        return (sum(ys) / len(ys), min(ys), max(ys), max(ys) - min(ys))

    def waveform(self, ch: int) -> tuple[list[float], list[float]]:
        pre = self.preamble(ch)
        fmt = str(pre.get("format", "BYTE")).upper()
        if fmt.startswith("ASC"):
            text = self.t.query(":WAVeform:DATA?")
            ys = [float(v) for v in text.replace(";", ",").split(",") if v.strip()]
            xs = []
            return xs, ys

        data = self.t.query_binary(":WAVeform:DATA?")
        # DHO series may return 16-bit samples when FORMat WORD is honoured
        if len(data) >= 2 and len(data) % 2 == 0 and pre.get("yincrement", 0) == 0:
            samples = list(struct.unpack("<%dh" % (len(data) // 2), data))
        elif len(data) % 2 == 0 and len(data) // 2 == int(pre.get("points", 0)):
            samples = list(struct.unpack("<%dh" % (len(data) // 2), data))
        else:
            samples = list(data)

        xinc = float(pre.get("xincrement", 0) or 0)
        xorg = float(pre.get("xorigin", 0) or 0)
        xref = float(pre.get("xreference", 0) or 0)
        yinc = float(pre.get("yincrement", 0) or 0)
        yorg = float(pre.get("yorigin", 0) or 0)
        yref = float(pre.get("yreference", 0) or 0)

        xs = [(i - xref) * xinc + xorg for i in range(len(samples))]
        ys = [(v - yref) * yinc + yorg for v in samples]
        return xs, ys

    def waveform_deep(self, ch: int, points: int = 200000,
                      chunk: int = 500000, reuse: bool = False,
                      window_s: float = None, target_fs: float = None,
                      decim: int = 1) -> tuple[list[float], list[float], float]:
        """Read the acquisition memory (RAW mode).

        Two ways to ask for a record:

        * ``points=N`` -- legacy: pin the memory depth to the smallest supported
          step >= N and read the whole record.
        * ``window_s=W, target_fs=F`` -- plan a *time window* instead: pick the
          shallowest depth that still yields >= F samples per second across W,
          set the time base so the window really is W, then box-average down to
          about F.  This is what deep captures need: the native 25 ms S2C frame
          is 1 s long and two frames do not fit in 1 Mpts at 500 kSa/s.

        Returns ``(xs, ys, sample_rate_hz)``; ``xs`` is ``None`` for very large
        records (build it from the sample rate instead of materialising it).
        """
        # RAW reads block forever while the instrument is still acquiring, so a
        # fresh frame has to be acquired and stopped first.  Use RUN -> wait ->
        # STOP (the sweep must be AUTO, otherwise the instrument waits forever for
        # a trigger edge that a DC signal never provides); :SINGle only works when
        # a real trigger event is guaranteed.  Two more traps learned here:
        #  * plain :STOP on an old screen freezes that stale frame -- every later
        #    measurement then silently returns the same old trace;
        #  * *OPC? answers "0" immediately on this firmware, long before the
        #    acquisition finished, so it must not be used as a "done" barrier.
        #
        # Plan the record *before* acquiring: the memory depth and the time base
        # decide the window and the sample rate, so changing them afterwards would
        # only re-arrange an acquisition that is already wrong.
        try:
            srate = float(self.t.query(":ACQuire:SRATe?").split(",")[0])
        except (ValueError, IndexError):
            srate = 0.0
        if window_s:
            # The record length is depth/srate, and this firmware always reports
            # a fixed 12.5 MSa/s whatever the time base (it does honour
            # :ACQuire:MDEPth, clamped to 10 Mpts).  So plan the depth from the
            # *measured* rate: 1 Mpts is only 80 ms, a 48 kSa/s frame is 200 ms.
            use_fs = max(float(target_fs or 0.0), srate, 1e6)
            want = int(window_s * use_fs)
        else:
            want = max(int(points), 200000)
        depth = pick_depth(want)
        self.t.write(":ACQuire:MDEPth %d" % depth)
        time.sleep(0.3)
        depth = int(float(self.t.query(":ACQuire:MDEPth?")))
        if window_s:
            # Ten horizontal divisions per screen, so this sets the window in one
            # step; the depth set above decides how many samples fill it.
            self.t.write(":TIMebase:MAIN:SCALe %.9g" % (window_s / 10.0))
            time.sleep(0.45)
        try:
            srate = float(self.t.query(":ACQuire:SRATe?").split(",")[0])
        except (ValueError, IndexError):
            srate = 0.0
        if window_s is None and srate > 0 and target_fs:
            decim = max(1, int(round(srate / float(target_fs))))
        if window_s and srate > 0 and target_fs:
            decim = max(1, int(round(srate / float(target_fs))))
        self.t.drain(0.2)
        if not reuse:
            self.t.write(":TRIGger:SWEep AUTO")
            self.t.write(":RUN")
        if not reuse:
            try:
                tb = float(self.t.query(":TIMebase:MAIN:SCALe?").split(",")[0])
            except (ValueError, IndexError):
                tb = 1e-3
            # After a memory-depth change this firmware needs a couple of seconds
            # before the record is really re-armed; a 12*tb wait produced empty
            # DATA? blocks at 10 Mpts.
            time.sleep(min(6.0, max(2.0, 12.0 * tb)))
            self.t.write(":STOP")
            for _ in range(50):
                if self.t.query(":TRIGger:STATus?").upper().startswith("STOP"):
                    break
                time.sleep(0.01)
        self.t.write(":WAVeform:SOURce CHANnel%d" % ch)
        self.t.write(":WAVeform:MODE RAW")
        self.t.write(":WAVeform:FORMat BYTE")
        # The RAW read always starts at the beginning of the acquisition memory, so
        # reading fewer points than the record length silently shrinks the time window
        # (a 300 kpts read of a 1.2 s window is 48 ms of signal).  The depth was
        # already pinned above; always read the *whole* record.
        points = depth
        raw = self.t.query(":WAVeform:PREamble?")
        vals = [v for v in raw.replace(";", ",").split(",") if v.strip()]
        keys = ["format", "type", "points", "count", "xincrement", "xorigin",
                "xreference", "yincrement", "yorigin", "yreference"]
        pre = {}
        for i, v in enumerate(vals):
            key = keys[i] if i < len(keys) else "f%d" % i
            try:
                pre[key] = float(v)
            except ValueError:
                pre[key] = v
        # NOTE: :WAVeform:PREamble?'s "points" field reports the *requested
        # START/STOP window*, not the record length, so it must not be used to
        # clamp the read; :ACQuire:MDEPth? is the record length.

        buf = bytearray()
        start = 1
        _retried = False
        while len(buf) < points:
            stop = min(start + chunk - 1, points)
            self.t.write(":WAVeform:STARt %d" % start)
            self.t.write(":WAVeform:STOP %d" % stop)
            try:
                blk = self.t.query_binary(":WAVeform:DATA?")
            except IOError as _exc:
                # The window can run past the end of the acquired record (the
                # DHO812 answers an empty block instead of clamping STOP), so
                # keep what we already have instead of failing the capture.
                if len(buf) == 0:
                    # Nothing at all came back: the acquisition was not armed yet
                    # (this firmware needs a moment after a depth change).  Re-arm
                    # once and retry the very first window before giving up.
                    self.t.write("*CLS")
                    self.t.drain(0.5)
                    if not _retried:
                        _retried = True
                        self.t.write(":TRIGger:SWEep AUTO")
                        self.t.write(":RUN")
                        time.sleep(2.0)
                        self.t.write(":STOP")
                        time.sleep(0.4)
                        continue
                    raise
                # The truncated DATA? query leaves the port out of sync (the next
                # command then reads a stale "#9..." block header), so clear the
                # instrument status and drain before anything else is queried.
                self.t.write("*CLS")
                self.t.drain(0.5)
                break
            if not blk:
                break
            buf += blk
            start = stop + 1
        self.t.write(":WAVeform:MODE NORMal")

        xinc = float(pre.get("xincrement", 0) or 0)
        xorg = float(pre.get("xorigin", 0) or 0)
        xref = float(pre.get("xreference", 0) or 0)
        yinc = float(pre.get("yincrement", 0) or 0)
        yorg = float(pre.get("yorigin", 0) or 0)
        yref = float(pre.get("yreference", 0) or 0)
        # Deep records (up to 25 Mpts on this model) must not be materialised as
        # 25 M Python floats, so decimate with a box average while converting.
        # The wanted signal sits at 8-16 kHz, far below the decimated Nyquist, so
        # a plain moving average is enough of an anti-alias filter here.
        n_raw = len(buf)
        decim = max(1, int(decim))
        if decim > 1 and n_raw >= decim:
            n = n_raw // decim
            ys = [(sum(buf[i * decim:(i + 1) * decim]) / float(decim) - yref) * yinc + yorg
                  for i in range(n)]
        else:
            ys = [(v - yref) * yinc + yorg for v in buf]
            n = len(ys)
        step = xinc * decim
        xs = None
        if n <= 4000000:
            xs = [(i - xref / float(decim)) * step + xorg for i in range(n)]
        fs = (1.0 / step) if step else 0.0
        return xs, ys, fs


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------
def save_csv(path: str, xs, ys, fs: float = 0.0) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True) if os.path.dirname(path) else None
    with open(path, "w", newline="", encoding="ascii") as fh:
        w = csv.writer(fh)
        w.writerow(["time_s", "volt_v"])
        for i, y in enumerate(ys):
            if xs is not None and i < len(xs):
                t = xs[i]
            else:                       # deep record: xs was not materialised
                t = i / fs if fs else 0.0
            w.writerow(["%.12g" % t, "%.9g" % y])


def welch_like_fft(ys, fs):
    """Return (freqs, db) of a Hann-windowed spectrum, 0 dB at the fundamental.

    numpy is used when present because it takes an arbitrary length; the pure
    Python radix-2 path is only a fallback.  The window is applied over exactly
    the transformed length -- applying a 10 000-point window and then cutting
    the array to 8 192 for a radix-2 FFT leaves a step at the end, and that step
    alone produces a -32 dBc skirt around the carrier.
    """
    try:
        import numpy as np
    except ImportError:  # pragma: no cover - numpy is present on this machine
        return _fft_py(ys, fs)
    x = np.asarray(ys, dtype=float)
    n = x.size
    if n < 32 or fs <= 0:
        return [], []
    win = np.hanning(n)
    mag = np.abs(np.fft.rfft((x - x.mean()) * win)) * 2.0 / win.sum()
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    peak = float(mag[1:].max()) if mag.size > 1 else float(mag.max())
    db = 20.0 * np.log10(np.maximum(mag, 1e-15) / max(peak, 1e-15))
    return freqs.tolist(), db.tolist()


def _fft_py(ys, fs):
    n = len(ys)
    if n < 32 or fs <= 0:
        return [], []
    nfft = 1
    while nfft * 2 <= n:
        nfft *= 2
    mean = sum(ys[:nfft]) / nfft
    win = [0.5 - 0.5 * math.cos(2 * math.pi * i / (nfft - 1)) for i in range(nfft)]
    a = [complex((ys[i] - mean) * win[i], 0.0) for i in range(nfft)]
    j = 0
    for i in range(1, nfft):
        bit = nfft >> 1
        while j & bit:
            j ^= bit
            bit >>= 1
        j |= bit
        if i < j:
            a[i], a[j] = a[j], a[i]
    length = 2
    while length <= nfft:
        ang = -2 * math.pi / length
        wl = complex(math.cos(ang), math.sin(ang))
        for i in range(0, nfft, length):
            w = 1 + 0j
            for k in range(i, i + length // 2):
                u, v = a[k], a[k + length // 2] * w
                a[k], a[k + length // 2] = u + v, u - v
                w *= wl
        length *= 2
    mags = [abs(a[i]) * 2.0 / sum(win) for i in range(nfft // 2)]
    freqs = [i * fs / nfft for i in range(nfft // 2)]
    peak = max(mags) if mags else 0.0
    db = [20 * math.log10((m / peak) if peak > 0 and m > 0 else 1e-12) for m in mags]
    return freqs, db


def report_spectrum(freqs, db, fundamental_hint=1e6, harmonics=6, main_lobe_bins=8.0,
                    fmax=None, gate_sfdr=45.0, gate_thd=-40.0):
    """Print fundamental / SFDR / THD and apply the (tunable) acceptance gate."""
    if not freqs:
        print("no data for FFT")
        return None
    df = freqs[1] - freqs[0] if len(freqs) > 1 else 0.0
    peak_idx = max(range(len(db)), key=lambda i: db[i])
    f0 = freqs[peak_idx]
    if fundamental_hint and abs(f0 - fundamental_hint) > 0.1 * fundamental_hint:
        near = [i for i, f in enumerate(freqs)
                if abs(f - fundamental_hint) <= max(4 * df, 0.02 * fundamental_hint)]
        if near:
            peak_idx = max(near, key=lambda i: db[i])
            f0 = freqs[peak_idx]
    print("fundamental : %.6g Hz (bin %d, %.2f dB, df = %.4g Hz)"
          % (f0, peak_idx, db[peak_idx], df))
    if f0 <= 0:
        return None
    guard = max(main_lobe_bins * df, 0.01 * f0)
    # the Nyquist bin is a real-valued edge bin and carries no physical spur;
    # a band limit also keeps sampler feedthrough out of an SFDR meant for the DAC
    top = fmax if fmax else 0.45 * freqs[-1]
    cand = [(db[i], freqs[i]) for i in range(1, len(db))
            if abs(freqs[i] - f0) > guard and freqs[i] > 3 * df and freqs[i] <= top]
    cand.sort(reverse=True)
    print("strongest spurs:")
    for mag, f in cand[:6]:
        print("  %10.4g Hz  %8.2f dBc" % (f, mag))
    sfdr = -cand[0][0] if cand else 0.0
    thd_terms = []
    for h in range(2, harmonics + 1):
        target = h * f0
        if target >= fs_nyquist(freqs):
            break
        near = [i for i, f in enumerate(freqs) if abs(f - target) <= max(2 * df, 0.005 * target)]
        if not near:
            continue
        i = max(near, key=lambda k: db[k])
        thd_terms.append(db[i])
    thd = 10 * math.log10(sum(10 ** (v / 10) for v in thd_terms)) if thd_terms else float("nan")
    print("SFDR        : %.2f dBc" % sfdr)
    print("THD (2..%d)  : %.2f dBc" % (harmonics, thd))
    ok = sfdr >= gate_sfdr and thd <= gate_thd
    print("gate        : SFDR >= %.0f dBc  %s | THD <= %.0f dBc  %s"
          % (gate_sfdr, "PASS" if sfdr >= gate_sfdr else "FAIL",
             gate_thd, "PASS" if thd <= gate_thd else "FAIL"))
    return {"f0": f0, "sfdr": sfdr, "thd": thd, "pass": ok, "df": df}


def fs_nyquist(freqs):
    return 2 * freqs[-1]


def scan(subnet: str, ports=WAVE_PORTS, timeout=0.25, workers=256):
    """Scan a subnet for open SCPI ports.

    Accepts ``192.168.1``, ``192.168.1.0/24``, ``169.254`` (whole /16) or
    ``169.254.1``.  Progress is printed every 4096 probes so a /16 sweep does
    not look like a hang.
    """
    prefix = subnet.split("/")[0].strip()
    parts = [p for p in prefix.split(".") if p != ""]
    if len(parts) == 4 and parts[3] == "0":
        parts = parts[:3]
    if len(parts) < 2 or len(parts) > 3:
        raise SystemExit("scan: need 2 or 3 octets, e.g. 192.168.1 or 169.254")
    base = ".".join(parts)
    hosts = []
    if len(parts) == 2:
        for a in range(0, 256):
            for b in range(1, 255):
                hosts.append("%s.%d.%d" % (base, a, b))
    else:
        for b in range(1, 255):
            hosts.append("%s.%d" % (base, b))
    hosts = [h for h in hosts if not h.endswith(".0") and not h.endswith(".255")]

    def probe (hp):
        host, port = hp
        s = socket.socket()
        s.settimeout(timeout)
        try:
            if s.connect_ex((host, port)) == 0:
                return (host, port)
        except OSError:
            return None
        finally:
            s.close()
        return None

    tasks = [(h, p) for h in hosts for p in ports]
    print("scanning %d hosts x %d ports = %d probes ..."
          % (len(hosts), len(ports), len(tasks)), flush=True)
    hits = []
    done = 0
    with ThreadPoolExecutor(max_workers=workers) as ex:
        for res in ex.map(probe, tasks):
            done += 1
            if res:
                hits.append(res)
                print("  OPEN %s:%d" % res, flush=True)
            if done % 4096 == 0:
                print("  ... %d/%d probes, %d hit(s)" % (done, len(tasks), len(hits)),
                      flush=True)
    if not hits:
        print("nothing found -- check the cable and the scope's LAN settings")
    return hits


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description="DHO812 SCPI helper (LAN)")
    ap.add_argument("--host", default=os.environ.get("SCOPE_HOST"))
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("scan", help="find instruments on a subnet")
    p.add_argument("subnet", help="e.g. 192.168.1, 192.168.1.0/24 or 169.254")
    p.add_argument("--ports", default=",".join(str(x) for x in WAVE_PORTS),
                   help="comma separated ports (default: %(default)s)")
    p.add_argument("--timeout", type=float, default=0.25)
    p.add_argument("--workers", type=int, default=256)

    sub.add_parser("idn", help="query *IDN?")
    sub.add_parser("status", help="dump timebase / channel state")

    p = sub.add_parser("cmd", help="send one SCPI command (or query if it ends with ?)")
    p.add_argument("scpi", nargs="+")

    p = sub.add_parser("wav", help="download one channel and save CSV")
    p.add_argument("ch", type=int, choices=(1, 2))
    p.add_argument("-o", "--out", default="")

    p = sub.add_parser("fft", help="download one channel and report SFDR/THD")
    p.add_argument("ch", type=int, choices=(1, 2))
    p.add_argument("--fs", type=float, default=0.0, help="override sample rate")
    p.add_argument("--fund", type=float, default=1e6)
    p.add_argument("--gate-sfdr", type=float, default=45.0, help="SFDR lower bound (dBc)")
    p.add_argument("--gate-thd", type=float, default=-40.0, help="THD upper bound (dBc)")

    p = sub.add_parser("setup-sine", help="configure one channel for a clean single tone")
    p.add_argument("ch", type=int, choices=(1, 2), default=1)
    p.add_argument("--vdiv", type=float, default=2.0)
    p.add_argument("--tdiv", type=float, default=2e-7)

    args = ap.parse_args(argv)

    if args.cmd == "scan":
        ports = tuple(int(x) for x in args.ports.split(",") if x.strip())
        scan(args.subnet, ports=ports, timeout=args.timeout, workers=args.workers)
        return 0

    if not args.host:
        print("error: --host (or SCOPE_HOST) is required", file=sys.stderr)
        return 2

    with SocketTransport(args.host, args.port, args.timeout) as t:
        sc = Scope(t)

        if args.cmd == "idn":
            print(sc.idn())

        elif args.cmd == "status":
            for k, v in sc.status().items():
                print("%-14s %s" % (k, v))

        elif args.cmd == "cmd":
            line = " ".join(args.scpi)
            if line.strip().endswith("?"):
                print(sc.t.query(line))
            else:
                sc.t.write(line)
                print("sent: %s" % line)

        elif args.cmd == "wav":
            xs, ys = sc.waveform(args.ch)
            out = args.out or "out/scope_ch%d.csv" % args.ch
            save_csv(out, xs, ys)
            if ys:
                print("points %d  min %.6g V  max %.6g V  pk-pk %.6g V"
                      % (len(ys), min(ys), max(ys), max(ys) - min(ys)))
            print("saved %s" % out)

        elif args.cmd == "fft":
            xs, ys = sc.waveform(args.ch)
            fs = args.fs or sc.sample_rate()
            if not fs and len(xs) > 1:
                fs = 1.0 / (xs[1] - xs[0])
            print("points %d  fs %.6g Sa/s" % (len(ys), fs))
            freqs, db = welch_like_fft(ys, fs)
            report_spectrum(freqs, db, args.fund,
                            gate_sfdr=args.gate_sfdr, gate_thd=args.gate_thd)

        elif args.cmd == "setup-sine":
            sc.stop()
            sc.channel(args.ch, scale=args.vdiv, offset=0.0, coupling="DC",
                       probe=1.0, on=True)
            sc.timebase(args.tdiv)
            print("ch%d: %.3g V/div, %.3g s/div, DC coupling" % (args.ch, args.vdiv, args.tdiv))
    return 0


if __name__ == "__main__":
    sys.exit(main())
