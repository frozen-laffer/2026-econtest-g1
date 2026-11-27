#!/usr/bin/env python3
# Stepped-response logger for DHO812: polls Vpp/frequency, re-asserts the scope
# setup every few seconds so a knob turn cannot silently corrupt the run, and
# appends to CSV as it goes (partial data survives a kill).
#   python _bode_capture.py --tdiv 20u --vdiv 0.02 --secs 300 --out _bode_step1.csv
import argparse, csv, os, sys, time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from dho_scope import SocketTransport, Scope          # noqa: E402


def pnum(txt):
    try:
        v = float(txt)
    except Exception:
        return None
    return None if abs(v) > 1e30 else v          # Rigol returns 9.9E+37 when invalid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=os.environ.get("SCOPE_HOST", "192.168.1.101"))
    ap.add_argument("--ch", type=int, default=1)
    ap.add_argument("--tdiv", default="20u")
    ap.add_argument("--vdiv", type=float, default=0.02)
    ap.add_argument("--offset", type=float, default=0.0)
    ap.add_argument("--secs", type=float, default=300.0)
    ap.add_argument("--interval", type=float, default=0.25)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    unit = {"u": 1e-6, "n": 1e-9, "m": 1e-3}
    tdiv = float(a.tdiv[:-1]) * unit[a.tdiv[-1]] if a.tdiv[-1] in unit else float(a.tdiv)

    with SocketTransport(a.host) as tr:
        sc = Scope(tr)
        print("idn:", sc.idn(), flush=True)

        def assert_setup():
            tr.write(":TRIGger:SWEep AUTO")
            tr.write(":WAVeform:MODE NORMal")
            tr.write(":CHANnel%d:COUPling DC" % a.ch)
            tr.write(":CHANnel%d:PROBe 1" % a.ch)
            tr.write(":CHANnel%d:SCALe %g" % (a.ch, a.vdiv))
            tr.write(":CHANnel%d:OFFSet %g" % (a.ch, a.offset))
            tr.write(":CHANnel%d:BWLimit OFF" % a.ch)
            tr.write(":TIMebase:MAIN:SCALe %g" % tdiv)
            tr.write(":RUN")

        assert_setup()
        time.sleep(0.8)
        print("tdiv=%g vdiv=%g offset=%g" % (tdiv, a.vdiv, a.offset), flush=True)

        fh = open(a.out, "w", newline="", encoding="utf-8")
        w = csv.writer(fh)
        w.writerow(["t_s", "freq_hz", "vpp_v"])
        fh.flush()

        fcmd = ":MEASure:FREQuency? CHANnel%d" % a.ch
        vcmd = ":MEASure:VPP? CHANnel%d" % a.ch
        t0 = nxt = time.time()
        last_assert = t0
        last_report = t0
        n = good = 0
        while time.time() - t0 < a.secs:
            if time.time() - last_assert > 2.0:
                assert_setup()
                last_assert = time.time()
            f = pnum(tr.query(fcmd))
            v = pnum(tr.query(vcmd))
            n += 1
            if f and v:
                good += 1
            w.writerow(["%.3f" % (time.time() - t0), "" if f is None else "%.1f" % f,
                        "" if v is None else "%.6g" % v])
            fh.flush()
            if time.time() - last_report > 5.0:
                print("t=%5.1fs n=%4d valid=%4d  last f=%-10s vpp=%s" % (
                    time.time() - t0, n, good,
                    "%.4g" % f if f else "-", "%.4g" % v if v else "-"), flush=True)
                last_report = time.time()
            nxt += a.interval
            d = nxt - time.time()
            if d > 0:
                time.sleep(d)
        fh.close()
        print("DONE rows=%d valid=%d -> %s" % (n, good, a.out), flush=True)


if __name__ == "__main__":
    main()
