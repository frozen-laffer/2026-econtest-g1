#!/usr/bin/env python3
"""Rediscover the DHO812 on the local wires.

The scope's address is not stable here (it answers 192.168.1.102 one minute and
nothing the next), so sweep the private /24s that a direct cable can produce,
find hosts via ARP (UDP probe -- the port never matters for ARP), then try
``*IDN?`` over TCP 5555 on every live host.

    python tools/find_scope.py                 # one round, default subnets
    python tools/find_scope.py --rounds 20     # keep looking for ~2 min
"""

from __future__ import annotations

import argparse
import socket
import subprocess
import sys
import time

SUBNETS = ["192.168.1", "192.168.0", "10.0.0", "192.168.10"]


def live_hosts(subnet: str) -> dict[str, str]:
    """Return {ip: mac} for hosts that answer ARP in ``subnet``."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for i in range(1, 255):
        try:
            probe.sendto(b"\x00", ("%s.%d" % (subnet, i), 5555))
        except OSError:
            pass
    probe.close()
    time.sleep(0.8)
    try:
        out = subprocess.run(["arp", "-a"], capture_output=True, timeout=15).stdout
    except OSError:
        return {}
    found = {}
    for line in out.decode("gbk", "replace").splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0].startswith(subnet + "."):
            mac = parts[1].lower()
            if mac not in ("ff-ff-ff-ff-ff-ff", "00-00-00-00-00-00"):
                found[parts[0]] = mac
    return found


def idn(host: str, port: int = 5555, timeout: float = 3.0) -> str | None:
    try:
        with socket.create_connection((host, port), timeout=timeout) as s:
            s.settimeout(timeout)
            s.sendall(b"*IDN?\n")
            data = b""
            while b"\n" not in data and len(data) < 256:
                chunk = s.recv(256)
                if not chunk:
                    break
                data += chunk
        return data.decode("ascii", "replace").strip() or None
    except OSError:
        return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rounds", type=int, default=1)
    ap.add_argument("--subnets", default=",".join(SUBNETS))
    ap.add_argument("--interval", type=float, default=6.0)
    args = ap.parse_args()
    subnets = [s.strip() for s in args.subnets.split(",") if s.strip()]

    for rnd in range(1, args.rounds + 1):
        for subnet in subnets:
            hosts = live_hosts(subnet)
            hits = ["%s (%s)" % (ip, mac) for ip, mac in sorted(hosts.items())]
            print("round %d  %s.*: %d host(s)%s"
                  % (rnd, subnet, len(hosts), ("  " + ", ".join(hits)) if hits else ""),
                  flush=True)
            for ip in sorted(hosts):
                ident = idn(ip)
                if ident:
                    print("  SCPI %s:5555 -> %s" % (ip, ident), flush=True)
                    print("  set SCOPE_HOST=%s" % ip, flush=True)
                    return 0
        if rnd < args.rounds:
            time.sleep(args.interval)
    return 1


if __name__ == "__main__":
    sys.exit(main())
