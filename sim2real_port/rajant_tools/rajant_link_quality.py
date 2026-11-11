#!/usr/bin/env python3
"""
rajant_link_quality.py — RF signal strength between Rajant radios.

Queries every radio, maps each peer MAC back to the radio that owns it, and
prints the per-interface link quality in both directions.

    export RAJANT_PW=<password>
    python3 rajant_link_quality.py                    # one snapshot
    python3 rajant_link_quality.py --watch 5          # refresh every 5s

Key fields, straight off the radio:
    signal  dBm, the received power for that peer      (closer to 0 = stronger)
    noise   dBm, the interface's noise floor
    rssi    dB, signal-to-noise margin == signal - noise   (higher = better)
    cost    InstaMesh routing cost for the hop          (LOWER = better)
    rate    negotiated link rate
"""
import argparse
import os
import sys
import time

from rajant_api import Breadcrumb

RADIOS = ["10.195.219.1", "10.195.220.1", "10.195.221.1"]
ROLE = os.environ.get("RAJANT_ROLE", "ADMIN")
PW = os.environ.get("RAJANT_PW")
PORT = int(os.environ.get("RAJANT_PORT", "2300"))


def macstr(b):
    return ":".join(f"{x:02x}" for x in b) if b else "-"


def collect(hosts):
    """Return {host: {'serial','state'}} for every radio that answers."""
    out = {}
    for host in hosts:
        bc = Breadcrumb(host=host, port=PORT, role=ROLE, password=PW)
        if not bc.reachable():
            continue
        if not bc.authenticate():
            print(f"  !! {host}: auth failed (check RAJANT_ROLE / RAJANT_PW)",
                  file=sys.stderr)
            continue
        st = bc.get_state()
        if st is False:
            continue
        out[host] = {"serial": bc.serial, "state": st}
    return out


def build_mac_index(radios):
    """Map every interface MAC -> (host, serial) so peers can be named."""
    idx = {}
    for host, r in radios.items():
        for w in r["state"].wireless:
            idx[macstr(w.mac)] = (host, r["serial"])
        for wd in r["state"].wired:
            idx[macstr(wd.mac)] = (host, r["serial"])
    return idx


def snapshot(hosts):
    radios = collect(hosts)
    if not radios:
        print("No radios answered.")
        return
    idx = build_mac_index(radios)

    hdr = (f"{'FROM':<22} {'IFACE':<7} {'CH':>4} {'-> TO':<22} "
           f"{'SIG':>6} {'NOISE':>6} {'SNR':>5} {'COST':>7} {'RATE':>6}")
    print(hdr)
    print("-" * len(hdr))

    for host, r in sorted(radios.items()):
        for w in r["state"].wireless:
            for p in w.peer:
                peer_mac = macstr(p.mac)
                who = idx.get(peer_mac)
                # peers reported by IP even when the MAC isn't in our index
                label = who[1] if who else (p.ipv4Address or peer_mac)
                snr = p.signal - w.noise if (p.signal and w.noise) else p.rssi
                print(f"{r['serial']:<22} {w.name:<7} {w.channel:>4} "
                      f"{label:<22} {p.signal:>5}d {w.noise:>5}d {snr:>5} "
                      f"{p.cost:>7} {p.rate:>6}")
    print()

    # flag the weakest link and the most expensive hop
    links = [(r["serial"], w.name, p, w.noise)
             for r in radios.values()
             for w in r["state"].wireless
             for p in w.peer]
    if links:
        worst_sig = min(links, key=lambda t: t[2].signal)
        worst_cost = max(links, key=lambda t: t[2].cost)
        print(f"weakest signal : {worst_sig[0]} {worst_sig[1]} "
              f"{worst_sig[2].signal} dBm (SNR {worst_sig[2].signal - worst_sig[3]})")
        print(f"costliest hop  : {worst_cost[0]} {worst_cost[1]} "
              f"cost {worst_cost[2].cost}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("hosts", nargs="*", default=None,
                    help="radio IPs (default: all three mesh radios)")
    ap.add_argument("--watch", type=float, metavar="SEC",
                    help="refresh continuously every SEC seconds")
    args = ap.parse_args()

    if not PW:
        sys.exit("Set RAJANT_PW (and optionally RAJANT_ROLE, default ADMIN).")

    hosts = args.hosts or RADIOS
    if args.watch:
        try:
            while True:
                print(f"\n=== {time.strftime('%H:%M:%S')} ===")
                snapshot(hosts)
                time.sleep(args.watch)
        except KeyboardInterrupt:
            pass
    else:
        snapshot(hosts)


if __name__ == "__main__":
    main()
