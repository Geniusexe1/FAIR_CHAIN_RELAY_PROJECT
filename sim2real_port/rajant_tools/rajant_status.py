#!/usr/bin/env python3
"""
rajant_status.py — read live status from Rajant BreadCrumb radios.

Usage:
    python3 rajant_status.py                      # all three mesh radios
    python3 rajant_status.py 10.195.220.1         # one radio
    RAJANT_ROLE=CO RAJANT_PW=<password> python3 rajant_status.py

Set the password to your radios' actual credential — the library confirms the
connection and TLS handshake work, but the radio rejects a wrong password with
"authentication failure".
"""
import os
import sys
from rajant_api import Breadcrumb

ROLE = os.environ.get("RAJANT_ROLE", "CO")
PW   = os.environ.get("RAJANT_PW", "admin")
PORT = int(os.environ.get("RAJANT_PORT", "2300"))

DEFAULT_HOSTS = ["10.195.219.1", "10.195.220.1", "10.195.221.1"]


def mac(b):
    return ":".join(f"{x:02x}" for x in b) if b else "-"


def report(host):
    bc = Breadcrumb(host=host, port=PORT, role=ROLE, password=PW)
    if not bc.reachable():
        print(f"\n{host:<16} UNREACHABLE (no ping)")
        return
    if not bc.authenticate():
        print(f"\n{host:<16} reachable, but AUTH FAILED "
              f"(check RAJANT_ROLE / RAJANT_PW)")
        return

    st = bc.get_state()
    sysm, im = st.system, st.instamesh

    print(f"\n{'='*54}")
    print(f"{bc.serial}   ({host})")
    print(f"{'='*54}")
    print(f"  version        {sysm.version}")
    print(f"  uptime         {sysm.uptime}s   bootCounter {sysm.bootCounter}")
    print(f"  temperature    {sysm.temperature} C")
    print(f"  rebooting      {sysm.isRebooting}")

    print("  -- InstaMesh flooding --")
    print(f"  floods dropped        {im.floodsDropped}")
    print(f"  source floods dropped {im.sourceFloodsDropped}   "
          f"<- the '537 Individual Flood Limit' counter")
    print(f"  packets dropped       {im.packetsDropped}")
    print(f"  undeliverables recv   {im.undeliverablesReceived}")

    for w in st.wireless:
        print(f"  -- wireless {w.name}  ch {w.channel}  noise {w.noise} --")
        for p in w.peer:
            print(f"     peer {mac(p.mac)}  rssi {p.rssi}  cost {p.cost}  "
                  f"ip {p.ipv4Address or '-'}")
    for wd in st.wired:
        print(f"  -- wired {wd.name}  linkup {wd.linkup} --")
        for p in wd.peer:
            print(f"     peer {mac(p.mac)}  cost {p.cost}  ip {p.ipv4Address or '-'}")


if __name__ == "__main__":
    hosts = sys.argv[1:] or DEFAULT_HOSTS
    for h in hosts:
        try:
            report(h)
        except Exception as e:
            print(f"\n{h:<16} ERROR: {e}")
