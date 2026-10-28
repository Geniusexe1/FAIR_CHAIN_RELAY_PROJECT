#!/usr/bin/env python3
"""Raise Gazebo's main window and give it a fixed geometry, for screen recording.

    .venv/bin/python rssi/ros/gz_window.py [x y w h]      default 0 40 980 1380

Prints the final 'x y w h' of the window on the screen (what ffmpeg x11grab should capture).
Gazebo Classic opens behind other windows; this brings it to the front.
"""
import sys
import time

from Xlib import X, display, protocol


def find(win, name):
    try:
        n = win.get_wm_name()
        if n == name and win.get_geometry().height > 200:
            return win
        for c in win.query_tree().children:
            f = find(c, name)
            if f:
                return f
    except Exception:
        pass
    return None


def main():
    x, y, w, h = (int(v) for v in sys.argv[1:5]) if len(sys.argv) >= 5 else (0, 40, 980, 1380)
    d = display.Display(":0")
    root = d.screen().root
    win = find(root, "Gazebo")
    if win is None:
        sys.exit("no Gazebo window on :0")
    # the window manager owns the frame: ask it (EWMH) to activate, then move + resize
    act = d.intern_atom("_NET_ACTIVE_WINDOW")
    ev = protocol.event.ClientMessage(window=win, client_type=act, data=(32, [2, X.CurrentTime, 0, 0, 0]))
    root.send_event(ev, event_mask=X.SubstructureRedirectMask | X.SubstructureNotifyMask)
    win.configure(x=x, y=y, width=w, height=h, stack_mode=X.Above)
    win.raise_window()
    d.sync()
    time.sleep(1.5)
    g = win.get_geometry()
    t = win.translate_coords(root, 0, 0)
    print(f"{-t.x} {-t.y} {g.width} {g.height}")


if __name__ == "__main__":
    main()
