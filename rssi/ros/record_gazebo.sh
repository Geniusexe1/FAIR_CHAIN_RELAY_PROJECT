#!/usr/bin/env bash
# Screen-record Gazebo's window during a live run.
#
#   rssi/ros/record_gazebo.sh start rssi/out/live_runs/<name>   -> <name>_gazebo_raw.mkv (2 fps)
#   rssi/ros/record_gazebo.sh stop  rssi/out/live_runs/<name>   -> <name>_gazebo.mp4 (time-lapse)
#
# Uses the static ffmpeg from the venv's imageio-ffmpeg (x11grab of the window on :0) and rssi/ros/gz_window.py
# to raise Gazebo and fix its geometry. The sim runs at RTF ~0.2, so the time-lapse is sped up
# SPEEDUP times (default 20) to a watchable length, and cropped (CROP=w:h:x:y, default = the 3D view
# of a 980x1376 window, without the side panel).
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PY="$ROOT/.venv/bin/python"
FF="$($PY -c 'import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())')"
CMD="${1:?start|stop}"; BASE="${2:?output base path}"; SPEEDUP="${SPEEDUP:-20}"
mkdir -p "$(dirname "$BASE")"

case "$CMD" in
  start)
    read -r X Y W H < <("$PY" "$ROOT/rssi/ros/gz_window.py" ${GZ_GEOM:-}) || { echo "no Gazebo window"; exit 1; }
    W=$((W / 2 * 2)); H=$((H / 2 * 2))                       # even sizes for x264
    echo "$X $Y $W $H" > "${BASE}_gazebo_geom.txt"
    # the window itself, not the screen: with no monitor plugged in the screen stays black
    ID=$(DISPLAY=:0 XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority" "$PY" -c "
from Xlib import display
def walk(w):
    try:
        if w.get_wm_name() == 'Gazebo' and w.get_geometry().height > 200: print(hex(w.id)); raise SystemExit
        for c in w.query_tree().children: walk(c)
    except SystemExit: raise
    except Exception: pass
walk(display.Display().screen().root)")
    [ -n "$ID" ] || { echo "no Gazebo window"; exit 1; }
    DISPLAY=:0 XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority" setsid nohup "$FF" -y -hide_banner -loglevel error -f x11grab -framerate 2 -window_id "$ID" \
      -i :0.0 -vf "crop=$W:$H:0:0" -c:v libx264 -preset veryfast -crf 26 -pix_fmt yuv420p \
      "${BASE}_gazebo_raw.mkv" > "${BASE}_gazebo_ffmpeg.log" 2>&1 < /dev/null &
    echo $! > "${BASE}_gazebo.pid"
    echo "recording ${W}x${H}+${X}+${Y} -> ${BASE}_gazebo_raw.mkv (pid $(cat "${BASE}_gazebo.pid"))" ;;
  stop)
    PID=$(cat "${BASE}_gazebo.pid" 2>/dev/null) && kill -INT "$PID" 2>/dev/null
    for _ in $(seq 30); do kill -0 "$PID" 2>/dev/null || break; sleep 1; done
    # 2 fps captured -> played at 2*SPEEDUP fps-equivalent: each frame shown 1/(2*SPEEDUP) s, output 30 fps
    "$FF" -hide_banner -loglevel error -y -i "${BASE}_gazebo_raw.mkv" \
      -vf "crop=${CROP:-690:1232:270:68},setpts=PTS/${SPEEDUP},fps=30" -c:v libx264 -preset medium -crf 28 -pix_fmt yuv420p -an \
      "${BASE}_gazebo.mp4" && echo "wrote ${BASE}_gazebo.mp4 (x${SPEEDUP})"
    rm -f "${BASE}_gazebo.pid" ;;
esac
