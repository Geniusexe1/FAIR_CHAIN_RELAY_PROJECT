#!/bin/bash
# Verifies the namespacing workarounds. Run INSIDE the container after launching
# multi_vehicle.launch.py. Each check maps to an issue in NAMESPACING.md.
echo "=== A2/A3: distinct control topics ==="
ros2 topic list | grep -E 'cmd_vel' || echo "  NONE - bridge not up?"

echo "=== A4: /clock must exist and be GLOBAL (not /robotN/clock) ==="
ros2 topic list | grep -E '^/clock$' && echo "  OK" || echo "  FAIL - clock got namespaced"
timeout 5 ros2 topic echo /clock --once >/dev/null 2>&1 && echo "  clock ticking OK" || echo "  FAIL - clock silent"

echo "=== A5: TF frames must ALL be prefixed. A bare base_link = half-prefixed tree ==="
timeout 5 ros2 topic echo /tf --once 2>/dev/null | grep -E 'frame_id' | sort -u

echo "=== A10: ONE tree rooted at map, no bare model frame ==="
timeout 8 ros2 topic echo /tf --once 2>/dev/null | grep -E 'frame_id' | sort -u
echo "  (a bare 'robot1'/'robot2' parent frame above = link poses back on = split tree)"
for ns in robot1 robot2; do
  timeout 5 ros2 run tf2_ros tf2_echo map $ns/base_link >/dev/null 2>&1 \
    && echo "  map -> $ns/base_link OK" || echo "  FAIL - map cannot reach $ns/base_link"
  timeout 5 ros2 topic echo /$ns/joint_states --once >/dev/null 2>&1 \
    && echo "  /$ns/joint_states OK" || echo "  FAIL - joint states unbridged, wheel frames missing"
done

echo "=== globals stayed global ==="
ros2 topic list | grep -E '^/(map|tf|tf_static|weakest_link|mesh_matrix)$'

echo "=== packages present ==="
ros2 pkg list | grep -E '^(saye_|rajant_mesh)' | tr '\n' ' '; echo
