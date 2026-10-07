# Runbook — Two-NUC Ackermann Sim + Rajant Mesh Monitor

PC1 = NUC1 (`192.168.7.143`, mesh `10.195.219.0`) — runs Gazebo + both robots
PC2 = NUC2 (`192.168.7.56`,  mesh `10.195.221.0`) — no Gazebo; sensors + navigation

> `192.168.7.56` is currently unreachable (no ping response). Use the mesh address
> `10.195.221.0` for ssh; every `192.168.7.56` below works the same with it.

Everything runs from one image: `mesh_sim:humble`. Both machines must be on
`ROS_DOMAIN_ID=42`.

---

## 0. One-time per boot, on PC1 only

Lets the container draw on your screen. Skip it and Gazebo dies with
"cannot open display".

```bash
xhost +local:docker
```

---

## 1. PC1 — start the simulation (both robots)

```bash
docker rm -f pc1_sim 2>/dev/null
docker run -d --name pc1_sim --network host \
  -e ROS_DOMAIN_ID=42 -e ROBOT_NS=robot1 \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri --privileged \
  mesh_sim:humble \
  bash -c 'ros2 launch saye_bringup multi_vehicle.launch.py rviz:=false > /tmp/launch.log 2>&1'
```

Gazebo takes 30-60 s to appear (Intel iGPU compiles shaders on first run).
Wait until both robots exist:

```bash
docker exec pc1_sim /entrypoint.sh ros2 topic list | grep robot2/odom
```

Add `rviz:=true` if you want RViz. `-e GZ_HEADLESS=1` runs with no GUI at all.

> The sim auto-starts running. If the clock is stuck at 0, it launched paused —
> press play in the GUI, or:
> `docker exec pc1_sim /entrypoint.sh gz service -s /world/my_world/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req "pause: false"`

## 2. PC2 — start the control/sensor container

```bash
ssh asl@10.195.221.0        # password: see ~/.fair_chain.env (NUC2_SSH_PASSWORD)
docker rm -f pc2_ctrl 2>/dev/null
docker run -d --name pc2_ctrl --network host \
  -e ROS_DOMAIN_ID=42 -e ROBOT_NS=robot2 \
  mesh_sim:humble sleep infinity
```

Confirm it sees PC1 (this is the distributed-setup check):

```bash
docker exec pc2_ctrl /entrypoint.sh ros2 topic list | grep robot
```

## 3. PC2 — SLAM (builds the shared map)

```bash
docker exec -d pc2_ctrl /entrypoint.sh bash -c \
  'ros2 launch saye_bringup slam.launch.py use_sim_time:=true > /tmp/slam.log 2>&1'
```
Check: `docker exec pc2_ctrl /entrypoint.sh ros2 topic list | grep '^/map$'`

Only **one** robot may run SLAM — `/map` is deliberately global and shared.

## 4. PC2 — Nav2 (obstacle avoidance + waypoints)

```bash
docker exec -d pc2_ctrl /entrypoint.sh bash -c \
  'ros2 launch saye_bringup nav_robot.launch.py > /tmp/nav.log 2>&1'
```

Wait ~45 s, then confirm:
```bash
docker exec pc2_ctrl bash -c 'grep "Managed nodes are active" /tmp/nav.log'
```

---

## Setting a waypoint

Coordinates are **metres in the `map` frame**. Run from any container on domain 42.

```bash
# single goal: drive to x=4, y=2
docker exec pc2_ctrl /entrypoint.sh \
  ros2 run saye_bringup send_waypoints.py robot2 4.0 2.0

# a route: visit each point in order
docker exec pc2_ctrl /entrypoint.sh \
  ros2 run saye_bringup send_waypoints.py robot2 1.0 5.0  4.0 5.0  4.0 2.0
```

One x/y pair uses `NavigateToPose`; several use `FollowWaypoints`.
Nav2 plans around obstacles automatically using the LiDAR.

**Run it detached for long routes** — if your shell dies, the action client dies
with it and the goal is cancelled:
```bash
docker exec -d pc2_ctrl /entrypoint.sh bash -c \
  'ros2 run saye_bringup send_waypoints.py robot2 4.0 5.0 > /tmp/wp.log 2>&1'
```

Equivalent with the plain CLI:
```bash
docker exec pc2_ctrl /entrypoint.sh ros2 action send_goal \
  /robot2/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 2.0}, orientation: {w: 1.0}}}}"
```

Watch progress:
```bash
docker exec pc1_sim /entrypoint.sh ros2 topic echo /robot2/odom --once | grep -A2 position
```

---

## STOPPING THINGS

### Stop the robot mid-drive (cancel navigation)
```bash
docker exec pc2_ctrl /entrypoint.sh ros2 run saye_bringup stop_robot.py robot2
```
Cancels any active NavigateToPose/FollowWaypoints goal AND zeroes cmd_vel.
Cancelling alone is not enough - the last command leaves the car coasting.

### Stop just the navigation stack (keep the sim running)
```bash
docker exec pc2_ctrl bash -c 'ps -eo pid,cmd | grep [n]av_robot | awk "{print \$1}" | xargs -r kill -9'
```
Kill by PID. `pkill -f nav_robot` does NOT work - it matches its own bash -c
wrapper and kills itself, leaving nav2 running. This trap cost hours.

### Stop everything
```bash
docker rm -f pc1_sim                            # on PC1 - kills sim + Gazebo
ssh asl@192.168.7.56 'docker rm -f pc2_ctrl'    # on PC2 - kills SLAM + Nav2
```

### Pause the simulation (freeze physics, keep everything loaded)
```bash
docker exec pc1_sim /entrypoint.sh gz service -s /world/my_world/control \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 \
  --req "pause: true"      # "pause: false" to resume
```

> **If the RViz axes blink or `map` is not the tree root, check the TF tree FIRST:**
> ```bash
> docker exec pc1_sim /entrypoint.sh ros2 run tf2_tools view_frames
> ```
> One tree rooted at `map`, and no bare `robot1` / `robot2` frame. A bare model frame
> means the Gazebo pose publisher is broadcasting link poses again, giving
> `<ns>/base_link` two parents at 20 Hz. See NAMESPACING.md A10.

> **If a robot drifts or a waypoint does nothing, check for duplicates FIRST:**
> ```bash
> docker exec pc1_sim /entrypoint.sh ros2 topic info /robot2/cmd_vel
> ```
> Exactly **1 publisher** is healthy. More means several Nav2 stacks are
> fighting over the robot, or ghost participants remain from containers that
> were killed. Fix by removing every stale container and starting clean.


---

## Driving manually instead

```bash
# robot1 from PC1
docker exec -it pc1_sim /entrypoint.sh \
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel

# robot2 from PC2
docker exec -it pc2_ctrl /entrypoint.sh \
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot2/cmd_vel
```

## Rajant mesh monitor

```bash
docker exec -d pc1_sim /entrypoint.sh bash -c \
  'ros2 launch rajant_mesh mesh_launch.py > /tmp/mesh.log 2>&1'
docker exec pc1_sim /entrypoint.sh ros2 topic echo /mesh_matrix --once
```

---

## Gotchas that will cost you time

| Symptom | Cause |
|---|---|
| `ros2: command not found` in `docker exec` | `docker exec` skips the ENTRYPOINT. Use `/entrypoint.sh <cmd>` or `docker exec -it ... bash` |
| Clock stuck at 0, nothing moves, no errors | Gazebo launched paused — unpause (see §1) |
| Waypoint stops halfway | Your shell was killed, taking the action client with it. Use `docker exec -d` |
| `pkill -f <pattern>` seems to do nothing | It matches its own `bash -c` wrapper and kills itself. Kill by PID |
| Robot reverses instead of turning | Correct — Ackermann cars can't pivot in place |
| Two publishers on `/mesh_matrix` from one host | Old `rajant_mesh_relay` still running on PC2; remove it |
| `tf2_buffer: Detected jump back in time` + RViz blinking + SLAM dropping every scan | **Two Gazebos on domain 42** = two `/clock` streams. Count them first: `ros2 topic info /clock` must show 1 publisher. `docker restart nuc1` |
| Every goal reports SUCCESS instantly, robot never moves | A Nav2 node is on the wall clock. `ros2 param get /<ns>/controller_server use_sim_time` must be True. NAMESPACING.md A11 |
| Robot sits ON the goal, action never completes, `Running spin` in the log | `yaw_goal_tolerance` too tight — an Ackermann car cannot rotate in place. NAMESPACING.md A11 |
| RViz axes blink; `map` is not the tree root; two copies of each robot | Gazebo pose publisher broadcasting link poses, so `<ns>/base_link` has two parents. NAMESPACING.md A10 |
| Wheel frames missing from the robot model | `/<ns>/joint_states` not bridged, so robot_state_publisher broadcasts nothing. NAMESPACING.md A10 |

## Full teardown

```bash
docker rm -f pc1_sim                                     # PC1
ssh asl@192.168.7.56 'docker rm -f pc2_ctrl'             # PC2
```
