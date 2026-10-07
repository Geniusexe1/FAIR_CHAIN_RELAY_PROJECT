# Daily Workflow

Two containers, one per NUC. They stay up (`--restart unless-stopped`) and do
nothing on their own — you exec in and launch what you want.

| | container | robot it manages |
|---|---|---|
| NUC1 (this PC) | `nuc1` | robot1 — **also runs Gazebo + spawns BOTH robots** |
| NUC2 (192.168.7.56) | `nuc2` | robot2 |

Both: `ROS_DOMAIN_ID=42`, host networking. ROS + both workspaces are sourced
automatically when you open a shell.

---

## Opening a terminal

```bash
docker exec -it nuc1 bash                       # NUC1
ssh asl@10.195.221.0 -t 'docker exec -it nuc2 bash'   # NUC2   (password: see ~/.fair_chain.env)
```
`ros2` works immediately — no sourcing needed.

> Only for **non-interactive** one-liners (`docker exec nuc1 ros2 ...`) you must
> prefix `/entrypoint.sh`, because bash skips `.bashrc` when not interactive.

If the containers are stopped: `docker start nuc1` / `docker start nuc2`.

---

## HOW THE NAMESPACE WORKS

Each robot owns a prefix. Everything belonging to robot1 lives under
`/robot1/...`, robot2 under `/robot2/...`. That is what stops two identical
robots colliding on topic and TF names.

**Where it comes from:** the `ROBOT_NS` environment variable, baked per
container (`robot1` on nuc1, `robot2` on nuc2). Every launch file reads it as
its default, so you normally never type a namespace at all.

**To override it for one launch**, pass the argument explicitly:
```bash
ros2 launch saye_bringup nav_robot.launch.py namespace:=robot1
```

**What gets prefixed** (per robot):

| | |
|---|---|
| topics | `/robot1/cmd_vel`, `/robot1/odom`, `/robot1/scan`, `/robot1/goal_pose` |
| TF frames | `robot1/base_link`, `robot1/odom`, `robot1/base_link/gpu_lidar` |
| joint states | `/robot1/joint_states` (feeds robot_state_publisher's wheel frames) |
| Gazebo model | `robot1` |
| nodes | `/robot1/controller_server`, ... |

**What is deliberately GLOBAL** (shared, never prefixed):

| | why |
|---|---|
| `/clock` | one Gazebo, one simulation clock |
| `/tf`, `/tf_static` | ONE tree; robots separated by frame prefix, not by topic |
| `/map` | one shared occupancy grid |
| `/weakest_link`, `/mesh_matrix` | mesh monitor, both NUCs publish |

So: `ros2 topic list` shows `/robot1/scan` and `/robot2/scan` separately, but a
single `/map` and a single `/tf`.

---

## Starting a session

### Everything at once (single machine)

```bash
docker exec -it nuc1 bash
ros2 launch saye_bringup bringup_all.launch.py
```

Gazebo + both robots + SLAM + a Nav2 stack each, launched in the right order
with the delays they need. Ready in ~90 s; watch for two `Managed nodes are
active` lines.

| argument | default | |
|---|---|---|
| `slam` / `slam_robot` | `true` / `robot1` | Which robot maps. The other gets the static `map -> <ns>/odom`. |
| `nav` | `both` | `both` \| `robot1` \| `robot2` \| `none` |
| `rviz` | `true` | Opens after Nav2, not against an empty TF tree |
| `preflight` | `true` | Refuses to start if the domain already has a stack on it |
| `slam_delay` `nav_delay` `nav_stagger` `rviz_delay` | 15 / 25 / 12 / 20 s | Raise on a cold machine |

> **The preflight is the important part.** It counts `/clock` publishers,
> `slam_toolbox` nodes and `parameter_bridge` nodes already on domain 42 and
> aborts if any exist. Two Gazebos means two `/clock` streams; subscribers
> interleave them, time appears to run backwards, and tf2 clears its whole
> buffer several times a second — SLAM drops every scan, the controller loses
> its transforms, and RViz blinks. `docker restart nuc1` clears it.

On the two-NUC setup this is still the nuc1 command, with `nav:=robot1` — nuc2
runs its own `nav_robot.launch.py`.

### Or step by step — NUC1, Gazebo with BOTH robots
```bash
docker exec -it nuc1 bash
ros2 launch saye_bringup multi_vehicle.launch.py
```
Takes 30-60 s. Add `rviz:=false` to skip RViz, `GZ_HEADLESS=1` env for no GUI.
Both robots spawn here regardless of which NUC drives them.

### NUC2 — SLAM (builds the shared map)
```bash
ros2 launch saye_bringup slam.launch.py use_sim_time:=true
```
Run this on **one** robot only — `/map` is global, two mappers would fight.

### Navigation — one per robot, on its own NUC
```bash
# NUC2 (robot2 — has SLAM, so it gets map->odom from there)
ros2 launch saye_bringup nav_robot.launch.py

# NUC1 (robot1 — no SLAM, so it needs the static map->odom link)
ros2 launch saye_bringup nav_robot.launch.py static_map_odom:=true
```
Wait for `Managed nodes are active` (~20-90 s).

> `static_map_odom` publishes an **identity** `map -> <ns>/odom`, and that is correct
> regardless of where the robot was spawned: Gazebo's OdometryPublisher reports the model's
> **world** pose, not a spawn-relative one, so `<ns>/odom` already coincides with the world
> origin. (Verified: with robot2 spawned at y=2.0, `/robot2/odom` matches the gz world pose
> exactly.) Do **not** pass the spawn pose via `map_odom_x/y/yaw` — that double-counts it
> and puts the robot metres off in the map. Those offsets are only for a map whose origin
> is deliberately not the world origin.
>
> Only the robot WITHOUT SLAM gets this — slam_toolbox owns `map -> <ns>/odom` for the
> mapping robot, and two publishers on one TF edge fight.

---

## SENDING A WAYPOINT (topic)

```bash
ros2 topic pub --once /robot2/goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 1.0, y: 2.0}, orientation: {w: 1.0}}}'
```

- `frame_id: map` — coordinates are metres in the **map** frame.
- **Orientation is ignored.** `yaw_goal_tolerance` is wide open on purpose: an Ackermann
  car cannot rotate in place, so demanding a heading strands it on the goal forever.
- Change `robot2` to `robot1` to drive the other car.
- **Publish once.** Every message restarts navigation, so `-r 1` makes the robot
  continually re-plan to the same point and never settle.
- If `--once` seems ignored, the publisher exited before discovery finished —
  use `-t 3` instead.

Nav2 plans around obstacles automatically from the LiDAR.

### Is Nav2 on the simulation clock?
```bash
ros2 param get /robot2/controller_server use_sim_time     # must be True
```
If this is False the robot never moves and every goal still reports SUCCESS. Passing
`use_sim_time:=true` to the launch file does NOT guarantee it — see NAMESPACING.md A11.

### Is the TF tree healthy?
```bash
ros2 run tf2_tools view_frames
```
There must be **one** tree rooted at `map`, and **no** bare `robot1` / `robot2` frame. A
bare model frame means the Gazebo pose publisher is broadcasting link poses again, which
gives `<ns>/base_link` a second parent — the tree splits in two and the RViz axes blink.
See NAMESPACING.md A10.

### Where is the robot really?
```bash
ros2 run tf2_ros tf2_echo map robot2/base_link
```
This is the ground truth, in the same frame as the goal. Do **not** trust the
action status — it has reported SUCCEEDED with the robot 2 m away.

### Stop it
```bash
ros2 run saye_bringup stop_robot.py robot2
```

---

## Mesh monitor (optional, either NUC)
```bash
ros2 launch rajant_mesh mesh_launch.py
ros2 topic echo /mesh_matrix --once
```

## Sanity check when something misbehaves
```bash
ros2 topic info /robot2/cmd_vel      # exactly 1 publisher is healthy
```
More than one means duplicate Nav2 stacks or leftover ghost participants from
killed containers — clean up and restart.
