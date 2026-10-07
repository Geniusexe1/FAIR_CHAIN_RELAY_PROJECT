# Multi-Robot Namespacing: Issues, Workarounds, and Verification

How two Ackermann robots are made to coexist in **one** Gazebo world without colliding
on topics or TF frames — and why the standard ROS 2 recipe does not apply to this repo.

**Namespaced:** ackermann topics, all TF frames, Gazebo model + gz-transport names.
**Deliberately global:** `/tf`, `/clock`, `/map`, `/weakest_link`, `/mesh_matrix`.

---

## Summary table

| # | Issue | Root cause | Workaround |
|---|---|---|---|
| A1 | No xacro/urdf to parameterise | Robot is pure SDF | `${NS}` tokens + Python `.replace()` at launch |
| A2 | No `ros2_control` to namespace | Motion is Gazebo system plugins | Per-robot values in the `<plugin>` blocks |
| A3 | Namespaces silently ignored | Bridge topics are **absolute** | Generate bridge YAML per robot |
| A4 | `use_sim_time` nodes stall | `/clock` got namespaced | Bridge `clock` **once**, globally |
| A5 | Half-broken TF tree | TF has **two** independent sources | Prefix both; `frame_prefix` covers only one |
| A6 | Spawn hangs forever | `-topic '/robot_description'` absolute | Use relative `robot_description` |
| A7 | RViz shows nothing | `.rviz` hardcodes `odom` / `saye` | Generate per-robot `.rviz` |
| A8 | Map should stay shared | SLAM would publish `/robot1/map` | Keep slam_toolbox global, namespace its *inputs* |
| A10 | Two TF trees per robot; RViz axes blink | Two gz plugins share `/model/<ns>/pose` | `publish_link_pose=false`; bridge joint states |
| A11 | Every goal "succeeds" instantly, robot never moves | Nav2 nodes on the WALL CLOCK | `use_sim_time` in every params block |

---

## A1 — There is no `.xacro` or `.urdf`

The usual instruction is "modify the xacro to accept a prefix argument." **This repo has zero
xacro/urdf files.** The robot is `saye_description/models/saye/model.sdf`, and plain SDF has no
`<arg>`/property mechanism — that is a xacro feature. There is nothing to pass an argument to.

**Workaround.** `model.sdf` now carries `${NS}` tokens, resolved in Python at launch by
`launch/robot_spawn.py::robot_description_for()`. The upstream launch file *already* read this
file into a string, so this adds no new tooling.

**Rejected:** converting SDF→URDF+xacro (large rewrite, and `sdformat-urdf` drops the `<plugin>`
blocks that provide all the motion); an `envsubst` entrypoint (extra moving part, and the launch
file would still need the per-robot string in memory).

## A2 — There is no `ros2_control`

`grep -rE 'ros2_control|controller_manager|ackermann_steering_controller|joint_state_broadcaster'`
returns **0 hits**. There are no controller YAMLs to push into a namespace. Motion comes from four
Gazebo system plugins.

**Workaround.** The equivalent is per-robot values inside `model.sdf`:

| Location | Was | Now |
|---|---|---|
| `<model name=...>` | `saye` | `${NS}` |
| ackermann `<topic>` (L732) | `/cmd_vel` | `/${NS}/cmd_vel` |
| odometry `<odom_frame>` (L743) | `odom` | `${NS}/odom` |
| odometry `<robot_base_frame>` (L744) | `saye` | `${NS}/base_link` |
| odometry `<odom_topic>` (L745) | `odom` | `${NS}/odom` |
| sensor `<topic>` (L66,176,229,276,…) | `imu`, `scan`, … | `${NS}/imu`, `${NS}/scan`, … |
| `<gz_frame_id>` (L7, L175) | `saye/base_link` | `${NS}/base_link` |

> **Why this matters *here*.** These are **gz-transport** names, not ROS names. Two *separate*
> Gazebo worlds would not collide. Because both robots share **one** world, they do.

Mesh URIs (`package://saye_description/...`) and visual/collision element names (`saye/TopVisual`)
are left alone — the former are asset paths, the latter are scoped inside their own model.

## A3 — Absolute topic names make namespaces a no-op

**The subtlest failure.** `config/ros_gz_bridge.yaml` declared `/cmd_vel`, `/odom`, `/scan`,
`/tf`, `/imu` **with a leading slash**. In ROS 2 an absolute name ignores `PushRosNamespace`
entirely. Wrapping the bridge in a namespace group therefore *looks* correct while both robots
keep publishing to the same global topics.

**Workaround.** `config/ros_gz_bridge.yaml.in` is a template; `robot_spawn.py::_substitute()`
writes a resolved copy to a temp file per robot and passes it as `config_file`.

**Rejected:** per-topic `remappings=` on the bridge node — works, but every topic must be listed
twice and it drifts out of sync with the YAML.

## A4 — `/clock` must stay global (the quiet killer)

`clock` was the one **relative** entry in the bridge YAML. Under a namespace it becomes
`/robot1/clock`. There is exactly one Gazebo, so there must be exactly one clock — and every node
with `use_sim_time:=true` subscribes to `/clock` and simply **stalls, with no error message**, if
it is namespaced away.

**Workaround.** `clock` was removed from the per-robot template into `config/clock_bridge.yaml`,
launched once by `robot_spawn.py::clock_bridge_node()` **outside** every `GroupAction`.

## A5 — TF has two independent sources

The reason naive prefixing yields a half-broken tree:

1. **`robot_state_publisher`** — link/joint transforms from `robot_description`. Controlled by the
   **`frame_prefix`** parameter; `base_link` and all six wheel links come along automatically.
2. **The Gazebo pose-publisher bridged into `/tf`** — frames named by the SDF's `gz_frame_id`,
   the model name, and the odometry plugin's `odom_frame`/`robot_base_frame`.
   **`frame_prefix` does not touch these.** They are fixed in the SDF (A2).

Prefix only one and you get `robot1/base_link` from one source and bare `base_link` from the
other — a disconnected tree. Both are handled.

> `/tf` itself stays **global**. ROS 2 multi-robot convention is one `/tf` tree separated by frame
> prefix, not one topic per robot.

## A6 — `-topic '/robot_description'` was absolute

`saye_spawn.launch.py:72` spawned with `'-topic', '/robot_description'`. Under a namespace,
`robot_state_publisher` publishes `/robot1/robot_description`, so `create` waits forever on a
topic nobody feeds. Now relative (`'robot_description'`), resolved inside the group.

## A7 — RViz config hardcodes frames

`rviz/saye.rviz` has `Fixed Frame: odom` (L313), `Reference Frame: saye` (L308),
`Target Frame: saye` (L372). With prefixed frames RViz opens to an error and displays nothing.
`robot_spawn.py::rviz_config_for()` writes a per-robot copy with those three substituted.

## A8 — `/map` stays global while the robot is namespaced

`slam.launch.py` keeps `namespace=''` so slam_toolbox publishes **global `/map`** (the occupancy
grid is shared by design). Only its *inputs* are namespaced, via `config/slam.yaml.in`:
`scan_topic: /${NS}/scan`, `odom_frame: ${NS}/odom`, `base_frame: ${NS}/base_link`, with
`map_frame: map` unchanged. TF then reads `map → robot1/odom`, correct for a shared map.

## A10 — Two gz plugins share one TF topic, so `base_link` had two parents

**The bug A5 set up but did not catch.** Both frame sources were prefixed correctly, and
the tree still broke — because the prefixes were never the problem. `<ns>/base_link` had
**two parents**, each published at 20 Hz:

| Publisher | Edge | Where the frame name comes from |
|---|---|---|
| `gz-sim-odometry-publisher-system` | `<ns>/odom → <ns>/base_link` | `<odom_frame>` / `<robot_base_frame>` |
| `gz-sim-pose-publisher-system` | `<ns> → <ns>/base_link` (and every other link) | the model name, automatically |

Neither plugin declares a `<tf_topic>`, and in Harmonic **both default to
`/model/<model_name>/pose`** — the single topic the bridge maps onto the global `/tf`.
Confirmed with `strings libgz-sim8-odometry-publisher-system.so | grep -E '/model/|tf_topic'`.

tf2 stores exactly one parent per frame, so `<ns>/base_link` was reparented back and forth
at 20 Hz. All three reported symptoms are that one fact:

- **`map` was not the root.** The bare `<ns>` frame is an extra root that nothing connects
  to `map`, and the whole link set hung off it. `view_frames` drew two disconnected copies
  of each robot — four "robots" for two.
- **RViz axes blinked.** Whenever `base_link` was parented to `<ns>`, the robot was
  unreachable from the fixed frame and RViz dropped it; the next message reattached it to
  `<ns>/odom` and it came back.
- Every `lookupTransform('map', '<ns>/base_link')` failed about half the time, which is
  what costmap updates and `bt_navigator` pose lookups are built on.

**Workaround.** `<publish_link_pose>false</publish_link_pose>` in the `PosePublisher` block.
The odometry plugin is then the only publisher of that edge. `<publish_sensor_pose>` stays
**true**: sensor poses are parented to their own link (`<ns>/base_link →
<ns>/base_link/gpu_lidar`), so they add no second parent, and `nav2_params.yaml.in`'s
`scan_frame_id` depends on them.

**The part that is easy to miss.** Dropping link poses removes the wheel and steering
frames, and `robot_state_publisher` was **not** filling that gap — it broadcasts a joint's
transform only when a `JointState` arrives, all six joints here are `revolute`, and
`/<ns>/joint_states` was not in the bridge. RSP was contributing *nothing* to `/tf`. So the
fix is two changes, not one: pin `<topic>${NS}/joint_states</topic>` on the gz
`JointStatePublisher` and bridge it (`gz.msgs.Model` → `sensor_msgs/msg/JointState`). RSP
already sits inside `PushRosNamespace(ns)` and subscribes to the relative `joint_states`,
so no remap is needed.

**Resulting tree — one root, one robot each:**

```
map ──┬── robot1/odom ── robot1/base_link ──┬── robot1/base_link/gpu_lidar   (gz sensor pose)
      │                                     └── robot1/*_wheel_link          (RSP + joint states)
      └── robot2/odom ── robot2/base_link ──┴── ...
```

`map → <ns>/odom` comes from slam_toolbox on the mapping robot. The other robot needs
`nav_robot.launch.py static_map_odom:=true`, which publishes an **identity** transform —
correct no matter where that robot was spawned, because Gazebo's `OdometryPublisher`
reports the model's **world** pose rather than a spawn-relative one. Measured with robot2
spawned at `y=2.0`:

```
/robot2/odom          x: 1.179261941007396    y: -0.8865667174750771
gz world pose         x: 1.1792619410073959   y: -0.88656671747507709
```

They are the same pose, so `<ns>/odom` is already coincident with the world origin. Passing
the spawn pose through `map_odom_x/y/yaw` double-counts it and puts the robot 2 m off; those
offsets exist only for a map whose origin is deliberately not the world origin.

## A11 — Nav2 ran on the wall clock, so every goal "succeeded" instantly

Passing `use_sim_time:=true` to `nav2_bringup`'s `navigation_launch.py` does **not**
reach the nodes. It applies the value through `RewrittenYaml(param_rewrites=...)`, which
only **replaces keys that already exist** in the params file and never inserts a missing
one. `nav2_params.yaml.in` had `use_sim_time` in exactly three blocks (`amcl`,
`map_server`, `velocity_smoother`), so six of the seven launched nodes came up on the
wall clock while the entire TF tree ran on sim time:

```
$ ros2 param get /robot1/controller_server use_sim_time
Boolean value is: False
```

The failure looks nothing like a clock bug. The controller stamps a pose with wall time,
the lookup against a sim-time tree fails, and the goal checker reads the resulting garbage
pose as "arrived":

```
[tf_help]: Transform data too old when converting from map to robot1/odom
[tf_help]: Data time: 1787819004s 966664771ns, Transform time: 134s 400000000ns
[controller_server]: Reached the goal!      <- robot never moved
[bt_navigator]: Goal succeeded
```

Nav2 reports SUCCESS on every goal while the robot sits still.

**Workaround.** `use_sim_time: True` in **every** `ros__parameters` block of
`nav2_params.yaml.in`, costmaps included. Verify per node, never by the launch argument:
`ros2 param get /<ns>/controller_server use_sim_time`.

**Two more Ackermann-specific fixes went in alongside it**, both in `nav2_params.yaml.in`:

- `general_goal_checker.yaw_goal_tolerance: 0.25 -> 3.15`. These are waypoints for a
  **car**: the goal is a position, and every caller leaves orientation at yaw=0 without
  meaning it. A real yaw tolerance is unreachable, because an Ackermann car cannot rotate
  in place. On arriving at xy with the wrong heading the controller logged `Failed to make
  progress`, aborted, and the BT fell into `Running spin` — a recovery the `behavior_server`
  block itself notes can never succeed on this vehicle. The robot sat **on** the goal while
  the action never completed.
- `progress_checker.movement_time_allowance: 10.0 -> 30.0`. A multi-point turn legitimately
  covers less than `required_movement_radius` (0.5 m) in 10 s, so the controller aborted
  goals that were progressing fine.

**And two bugs in `scripts/send_waypoints.py`**, which made working goals *look* dead:

- It preferred Nav2 feedback over TF for measuring arrival. Feedback stops the moment the
  goal completes, so the last value seen was often an early pose; the measured distance
  never dropped and the client spun out its full 300 s deadline while Nav2 had already
  logged `Goal succeeded`. TF is the ground truth and keeps updating — it is now first.
- `TransformListener(..., spin_thread=True)` gave the listener its own executor for a node
  the main loop was also spinning with `rclpy.spin_once()`. rclpy does not support one node
  in two executors: `/tf` callbacks stopped being serviced and *every* `lookup_transform`
  raised. Now `spin_thread=False`, with the one loop driving both feedback and TF.

## A9 — Two things needed no work

- **The world contains no robot.** `saye_world.sdf` holds only `ground_plane`, the warehouse, and
  the A/I/L letters; the robot is spawned via `create`. Spawning twice is clean.
- **`rajant_mesh` is untouched.** Its nodes already use *relative* topic names
  (`create_publisher(String, 'mesh_matrix', 10)`) and are deliberately left global so both NUCs
  publish to the same topics.

---

## Verification

```bash
# 1. Distinct control topics
ros2 topic list | grep cmd_vel          # /robot1/cmd_vel AND /robot2/cmd_vel

# 2. TF separated -- the key check for A5. A bare `base_link` means a half-prefixed tree.
ros2 run tf2_tools view_frames
ros2 topic echo /tf --once | grep frame_id

# 3. Globals stayed global (A4/A8)
ros2 topic list | grep -E '^/(clock|map|tf|weakest_link|mesh_matrix)$'

# 4. Control isolation -- regression test for BIDIRECTIONAL /cmd_vel
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/robot1/cmd_vel     # robot2 must NOT move

# 5. Sim clock alive (A4). If this hangs, /clock got namespaced.
ros2 topic echo /clock --once
```

## Files changed

| File | Change |
|---|---|
| `saye_description/models/saye/model.sdf` | `${NS}` tokens (A1/A2). `.orig` kept alongside |
| `saye_bringup/config/ros_gz_bridge.yaml.in` | new — per-robot template, no clock (A3) |
| `saye_bringup/config/clock_bridge.yaml` | new — the single global clock bridge (A4) |
| `saye_bringup/config/slam.yaml.in` | new — namespaced inputs, global `map_frame` (A8) |
| `saye_bringup/launch/robot_spawn.py` | new — shared spawn/substitution helper |
| `saye_bringup/launch/multi_vehicle.launch.py` | new — Gazebo once + two namespaced robots |
| `saye_bringup/launch/saye_spawn.launch.py` | namespace-aware single robot (defaults `$ROBOT_NS`) |
| `saye_bringup/launch/slam.launch.py` | `${NS}` substitution, stays in global namespace |
| `saye_description/models/saye/model.sdf` | `publish_link_pose=false` + pinned joint-state topic (A10) |
| `saye_bringup/config/ros_gz_bridge.yaml.in` | joint-states entry, required by A10 |
| `saye_bringup/launch/nav_robot.launch.py` | static `map-><ns>/odom` takes the spawn pose, not identity (A10) |
| `saye_bringup/config/nav2_params.yaml.in` | `use_sim_time` everywhere; Ackermann goal/progress checkers (A11) |
| `saye_bringup/scripts/send_waypoints.py` | TF-first arrival check; single-executor TF listener (A11) |
