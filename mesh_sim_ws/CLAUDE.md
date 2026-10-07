# mesh_sim_ws

Two Ackermann robots (`robot1`, `robot2`) in **one** Gazebo world, driven from two NUCs
linked by a Rajant mesh. ROS 2 Humble + Gazebo Harmonic, everything on `ROS_DOMAIN_ID=42`.

Multi-robot separation is by **frame prefix**, not by topic: one global `/tf`, one `/map`,
one `/clock`, and per-robot `/robot1/...` `/robot2/...` for everything else.

---

## Machines

| | host | mesh IP | container | `ROBOT_NS` | runs |
|---|---|---|---|---|---|
| NUC1 | `192.168.7.143` | `10.195.219.0` | `nuc1` | `robot1` | Gazebo + **both** robot models, SLAM, robot1 Nav2 |
| NUC2 | `192.168.7.56` ⚠️ | `10.195.221.0` | `nuc2` | `robot2` | robot2 Nav2 only. **Never** Gazebo |

⚠️ `192.168.7.56` does not respond — **ssh to `10.195.221.0`**, user `asl`, password in `~/.fair_chain.env` (`NUC2_SSH_PASSWORD`, not in the repo).

Gazebo only ever runs on NUC1 and spawns *both* robots. What moves to NUC2 is only the
decision-making (Nav2). That is why robot2 drives fine with NUC2 switched off.

```bash
docker exec -it nuc1 bash                                # NUC1
ssh asl@10.195.221.0 -t 'docker exec -it nuc2 bash'      # NUC2
```
`ros2` works immediately in an interactive shell. For **non-interactive** one-liners
(`docker exec nuc1 ros2 ...`) prefix `/entrypoint.sh` — bash skips `.bashrc` there.

---

## Running it

```bash
# ALWAYS first — nothing else may be on domain 42
docker restart nuc1 && ssh asl@10.195.221.0 'docker restart nuc2'
```

**Single machine** (NUC2 idle; what to use for developing nav/TF):
```bash
ros2 launch saye_bringup bringup_all.launch.py
```

**Two NUCs** (the real setup) — two commands, one per machine:
```bash
# NUC1
ros2 launch saye_bringup bringup_all.launch.py nav:=robot1
# NUC2 (ROBOT_NS is already robot2)
ros2 launch saye_bringup nav_robot.launch.py static_map_odom:=true
```
Start NUC2's side **after** NUC1 is up, or its costmaps come up with no `/clock` and no TF.
Do not use `bringup_all` on NUC2 — its preflight would (correctly) see NUC1's healthy stack
as a busy domain and refuse.

Ready in ~90 s. Watch for two `Managed nodes are active` lines.

### Waypoints

The robot is picked by **namespace prefix** — there is no "which robot" field anywhere.
Coordinates are metres in the shared `map` frame for both robots. **Orientation is ignored**
(see Ackermann note below). `robot2` spawns at `(0, 2.0)`.

```bash
ros2 run saye_bringup send_waypoints.py robot1 2.0 0.0          # 1 pair  → NavigateToPose
ros2 run saye_bringup send_waypoints.py robot2 1.0 5.0 4.0 5.0  # several → FollowWaypoints
ros2 run saye_bringup stop_robot.py robot2                      # cancel + zero cmd_vel
ros2 topic pub --once /robot2/goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 1.0, y: 2.0}, orientation: {w: 1.0}}}'
```

---

## Invariants — break these and the system fails quietly

1. **`publish_link_pose` in the SDF `PosePublisher` MUST stay `false`.** It and
   `OdometryPublisher` share the gz topic `/model/<ns>/pose`; with link poses on,
   `<ns>/base_link` gets two parents (`<ns>/odom` and the bare model frame `<ns>`), tf2
   reparents at 20 Hz, and you get two disconnected trees plus blinking RViz axes.
2. **`/<ns>/joint_states` must stay bridged.** All six joints are `revolute`, so
   `robot_state_publisher` emits *no* TF without it and the wheel frames vanish.
3. **Every node block in `nav2_params.yaml.in` needs its own `use_sim_time: True`.**
   `nav2_bringup`'s `RewrittenYaml` only *replaces* existing keys, never inserts — so
   `use_sim_time:=true` on the launch line silently does nothing for a block that lacks it.
   A node on the wall clock reports `Reached the goal!` without moving.
4. **The static `map -> <ns>/odom` is IDENTITY.** Gazebo's `OdometryPublisher` reports the
   model's **world** pose, not a spawn-relative one. Passing the spawn pose double-counts it.
5. **Exactly one publisher per TF edge, and exactly one Gazebo.** Only one robot runs SLAM
   (it owns `map -> <ns>/odom`); the other gets the static transform.
6. **Ackermann cars cannot rotate in place.** `yaw_goal_tolerance` is deliberately wide open
   (3.15). A real yaw tolerance strands the robot sitting *on* the goal forever, thrashing in
   a `spin` recovery that can never succeed.

Verify with:
```bash
ros2 run tf2_tools view_frames      # ONE root: map. No bare robot1/robot2 frame. 31 frames
ros2 topic info /clock              # exactly 1 publisher
ros2 param get /robot1/controller_server use_sim_time    # must be True
bash verify.sh                      # inside the container
```

---

## Layout

```
mesh_sim_ws/
├── Dockerfile              the ONE image, mesh_sim:humble (ros:humble + gz-harmonic + ros_gz built from source)
├── RUNBOOK.md              step-by-step ops, stopping things, gotcha table
├── WORKFLOW.md             daily use, namespace explanation, waypoints
├── verify.sh               post-launch sanity checks; run INSIDE the container
├── vendor/rajant_api/      vendored Rajant protobuf API (not on PyPI)
└── src/
    ├── rajant_mesh/                    mesh link monitor (own package, NOT a git repo)
    │   ├── launch/mesh_launch.py       starts both nodes; namespace:= / $MESH_NS
    │   ├── rajant_mesh/weakest_link.py publishes /weakest_link
    │   └── rajant_mesh/mesh_matrix.py  publishes /mesh_matrix
    └── ackermann-vehicle-gzsim-ros2/   git repo, branch main
        ├── NAMESPACING.md              ** the design doc. A1–A11 = every trap and why **
        ├── saye_bringup/               all launch files + config  ← most work happens here
        ├── saye_description/           robot SDF, meshes, world
        ├── saye_control/               C++ control client — DORMANT, no launch file uses it
        └── saye_msgs/                  interfaces — only consumer is saye_control, so also dormant
```

### `saye_bringup/launch/`

| File | What it does |
|---|---|
| `bringup_all.launch.py` | **Main entry point.** Gazebo + both robots + SLAM + a Nav2 stack each, staged with delays. Args: `slam` `slam_robot` `nav` `rviz` `preflight` `slam_delay` `nav_delay` `nav_stagger` `rviz_delay` |
| `multi_vehicle.launch.py` | Gazebo once + both robots namespaced. `rviz:=`, `GZ_HEADLESS=1` env |
| `nav_robot.launch.py` | Nav2 for ONE robot. `namespace:=` `static_map_odom:=` `map_odom_x/y/yaw:=`. Uses `OpaqueFunction` so the *resolved* namespace drives frames and params |
| `slam.launch.py` | slam_toolbox, stays in the global namespace so `/map` is shared |
| `saye_spawn.launch.py` | Gazebo + a single robot from `$ROBOT_NS` |
| `robot_spawn.py` | **NOT a launch file** — helper module (`spawn_robot_group`, `clock_bridge_node`, `rviz_config_for`) imported by the spawn launches |
| `navigation_bringup.launch.py` | ⚠️ **LEGACY, do not use.** Pre-namespacing; wraps nav2 `bringup_launch.py`, which starts map_server + AMCL and would publish a second `map -> odom` against SLAM |
| `amcl.launch.py` | ⚠️ **LEGACY.** Not namespace-aware, `use_sim_time` defaults to *false* |

### `saye_bringup/config/`

`*.yaml.in` are **templates**: `${NS}` is substituted in Python at launch and written to a
temp file. The plain `.yaml` next to them are the originals kept for reference.

| File | Notes |
|---|---|
| `ros_gz_bridge.yaml.in` | per-robot bridge. `/tf` and `joint_states` live here; **`clock` deliberately does not** |
| `clock_bridge.yaml` | the single global `/clock` bridge, launched outside every namespace |
| `nav2_params.yaml.in` | Nav2 for one robot. MPPI + Smac Hybrid, Ackermann model. `use_sim_time` in every block |
| `slam.yaml.in` | namespaced *inputs* (`scan_topic`, `odom_frame`, `base_frame`), global `map_frame: map` |

### Other paths worth knowing

- `saye_description/models/saye/model.sdf` — the robot. `${NS}` tokens; the four gz plugins
  (AckermannSteering, PosePublisher, OdometryPublisher, JointStatePublisher) provide all motion.
  There is **no xacro/urdf and no ros2_control** anywhere in this repo.
- `saye_description/worlds/saye_world.sdf` — world `my_world`. Contains no robot.
- `saye_bringup/scripts/send_waypoints.py` / `stop_robot.py` — the only executables in this
  package (`rajant_mesh` adds `mesh_matrix` and `weakest_link`). Everything else is a launch file.
- `saye_bringup/rviz/saye.rviz` — per-robot substituted at launch; Fixed Frame becomes `map`.
- `saye_bringup/rviz/navigation.rviz` — ⚠️ **un-namespaced legacy** (`/scan`, `/plan`,
  `TF Prefix: saye`). Shows almost nothing as-is.

---

## Namespacing

**Global, never prefixed:** `/clock` (one Gazebo, one clock — namespacing it stalls every
`use_sim_time` node with no error), `/tf` `/tf_static` (one tree, split by prefix), `/map`
`/map_metadata`, `/pose`, `/mesh_matrix` `/weakest_link`.

**Per robot:** `cmd_vel`, `odom`, `scan`, `imu`, `joint_states`, `robot_description`,
`goal_pose`, all Nav2 nodes and actions, and all TF frames (`robot1/base_link`,
`robot1/base_link/gpu_lidar`, …).

TF tree — one root, 31 frames:
```
map ─┬─ robot1/odom ── robot1/base_link ─┬─ base_link/{gpu_lidar,imu_sensor,rs_front,camera_*}
     │                                   └─ {front_*_wheel_steering_link → *_wheel_link, rear_*_wheel_link}
     └─ robot2/odom ── robot2/base_link ─┴─ (same)
```
`map→odom` from slam_toolbox (mapping robot) or `static_transform_publisher` (the other);
`odom→base_link` from gz OdometryPublisher; sensors from gz PosePublisher; wheels from
robot_state_publisher via bridged joint_states.

---

## Deploying a change

**The containers do NOT bind-mount the workspace** — code is baked into the image. Any edit
needs a rebuild *and* a transfer. Both NUCs must run a byte-identical image.

```bash
cd /home/asl/mesh_sim_ws && docker build -t mesh_sim:humble .
docker rm -f nuc1 && docker run -d --name nuc1 --network host \
  -e ROS_DOMAIN_ID=42 -e ROBOT_NS=robot1 -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --privileged \
  --restart unless-stopped mesh_sim:humble sleep infinity

docker save mesh_sim:humble | gzip -1 | ssh asl@10.195.221.0 'gunzip -c | docker load'
ssh asl@10.195.221.0 'docker rm -f nuc2 && docker run -d --name nuc2 --network host \
  -e ROS_DOMAIN_ID=42 -e ROBOT_NS=robot2 --restart unless-stopped mesh_sim:humble sleep infinity'
```
~1.6 GB compressed; 4–14 min depending on mesh throughput. `xhost +local:docker` on NUC1
once per boot or Gazebo dies with "cannot open display".

**`docker restart` does NOT pick up a new image** — it keeps the old one. Always
`rm -f` + `run`. Check drift with:
`docker inspect nuc1 --format '{{.Image}}'` vs `docker images mesh_sim:humble`.

---

## Debugging

| Symptom | Cause |
|---|---|
| `tf2_buffer: Detected jump back in time` + blinking RViz + SLAM dropping every scan | **Two Gazebos on domain 42** = two `/clock` streams. `jump back in time` means **clocks, not frames**. Count them first |
| Every goal reports SUCCESS instantly, robot never moves | A Nav2 node is on the wall clock (invariant 3) |
| Robot sits ON the goal, `Failed to make progress` → `Running spin` | yaw tolerance too tight (invariant 6) |
| Bare `robot1`/`robot2` frame in `view_frames` | link poses back on (invariant 1) |
| Wheel frames missing | joint_states unbridged (invariant 2) |
| Clock stuck at 0, nothing moves, no errors | Gazebo launched paused — needs `-r` |
| `pkill -f <pattern>` does nothing | It matches its own `bash -c` wrapper and kills itself. Kill by PID, or `docker restart` |
| `ros2: command not found` in `docker exec` | Use `/entrypoint.sh <cmd>` |
| `ros2 node list` misses nodes | Single-shot discovery is unreliable — sample 2–3× and union |
| gz `Exception sending a multicast message` | gz-transport discovery on an interface with no multicast route. Noisy but harmless; pin with `GZ_IP=` if gz topics go missing |

---

## Housekeeping

- Most of the namespacing work is **uncommitted** on `main` (`git status` in
  `src/ackermann-vehicle-gzsim-ros2` shows 4 modified + many untracked).
- Five `*.orig` backups (`model.sdf.orig`, `slam.yaml.orig`, `ros_gz_bridge.yaml.orig`,
  `saye_spawn.launch.py.orig`, `slam.launch.py.orig`) sit next to the live files and ship
  inside the image, because `CMakeLists.txt` installs whole directories. Git has the history;
  they can go.
- `saye_control` + `saye_msgs` build but nothing launches them.

---

# SECOND PROJECT — chain_robot + hospital world (Gazebo Classic)

Added 2026-09-01. **This is a different sim from everything above.** `mesh_sim_ws` is
Gazebo **Harmonic**; this is Gazebo **Classic 11**. They cannot share a world.

Full detail lives in **`/home/asl/hospital_world/CHECKPOINT.md`** (git repo, tagged
`hospital-working`). Read that first — this section is the summary.

## Roles

- **chain_robot** — articulated 4-module tracked "mission robot" (head). Ships in the
  image `gudwls124z/safe_robot:260826`.
- **relay1 / relay2** — the Ackermann `saye` cars, **ported from Harmonic SDF 1.8 to
  Classic SDF 1.7** so they can share chain_robot's world. Port lives in
  `/home/asl/hospital_world/models/saye_classic/`, generated by `port_saye.py`.
  **Edit `port_saye.py` and re-run it — never hand-edit `model.sdf`, it is generated.**

## Machines / containers

| container | machine | role |
|---|---|---|
| `safe_robot` | NUC1 | Gazebo + physics + chain_robot + **both relay models** |
| `nuc1` (mesh_sim:humble) | NUC1 | relay1's Nav2 + `send_waypoints.py` |
| `nuc2` (mesh_sim:humble) | NUC2 `10.195.221.0` | relay2's Nav2 |

`safe_robot` has `nav2_bringup` but **not** `saye_bringup`, which is why the relay brains
run in the mesh_sim containers. All three are `--network host` on `ROS_DOMAIN_ID=42`.

## Running it

    cd /home/asl/hospital_world
    ./run_chain_stack.sh              # 1 command; exploration is OPT-IN (--explore)
    ./spawn_saye.sh relay1 2.5 12.0
    ./spawn_saye.sh relay2 3.5 12.0
    ./run_relay1_nuc1.sh relay1       # AFTER the relay exists - see ordering below
    ./run_relay2_nuc2.sh relay2
    ./waypoint.sh relay1 5.0 -1.0     # map frame
    ./stop_all.sh

## Traps that cost hours — do not rediscover these

1. **`GAZEBO_IP=127.0.0.1` is mandatory.** `enp86s0` is UP with **no IP address**, so every
   gzserver multicast send fails and the retries block the physics thread. Symptom: the log
   fills with `Exception sending a multicast message:Network is unreachable` and the sim
   looks **frozen**. Fixing this took RTF from 1.16 Hz to 4.25 Hz. Already in the scripts.
2. **`GAZEBO_MODEL_DATABASE_URI=` (empty) is mandatory.** Otherwise a missing model sends
   Gazebo to `models.gazebosim.org`, which is retired and never answers — it hangs forever
   with no error.
3. **`GAZEBO_MODEL_PATH` must include `/hw/models`** or the hospital loads empty.
4. **Gazebo's window opens BEHIND VS Code** at 576x320 and GNOME will not raise it; you get
   a "Gazebo is ready" notification instead. **Alt+Tab.** This looked like "gazebo never
   starts" for a long time.
5. **Start a relay's Nav2 only AFTER that relay is spawned.** Its costmaps need
   `relay*/base_link` on TF at configure time; if it starts first the lifecycle nodes sit
   in `inactive` forever and never retry. Restarting the brain fixes it.
6. **`ROS_LOCALHOST_ONLY=1` is baked into the safe_robot image's `.bashrc`** and would stop
   NUC2 seeing anything. `run_safe_robot.sh` appends an override at the END of `.bashrc`.
8. **`--ipc host` on BOTH `safe_robot` and `nuc1` is MANDATORY.** They are both
   `--network host`, so FastDDS treats them as same-host participants and prefers the
   **shared-memory** transport. With separate IPC namespaces their `/dev/shm` differ
   (`27:1` vs `110:1`), SHM segments written by one are invisible to the other, and you get
   the nastiest failure mode available: **discovery succeeds over UDP — `ros2 topic list`,
   `node list`, `topic info -v` all look perfectly correct — while every message is silently
   dropped.** `topic hz` reports nothing and Nav2 costmaps never get TF, so the brain never
   activates. NUC2 is unaffected because it is genuinely remote and uses UDP.
   Fix: `./run_nuc1_container.sh` (recreates nuc1 with `--ipc host`). Verify with
   `docker exec nuc1 /entrypoint.sh bash -c 'export ROS_DOMAIN_ID=42; ros2 topic hz /tf'` —
   **`topic list` is NOT a sufficient check; it passes while the path is dead.**

7. `map` is **not** the world frame. It is chain_robot's spawn, world `(0, 12)`. Both relay
   launchers pass `map_odom_x:=0.0 map_odom_y:=-12.0`. Verified correct: relay1 at world
   (4.98, 12.00) reads map (4.98, 0.00).

## State

**Working:** hospital world + chain_robot; both relays spawn; relay TF, odom, scan, IMU all
publish and cross to NUC2; relay1's Nav2 activates and **does steer toward a goal**
(commanded map (5.0, -1.0), it tracked from y=-0.004 to y=-0.480).

**Broken — the one thing to fix next: relay top speed.**
Commanded 0.3 m/s, achieves ~0.056 m/s. Wheels spin at 7.8 rad/s (= 0.286 m/s of tread)
while the body does 0.056 → ~80% wheel slip **under torque only**. At rest wheels and
ground match exactly, so it is not a static friction problem.

Already tried and ruled out:
- `linear_velocity_pid_gain` sweep 10 / 100 / 150 / 400 → plateaus ~0.17 m/s, 400 unstable.
- `mu` 0.5 → 1.2 → **5.0** (absurd): only 0.056 → 0.075 m/s. **Not traction-limited.**
- Drive-joint `<effort>` was absent, added (2 N.m). No change to top speed.
- Contact `kp 1e6 / kd 100 / min_depth 0.001`. No change to top speed.

Next leads: how `libgazebo_ros_ackermann_drive.so` actually applies wheel motion
(`SetParam("vel"/"fmax")` vs force), and whether the world's `<ode>` block
(`iters 100`, `contact_max_correcting_vel 0.2`) is fighting it.

**chain_robot itself is still broken and untouched** — it spins on the spot in its own
`UNIST_indoor.world`. That is its author's bug, not ours. All my debugging edits to the
chain_robot package were reverted; the image's packages are pristine.

## Fixed this session (do not regress)

| symptom | cause |
|---|---|
| relay steered itself in circles with nothing on cmd_vel | steering knuckles authored at 0.005 kg / izz 1.8e-6 (gz drove them kinematically); a Classic torque PID gave 333,000 rad/s2. Now 0.1 kg / izz 1e-4, PID `2.0 0 0.2`. **Do NOT add `<damping>`** — ODE emits NaN. |
| gzserver segfault on relay respawn | Classic's **GPU** ray caps horizontal FOV at 180 deg, model wants 360. Switched to CPU `<ray>`. Also `vertical samples 1` needs min_angle == max_angle. |
| relays crept ~4 mm/s parked | `linear_velocity_pid_gain` D term at a 1 ms step (×1000 noise), plus my own `max_vel 0.1` letting ODE inject contact-correction velocity. Now `100 0 0` and `max_vel 0.01`. Drift: 17.1 cm → 1.5 cm per 40 s. |
| car would not drive at all | `linear_velocity_pid_gain` was **absent**, and it defaults to 0 = no wheel torque. |
| blue lidar rays / lag | `<visualize>false</visualize>`, samples 1024 → 360. |

## Fixes folded back INTO mesh_sim_ws (2026-09-03)

These were found while debugging the Classic port but are **mesh_sim_ws bugs** — they
affect the Harmonic robots too. All three are now applied to the source here, but the
running `nuc1`/`nuc2` containers have the code **baked into `mesh_sim:humble`**, so the
image must be rebuilt for them to take effect there.

1. **`saye_description/models/saye/model.sdf`** — `AckermannSteering` listed
   `front_right_wheel_joint` **twice** and never named `rear_right_wheel_joint`, so the
   rear right wheel was undriven. Left side correctly listed front+rear. Fixed.

2. **`saye_bringup/config/nav2_params.yaml.in` — `min_turning_r` 0.2 → 0.70.**
   Geometry allows only `wheel_base 0.2255 / tan(0.4) = 0.533 m`; 0.2 claimed a turn 2.7x
   tighter than the car can make, so MPPI planned arcs it could never execute. 0.70 is the
   value measured on the Classic port (0.682 with slip). Over-estimating is the safe
   direction — the car can always turn tighter than planned, never wider.

3. **`nav2_params.yaml.in` — `GoalAngleCritic` disabled.** It was `enabled: true` with
   `threshold_to_consider: 0.5`, contradicting this same file's `yaw_goal_tolerance: 3.15`
   ("effectively any heading", because an Ackermann car cannot rotate in place). Waypoints
   stalled repeatably at ~0.47–0.51 m — matching that 0.5 m threshold — with tiny
   sign-flipping `cmd_vel` as it fought `GoalCritic`.

**Not yet verified against the Harmonic robots.** Fixes 2 and 3 were validated on the
Classic port only (turning waypoints went from 300 s timeout to ARRIVED in 20.6 s). Worth
re-running a `robot1` waypoint in the Harmonic sim to confirm no regression.

## The Classic port bug (fixed) — `use_parent_model_frame`

Every joint carries `<use_parent_model_frame>1</use_parent_model_frame>`. The Harmonic model
is **SDF 1.8**, where that tag was removed and sdformat **ignores** it. The Classic port is
**SDF 1.7**, where Gazebo Classic's sdformat9 still **honors** it — pinning the front wheels'
spin axis to the model frame so it does NOT follow the steering link. The wheels spun about
a fixed axis regardless of steering: measured turning radius **−1.835 m** (wrong sign, 3.4x
too wide) vs 0.533 m geometric. Forward waypoints arrived in 12 s; every turning waypoint
timed out. `port_saye.py` now strips the tag → radius **+0.682 m**, turning waypoints arrive.

**Lesson: downgrading SDF 1.8 → 1.7 for Classic can revive dead tags that were inert
upstream.** Check for removed-then-reinterpreted elements when porting.
