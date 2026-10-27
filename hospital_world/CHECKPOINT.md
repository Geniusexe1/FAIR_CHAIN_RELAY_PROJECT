# CHECKPOINT — hospital world + chain_robot, working

Verified 2026-08-31. Fall back to this.

## Run it

    /home/asl/hospital_world/run_hospital.sh

Gazebo's window opens **behind VS Code** at 576x320 and GNOME will not raise it —
you get a "Gazebo is ready" notification instead. **Alt+Tab to it.** Resize once;
Gazebo remembers geometry in ~/.gazebo/gui.ini. Allow ~60 s for the meshes.

## State this checkpoint describes

- Container `safe_robot` from `gudwls124z/safe_robot:260826` (Gazebo Classic 11.10.2,
  ROS 2 Humble, 52.5 GB). Created by `run_safe_robot.sh`.
- **The image's own packages are PRISTINE.** All debugging edits to
  `chain_robot_plugin.cpp`, `chain_robot2.launch.py` and `fast_lio/config/mid360.yaml`
  were reverted from the image and `chain_robot` rebuilt. Nothing of mine is in there.
- The only artifacts are in this directory, mounted read-only at `/hw`.

## The two exports are not optional

`run_hospital.sh` sets both. If you launch by hand and skip them:

- **no `GAZEBO_MODEL_PATH=/hw/models`** -> Gazebo cannot resolve `model://aws_robomaker_*`
  and falls back to the online database.
- **no `GAZEBO_MODEL_DATABASE_URI=`** -> that database is `models.gazebosim.org`, which
  was retired and does not respond. Gazebo blocks on it indefinitely. Symptom is
  `[Wrn] [ModelDatabase.cc:340] Getting models from[http://models.gazebosim.org/]`
  followed by a permanent hang with no error.

## hospital_chain.world — what it is

`hospital.world` (AWS RoboMaker Hospital, single floor, from
mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps) plus:

1. The `chain_robot` `<include>` block copied verbatim from `UNIST_indoor.world:2441`,
   pose changed to `0 12 0.15`. **Required** — `chain_robot2.launch.py` has
   `spawn_entity_node` commented out, so the robot must come from the world file.
   A bare `hospital.world` gives an empty hospital with no robot.
2. SDF bumped 1.6 -> 1.7. `<plugin>` inside `<include>` needs SDFormat >= 1.7.
3. A tuned `<ode>` solver block copied from `UNIST_indoor.world`. hospital.world ships
   with none, and its `<physics>` had `default="0"` so it was not even selected.
   `contact_max_correcting_vel` defaults to 100 m/s, which launches contact-heavy
   models into the air.
4. Spawn z raised 0.0 -> 0.15. At z=0 the robot sometimes fell through the floor
   before the heavy collision meshes finished loading (seen once at z=-1067650).

## Restoring models/ (gitignored, 169 MB)

    BASE=https://raw.githubusercontent.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps/master/worlds/hospital
    for i in 1 2 3 4; do curl -LO $BASE/models_part$i.zip; done
    mkdir -p models
    for i in 1 2 3 4; do unzip -q -o models_part$i.zip -d tmp; mv tmp/models_part$i/* models/; rm -rf tmp; done

The zips extract to `models_partN/`, NOT `models/` — they must be merged flat.
Gate: 64 models referenced, 100 present, `MISSING: none`.

## Known-broken, NOT fixed here

`chain_robot` cannot drive. Commanded straight at 30 RPM it spins on the spot in its
own `UNIST_indoor.world`, and in the hospital it drives ~5 m then tumbles. Verified
with Nav2, exploration and FAST-LIO all stopped, joint references at zero, and nothing
in its path — so it is the robot model, not this world. Left for its author.

Suspects found while debugging (all reverted, none fixed it):
`chain_robot_plugin.cpp` negates every articulation torque via `joint_dir[0] == -1`;
the pitch/yaw PID banks are swapped; yaw runs D=25.0 at a ~1 ms step with +-200 N.m.

---

# Ackermann relay (saye) ported to Gazebo Classic

`models/saye_classic/` is the Harmonic `saye` model from `mesh_sim_ws`, ported so it
can share this Classic world with `chain_robot`. `chain_robot` itself is untouched.

## Spawn one

    ./spawn_saye.sh relay1  2.0 12.0            # ns, x, y  (z, yaw optional)
    ./spawn_saye.sh relay2 -2.0 12.0 0.10 1.57

Additive — call once per relay with a different namespace. The namespace becomes the
Gazebo entity name, the ROS namespace and the TF frame prefix.

Topics per relay: `/<ns>/cmd_vel` `/<ns>/odom` `/<ns>/points` `/<ns>/imu`
`/<ns>/joint_states` `/<ns>/camera_{front,rear,left,right}/image_raw` `/<ns>/rs_front/*`

## What the port changed

| Harmonic | Classic |
|---|---|
| `<sdf version="1.8">` | `1.7` (Classic 11's ceiling) |
| `gz-sim-ackermann-steering-system` | `libgazebo_ros_ackermann_drive.so` |
| `gz-sim-odometry-publisher-system` | folded in (`publish_odom` + `publish_odom_tf`) |
| `gz-sim-joint-state-publisher-system` | `libgazebo_ros_joint_state_publisher.so` |
| `gz-sim-pose-publisher-system` | **dropped** — see below |
| `<sensor type="gpu_lidar">` | `type="gpu_ray"` + `libgazebo_ros_ray_sensor.so` |
| `<sensor type="rgbd_camera">` | `type="depth"` + `libgazebo_ros_camera.so` |
| `<lidar>` / `<depth_camera>` | `<ray>` / removed |
| `<gz_frame_id>` (10 of them) | removed, no Classic equivalent |
| `package://saye_description/...` | `model://saye_classic/meshes/...` |

The pose publisher was dropped deliberately. In Classic each sensor plugin declares its
own `<frame_name>`, so re-adding one would recreate the two-parents-on-`base_link` bug
documented as A10 in `NAMESPACING.md`.

## Bug found in the upstream Harmonic model

`saye_description/models/saye/model.sdf` lists:

    <right_joint>front_right_wheel_joint</right_joint>
    <right_joint>front_right_wheel_joint</right_joint>

`front_right_wheel_joint` twice, and `rear_right_wheel_joint` never. **The rear right
wheel is undriven in the Harmonic sim too.** Fixed in the Classic port; still needs
fixing in `mesh_sim_ws`.

## Known gaps

- **No `robot_state_publisher`.** The ackermann plugin publishes `odom -> base_link`
  and each sensor declares its frame, but the wheel/steering link frames have no TF.
  Needs a URDF; the saye package ships SDF only.
- **Lidar is PointCloud2, not LaserScan** (128 vertical samples = 3D). For 2D nav, run
  `pointcloud_to_laserscan` on `/<ns>/points`, as the chain_robot stack already does
  for its Livox.
- **Untested in a live sim** as of this commit — sdformat parses it clean (exit 0, no
  warnings) but it has not yet been spawned.

---

# Two-NUC split

Mirrors `mesh_sim_ws`. **Gazebo only ever runs on NUC1.** Physics cannot be split, so
every robot's *model* — chain_robot, relay1 and relay2 alike — lives in NUC1's gzserver.
What moves to NUC2 is only relay2's decision-making.

| | mesh IP | container | runs |
|---|---|---|---|
| NUC1 | `10.195.219.0` | `safe_robot` | Gazebo + physics + chain_robot + relay1, and relay2's model |
| NUC2 | `10.195.221.0` | `nuc2` | relay2's Nav2 only. **Never** Gazebo |

`192.168.7.56` does not respond — ssh `10.195.221.0`, user `asl`, password in `~/.fair_chain.env` (`NUC2_SSH_PASSWORD`, not in the repo).

## Domain 42 — and the trap

Both sides are on `ROS_DOMAIN_ID=42`. The `safe_robot` image ships with
`ROS_DOMAIN_ID=52` **and `ROS_LOCALHOST_ONLY=1`** set in `/root/.bashrc`.
`ROS_LOCALHOST_ONLY=1` confines DDS to loopback, so NUC2 can never see NUC1 no matter
what — host networking and a healthy mesh included. `run_safe_robot.sh` appends an
override block at the **end** of `.bashrc` so it wins. Re-applied on every container
recreation; check with:

    docker exec safe_robot bash -ic 'echo $ROS_DOMAIN_ID $ROS_LOCALHOST_ONLY'   # 42 0

Verified working: a topic published on NUC1 is visible from inside `nuc2`.

## Order of operations

    # NUC1
    ./run_hospital.sh                          # gazebo + chain_robot
    ./spawn_saye.sh relay1  2.0 12.0
    ./spawn_saye.sh relay2 -2.0 12.0

    # NUC2 (after NUC1 is up AND relay2 is spawned)
    ./run_relay2_nuc2.sh relay2

Start NUC2 last, as in mesh_sim — otherwise its costmaps come up with no `/clock`
and no TF.

## Sensors: cameras dropped, lidar made 2D

Both for the mesh link, and because mapping is the goal.

| | before | after |
|---|---|---|
| 4x RGB + RGBD | ~1.1 Gbps/relay | **removed** |
| lidar | 1024x128 PointCloud2, ~21 MB/s | 1024x1 **LaserScan**, ~41 KB/s |

LaserScan also removes a dependency: NUC2 has slam_toolbox and 30 nav2 packages but
**no `pointcloud_to_laserscan`**, so a 3D cloud would have had nothing to consume it.
It matches mesh_sim, which already fed slam_toolbox on `<ns>/scan`.

Restore either with `DROP_CAMERAS = False` / `LIDAR_2D = False` in `port_saye.py`
(kept in the session scratchpad) and re-run it.

## Not yet tested

- relay2 has never been spawned into a live sim; the model only passes `gz sdf -p`.
- `nav_robot.launch.py namespace:=relay2` on NUC2 is reused from mesh_sim and has the
  right `namespace` argument, but has not been run against this Classic model.

---

# Full run sequence

Three containers on two machines. Gazebo only ever runs in `safe_robot`.

| container | machine | role |
|---|---|---|
| `safe_robot` | NUC1 | Gazebo + physics + chain_robot + **both relay models** |
| `nuc1` (mesh_sim) | NUC1 | relay1's Nav2 + waypoint tools |
| `nuc2` (mesh_sim) | NUC2 | relay2's Nav2 + waypoint tools |

`safe_robot` has `nav2_bringup` but **not** `saye_bringup`, which is why the relay brains
live in the mesh_sim containers. All three are `--network host` on domain 42.

    # --- NUC1 ---
    ./run_chain_stack.sh                    # 1 command, was 7 terminals
    ./spawn_saye.sh relay1 2.5 12.0         # 1 m apart, clear of the 1.3 m chain robot
    ./spawn_saye.sh relay2 3.5 12.0
    ./run_relay1_nuc1.sh relay1             # relay1 brain, container nuc1

    # --- NUC2 (last: needs /clock + TF from NUC1) ---
    ./run_relay2_nuc2.sh relay2             # ssh's over for you

    # --- waypoints (from NUC1; routes to the right machine itself) ---
    ./waypoint.sh relay1 5.0 0.0
    ./waypoint.sh relay2 3.0 0.0  6.0 2.0
    ./waypoint.sh relay1 stop

    ./stop_all.sh                           # stop everything, both NUCs

## The map frame is NOT the world frame

`map` is the **chain robot's** map origin, which is its spawn at world **(0, 12)** —
FAST-LIO starts its odom at zero there. So `map (0,0)` is `world (0,12)`, and a waypoint
of `5.0 0.0` means world `(5, 12)`.

The relays complicate this: `libgazebo_ros_ackermann_drive.so` has only
`UpdateOdometryWorld()` (no encoder variant), so relay odom is the **world** frame —
same as Harmonic's `OdometryPublisher`, so CLAUDE.md invariant 4 still holds and the
transform is not the spawn pose. But it is not identity either, because `map` is offset
from world by the chain robot's spawn. Hence both relay launchers pass:

    map_odom_x:=0.0 map_odom_y:=-12.0

**Verify this before trusting a waypoint** — compare TF against Gazebo ground truth:

    docker exec safe_robot bash -ic 'ros2 run tf2_ros tf2_echo map relay1/base_link'
    docker exec safe_robot bash -ic 'gz model -m relay1 -i | sed -n "4,8p"'

Spawned at world (2.5, 12.0), relay1 should read `map` ≈ **(2.5, 0.0)**. If it reads
(2.5, 12.0) the offset is not being applied; if it reads (2.5, 24.0) it is being applied
with the wrong sign.

---

# relay wheel/steering debugging (partial)

## Namespaces are clean — checked, not assumed

    /relay1/cmd_vel  /relay1/odom  /relay1/scan  /relay1/imu  /relay1/joint_states
    nodes: /relay1/relay1_ackermann /relay1/relay1_imu /relay1/relay1_joint_state /relay1/relay1_lidar

Nothing collides with the chain robot's global `/cmd_vel` `/odom` `/scan` `/imu`
`/joint_states`. TF is shared but the frames are prefixed. **Not a namespace problem.**

## FIXED — steering was driving the car in circles

Root cause: the steering knuckles were authored at **0.005 kg, izz 1.8e-6** because gz's
`AckermannSteering` drives those joints **kinematically** — inertia never mattered. Under
a Classic *torque* PID it does. 0.6 N.m into 1.8e-6 kg.m2 is 333,000 rad/s2, so the joint
slammed into its +-0.6 stop at 8 rad/s (its own limit is 1.0) and steered the car in
circles with **nothing published on cmd_vel**.

| | before | after |
|---|---|---|
| steering knuckle | 0.005 kg, izz 1.8e-6 | 0.1 kg, izz 1e-4 |
| steering PID | `1500 0 1` (my error — that is the full-size Prius demo value) | `2.0 0 0.2` |
| idle steering velocity | -8.66 rad/s | **-0.0005 rad/s** |

Steering now responds correctly under command: `angular.z 0.4` gives 0.476 / 0.355 rad,
properly asymmetric Ackermann.

**Do NOT add `<damping>` to the steering joints.** Tried it; ODE emitted NaN for every
joint velocity and the model froze at the origin. The inertia fix is the correct one.

## Also fixed

- **Drive wheels free-ran at 101 rad/s on a fresh spawn.** The plugin has no target until
  it receives its first `cmd_vel`. `spawn_saye.sh` now publishes one latched zero.
- **Drive joints had no `<effort>` limit** (only lower/upper). Added `effort 20`,
  `velocity 100`.

## STILL BROKEN — drive is ~25x too weak

Commanded 0.3 m/s, achieves ~0.012 m/s. The effort limit did not fix it. Also a slow idle
creep remains (~8 cm in 20 s) that continuous zero `cmd_vel` does not brake, with `z` pinned
at 0.0365 so it is not a floor slope.

Next things to try, untested:
- `libgazebo_ros_ackermann_drive.so` drives wheels via ODE joint motors (`SetParam vel/fmax`);
  check whether Gazebo's `<implicit_spring_damper>` or the `<ode>` solver block interacts.
- Wheel `<surface><friction>` is `mu 0.5 / mu2 1.0` with `fdir1 0 0 1` — an fdir1 of
  **0 0 1** on a wheel is suspicious; it usually points along the roll axis.
- Compare against the `gazebo_ros_ackermann_drive` demo model to see what else it sets.

---

# nuc1 saw topics but received zero data — IPC namespace mismatch

Symptom: `nuc1` could list every safe_robot topic and node and `topic info -v` looked
correct, but `topic hz` on `/tf`, `/clock`, `/relay1/odom` returned nothing. relay1's Nav2
costmaps therefore never got TF and the stack never activated. `nuc2`, over the real Rajant
mesh, was completely fine.

Cause: `safe_robot` ran with `IpcMode=host`, `nuc1` with `IpcMode=private`.

    safe_robot  /dev/shm = 27:1   (host's)
    nuc1        /dev/shm = 110:1  (its own)

Both are `--network host`, so FastDDS classes them as same-host participants and selects
the **shared-memory** transport for data. Discovery runs over UDP and succeeds; SHM data
delivery silently fails because the segments live in different namespaces. nuc2 is a real
remote host, so it negotiates UDP and works.

Fix: recreate nuc1 with `--ipc host` (`./run_nuc1_container.sh`). Result:

    /tf                46.3 Hz   (was 0)
    relay1 TF frames   504 / 15 s (was 0)

**`ros2 topic list` is not a health check for this** — it passes while the path is dead.
Use `ros2 topic hz`.

---

# THE PORT BUG: `use_parent_model_frame` (found 2026-09-03)

**Symptom:** relay waypoints requiring any turn stalled ~0.45 m out and timed out at 300 s.
Pure-forward waypoints arrived fine in ~12 s.

**Cause:** every joint in the saye model carries
`<use_parent_model_frame>1</use_parent_model_frame>`.

- The **Harmonic original is SDF 1.8**. The tag was removed in SDF 1.7+, so modern
  sdformat **ignores it** — a steered wheel's spin axis correctly follows its steering link.
- The **Classic port is SDF 1.7**, and Gazebo Classic's sdformat9 still **honors it**.
  `front_*_wheel_joint`'s axis (`0 1 0`) gets pinned to the **model** frame and does NOT
  rotate with the steering link. The front wheels therefore always spin about the car's
  fixed Y axis no matter how far the steering deflects.

Inert in Harmonic, actively destructive in Classic. This is the thing that broke in the port.

**Measured, steering at full lock (cmd vx=0.3, wz=+0.562 → radius 0.533 m LEFT):**

| | turning radius | note |
|---|---|---|
| before fix | **−1.835 m** | wrong sign, 3.4x too wide; residual yaw was tyre scrub, not steering |
| after fix | **+0.682 m** | correct sign, near the 0.533 m geometric minimum (gap = slip + PID lag) |

**Fix:** `port_saye.py` strips every `<use_parent_model_frame>` tag.

## Waypoint results after the fix

| goal (map) | maneuver | result |
|---|---|---|
| (4.0, 0.0) | pure forward | **ARRIVED 11.7 s** |
| (4.0, −1.5) | forward + right, one arc | **ARRIVED 20.6 s** |
| (2.5, −1.0) | **pure lateral** | still times out, ~0.43 m short |

The pure-lateral case is a genuine near-worst-case, not a bug: an Ackermann car can only go
sideways by arcing forward-and-right, which overshoots x, then needs to reverse/loop back.
With a 0.7 m turning radius and MPPI's 2.8 s horizon (`time_steps 56 x model_dt 0.05`) it
cannot plan that parallel-parking maneuver. Avoid pure-lateral waypoints, or give the car
room to loop.

## Also fixed this session

- `AckermannConstraints.min_turning_r` was `0.2`; geometry allows only `0.533`, measured
  `0.682`. Now **0.70**. (Pre-existing in the Harmonic config too — not a port bug.)
- `GoalAngleCritic` was `enabled: true` with `threshold_to_consider: 0.5`, contradicting
  this same file's documented `yaw_goal_tolerance: 3.15` ("effectively any heading" for a
  car that cannot rotate in place). That 0.5 m matched the stall distance exactly. Now
  disabled. (Also pre-existing in Harmonic.)
- Killed a leftover `dds_bridge.py` from the abandoned two-hop bridge experiment (17% CPU).
- chain_robot's `path_planner_exploration_node` busy-polls a full core permanently.

**Both nav2 edits were made inside the `nuc1` container only** (`/root/colcon_ws/{src,install}/
.../nav2_params.yaml.in`) and are LOST if that container is recreated. They should be folded
into `mesh_sim_ws` proper — they apply to the Harmonic robots too.

---

# Second port regression: the lidar frame was in NO TF tree

**Symptom:** Smac refused to plan — `Starting point in lethal space! Cannot create feasible
plan`. The nav log was full of `Message Filter dropping message: frame
'relay1/base_link/gpu_lidar' ... timestamp earlier than all the data in the transform cache`.

**Cause:** the ray plugin stamped scans with `frame_name = ${NS}/base_link/gpu_lidar`, and
**that frame exists nowhere**. TF held only `relay1/odom` and `relay1/base_link`.

In Harmonic, `PosePublisher` had `publish_sensor_pose: true`, which emitted exactly that
sensor frame. This port dropped `PosePublisher` (correctly — it caused the A10 two-parent
bug) on the reasoning that `robot_state_publisher` would supply link frames. **But the
relays have no `robot_state_publisher`** (the saye package ships SDF, not URDF). So nothing
published the sensor frame.

Consequence: every scan unresolvable -> costmap received no obstacles AND never cleared ->
robot appeared to stand in lethal/unknown space -> Smac refused to plan.

**Fix:** `frame_name` is now `${NS}/base_link`. The lidar pose is `(0, 0, 0.21)` — a pure Z
offset — so x/y is exactly right, which is all a 2D costmap uses. Scan drops: constant -> **0**.

# Planner: the Reeds-Shepp planner was configured but never invoked

`planner_plugins: ["GridBased", "SmacPlannerHybrid"]`. Nav2's stock navigate_to_pose BT
hardcodes `planner_id="GridBased"`, so **only NavFn ever ran** — a holonomic grid planner
that emits straight sideways paths an Ackermann car cannot follow. The Smac Hybrid config
below it (`REEDS_SHEPP`, `reverse_penalty: 0.3`, author's comment "allow reversing when
turning around") was dead config. IDs swapped so `GridBased` -> SmacPlannerHybrid, NavFn
kept as `NavFn`. Its `minimum_turning_radius` had the same 0.20 error; now 0.70.

# Waypoint status after all fixes

| goal (map) | maneuver | result |
|---|---|---|
| (4.0, 0.0) | pure forward | **ARRIVED 11.7 s** |
| (4.0, −1.5) | forward + right, one arc | **ARRIVED 20.6 s** |
| (2.5, −1.0) | pure lateral | still ~0.5 m short after 300 s |

All four physical primitives verified working: forward, reverse (−0.303 m/s commanded
−0.3), forward+turn (radius 0.682 m), reverse+turn (radius 0.420 m). The car can execute a
K-turn; MPPI still will not close a pure-sideways 1 m goal within the 0.25 m tolerance.
Reverse is NOT broken — that hypothesis was tested and disproved.

**Recommendation:** avoid pure-lateral waypoints for this vehicle, or raise
`xy_goal_tolerance` for them. Everything else now works.

---

# Waypoint accuracy: it is a PRECISION limit, not a direction limit

`send_waypoints.py` declares ARRIVED at **< 0.30 m** (line 145/168). Every result clusters
around that line:

| goal (map) | distance | final error | verdict |
|---|---|---|---|
| (4.0, 0.0) forward | 1.5 m | **0.298** | ARRIVED 11.7 s |
| (4.0, −1.5) arc | 2.1 m | **0.299** | ARRIVED 20.6 s |
| (2.5, −2.0) lateral | 2.0 m | **0.297** | ARRIVED 89.9 s |
| (3.21, −0.71) diagonal | 1.0 m | 0.371 | timeout |
| (2.5, −1.0) lateral | 1.0 m | 0.405–0.497 | timeout |

The three "successes" land at 0.297/0.298/0.299 — barely under threshold. So this is **not**
direction-dependent: the vehicle has a **consistent ~0.3–0.5 m steady-state positioning
error on every goal**. Short goals fail more often simply because there is less run-up to
damp the final oscillation before it settles.

Note the 2 m lateral goal SUCCEEDED where the 1 m lateral failed — more lateral room lets
the car arc instead of parallel-park. It visibly reversed and executed a real K-turn
(path went 2.32 → 1.96 backwards, then swung to −2.70 and back to −1.72), confirming the
Reeds-Shepp planner + relaxed PreferForwardCritic are working.

**Practical read:** for the relay's actual job — trailing the head robot to hold a mesh
link — 0.5 m accuracy is almost certainly fine. If tighter accuracy is genuinely needed,
that is a separate fine-positioning tuning problem (MPPI near-goal behaviour, minimum
practical speed, velocity smoother deadband), not a port bug.

---

# Accuracy: 30 cm -> under 10 cm, and 1 m lateral now arrives

## What actually capped accuracy

Three hard config floors, none of which any amount of controller tuning could beat:

| | was | now | why it capped accuracy |
|---|---|---|---|
| `xy_goal_tolerance` | 0.25 | **0.08** | controller declared success and stopped 25 cm out |
| Smac planner `tolerance` | 0.25 | **0.05** | the PLAN itself was allowed to end 25 cm short |
| `GoalCritic.cost_weight` | 5.0 | **24.0** | goal term was swamped by path-following critics on final approach |

## The drive PID could not fine-position

`linear_velocity_pid_gain` was `10 0 1` with `linear_velocity_i_range: 0 0` — P-only with
integral action fully disabled. A P-only loop cannot break static friction at small velocity
error, giving a hard low-speed deadband. Measured:

    cmd 0.05 m/s -> -0.141 m in 12 s   (wrong direction)
    cmd 0.10 m/s -> +0.007 m in 12 s   (stalled)
    cmd 0.15 m/s -> +0.206 m in 12 s

Near a goal MPPI commands exactly those small velocities, so the car physically could not
fine-position. Now `100 50 1` with `i_range -20 20`:

    cmd 0.10 m/s -> +0.071 m   (10x better)
    cmd 0.15 m/s -> +0.300 m

`150 250 1` was tried and is WORSE — integral windup drove the car backward at every
command. Do not raise I past ~50.

## send_waypoints.py was hiding the real accuracy

It broke out of its wait loop at `last_d < 0.30` and reported that first sample as final.
Every "0.297 / 0.298 / 0.299" result was this artifact, not the controller's capability.
Measured properly, the approach converges:

    err 1.008 -> 0.589 -> 0.051 -> 0.114 (overshoot) -> 0.087 -> 0.064 -> 0.040 -> 0.021

Threshold now 0.08 to match `xy_goal_tolerance`. Timeout also raised 300 s -> 900 s wall:
the sim runs at RTF ~0.27, so 300 s wall was only ~80 s of sim time — enough for a straight
run but not a multi-point Ackermann maneuver, which reported a false TIMEOUT while the robot
was still legitimately converging.

## Verified results (8 cm threshold)

| goal (map) | maneuver | result |
|---|---|---|
| (3.207, −0.707) | 1 m diagonal | **ARRIVED 0.079 m in 9.0 s** |
| (2.5, −1.0) | **1 m lateral** | **ARRIVED 0.080 m in 877 s** |
| direct action, same lateral goal | | settled to **0.021 m** |

The 1 m lateral — which previously never arrived in any configuration — now completes,
executing the reversing K-turn ("egging") it needs. It is slow (877 s wall ≈ 240 s sim)
because a 1 m pure-sideways shift genuinely requires multi-point maneuvering at a 0.68 m
turning radius.

---

# Ground-truth occupancy grid (`tools/gt_occupancy.py`)

True GT: rasterizes the same **collision** geometry the physics engine uses, not SLAM
output. Run inside `safe_robot` (needs numpy + trimesh + pycollada; the world at `/hw`).

    docker cp tools/gt_occupancy.py safe_robot:/tmp/
    docker exec safe_robot bash -lc 'cd /tmp && python3 gt_occupancy.py \
      --world /hw/hospital_chain.world --models /hw/models \
      --out /tmp/hospital_gt --res 0.05 --zlo 0.05 --zhi 0.60'
    docker cp safe_robot:/tmp/hospital_gt.npy gt/

Output in `gt/`: `.npy` (uint8, 1 = occupied, 0 = free), `.json` (resolution + origin, so
cells map to world coords), `.png` (visual check).

    grid   1181 x 543  =  27.2 x 59.1 m at 0.05 m/cell
    occupied 9.4%
    world_x = origin_xy[0] + (col+0.5)*res
    world_y = origin_xy[1] + (row+0.5)*res

Floor and ceiling are excluded via `--skip` (either would fill the whole grid).

## Two traps that produced convincing but WRONG grids

1. **Missing `pycollada` silently dropped every `.dae` collision mesh** — 37 of them,
   including the building walls. The result was furniture floating in empty space and it
   looked plausible at a glance. The script now **aborts** on any mesh load failure rather
   than skipping silently.
2. **COLLADA `<unit meter="0.01">`** — 34 of the collision `.dae` files are in centimetres.
   Gazebo honours the unit; **trimesh does not**. Without applying it those meshes load 100x
   oversized: the walls alone spanned 5.6 km and the grid came out 2514 x 5614 m for a
   25 x 51 m building. Now parsed and applied.

## Validation

    relay1 spawn (2.5, 12.0)  -> free      (correct)
    chain_robot spawn (0, 12) -> free      (correct)
    horizontal cut at y=12.0  -> walls at x -12.57 .. 12.53 m
                                 (building extent from model poses: -12.4 .. 12.4)

NOTE: `trimesh` and `pycollada` were pip-installed into the running `safe_robot` container
and are **lost if it is recreated**. Reinstall with
`pip install trimesh pycollada` before re-running.

---

# 2026-10-27/28 — live exploration fixes (sim side)

**Relay lidar ray fan made the GUI lag.** `<visualize>true</visualize>` on the relay lidar had
gzclient draw every ray. `port_saye.py` now sets it false (sensor unaffected).

**RTF.** Measured: gzserver physics thread ~67 %, sensor thread ~40 % (chain_robot's MID-360
casting 12 000 rays per scan against the hospital meshes). The MID-360's `<downsample>` was
raised 2 -> 4 **inside the safe_robot container**
(`/root/.gazebo/models/chain_robot/model.sdf`, original kept as `model.sdf.orig`; lost if the
container is recreated). Sensor thread -> ~7 %. RTF 0.36 without the relays, ~0.2 with them;
the rest is physics (100 ODE iterations, 1 ms step, 24 track wheels in contact) -- solver
iterations 100 -> 50 would be the next lever, not applied.

**chain_robot drove into the relays.** Gazebo lidars hit COLLISION shapes only, and the saye
model (upstream too) has a 0.30 x 0.09 x 0.12 m spine box as its body collision, so the relays
were nearly invisible to chain_robot's lidar. `port_saye.py` now gives the body a
0.32 x 0.23 x 0.18 m box (bottom at the axle; wheels still carry the floor contact). Relays
now park off the road graph, west of the GCS (`rssi/ros/nodes.yaml` `park`), and the planner
keeps robots 0.8 m apart. Live run 3: closest approach 0.82 m (before: relay1 pushed ~4 m).

**/map silently stopped arriving.** occ_grid_mapping's default grid is 1200 x 1300 cells =
1.5 MB per message. That does not fit FastDDS's 512 KB shared-memory segment, and over UDP the
host's default socket buffers (`net.core.rmem_max` 212992, changing it needs sudo) drop the
fragments: the frontier detector, the explorer ("No map received yet" -> "No frontiers" -> its
`is_goal_occupied()` crashes on a None map) and the coordinator never got it. Fix:
`hospital_mapping.launch.xml` (copy of the package launch) + `occ_grid_hospital.yaml`, a
32 x 63 m grid sized to the hospital = 197 KB. `run_chain_stack.sh` uses it.
(An 8 MB SHM segment profile for the mapper worked briefly but not reliably: some participants
log `RTPS_TRANSPORT_SHM ... open_and_lock_file failed` and fall back to UDP.)

**Nav2 shuts itself down at low RTF.** A node that misses the lifecycle manager's 4 s bond
heartbeat (e.g. during a CPU spike) makes it declare CRITICAL FAILURE and deactivate the whole
stack; once its manager hung mid-reset for 30+ min and only relaunching the Nav2 stage helped.
`rssi/ros/run_full_live.sh` checks every Nav2 is ACTIVE after start-up (STARTUP, then
RESET + STARTUP), and the exploration coordinator does the same whenever a goal is rejected.

**Partial restarts leave a mess.** Relaunching only the mapper left its child
`pointcloud_to_laserscan` running -> two same-named nodes and a silent `/scan`; killed nodes
pile up as zombies and FastDDS SHM files. When things get strange: `stop_all.sh`,
`docker restart safe_robot nuc1`, `fastdds shm clean`, then `run_full_live.sh`.
Also: `pkill -f <pattern>` inside `bash -c` matches its own shell -- use `[p]attern`.

**chain_robot wedges against walls.** Its own controller brought it to 0.10 m from a wall in the
north-east room; with Nav2's 0.5 m global inflation the start pose is in an obstacle, so no plan
and no backup succeed. Still open (its stack, unmodified).
