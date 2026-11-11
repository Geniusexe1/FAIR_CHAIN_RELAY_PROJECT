# Porting the relay code from the simulator to the real robots

This guide is for an agent (or person) who has to make the relay coordination code in this folder run on real
hardware. It assumes you know ROS 2 and Nav2 but have never seen this project.

Two things about the real system differ from the simulator, and the whole guide is organised around them:

1. **The real relays are not Ackermann cars.** The simulated relays are (0.70 m turning radius), and several
   settings exist only because of that.
2. **The chain robot's navigation code will come from another lab member.** In the simulator the chain robot
   runs stock Nav2 with a frontier explorer. The relay code must not depend on how that stack works inside,
   only on a small interface (section 4).

The code here is the version that produced the best simulated run, **live14: 99.7 % of the floor explored in
96 minutes with two relays at P0 = −57 dBm**. `MANIFEST.md` lists where each file came from and its checksum.

## 1. Rules

- **The chain robot's navigation belongs to its owner.** Do not edit it. If the relay code needs something from
  it, write the need down and hand it to the owner. Report chain problems; do not fix them.
- **Secrets never go in the repository.** The Rajant admin password is read from the environment variable
  `RAJANT_ADMIN_PASSWORD`. Host secrets live in `~/.fair_chain.env`, which git ignores.
- **Exploration goals and relay positions come only from the map the robots have explored.** The prior map of
  the building may be used for the radio model and for nothing else. The coordinator already works this way;
  keep it so.
- **Ask the user before changing the radio threshold or P0.** They chose P0 = −57 on purpose for the simulator
  (the weakest value at which both relays are still needed).
- Section 9 lists facts this guide does not have. Ask the user for them. Do not guess them.

## 2. What the system does

A chain robot (the "client", named `head`) explores a building. A ground control station (GCS) must stay
connected to it by radio. Two relay robots carry radios and are placed so that a route GCS → relay(s) → client
always has enough signal.

The method is from Seo et al., RA-L 2025, Section IV-B. One node, the **coordinator**, sits between whatever
picks exploration goals and the robots' navigation:

```
goal source ──goal──▶ coordinator ──waypoints, speed limits──▶ client navigation
(frontier explorer)       │      └─────waypoints, speed limits──▶ relay navigation (one per relay)
                          │
        explored map ─────┤   (where robots may go)
        prior radio grid ─┤   (how signal travels)
        robot poses ──────┘
```

For each goal the coordinator decides one of three things:

| Decision | Meaning |
|---|---|
| `CLIENT_ONLY` | The signal holds with the relays where they are. Only the client moves. |
| `DEPLOY` | Relays must move. The fewest relays needed are placed, and a trajectory is planned for every robot. |
| `SKIP` | The goal cannot be reached with signal even with every relay placed. The goal is rejected. |

A plan has `t_f = 5` intervals. In each interval every moving robot gets one waypoint, and nobody starts
interval *t+1* until everyone has reached waypoint *t*. That gate is what keeps the chain connected. Speed
limits are also adjusted every 0.5 s so the robots arrive together, but the gate works without them.

The signal between two points is predicted by a model, not measured:

```
P = P0 − 10·n·log10(dE) − alpha·log10(dbar + 1)        [dBm]
```

`dE` is the straight-line distance and `dbar` is how much longer the shortest path around walls is. A link is
good enough when the predicted value is at least **−81.68 dBm** (−85 dBm threshold plus a 3.3 dB margin for
noise). Section 6.4 covers what changes with real radios.

## 3. What is in this folder

Paths mirror the simulator repository, because the code finds its modules and maps by relative path. Keep the
layout. Every file is an unmodified copy; the changes this guide asks for are yours to make.

### Use these (the relay system)

| File | What it is | Port status |
|---|---|---|
| `rssi/relay_planner.py` | The planner: relay placement, assignment, trajectories, routing | Use as is |
| `rssi/propagation.py`, `rssi/network.py` | Signal model and noise model | Use as is; calibrate parameters (6.4) |
| `rssi/roadgraph.py` | Roads through free space that plans follow | Use as is; check clearance (5.3) |
| `rssi/ros/exploration_coordinator.py` | The coordinator ROS node | Adapt: settings in 5 and 6, clock in 6.3 |
| `rssi/ros/nodes_real_env.yaml` | GCS position, relay names, parking spots, pose topics | Rewrite for the real robots (6.1) |
| `rssi/ros/run_coordinator.sh`, `rssi/ros/Dockerfile` | How the coordinator is started in its container | Adapt: clock, mounts |
| `hospital_world/real_env/gt/real_env_radio.{npy,json}` | Prior grid for the radio model: walls only | Use if the frame matches (6.2) |
| `hospital_world/real_env/gt/real_env_gt.{npy,json,png}` | Prior grid with desks; used to score coverage | Scoring only |
| `hospital_world/tools/real_env_layout.py` | How both grids were made from the FAST-LIO scan | Reference for making a grid of another site |
| `hospital_world/mesh_peers.xml` | DDS discovery without multicast | Use; update addresses (6.5) |
| `hospital_world/tf_relay.py` | Gives each relay its own TF topic | Use if relays run Nav2 across the mesh (6.5) |
| `hospital_world/map_throttle.py` | Republishes the map every 4 s as `/map_mesh` | Use (6.5) |
| `hospital_world/relay_hosts.conf` | Which computer runs which relay | Rewrite |
| `rajant_mesh/` | ROS 2 package that reads the real Rajant radios and publishes `mesh_matrix` and `weakest_link` | Use; see 6.4 |
| `rajant_tools/` | Two command-line readers for the same radios, no ROS: `rajant_link_quality.py` (every link) and `rajant_status.py` (radio health) | Use for checks and calibration; see 6.4 |

### Use these to check your work

| File | What it is |
|---|---|
| `smoke_test.py`, `smoke_test_expected.txt`, `test_data/` | Runs the planner offline on a saved map. No ROS needed. |
| `rssi/explore_sim.py` | Offline exploration simulator around the same planner. Use it to try settings on a new map. |
| `rssi/map_coverage.py` | Scores how much of the prior grid a saved map covers |
| `rssi/ros/rssi_monitor.py`, `rssi/ros/run_rssi_node.sh`, `rssi/margin.py` | Publishes the *predicted* signal of every link from live poses. On real robots it is the prediction to compare against measurements. |
| `reference/live14/coordinator.log` | The coordinator's log from the best simulated run. It shows what a healthy run looks like. |

### Read these, then replace them

| File | Why it is here |
|---|---|
| `reference/ackermann_relay/run_relay.sh` | How a simulated relay's Nav2 is started: namespace, TF, map topic, frame alignment |
| `reference/ackermann_relay/tuned_tol25.yaml.in` | The simulated relay's Nav2 parameters. The Ackermann parts are listed in 5.1. |
| `reference/chain_explorer/run_exploration.sh` | How the simulator's explorer is wired to the coordinator. The new chain stack must provide the same wiring (4.1). |
| `reference/sim_run/run_full_live.sh`, `overnight.sh` | The exact start order and arguments of the simulated runs |
| `reference/rssi_README.md` | Model parameters and how P0 was measured |
| `reference/sim_relay_CHECKPOINT.md` | Notes on the simulated relay: drive deadband, creep while parked |

### Left out on purpose (simulator only)

These exist in the simulator repository and have no use on real robots: the Gazebo world and spawn scripts
(`real_env.world`, `spawn_saye.sh`, `port_saye.py`), `parking_brake.py` (stops simulated cars creeping),
`contact_log.py` (Gazebo contacts), `record_gazebo.sh`, `gz_window.py`, `live_logger.sh` (Gazebo ground-truth
poses), `replay_live_run.py` (needs that ground truth), `state_strip.py`, and `run_chain_stack.sh` (the
simulated chain stack, which the other lab member's code replaces).

## 4. The interface the coordinator needs

This section is the contract. Everything the coordinator touches is listed here. If the real system provides
these, the coordinator runs; nothing else about the robots matters to it.

All positions the coordinator works in are in one flat frame it calls **world**. Each input is turned into
world coordinates by adding a constant offset. There is **no rotation** anywhere, see 6.2.

### 4.1 From the chain robot's side

| What | Name in the simulator | Type | Notes |
|---|---|---|---|
| Goals | coordinator serves `/coord/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | The goal source sends its goals here instead of to the robot. Position only; frame = the map frame. |
| Map | `--map-topic`, default `/map_mesh` | `nav_msgs/OccupancyGrid` | QoS reliable + transient local. Free is 0–49, unknown is negative, the rest is occupied. |
| Client pose | topic in the nodes file, `/Odometry` | `nav_msgs/Odometry` | Position and yaw are read. World = pose + `offset`. |
| Client navigation | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Root namespace. Goals are in frame `map`, world − `--map-offset`. See below for feedback. |
| Client speed limit | `/speed_limit` | `nav2_msgs/SpeedLimit` | Absolute m/s, `percentage = false`. Optional but wanted. |
| Client navigation health | `/bt_navigator/get_state`, `/lifecycle_manager_navigation/manage_nodes` | lifecycle services | Used only after a goal is refused, to restart Nav2. |
| Client un-stick | `/cmd_vel`, `/backup` | `Twist`, `nav2_msgs/action/BackUp` | The coordinator drives the chain itself when it is stuck. **Turn this off**, see below. |

What the goal source must do:

- Send a goal, and when it is **rejected**, send a different one. Rejection is how the coordinator says "not
  that one" (already visited, no signal there, not connected to the explored floor, no room).
- When a goal finishes (succeeded or aborted), send the next one. "Succeeded" means the plan finished. The
  client may have stopped short of the goal on purpose because the signal would not hold further.
- If nothing is accepted for 60 s (`--own-goal-after`), the coordinator accepts the next request anyway and
  substitutes a frontier of its own from the map ("OWN GOAL" in the log). It can only do this when a request
  arrives, so the goal source must keep asking.
- The coordinator publishes `/coord/map`, the map cropped to the explored area. The simulator's explorer reads
  that instead of the full map. A new goal source may read either.

What the client's navigation must do:

- Accept a new goal while one is running, and accept cancel.
- Report `distance_remaining` in the action feedback. The coordinator uses it to match speeds and to notice a
  client that moves without getting closer (`--progress-s 30`). If the field stays 0 the speed matching and
  that check are simply inactive; the stuck check on the pose still works.
- Finish with status SUCCEEDED or ABORTED.

Settings to change because the chain navigation is someone else's:

- **`--stuck-tries 0`.** With the default 6, a client that has not moved for 40 s is driven directly by the
  coordinator on `/cmd_vel` (a forward arc or straight back) or sent a Nav2 `BackUp`. That bypasses the chain
  owner's planner and its collision checks. With 0 the coordinator only gives up the plan and counts the goal
  as failed. Recovery is then the chain stack's job.
- **`--v-client`** must equal the chain's real cruising speed. Plans and time limits are computed from it.
- **`--v-min`** (default 0.22) is the lowest speed limit ever sent to any robot. It was set for the simulated
  chain's controller, which cannot drive below 0.2 m/s. Set it to what the real chain can do.
- If the chain stack has no `bt_navigator` and lifecycle manager, a refused goal costs about 100 s of waiting
  before the coordinator gives up on it (`_revive`). Shorten `_revive` or skip it for the client in that case.
- `--goal-clear 0.6` and `--goal-snap 1.0` move a goal that is too close to obstacles for the chain's body.
  Ask the chain owner what clearance their planner needs.

Write these needs down for the chain owner early. The one most likely to be missing is the
`/coord/navigate_to_pose` wiring: their goal source must be able to send goals to a different action name and
cope with rejections.

### 4.2 From each relay

A relay named `relay1` in the nodes file must provide all of the following under `/relay1/`.

| What | Name | Type | Notes |
|---|---|---|---|
| Pose | topic in the nodes file, `/relay1/odom` | `nav_msgs/Odometry` | Must be a pose in the shared frame, not raw wheel odometry. See 6.2. |
| Navigation | `/relay1/navigate_to_pose` | `NavigateToPose` | Goals in frame `map`. Feedback `distance_remaining` is used to tell when a waypoint is passed. |
| Speed limit | `/relay1/speed_limit` | `nav2_msgs/SpeedLimit` | Absolute m/s |
| Health | `/relay1/bt_navigator/get_state`, `/relay1/lifecycle_manager_navigation/manage_nodes` | lifecycle services | Restart after a refused goal |
| Nudge | `/relay1/cmd_vel` | `Twist` | `_free_relay`: a straight move of at most 0.5 m. See 5.2. |

If the real relays run Nav2, all of this exists once Nav2 is started in the relay's namespace. If they run
something else, the cheapest path is a thin adapter node per relay that offers these names.

### 4.3 What the coordinator publishes

| Topic | Content |
|---|---|
| `/coord/chain` | JSON list, the current best route, for example `["gcs", "relay1", "head"]` |
| `/coord/state` | JSON events: decisions, intervals, aborts, relay state changes |
| `/coord/plan_markers` | Planned trajectories for RViz |
| `/coord/map` | The cropped map |
| `--state-log FILE` | One JSON line per second: route, each relay's state and position |
| `--map-log DIR` | A map snapshot every 5 s (`.npz`), the input of `smoke_test.py` and `map_coverage.py` |

Relay states in the log: `PARKED`, `DEPLOYING` (moving in a plan), `RELAYING` (standing on the route),
`RETURNING` (unused for 20 s, going back to park), `HOLDING`.

## 5. What changes because the relays are not Ackermann

Find out first what the real relays are (differential, omnidirectional, tracked), their footprint, their top
and lowest steady speed, and whether they can turn on the spot. Then go through this section.

### 5.1 The relay's own navigation

`reference/ackermann_relay/tuned_tol25.yaml.in` is tied to the car in these places. Do not carry them over.

| Parameter | Simulated value | Why it is Ackermann-specific |
|---|---|---|
| MPPI `motion_model` | `"Ackermann"` | Use `"DiffDrive"` or `"Omni"` |
| MPPI `min_turning_r` | 0.70 | Only exists for Ackermann |
| Planner | `SmacPlannerHybrid`, `REEDS_SHEPP` | Needed for a car. A robot that turns on the spot can use a plain grid planner. |
| `footprint` | 0.30 × 0.09 m rectangle | Use the real footprint |
| `xy_goal_tolerance` | 0.25 | Was raised because the car could not finish on a point. Keep ≤ `--relay-arrive`. |

What to keep from `run_relay.sh`, whatever the navigation stack: the relay runs in its own namespace, reads the
shared map from `/map_mesh` (not `/map`), reads TF from its own `/<ns>/tf`, and its `map` frame is the same
frame as the chain's map (6.2).

### 5.2 Coordinator settings that exist because of the car

Each setting below was added after a specific failure of the simulated car. For a relay that can turn on the
spot most can be relaxed. Change one at a time and re-run.

| Setting | Default | Why it exists | For a non-Ackermann relay |
|---|---|---|---|
| `--relay-lookahead` | 1.5 m | Plan waypoints are 0.2–0.9 m apart. The car could not stop at each, so it aims further ahead and drives through. | Harmless to keep. May be lowered if the relay cuts corners. |
| `--relay-min-move` | 0.6 m | The car shuffled for minutes at moves shorter than its turning circle. Shorter moves are dropped when the link holds anyway. | Lower to about the goal tolerance. |
| `--relay-arrive` | 0.6 m | Counts as "at the waypoint" | Lower with a tighter goal tolerance, never below it. |
| `--keep-tol`, `--park-tol` | 1.0 m, 0.6 m | Station keeping: a relay that drifts this far from its spot is sent back. Simulated cars creep while parked. | Real robots should not creep. Keep the feature, it also covers a relay that was pushed. |
| `--v-floor-relay` | 0.25 m/s | The simulated drive stalls at 0.10 m/s. A relay never gets a speed limit below this; it starts later instead. | Set to the lowest speed the real relay holds steadily. |
| `--v-relay` | 0.5 m/s | Top speed used for scheduling | Set to the real value. |
| `--relay-retries` | 2 | The car's Nav2 often aborted while manoeuvring; the waypoint is sent again. | Keep. |
| `--interval-slack` | 90 s | Time added per interval because the car needs long to plan and turn before it moves | Can be much lower; it bounds how long a failed interval takes. |
| Goal heading (`launch()` in `_execute_plan`) | along the direction of travel | A heading of 0 made the car plan a loop to arrive facing east. | Irrelevant if the goal checker ignores yaw. Otherwise keep. |
| `park_yaw` in the nodes file | −1.5708 | The car parks nose-out so it can leave without reversing. | Optional. |
| `_free_relay` | straight nudge ≤ 0.5 m along its heading | A relay stopped against a door jamb sits inside its own costmap's lethal band and every plan fails. A car can only move along its heading. | Keep the idea. An omnidirectional relay can move straight to the roomiest spot. The nudge skips collision checking, so on real hardware either keep it short and slow or route it through the relay's safety layer. |

### 5.3 Sizes

These depend on the robots' bodies, not on Ackermann steering, but they were set for the simulated robots
(the planner takes a body as 0.35 m across) and must be re-derived.

| Setting | Default | Meaning |
|---|---|---|
| `--d-sep` | 0.5 m | Planned minimum distance between two robots at any waypoint |
| `--guard` | 0.38 m | The plan is stopped if the client comes this close to a *moving* relay |
| `--yield-r` | 0.8 m | A lower-priority robot closing within this distance of another stops and lets it pass. Priority: client, then relays by name. |
| `--pass-clear` | 0.6 m | A standing robot does not block a corridor at least twice this wide |
| `RoadGraph.build(..., clearance=0.45)` | 0.45 m | Roads keep this far from obstacles. One road graph serves all robots. |

One problem is still open in the simulator: in a 1.46 m wide corridor the chain passes very close to a parked
relay (0.23 m once in live14). Relays do not step aside. Wider real relays make this worse. If the real
corridor is that narrow, plan for it: raise `--pass-clear` so the planner treats a parked relay as blocking.

## 6. What changes because it is real

### 6.1 The nodes file

Order matters: the **first** entry is the GCS, the **last** is the client, everything between is a relay. The
relay names are also their ROS namespaces.

```yaml
gcs:
  xy: [5.5, 7.7]            # where the GCS radio stands, world frame, measured on site

relay1:
  topic: /relay1/odom       # nav_msgs/Odometry, a pose in the shared frame
  offset: [0.0, 0.0]        # added to that pose to get world
  park: [4.0, 7.4]          # where it waits when unused, world frame
  park_yaw: -1.5708         # optional

relay2:
  topic: /relay2/odom
  offset: [0.0, 0.0]
  park: [4.9, 7.4]

head:
  topic: /Odometry          # the chain's pose
  offset: [4.4, 5.0]        # the chain's start position in world; also where it regroups
```

Parking spots must be on floor the chain maps early (the planner only sends relays through explored floor) and
close enough to the GCS that the link holds from there.

### 6.2 One frame for everybody

This is the largest gap between the simulator and reality.

In the simulator the relay's odometry is the true pose from Gazebo, and its Nav2 gets a fixed
`map → relay/odom` transform (`static_map_odom:=true`). The chain's pose is FAST-LIO odometry plus its known
spawn position. So everything agrees for free. On real robots it does not.

What must hold:

1. **The chain's map frame is the reference.** Goals to every robot are in frame `map` at
   world − `--map-offset`. Each relay's navigation must therefore use a `map` frame that is the same physical
   frame as the chain's map.
2. **Each relay must know its pose in that frame** and publish it as Odometry. Options, roughly from most to
   least robust: localise each relay against the chain's shared map with its own LiDAR (for example AMCL on
   `/map_mesh`); start each relay from a surveyed parking pose and dead-reckon (drifts over a long run); run
   SLAM per relay and align once at the start. Ask the user which sensors the relays have before choosing.
3. **World and map differ by a translation only.** `--map-offset X Y` is added to map coordinates; there is no
   rotation term in the coordinator. The prior grids are in the frame of the original FAST-LIO scan, turned by
   −0.8° so the walls lie on the axes, with the origin where the robot started that scan. In the simulator the
   chain starts at world (4.4, 5.0) facing +x. If the real chain starts somewhere else, change the offset. If
   it starts with a **different heading**, the map is rotated against the prior grid and the radio model is
   wrong everywhere. Then either start the chain at the surveyed pose and heading, or add a rotation where the
   coordinator converts between map and world (`_on_goal`, `_pose`, `_odom`, `_known_free`).

Check for this step: with all robots standing still at known spots, the coordinator's log line
`relayN: start -> PARKED at (x, y)` and `/coord/chain` must show positions that match a tape measure.

A wrong pose is not caught anywhere. The coordinator plans from whatever poses it receives.

### 6.3 The clock

`run_coordinator.sh` starts the node with `use_sim_time:=true`. On real robots there is no `/clock` topic, the
node's time stays at zero, no timer fires, and the coordinator sits silent with no error. **Remove that
parameter.** Every duration the help text calls "sim s" then means ordinary seconds. The time-outs were tuned
at a simulator speed of about 0.3 × real time, so in real time they are generous; leave them until you have
real runs to judge by.

### 6.4 The radio

Three separate things are simulated today.

**Model parameters.** `--P0`, `--n`, `--alpha`. P0 was measured once on the real radios: −60.8 dBm at 1 m
(Rajant Cardinal pair, 5 GHz channel 161; details in `reference/rssi_README.md`). `n = 2.0` and `alpha = 35`
are the paper's values and have **not** been fitted to this building. The simulated runs used P0 = −57. Before
trusting relay placement on site, measure the signal at a spread of known positions, including around corners
and through walls, and fit the three parameters.

**The prior grid.** The radio model needs to know where the walls are before they are explored
(`--radio-gt`). `real_env_radio` is the walls of the real site, without desks, because signal passes over a
desk that blocks a robot. If the deployment is in the same building and the frame matches (6.2), use it. For
another site you need a floor plan or an earlier scan turned into the same grid format;
`real_env_layout.py` shows how this one was made. `--gt` is only needed when `--radio-gt` is not given.

**Measurements.** In the simulator `rssi_monitor.py` *invents* measurements from the model plus noise. On real
robots `rajant_mesh/mesh_matrix.py` reads the radios instead. Know these facts about it:

- It needs the proprietary `rajant_api` Python library, which is not in the repository (obtain it from
  Rajant), and the `RAJANT_ADMIN_PASSWORD` environment variable.
- The radio addresses are a list at the top of the file (`RADIOS`). Update it.
- It publishes on `mesh_matrix` a JSON matrix keyed by radio **serial number**, and the values are **SNR**
  (`signal − noise`, in dB), not signal strength. The model and the −85 dBm threshold are in dBm. The radio
  API gives both `signal` and `noise` per peer, so publishing `signal` as well is a small change.
- You need a table from radio serial number to robot name.

The same package has a second node, `rajant_mesh/weakest_link.py`. Each cycle it publishes on `weakest_link`
one line of text for the weakest link only: the two radios, signal (dBm), noise (dBm), SNR, routing cost and
rate. `launch/mesh_launch.py` starts both nodes and takes a `namespace` argument (default: the `MESH_NS`
environment variable), so each computer can publish under its own prefix.

No node publishes the signal strength in dBm of **every** link. `mesh_matrix` has every link but as SNR;
`weakest_link` has dBm but for one link. To compare with the model you need dBm for every link, so add
`signal` to the matrix (or publish a second matrix) before step 10 of section 7.

`rajant_tools/` holds two scripts that talk to the radios without ROS. They read the password from
`RAJANT_PW` (not `RAJANT_ADMIN_PASSWORD`) and the role from `RAJANT_ROLE`, and each has its own radio address
list to update.

- `rajant_link_quality.py` prints signal, noise, SNR, cost and rate for every link in both directions;
  `--watch 5` refreshes every 5 s. Use it to take the calibration measurements described above.
- `rajant_status.py` prints each radio's version, uptime, temperature, dropped-flood counters and peers. Use
  it to check that the mesh is healthy (6.5).

**Nothing in the coordinator reads measurements yet.** It plans from the model only. The paper corrects the
model online with a Kalman filter and Gaussian process regression; that part is not implemented. A sensible
first step on real hardware is passive: log `mesh_matrix` next to `/rssi/links` (the prediction from
`rssi_monitor.py --no-noise`) during a run and compare. Only close the loop after the user agrees on how.

### 6.5 The mesh network

The Rajant radios collapse under multicast and under chatty DDS traffic. A few multicast packets per second
gave 50–80 % packet loss on the whole mesh; without them it was 0 %. All three measures below are needed
whenever ROS traffic crosses the radios:

- **No multicast discovery.** Every participant uses `mesh_peers.xml`
  (`FASTRTPS_DEFAULT_PROFILES_FILE`, `ROS_LOCALHOST_ONLY=0`, `ROS_DOMAIN_ID=42`). The file lists every host's
  address and participant ports. Update the addresses for the real computers.
- **No global `/tf` across the mesh.** Each relay runs `tf_relay.py <name>` and its navigation reads
  `/<name>/tf` and `/<name>/tf_static`.
- **The map at a rate the radios can carry.** `map_throttle.py` runs on the chain's computer and republishes
  `/map` as `/map_mesh` every 4 s. The coordinator and the relays read `/map_mesh`.

On real robots each relay's navigation will most likely run on the relay's own computer, which removes the
scan and odometry traffic the simulator had to push across the mesh. The coordinator's traffic (goals,
feedback, speed limits, poses, the map) is small. Docker containers need `--network host --ipc host`.

## 7. Order of work

Do the steps in order. Do not start a step until the check of the one before passes.

| # | Step | Check |
|---|---|---|
| 1 | Install the Python dependencies (`rssi/ros/Dockerfile` has the versions) and run `python3 smoke_test.py` in this folder. | Prints `PASS`. |
| 2 | Collect the facts in section 9 from the user and the chain owner. | Written down. |
| 3 | Write the nodes file (6.1) and fix the clock (6.3). | Coordinator starts and logs `P0 … \| req -81.68 dBm \| client head, relays [...]`. |
| 4 | Bring up one relay's navigation alone, in its namespace, on the shared map (5.1, 6.2). | `ros2 action send_goal /relay1/navigate_to_pose …` drives it to a goal given in the chain's map frame, and it stops where a tape measure says it should. |
| 5 | Start the coordinator with the relays and the chain standing still. | Log shows `relayN: start -> PARKED at (…)` with correct positions; `/coord/chain` shows a route. |
| 6 | Push a parked relay 1 m by hand. | Within about 10 s the log says it is going back to its parking spot, and it does. |
| 7 | Send one goal by hand to `/coord/navigate_to_pose`, close to the client. | Log shows `CLIENT_ONLY`, the chain drives there, the action succeeds. |
| 8 | Send a goal far enough that a relay is needed. | Log shows `DEPLOY`, the relay and the chain move in steps and wait for each other, the action succeeds. |
| 9 | Connect the chain owner's goal source (4.1) with `--stuck-tries 0`. | Goals arrive, some are rejected, the source carries on with others. |
| 10 | Record `mesh_matrix` and `/rssi/links` during a short run and compare (6.4). | Predicted and measured values agree within a few dB after fitting, or the user decides what to do about the difference. |
| 11 | Full run with `--map-log` and `--state-log`. | Compare the log with `reference/live14/coordinator.log`. |

A real-robot start line, as a starting point (add the settings from sections 4 and 5):

```
python3 rssi/ros/exploration_coordinator.py \
  --config rssi/ros/nodes_real.yaml \
  --radio-gt hospital_world/real_env/gt/real_env_radio --gt hospital_world/real_env/gt/real_env_gt \
  --map-offset 4.4 5.0 --P0 <fitted> --n <fitted> --alpha <fitted> \
  --stuck-tries 0 --v-client <m/s> --v-relay <m/s> --v-floor-relay <m/s> \
  --map-log <dir> --state-log <file>
```

The simulated run live14 used all defaults plus
`--P0 -57 --config …/nodes_real_env.yaml --gt …/real_env_gt --radio-gt …/real_env_radio --map-offset 4.4 5.0`.

## 8. Things that went wrong in the simulator and will go wrong again

- **Single runs mislead.** Runs with the same settings differ a lot, and a repeat of the best setting has
  wedged outright. Repeat before you conclude a change helped or hurt.
- **A parameter set at run time may not have been applied.** `ros2 param set` fails without an error code.
  Read the value back.
- **A leftover static transform publisher** from an earlier start keeps publishing an old `map → odom` and the
  relay localises itself off its costmap; every goal then aborts. Kill old processes before starting a relay.
- **"Starting point in lethal space".** A relay stopped next to a wall cannot plan at all. `_free_relay` is the
  fix; make sure it still works with the new footprint and inflation.
- **Goals seen through gaps.** Frontiers behind glass or outside a doorway that the chain cannot reach are
  rejected as "not connected to the explored floor". Expect many such rejections; they are normal.
- **A plan that "succeeds" short of the goal** is normal too. The goal is only marked visited when the client
  ends within 0.6 m of it (`--visited-r`).
- **A relay that misses its waypoints five plans in a row is benched** for 300 s: it keeps forwarding where it
  stands and the planner works around it. If you see this on real hardware, the relay's navigation is the
  problem, not the plan.
- **The chain got wedged when it was not allowed to reverse** (run live15, 63 %). That is chain-side and the
  chain owner's to solve, but it means the coordinator will see a client that stops making progress. With
  `--stuck-tries 0` it gives up the goal after 40 s and moves on.

## 9. What this guide does not know

Ask the user. Do not assume.

- What the real relays are: drive type, footprint, speeds, sensors, on-board computer, and whether they will
  run Nav2.
- How the relays will localise in the chain's map (6.2).
- What the new chain navigation offers: action name and type, feedback fields, how goals are chosen, map topic
  and frame, cruising speed, whether it honours a speed limit.
- Where the chain will start on site, and facing which way (6.2).
- Where the GCS radio stands, and the relays' parking spots.
- Which radio (serial number, address) is on which robot.
- Whether the user wants the planner to stay model-only for the first real runs, or to use measured signal.

## 10. Keeping this folder in step with the simulator

`sync_from_sim.sh` copies every file again from the simulator tree and rewrites `MANIFEST.md`.
`sync_from_sim.sh --check` lists copies that differ from the simulator tree without changing anything. Once
you start editing files here for the real robots, do not run the sync without `--check`: it overwrites your
changes. `PORTING_GUIDE.md`, `smoke_test.py`, `smoke_test_expected.txt` and `rajant_tools/` exist only here
(the two Rajant tools were never part of the simulator tree).
