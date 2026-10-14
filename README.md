# FAIR Chain–Relay Project

Multi-robot simulation where an articulated **mission robot** (the "head") explores an
indoor environment while **Ackermann relay cars** trail it to keep a Rajant mesh radio
link alive. Two NUCs, linked by the Rajant mesh, share one simulated world.

| role | robot | sim |
|---|---|---|
| head / mission robot | `chain_robot` — 4-module tracked snake | Gazebo **Classic 11** |
| relays | `relay1`, `relay2` — Ackermann "saye" cars | ported **Harmonic → Classic** |

ROS 2 Humble, `ROS_DOMAIN_ID=42`, all containers `--network host --ipc host`.

---

## Repository layout

```
FAIR_CHAIN_RELAY_PROJECT/
├── README.md                  you are here
├── scripts/fetch_assets.sh    restores the large third-party assets (run this first)
├── hospital_world/            Gazebo Classic world: chain_robot + relays in the AWS hospital
│   ├── CHECKPOINT.md          ** detailed log: every bug, fix and measurement **
│   ├── hospital_chain.world   hospital + chain_robot + tuned physics block
│   ├── models/saye_classic/   the Ackermann relay, ported to Classic (model.sdf is GENERATED)
│   ├── port_saye.py           generates saye_classic/model.sdf from the Harmonic model
│   ├── run_*.sh, spawn_saye.sh, waypoint.sh, stop_all.sh   operational scripts
│   ├── tools/gt_occupancy.py  ground-truth occupancy grid from collision geometry
│   └── gt/                    the GT grid: .npy + .json metadata + .png renders
├── rssi/                      RSSI model (paper eq. 1), heatmaps, noise, live ROS 2 node
│   └── README.md              ** parameters, measured P0, how to run **
└── mesh_sim_ws/               the original Gazebo Harmonic two-robot sim + shared Nav2 config
    ├── CLAUDE.md              ** operator notes: machines, invariants, traps **
    ├── RUNBOOK.md, WORKFLOW.md
    ├── Dockerfile             builds the mesh_sim:humble image used on both NUCs
    └── src/
        ├── ackermann-vehicle-gzsim-ros2/   fork of alitekes1's package (see Credits)
        └── rajant_mesh/                    mesh link monitor nodes
```

Start with **`hospital_world/CHECKPOINT.md`** for the current system and
**`mesh_sim_ws/CLAUDE.md`** for operator notes and the list of traps.

---

## Setup

### 1. Restore assets

```bash
./scripts/fetch_assets.sh
```

Downloads the AWS hospital models (~170 MB) and the saye meshes (~100 MB, pinned to the
upstream commit this fork is based on), then checks every `model://` in the world resolves.

### 2. Secrets — local file, never committed

```bash
cat > ~/.fair_chain.env <<'EOF'
export NUC2_SSH_PASSWORD='...'
export RAJANT_ADMIN_PASSWORD='...'
EOF
chmod 600 ~/.fair_chain.env
```

The scripts `source` this. Nothing secret lives in the repo.

### 3. Things NOT in this repo

| what | where to get it |
|---|---|
| `mesh_sim_ws/vendor/rajant_api/` | Rajant's protobuf API — proprietary; obtain from Rajant / BC\|Commander. The `Dockerfile` `COPY`s it, so the image will not build without it. |
| `chain_robot` | ships inside the `gudwls124z/safe_robot:260826` Docker image |

### 4. Containers

| container | machine | image | role |
|---|---|---|---|
| `safe_robot` | NUC1 | `gudwls124z/safe_robot:260826` | Gazebo + physics + chain_robot + **both relay models** |
| `nuc1` | NUC1 | `mesh_sim:humble` | relay1's Nav2 + waypoint tools |
| `nuc2` | NUC2 | `mesh_sim:humble` | relay2's Nav2 |

```bash
hospital_world/run_safe_robot.sh        # create safe_robot
hospital_world/run_nuc1_container.sh    # create nuc1  (--ipc host is MANDATORY, see traps)
docker build -t mesh_sim:humble mesh_sim_ws/   # then docker save | ssh | docker load to NUC2
```

`safe_robot` also needs `pip install trimesh pycollada` for the GT tool (lost on recreate).

---

## Running

```bash
cd hospital_world
./run_chain_stack.sh               # gazebo + hospital + chain_robot stack (exploration opt-in: --explore)
./spawn_saye.sh relay1 2.5 12.0    # relays 1 m apart, clear of chain_robot
./spawn_saye.sh relay2 3.5 12.0
./run_relay1_nuc1.sh relay1        # only AFTER the relay is spawned
./run_relay2_nuc2.sh relay2        # NUC2, last

./waypoint.sh relay1 4.0 -1.5      # map frame
./stop_all.sh
```

Gazebo's window opens **behind other windows** — Alt+Tab to it.

**`map` is not the world frame.** It is chain_robot's spawn, world (0, 12). A waypoint of
`(4.0, -1.5)` is world `(4.0, 10.5)` in the Gazebo GUI.

---

## Status

**Working**
- Hospital world + chain_robot load and run.
- Relays spawn, sense, and navigate across the two NUCs.
- Relay waypoint accuracy **~2–8 cm**, including pure 1 m sideways goals (via reversing K-turns).
- RSSI prediction (Seo et al., RA-L 2025, eq. 1) on the GT map: heatmaps, noise, link
  topology, client-to-GCS connectivity margin maps, and a live ROS 2 node publishing every
  link and the margin map from the robots' sim positions.
  P0 measured on our Rajant Cardinals: -60.8 dBm. See `rssi/README.md`.
- Ground-truth occupancy grid: `hospital_world/gt/hospital_gt.npy`
  (uint8, 1 = occupied, 0.05 m/cell, 27.2 × 59.1 m).

**Known issues**
- **chain_robot cannot drive** — spins in place in its own shipped world, tumbles in the
  hospital. Bug is in its Gazebo plugin (not ours; left unmodified).
- The Harmonic `robot1`/`robot2` sim has not been re-tested since the shared Nav2 tuning changed.
- Sim real-time factor is ~0.27 with the full stack loaded.

---

## The traps (short version — detail in CHECKPOINT.md / CLAUDE.md)

1. `GAZEBO_IP=127.0.0.1` or multicast failures freeze the sim.
2. `GAZEBO_MODEL_DATABASE_URI=` (empty) or a missing model hangs Gazebo forever.
3. `--ipc host` on every container, or FastDDS shared memory silently drops **all** data
   while `ros2 topic list` still looks healthy. Check with `ros2 topic hz`, not `topic list`.
4. Start a relay's Nav2 only after the relay exists, or it sits `inactive` forever.
5. Porting SDF 1.8 → 1.7 revived `<use_parent_model_frame>`, which silently disabled steering.
6. COLLADA `<unit meter="0.01">` is honoured by Gazebo but not by trimesh.
7. `hospital_world/models/saye_classic/model.sdf` is **generated** — edit `port_saye.py`.

---

## Credits

- `mesh_sim_ws/src/ackermann-vehicle-gzsim-ros2` is a modified fork of
  [alitekes1/ackermann-vehicle-gzsim-ros2](https://github.com/alitekes1/ackermann-vehicle-gzsim-ros2)
  at commit `f61ab5a`.
- Hospital world and models: [mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps)
  (AWS RoboMaker hospital). Not redistributed here; fetched by `scripts/fetch_assets.sh`.
- `chain_robot`: from the `gudwls124z/safe_robot` image. Not redistributed here.
