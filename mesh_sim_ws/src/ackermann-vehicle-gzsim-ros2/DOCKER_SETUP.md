# Docker Installation (corrected for the ASL NUCs)

Replaces the upstream README's "Docker Installation" section. Three things in that
section do not work on our hardware; each correction below is verified.

## What changed and why

| Upstream README says | Reality | Fix |
|---|---|---|
| `docker run ... alitekes1/ackermann_sim:latest` | **That image does not exist.** `hub.docker.com/v2/repositories/alitekes1/ackermann_sim/` returns **404**, and user `alitekes1` has **0 public repos** | You must **build the image yourself** |
| Prerequisite: "NVIDIA Container Toolkit (for GPU support)" | **Neither NUC has an NVIDIA GPU** (no `nvidia-smi`). Both have Intel iGPU at `/dev/dri/renderD128` | Use `--device /dev/dri`, drop all `NVIDIA_*` vars |
| Requirements: ROS 2 **Humble** + Gazebo **Harmonic** | Correct as a target, but there is **no apt package** for that pairing. rosdistro releases `ros_gz` for humble at `0.244.x` (the Fortress-era `ros_ign*` line); the OSRF repo ships only `gz-*` binaries, zero ROS packages | `Dockerfile.humble` **compiles `ros_gz` from source** against Harmonic |

`docker-compose.yaml` is also unusable as-is: it pins `ROS_DISTRO: jazzy` and hard-requires
`runtime: nvidia` + a `deploy.resources.devices: [gpu]` block, so it will refuse to start here.
Use the `docker build` / `docker run` commands below instead.

---

## 1. Clone

```bash
mkdir -p ~/ackermann_ws/src && cd ~/ackermann_ws/src
git clone https://github.com/alitekes1/ackermann-vehicle-gzsim-ros2
cd ackermann-vehicle-gzsim-ros2
```

> **Watch the nesting.** Running the README's `mkdir -p ackermann_sim/src` from *inside* an
> existing `ackermann_sim/` produces `ackermann_sim/src/ackermann_sim/src/...`, which confuses
> the colcon workspace root. The repo must sit exactly one level under a `src/`.

## 2. Build the image

```bash
docker build -f Dockerfile.humble -t ackermann_sim:humble .
```

Slow on first run (~15 min; it compiles `ros_gz`). Verify it produced Harmonic, not Fortress:

```bash
docker run --rm ackermann_sim:humble bash -c 'gz sim --version; ros2 pkg prefix ros_gz_sim'
```

**Verified output on this machine:**
```
Gazebo Sim, version 8.15.0          <- Harmonic. If this says 6.x, Fortress leaked in.
/root/ros_gz_ws/install/ros_gz_sim
```

Optional deeper check — loads the actual world headless, no display needed:
```bash
docker run --rm ackermann_sim:humble bash -c \
  'timeout 10 gz sim -s -r -v 2 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/saye_description/worlds/saye_world.sdf'
```
Should exit 0. A `laser_retro ... not defined in SDF` warning is benign (upstream schema nit).

> The image sets an **ENTRYPOINT** that sources `/opt/ros/humble`, the `ros_gz` workspace, and the
> app workspace. This matters: Ubuntu's `.bashrc` returns early for non-interactive shells, so
> relying on `.bashrc` alone silently breaks `docker run ... bash -c` and any scripted use.

## 3. Allow the container to draw on your screen

Required for Gazebo/RViz; the upstream README omits it and this is the usual reason
you get `cannot open display`.

```bash
xhost +local:docker
```

## 4. Run

```bash
docker run -it \
  --name ackermann_sim \
  --network host \
  --env ROS_DOMAIN_ID=42 \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  --privileged \
  ackermann_sim:humble
```

Flag notes:
- `--network host` — required for DDS to reach the other NUC. Also makes `--hostname` pointless, so it's dropped.
- `ROS_DOMAIN_ID=42` — must match on both NUCs. 42 matches the existing `weakest_link`/`mesh_matrix` containers, and since this image is Humble too, there is no cross-distro risk.
- `--device /dev/dri` — Intel iGPU, replaces the NVIDIA runtime.
- If Gazebo fails to render, add `--env LIBGL_ALWAYS_SOFTWARE=1` (slow but works).

## 5. Extra terminal in the same container

```bash
docker exec -it ackermann_sim bash
```

## 6. Run the simulation

All three shells are already sourced via `.bashrc`.

```bash
ros2 launch saye_bringup saye_spawn.launch.py
# second terminal:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Second NUC

Build once, ship over the **LAN, not the Rajant mesh** (keeps a multi-GB transfer off the radio):

```bash
docker save ackermann_sim:humble | ssh asl@192.168.7.56 docker load
```

Then run the same command there with a different `--name`. Verify they see each other:

```bash
# on NUC2
ros2 topic list          # should show topics published by NUC1
```

> **Heads-up for mesh work:** both NUCs also sit on `192.168.7.0/24`. With `--network host`
> and no DDS interface whitelist, Fast DDS will route everything over wifi/ethernet and the
> **Rajant mesh will carry nothing** — while appearing to work fine. Forcing traffic onto the
> mesh needs a Fast DDS profile whitelisting `10.195.219.0` (NUC1) / `10.195.221.0` (NUC2).
> Not needed to get the sim running; needed before any mesh measurement.

## Not yet done: two robots

This guide gets **one** robot running per machine. Spawning two non-conflicting robots still
requires namespacing work — the repo hardcodes the Gazebo model name (`saye`), all TF frames
(`base_link`, `odom`), and uses **absolute** topic names in `saye_bringup/config/ros_gz_bridge.yaml`
(`/cmd_vel`, `/odom`, `/tf`), which ROS namespaces cannot override. `/cmd_vel` is also bridged
`BIDIRECTIONAL`, so today one teleop would drive both cars.
