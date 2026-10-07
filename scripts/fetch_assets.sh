#!/usr/bin/env bash
# Restore the large third-party assets that are deliberately NOT in this repo.
#
#   ./scripts/fetch_assets.sh
#
#   1. AWS RoboMaker hospital models (~170 MB)   -> hospital_world/models/
#   2. saye Ackermann meshes from upstream (~100 MB, pinned commit)
#        -> mesh_sim_ws/src/ackermann-vehicle-gzsim-ros2/saye_description/models/saye/meshes/
#        -> hospital_world/models/saye_classic/meshes/   (the Classic port reuses them)
#
# Not fetched (proprietary / not ours): vendor/rajant_api, the chain_robot image.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

AWS=https://raw.githubusercontent.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps/master/worlds/hospital
UPSTREAM=https://raw.githubusercontent.com/alitekes1/ackermann-vehicle-gzsim-ros2
UPSTREAM_REV=f61ab5aa6f989996559b5f8d5a65a7839c95cb72   # the commit our fork is based on

# ---- 1. hospital models ------------------------------------------------------
HM="$ROOT/hospital_world/models"
if [ -d "$HM/aws_robomaker_hospital_floor_01_walls" ]; then
  echo "[1/2] hospital models already present"
else
  echo "[1/2] downloading hospital models..."
  tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
  for i in 1 2 3 4; do curl -fL --retry 3 -o "$tmp/p$i.zip" "$AWS/models_part$i.zip"; done
  mkdir -p "$HM"
  # Each zip extracts to models_partN/ -- they must be MERGED FLAT into models/.
  for i in 1 2 3 4; do
    unzip -q -o "$tmp/p$i.zip" -d "$tmp/x$i"
    cp -r "$tmp/x$i/models_part$i/"* "$HM/"
  done
fi

# ---- 2. saye meshes ----------------------------------------------------------
UM="$ROOT/mesh_sim_ws/src/ackermann-vehicle-gzsim-ros2/saye_description/models/saye/meshes"
CM="$ROOT/hospital_world/models/saye_classic/meshes"
mkdir -p "$UM" "$CM"
echo "[2/2] saye meshes @ ${UPSTREAM_REV:0:7}..."
for m in BaseReduced.dae TopReduced.dae STL27L.dae Wheel.dae \
         sensor_rack_lowres_shallow_laser.dae sensor_rack_support_lowres.dae; do
  [ -s "$UM/$m" ] || curl -fL --retry 3 -o "$UM/$m" \
      "$UPSTREAM/$UPSTREAM_REV/saye_description/models/saye/meshes/$m"
  cp -f "$UM/$m" "$CM/$m"
done

# ---- gate: every model:// in the world must resolve --------------------------
python3 - "$ROOT/hospital_world" <<'PY'
import os, sys, xml.etree.ElementTree as ET
hw = sys.argv[1]
want = {u.text.strip()[len('model://'):].split('/')[0]
        for u in ET.parse(os.path.join(hw, 'hospital_chain.world')).getroot().iter('uri')
        if u.text and u.text.strip().startswith('model://')}
have = set(os.listdir(os.path.join(hw, 'models')))
missing = sorted(want - have - {'chain_robot'})   # chain_robot ships in the safe_robot image
print(f'  {len(want)} models referenced, MISSING: {missing or "none"}')
sys.exit(1 if missing else 0)
PY
echo "done."
