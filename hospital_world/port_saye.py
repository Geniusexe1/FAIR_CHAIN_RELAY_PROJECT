#!/usr/bin/env python3
"""Port the Harmonic saye model.sdf to Gazebo Classic 11.

Text-level transform (not an XML round-trip) so the original's comments,
formatting and sensor-noise blocks survive intact.
"""
import os, re, sys

_HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(_HERE, '..', 'mesh_sim_ws', 'src', 'ackermann-vehicle-gzsim-ros2',
                   'saye_description', 'models', 'saye', 'model.sdf')
DST = os.path.join(_HERE, 'models', 'saye_classic', 'model.sdf')

s = open(SRC).read()
n = lambda tag: s.count(tag)

# ---- 1. SDF version: Classic 11 tops out at 1.7 ------------------------------
s = s.replace('<sdf version="1.8">', '<sdf version="1.7">')

# ---- 2. gz_frame_id is a gz-sim-only element; Classic's parser rejects it ----
before = len(re.findall(r'<gz_frame_id>', s))
s = re.sub(r'[ \t]*<gz_frame_id>.*?</gz_frame_id>\n', '', s)

# ---- 3. mesh URIs: package:// is not resolvable by Classic ------------------
s = s.replace('package://saye_description/models/saye/meshes/', 'model://saye_classic/meshes/')

# ---- 4. stray text node inside a <mesh> in the original ---------------------
s = s.replace('          /meshes/BaseReduced.dae\n', '')

# ---- 5. sensor types: gz names -> Classic names -----------------------------
s = s.replace('type="gpu_lidar"', 'type="ray"')
s = s.replace('type="rgbd_camera"', 'type="depth"')
s = s.replace('<lidar>', '<ray>').replace('</lidar>', '</ray>')
# gz-only depth block inside the rgbd sensor
s = re.sub(r'[ \t]*<depth_camera>.*?</depth_camera>\n', '', s, flags=re.S)

# ---- 6. attach a gazebo_ros plugin to every sensor --------------------------
SENSOR_PLUGINS = {
 'imu_sensor': '''        <plugin name="${NS}_imu" filename="libgazebo_ros_imu_sensor.so">
          <ros><namespace>/${NS}</namespace><remapping>~/out:=imu</remapping></ros>
          <frame_name>${NS}/base_link</frame_name>
        </plugin>
''',
 'gpu_lidar': '''        <plugin name="${NS}_lidar" filename="libgazebo_ros_ray_sensor.so">
          <ros><namespace>/${NS}</namespace><remapping>~/out:=scan</remapping></ros>
          <!-- LaserScan, not PointCloud2. Mapping is the goal, slam_toolbox and Nav2
               both want LaserScan, NUC2 has no pointcloud_to_laserscan, and this
               matches what mesh_sim already fed slam_toolbox on <ns>/scan.
               Bandwidth over the Rajant mesh: ~41 KB/s here vs ~21 MB/s for the
               1024x128 cloud. Set LIDAR_2D=False below to go back to 3D. -->
          <output_type>sensor_msgs/LaserScan</output_type>
          <frame_name>${NS}/base_link</frame_name>
        </plugin>
''',
 'rs_front': '''        <plugin name="${NS}_rs_front" filename="libgazebo_ros_camera.so">
          <ros><namespace>/${NS}</namespace></ros>
          <camera_name>rs_front</camera_name>
          <frame_name>${NS}/base_link</frame_name>
        </plugin>
''',
}
for cam in ('camera_front', 'camera_rear', 'camera_left', 'camera_right'):
    SENSOR_PLUGINS[cam] = f'''        <plugin name="${{NS}}_{cam}" filename="libgazebo_ros_camera.so">
          <ros><namespace>/${{NS}}</namespace></ros>
          <camera_name>{cam}</camera_name>
          <frame_name>${{NS}}/base_link</frame_name>
        </plugin>
'''

def add_sensor_plugins(text):
    out, added = [], []
    i, cur = 0, None
    for line in text.split('\n'):
        m = re.search(r'<sensor name="([^"]+)"', line)
        if m:
            cur = m.group(1)
        if '</sensor>' in line and cur in SENSOR_PLUGINS:
            out.append(SENSOR_PLUGINS[cur].rstrip('\n'))
            added.append(cur)
            cur = None
        out.append(line)
    return '\n'.join(out), added

s, added = add_sensor_plugins(s)

# ---- 6b. drop the camera sensors -------------------------------------------
# Mapping needs lidar + IMU. The 4 RGB cameras (720x1080 @10 Hz) plus the RGBD
# come to roughly 1.1 Gbps of raw topics per relay, which a Rajant mesh cannot
# carry - and relays saturating the very mesh they exist to maintain is
# self-defeating. Recoverable: set DROP_CAMERAS = False and re-run.
LIDAR_2D = True
if LIDAR_2D:
    # 128 vertical samples make this a 3D lidar; a LaserScan is a single ring.
    s = re.sub(r'(<vertical>\s*<samples>)128(</samples>)', r'\g<1>1\g<2>', s)
    # One ray, so min and max MUST match. Leaving them at -+0.5236 made Gazebo warn
    # "Only one vertical ray but vertical min. and max. angle are not equal" and,
    # together with the GPU-ray FOV cap, segfaulted gzserver on respawn.
    s = re.sub(r'(<vertical>.*?<min_angle>)[^<]+(</min_angle>\s*<max_angle>)[^<]+(</max_angle>)',
               r'\g<1>0\g<2>0\g<3>', s, flags=re.S)

DROP_CAMERAS = True
CAMERA_SENSORS = ('rs_front', 'camera_front', 'camera_rear', 'camera_left', 'camera_right')
if DROP_CAMERAS:
    for name in CAMERA_SENSORS:
        s = re.sub(r'[ \t]*<sensor name="%s".*?</sensor>\n' % re.escape(name),
                   '', s, flags=re.S)
    s = s.replace('    <link name="base_link">',
                  '    <!-- Camera sensors (rs_front + 4x RGB) were dropped in this port:\n'
                  '         ~1.1 Gbps of raw image topics per relay will not cross a Rajant\n'
                  '         mesh. Lidar + IMU are kept - they are what mapping needs.\n'
                  '         Regenerate with DROP_CAMERAS=False in port_saye.py to restore. -->\n'
                  '    <link name="base_link">')
    print(f'  cameras dropped     : {len(CAMERA_SENSORS)}')

# ---- 6c. physical inertia for the steering knuckles -------------------------
# Authored as 0.005 kg / izz 1.8e-6 because gz's AckermannSteering drives these
# joints KINEMATICALLY - inertia never mattered. Under a Classic torque PID it
# does: 0.6 N.m into 1.8e-6 kg.m2 is 333,000 rad/s2, so the joint slammed into
# its +-0.6 stop at 8 rad/s (own limit: 1.0) and steered the car in circles with
# nothing on cmd_vel. Adding <damping> instead made ODE emit NaN outright.
# 0.1 kg / 1e-4 is a plausible steering knuckle and gives the PID something real.
def fix_steering_inertia(text):
    """Rewrite mass+inertia ONLY inside the two steering links."""
    def repl(m):
        block = m.group(0)
        block = re.sub(r'<mass>[^<]+</mass>', '<mass>0.1</mass>', block)
        for t in ('ixx', 'iyy', 'izz'):
            block = re.sub(r'<%s>[^<]+</%s>' % (t, t), '<%s>0.0001</%s>' % (t, t), block)
        return block
    return re.subn(r'<link name="[a-z_]*steering_link">.*?</inertial>', repl, text, flags=re.S)

s, nfix = fix_steering_inertia(s)
print(f'  steering links re-inertia\'d: {nfix}')

# ---- 6d. effort limits on the DRIVE wheels ----------------------------------
# The four drive joints were authored with only lower/upper and no <effort>.
# gz's AckermannSteering never cared (kinematic), but libgazebo_ros_ackermann_
# drive.so derives the joint's max motor force from <effort>, so with none the
# wheels were driven with ~zero torque: commanded 0.3 m/s, achieved 0.0056 m/s.
def add_wheel_effort(text):
    def repl(m):
        return m.group(0).replace(
            '          <upper>1.79769e+308</upper>\n',
            '          <upper>1.79769e+308</upper>\n'
            '          <effort>2</effort>\n'
            '          <velocity>100</velocity>\n')
    return re.subn(r'<joint name="(?:front|rear)_(?:left|right)_wheel_joint".*?</joint>',
                   repl, text, flags=re.S)

s, neff = add_wheel_effort(s)
print(f'  drive wheels given effort limits: {neff}')

# ---- 6f. strip use_parent_model_frame (THE PORT BUG) -----------------------
# Inert in the SDF 1.8 Harmonic original (tag removed in SDF 1.7+, sdformat ignores
# it) but HONORED by Gazebo Classic's sdformat9 at SDF 1.7: it pins the front wheel
# spin axis to the MODEL frame so it does not follow the steering link. Steering
# then does almost nothing: measured radius -1.835 m (wrong sign, 3.4x too wide)
# vs 0.533 m geometric. Stripping it gives +0.682 m and turning waypoints arrive.
s = re.sub(r'[ \t]*<use_parent_model_frame>[^<]*</use_parent_model_frame>\n', '', s)

# ---- 7. replace the four gz system plugins with Classic equivalents ---------
gz_block_start = s.index('    <plugin\n      filename="gz-sim-ackermann-steering-system"')
gz_block_end   = s.index('  </model>')
classic_plugins = '''    <!-- ==================================================================
         Ported from Gazebo Harmonic to Gazebo Classic 11.
         The four gz-sim system plugins have no Classic equivalent and were
         replaced as follows:
           gz-sim-ackermann-steering-system   -> libgazebo_ros_ackermann_drive.so
           gz-sim-odometry-publisher-system   -> folded into ackermann_drive
                                                 (publish_odom + publish_odom_tf)
           gz-sim-joint-state-publisher-system-> libgazebo_ros_joint_state_publisher.so
           gz-sim-pose-publisher-system       -> dropped. Its only remaining job in
                 the Harmonic model was sensor poses; in Classic each sensor plugin
                 declares its own <frame_name>, and robot_state_publisher supplies
                 the link frames. Re-adding a pose publisher here would recreate the
                 two-parents-on-base_link bug documented in the original (A10).
         ================================================================== -->

    <plugin name="${NS}_ackermann" filename="libgazebo_ros_ackermann_drive.so">
      <ros>
        <namespace>/${NS}</namespace>
        <remapping>cmd_vel:=cmd_vel</remapping>
        <remapping>odom:=odom</remapping>
        <remapping>distance:=distance</remapping>
      </ros>
      <update_rate>100.0</update_rate>

      <front_left_joint>front_left_wheel_joint</front_left_joint>
      <front_right_joint>front_right_wheel_joint</front_right_joint>
      <rear_left_joint>rear_left_wheel_joint</rear_left_joint>
      <!-- NOTE: the Harmonic original lists <right_joint>front_right_wheel_joint</right_joint>
           TWICE and never names rear_right_wheel_joint, so its rear right wheel is
           undriven. Fixed here. Worth fixing upstream in mesh_sim_ws too. -->
      <rear_right_joint>rear_right_wheel_joint</rear_right_joint>
      <left_steering_joint>front_left_wheel_steering_joint</left_steering_joint>
      <right_steering_joint>front_right_wheel_steering_joint</right_steering_joint>
      <!-- steering_wheel_joint intentionally omitted: this model has no steering
           column link. The plugin only warns for that one; the other six joints
           are the ones whose absence disables it. -->

      <max_steer>0.4</max_steer>
      <max_steering_angle>0.4</max_steering_angle>
      <max_speed>10.0</max_speed>
      <!-- Scaled to THIS robot. The gazebo_ros demo value (1500 0 1) is for the
           full-size Prius; here the steering links weigh 0.005 kg with a 25 N.m
           effort limit and no damping, so 1500 drove them straight into the
           +-0.6 rad stops at 2 rad/s (past their own 1.0 rad/s limit) and the car
           steered itself in circles with nothing published on cmd_vel.
           The Harmonic model never needed gains at all - gz's AckermannSteering
           drives the steering kinematically, which is also why these joints were
           authored with no damping. -->
      <left_steering_pid_gain>2.0 0 0.2</left_steering_pid_gain>
      <left_steering_i_range>0 0</left_steering_i_range>
      <right_steering_pid_gain>2.0 0 0.2</right_steering_pid_gain>
      <right_steering_i_range>0 0</right_steering_i_range>
      <!-- WITHOUT THIS THE CAR DOES NOT DRIVE. The gain defaults to 0, so the
           plugin applies no wheel torque at all: commanded 0.3 m/s, achieved
           0.012 m/s. The shipped demo world uses 1000, but that is a ~1500 kg
           Prius on 0.3 m wheels; this is a 3 kg car on 0.0365 m wheels, so the
           gain is scaled down by roughly the same ratio. -->
      <!-- P=150 I=50 D=1 with a real i_range. The stock 10/0/0-range was P-only
           with integral DISABLED, which could not break static friction at small
           velocity error: cmd 0.05 m/s drove BACKWARD, cmd 0.10 m/s moved 0.007 m
           in 12 s (stalled), only >=0.15 tracked. Near a goal MPPI commands exactly
           those velocities, so the car could never fine-position.
           Higher P also holds position far better at rest: measured idle drift
           12 mm/s at P=10 vs ~1.5 mm/s at P=100.
           Do NOT raise I much past 50: at I=250 integral windup drove the car
           backward on every command. -->
      <linear_velocity_pid_gain>100 50 1</linear_velocity_pid_gain>
      <linear_velocity_i_range>-20 20</linear_velocity_i_range>

      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>false</publish_wheel_tf>
      <publish_distance>true</publish_distance>
      <odometry_frame>${NS}/odom</odometry_frame>
      <robot_base_frame>${NS}/base_link</robot_base_frame>
    </plugin>

    <plugin name="${NS}_joint_state" filename="libgazebo_ros_joint_state_publisher.so">
      <ros><namespace>/${NS}</namespace><remapping>~/out:=joint_states</remapping></ros>
      <update_rate>20</update_rate>
      <joint_name>front_left_wheel_joint</joint_name>
      <joint_name>front_right_wheel_joint</joint_name>
      <joint_name>rear_left_wheel_joint</joint_name>
      <joint_name>rear_right_wheel_joint</joint_name>
      <joint_name>front_left_wheel_steering_joint</joint_name>
      <joint_name>front_right_wheel_steering_joint</joint_name>
    </plugin>

'''
s = s[:gz_block_start] + classic_plugins + s[gz_block_end:]

open(DST, 'w').write(s)
print(f'wrote {DST}')
print(f'  gz_frame_id removed : {before}')
print(f'  sensor plugins added: {len(added)} -> {added}')
for bad in ('gz-sim-', 'gz_frame_id', 'package://', 'gpu_lidar"', 'rgbd_camera', '<lidar>', '<depth_camera>'):
    c = s.count(bad)
    print(f'  residual {bad!r:22}: {c}' + ('   <-- PROBLEM' if c else ''))
