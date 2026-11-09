#!/bin/bash
# Sim fallback demo: Neato finds an orange ball and pushes it into AprilTag #4.
# Ball and tag are offset from the robot's start line on purpose, so it's a
# real aim/approach, not a straight-line coast.
#
# Usage:
#   bash run_sim_demo.sh
#
# Watch it in the Gazebo GUI window that opens. Ctrl+C in this terminal to
# tear everything down when you're done.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TAG_PNG="$SCRIPT_DIR/tag4.png"
WS=~/ros2_ws

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=23

echo "== cleaning up any previous sim =="
for pat in "lib/machine_vision_pkg/vision_node" "gz sim" "parameter_bridge" "robot_state_publisher" "scan_to_pc2" "simulator_adapter"; do
  pids=$(pgrep -f "$pat" || true)
  [ -n "$pids" ] && kill $pids 2>/dev/null || true
done
sleep 2

echo "== launching empty world =="
ros2 launch neato2_gazebo harmonic_world.py world_file:=empty.world > /tmp/sim_demo_launch.log 2>&1 &
SIM_PID=$!
sleep 10

echo "== spawning ball and AprilTag #4 (offset from the robot's start line) =="
cat > /tmp/demo_ball.sdf << 'EOF'
<?xml version="1.0"?>
<sdf version="1.8">
  <model name="orange_ball">
    <pose>0 0 0.1 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>0.4</mass>
        <inertia><ixx>0.0016</ixx><iyy>0.0016</iyy><izz>0.0016</izz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><sphere><radius>0.1</radius></sphere></geometry>
        <surface>
          <friction>
            <ode><mu>0.8</mu><mu2>0.8</mu2></ode>
            <torsional>
              <coefficient>0.3</coefficient>
              <surface_radius>0.1</surface_radius>
              <use_patch_radius>false</use_patch_radius>
            </torsional>
          </friction>
          <contact><ode><max_vel>1.0</max_vel><min_depth>0.001</min_depth></ode></contact>
        </surface>
      </collision>
      <visual name="visual">
        <geometry><sphere><radius>0.1</radius></sphere></geometry>
        <material>
          <ambient>1 0.4 0 1</ambient>
          <diffuse>1 0.4 0 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
EOF

cat > /tmp/demo_tag.sdf << EOF
<?xml version="1.0"?>
<sdf version="1.8">
  <model name="apriltag4_box">
    <static>true</static>
    <pose>0 0 0 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>0.02 0.26 0.26</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.02 0.26 0.26</size></box></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <pbr>
            <metal>
              <albedo_map>file://${TAG_PNG}</albedo_map>
              <metalness>0.0</metalness>
              <roughness>1.0</roughness>
            </metal>
          </pbr>
        </material>
        <cast_shadows>false</cast_shadows>
      </visual>
    </link>
  </model>
</sdf>
EOF

# Goal structure (two posts + crossbar) mounted under the tag. Colored
# black on purpose: it sits close to / flush with the tag's white quiet-
# zone border, and a white-on-white goal washed the tag's edges out
# enough that cv2.aruco couldn't decode it (confirmed empirically -- a
# tag overlapping or nearly touching a same-colored structure fails to
# detect even though it looks fine to the eye).
cat > /tmp/demo_goal.sdf << 'EOF'
<?xml version="1.0"?>
<sdf version="1.8">
  <model name="goal_structure">
    <static>true</static>
    <pose>0 0 0 0 0 0</pose>
    <link name="link">
      <collision name="left_post_collision">
        <pose>0 -0.7 0.15 0 0 0</pose>
        <geometry><box><size>0.04 0.04 0.3</size></box></geometry>
      </collision>
      <visual name="left_post_visual">
        <pose>0 -0.7 0.15 0 0 0</pose>
        <geometry><box><size>0.04 0.04 0.3</size></box></geometry>
        <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material>
      </visual>
      <collision name="right_post_collision">
        <pose>0 0.7 0.15 0 0 0</pose>
        <geometry><box><size>0.04 0.04 0.3</size></box></geometry>
      </collision>
      <visual name="right_post_visual">
        <pose>0 0.7 0.15 0 0 0</pose>
        <geometry><box><size>0.04 0.04 0.3</size></box></geometry>
        <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material>
      </visual>
      <collision name="crossbar_collision">
        <pose>0 0 0.3 0 0 0</pose>
        <geometry><box><size>0.04 1.44 0.04</size></box></geometry>
      </collision>
      <visual name="crossbar_visual">
        <pose>0 0 0.3 0 0 0</pose>
        <geometry><box><size>0.04 1.44 0.04</size></box></geometry>
        <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
EOF

# ball: 1.3m forward, 0.6m left of the robot's start line
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'sdf_filename: "/tmp/demo_ball.sdf", name: "orange_ball", pose: {position: {x: 1.3, y: 0.6, z: 0.1}}'

# goal structure: 3.0m forward, 0.6m RIGHT of the robot's start line (opposite side from the ball)
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'sdf_filename: "/tmp/demo_goal.sdf", name: "goal_structure", pose: {position: {x: 3.0, y: -0.6, z: 0.0}}'

# goal tag: mounted flush on top of the crossbar (crossbar top is at z=0.32)
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'sdf_filename: "/tmp/demo_tag.sdf", name: "apriltag4_box", pose: {position: {x: 3.0, y: -0.6, z: 0.45}}'

sleep 1

echo "== starting vision_node =="
ros2 run machine_vision_pkg vision_node --ros-args \
  -r camera/image_raw:=/camera/image \
  -p goal_tag_size:=0.2 \
  -p use_sim_time:=true

wait $SIM_PID
