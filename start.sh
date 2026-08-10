#!/usr/bin/env bash
#
# Start the warehouse simulation (Gazebo Harmonic + ros_gz bridge).
#
#   ./start.sh                 GUI + bridge
#   ./start.sh --headless      server only, no GUI window
#   ./start.sh --paused        start paused (step with the GUI play button)
#   ./start.sh --no-bridge     Gazebo only, no ROS 2 bridge / static TF
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_FILE="$SCRIPT_DIR/warehouse.sdf"
RUN_DIR="$SCRIPT_DIR/.sim"
ROS_SETUP="/opt/ros/humble/setup.bash"

HEADLESS=0
PAUSED=0
BRIDGE=1

for arg in "$@"; do
  case "$arg" in
    --headless)  HEADLESS=1 ;;
    --paused)    PAUSED=1 ;;
    --no-bridge) BRIDGE=0 ;;
    -h|--help)   sed -n '2,10p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "Unknown option: $arg (try --help)" >&2; exit 1 ;;
  esac
done

# --------------------------------------------------------------------------
# Pre-flight
# --------------------------------------------------------------------------
[[ -f "$WORLD_FILE" ]] || { echo "ERROR: world not found: $WORLD_FILE" >&2; exit 1; }
command -v gz >/dev/null || { echo "ERROR: 'gz' not on PATH (install Gazebo Harmonic)." >&2; exit 1; }

if [[ -f "$RUN_DIR/gz.pid" ]] && kill -0 "$(cat "$RUN_DIR/gz.pid")" 2>/dev/null; then
  echo "A simulation already seems to be running (PID $(cat "$RUN_DIR/gz.pid"))."
  echo "Run ./stop.sh first."
  exit 1
fi

mkdir -p "$RUN_DIR"
rm -f "$RUN_DIR"/*.pid

# --------------------------------------------------------------------------
# Gazebo
# --------------------------------------------------------------------------
# The Dingo meshes are referenced as model://dd100/... -- let gz find them.
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

GZ_ARGS=(-v 3 --force-version 8)
[[ $PAUSED -eq 0 ]] && GZ_ARGS+=(-r)
[[ $HEADLESS -eq 1 ]] && GZ_ARGS+=(-s --headless-rendering)

echo "==> Starting Gazebo  (world: $(basename "$WORLD_FILE"))"
gz sim "${GZ_ARGS[@]}" "$WORLD_FILE" >"$RUN_DIR/gazebo.log" 2>&1 &
echo $! >"$RUN_DIR/gz.pid"

# Wait for the world to come up on the gz transport bus.
echo -n "==> Waiting for Gazebo transport"
for _ in $(seq 1 60); do
  if gz topic -l 2>/dev/null | grep -q '/world/warehouse'; then
    echo " ok"
    break
  fi
  if ! kill -0 "$(cat "$RUN_DIR/gz.pid")" 2>/dev/null; then
    echo " FAILED"
    echo "Gazebo exited early. Last lines of $RUN_DIR/gazebo.log:" >&2
    tail -n 20 "$RUN_DIR/gazebo.log" >&2
    exit 1
  fi
  echo -n "."
  sleep 1
done

# --------------------------------------------------------------------------
# ROS 2 bridge + static TF
# --------------------------------------------------------------------------
if [[ $BRIDGE -eq 1 ]]; then
  if [[ ! -f "$ROS_SETUP" ]]; then
    echo "WARN: $ROS_SETUP not found -- skipping ROS 2 bridge."
  else
    # ROS setup scripts reference unset vars; relax -u while sourcing.
    set +u
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    set -u

    echo "==> Starting ros_gz bridge"
    ros2 run ros_gz_bridge parameter_bridge \
      /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
      /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
      /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
      /tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
      /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
      /imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
      /joint_states@sensor_msgs/msg/JointState[gz.msgs.Model \
      --ros-args -p use_sim_time:=true \
      >"$RUN_DIR/bridge.log" 2>&1 &
    echo $! >"$RUN_DIR/bridge.pid"

    # base_link -> sensor frames (fixed joints, so not covered by /tf from DiffDrive)
    echo "==> Publishing static sensor transforms"
    ros2 run tf2_ros static_transform_publisher \
      --x 0 --y 0 --z 0.3963 --frame-id base_link --child-frame-id lidar_link \
      --ros-args -p use_sim_time:=true >"$RUN_DIR/tf_lidar.log" 2>&1 &
    echo $! >"$RUN_DIR/tf_lidar.pid"

    ros2 run tf2_ros static_transform_publisher \
      --x 0 --y 0 --z 0 --frame-id base_link --child-frame-id imu_link \
      --ros-args -p use_sim_time:=true >"$RUN_DIR/tf_imu.log" 2>&1 &
    echo $! >"$RUN_DIR/tf_imu.pid"
  fi
fi

sleep 2

cat <<EOF

Simulation is up.  Logs: $RUN_DIR/

  ROS 2 topics   /scan  /odom  /tf  /imu  /joint_states  /clock   (sub)
                 /cmd_vel                                          (pub)

  Drive it:      ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
                   --ros-args -p use_sim_time:=true
  Inspect:       ros2 topic echo /scan --once
  Stop:          ./stop.sh

EOF
