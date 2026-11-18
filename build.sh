#!/usr/bin/env bash
set -euo pipefail

BUILD_TYPE=Release
SIMULATION_ONLY=false
GHAND_PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
usage() {
  cat <<'HELP'
Usage: ./src/ghand-ros2/build.sh [OPTIONS]
Run from the ROS workspace root after sourcing ROS 2 Humble.
  --simulation        Build preview/Gazebo/bringup without hardware SDK
  Hardware builds use the bundled ghand_sdk_vendor dynamic library.
  --release           Release build (default)
  --debug             Debug build
  --help              Show this help
Transport support is owned by the SDK; no per-transport ROS build flags are needed.
HELP
}
while (($#)); do
  case "$1" in
    --simulation) SIMULATION_ONLY=true; shift ;;
    --release) BUILD_TYPE=Release; shift ;;
    --debug) BUILD_TYPE=Debug; shift ;;
    --canfd|--ethercat)
      echo "$1 is obsolete: transport support is supplied by GHandSDK." >&2; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done
if [[ ! -d src ]]; then echo 'Run this script from the workspace root (containing src/).' >&2; exit 2; fi
if [[ ${ROS_DISTRO:-} != humble ]]; then echo 'Source /opt/ros/humble/setup.bash first.' >&2; exit 2; fi
if $SIMULATION_ONLY; then
  colcon build --base-paths "$GHAND_PROJECT_DIR" --symlink-install --packages-select ghand_description \
    ghand_bringup ghand_moveit_config ghand_gazebo \
    --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DBUILD_TESTING=OFF"
  exit
fi
colcon build --base-paths "$GHAND_PROJECT_DIR" --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" -DBUILD_TESTING=OFF
