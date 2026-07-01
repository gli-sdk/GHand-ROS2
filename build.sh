#!/bin/bash
set -e

BUILD_TYPE="Debug"
ENABLE_CANFD="OFF"
ENABLE_ETHERCAT="OFF"

usage() {
  cat <<EOF
Usage: $0 [OPTIONS]

Build the GHand-ROS2 workspace with colcon.

Options:
  --canfd      Enable CANFD backend (requires ZLG CAN vendor libraries)
  --ethercat   Enable EtherCAT backend (requires SOEM)
  --release    Build in Release mode
  --help, -h   Show this help message

By default the script builds in Debug mode with both CANFD and EtherCAT
disabled, which is suitable for mock-hardware development and CI.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --canfd)
      ENABLE_CANFD="ON"
      shift
      ;;
    --ethercat)
      ENABLE_ETHERCAT="ON"
      shift
      ;;
    --release)
      BUILD_TYPE="Release"
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      usage
      exit 1
      ;;
  esac
done

CMAKE_ARGS=(
  "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
  "-DGHAND_ENABLE_CANFD=${ENABLE_CANFD}"
  "-DGHAND_ENABLE_ETHERCAT=${ENABLE_ETHERCAT}"
)

echo "Building GHand-ROS2..."
echo "  Build type: ${BUILD_TYPE}"
echo "  CANFD:      ${ENABLE_CANFD}"
echo "  EtherCAT:   ${ENABLE_ETHERCAT}"

colcon build --symlink-install --cmake-args "${CMAKE_ARGS[@]}"
