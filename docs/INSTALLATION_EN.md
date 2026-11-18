## Installation and build

Physical hardware uses GHand C++ SDK 3.1 or compatible 3.x. The SDK owns all transport dependencies. Preview and Gazebo do not need it.

### 1. Environment Setup

```bash
# Verify ROS 2 environment
source /opt/ros/jazzy/setup.bash
echo $ROS_DISTRO  # should output: jazzy

# Create workspace
mkdir -p <workspace>/src
cd <workspace>
```

**Note**: Replace `<workspace>` with your workspace path, e.g., `~/ghand-ros2`.

### 2. Clone Source and Model Assets

Model assets are provided in a separate repository. Use Gitee by default and clone the Jazzy source branch followed by the model assets:

```bash
cd <workspace>/src

# Clone the Jazzy branch
git clone -b jazzy https://gitee.com/glitech/ghand-ros2.git
# Clone model assets
git clone --depth 1 https://gitee.com/glitech/ghand-assets.git ghand_assets

# GitHub alternatives: replace the corresponding commands above
# Confirm the GitHub source mirror provides jazzy before using it
# git clone -b jazzy https://github.com/gli-sdk/ghand-ros2.git ghand-ros2
# git clone --depth 1 https://github.com/gli-sdk/GHand-Assets.git ghand_assets
```

Verify that the cloned assets contain the GHand5 and Lite1 URDF files. If
the Lite1 files are absent, obtain `ghand_lite1_system` from the product asset
bundle before building.

### 3. Install ROS Dependencies

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash

# Initialize rosdep only when the system-wide source list does not exist.
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi

# Run rosdep update and install as your regular user.
rosdep update
rosdep install --from-paths src/ghand-ros2 \
  --ignore-src -r -y
```

rosdep installs the ros2_control, MoveIt, Gazebo, and libmodbus dependencies
declared by the packages.

### 4. Build

The project includes the SDK-generated Linux x86_64 dynamic library, matching
headers and product JSON under `ghand_hardware/ghand_sdk_vendor/sdk/`.
No local SDK source checkout, SDK build, or external SDK path is required.

From the workspace root:

```bash
./src/ghand-ros2/build.sh
source install/setup.bash
# Preview/Gazebo only, omitting the hardware SDK package:
# ./src/ghand-ros2/build.sh --simulation
```

The hardware plugin links the bundled dynamic library installed into the ROS
workspace. Update the library, headers and JSON together when upgrading SDK.
Other architectures require matching SDK artifacts. Transport build flags are
no longer needed. Use a fresh workspace when migrating to avoid stale installs.

In each new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source <workspace>/install/setup.bash
```
