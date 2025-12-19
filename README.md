# Scout + xArm6 Complete Package

<div align="center">

![Scout + xArm6 Simulation Demo](docs/images/output.gif)

*Demonstration of the Scout mobile robot with xArm6 robotic arm, gripper, and RealSense D435i camera. The camera is mounted on link6 above the gripper for eye-in-hand manipulation. The simulation shows the robot's sensor visualizations, including the laser scan field of view and camera capabilities.*

**A self-contained ROS package for Gazebo simulation of AgileX Scout mobile base with xArm6 robotic arm, gripper, and RealSense D435i camera.**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS Melodic](https://img.shields.io/badge/ROS-Melodic-22314E)](https://www.ros.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-9.0-orange)](http://gazebosim.org/)

</div>

## What's Included

This package contains **everything** needed to run the complete robot simulation in Gazebo:
- Scout v2 mobile base URDF and meshes
- xArm6 robotic arm URDF and meshes  
- xArm gripper URDF and meshes
- RealSense D435i camera URDF
- All controller configurations (base, arm, gripper)
- All launch files
- Utility scripts (teleoperation, initialization)
- Gazebo world files
- No external dependencies on other custom packages!

## Quick Start

### 1. Install System Dependencies

```bash
# ROS Melodic (Ubuntu 18.04)
sudo apt-get update
sudo apt-get install -y \
  ros-melodic-gazebo-ros-pkgs \
  ros-melodic-gazebo-ros-control \
  ros-melodic-joint-trajectory-controller \
  ros-melodic-position-controllers \
  ros-melodic-velocity-controllers \
  ros-melodic-effort-controllers \
  ros-melodic-robot-state-publisher \
  ros-melodic-joint-state-publisher \
  ros-melodic-xacro \
  ros-melodic-controller-manager \
  ros-melodic-tf \
  ros-melodic-tf2-ros \
  ros-melodic-cv-bridge \
  ros-melodic-image-transport
```

### 2. Install Gazebo Plugins

```bash
# Install RealSense Gazebo plugin
cd ~/catkin_ws/src
git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git
cd ~/catkin_ws
catkin_make

# Install roboticsgroup_gazebo_plugins (for mimic joints)
cd ~/catkin_ws/src
git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
cd ~/catkin_ws
catkin_make
```

### 3. Setup Package

```bash
# Clone or copy this package to your catkin workspace
cd ~/catkin_ws/src
# If using git:
# git clone <your-repo-url> scout_xarm_complete
# Or copy the folder:
# cp -r /path/to/scout_xarm_complete ~/catkin_ws/src/

# Build
cd ~/catkin_ws
catkin_make

# Source
source ~/catkin_ws/devel/setup.bash
```

### 4. Launch Simulation

```bash
# Launch Gazebo with robot
roslaunch scout_xarm_complete scout_with_xarm_empty_world.launch
```

## Docker Setup (Alternative for Non-Ubuntu 18.04 Users)

If you're not using Ubuntu 18.04, you can run the simulation in a Docker container. This provides a complete ROS Melodic environment with all dependencies pre-installed.

### Prerequisites

- Docker and Docker Compose installed
- NVIDIA Docker runtime (for GPU support and Gazebo visualization)
- X11 server running (for GUI applications like Gazebo and RViz)

### Docker Setup Files

Create the following files in your project root:

**docker/compose.yaml:**
```yaml
services:
  melodic:
    build:
      context: ..
      dockerfile: docker/melodic.Dockerfile
    image: scout_xarm_melodic:latest
    container_name: scout_xarm_melodic
    network_mode: host
    privileged: true
    tty: true
    stdin_open: true
    gpus: all
    group_add:
      - video
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_MASTER_URI=http://YOUR_DEVICE_IP:11311  # Replace with your device IP
      - ROS_IP=YOUR_DEVICE_IP  # Replace with your device IP
      - XAUTHORITY=/home/ros/.Xauthority
      - QT_X11_NO_MITSHM=1
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
    volumes:
      - ../catkin_ws:/home/ros/catkin_ws  # Mount your catkin workspace
      - /tmp/.X11-unix:/tmp/.X11-unix
      - ${XAUTHORITY:-$HOME/.Xauthority}:/home/ros/.Xauthority:ro
      - /etc/localtime:/etc/localtime:ro
      - /etc/timezone:/etc/timezone:ro
      - /dev/dri:/dev/dri
    command: ["sleep", "infinity"]
```

**docker/melodic.Dockerfile:**
```dockerfile
FROM osrf/ros:melodic-desktop-full

ENV ROS_DISTRO=melodic

RUN apt-get update && \
    apt-get install -y \
        build-essential \
        python-catkin-tools \
        sudo \
        git \
        curl \
        wget \
        libgl1-mesa-dri \
        mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install ROS packages needed for simulation
RUN apt-get update && \
    apt-get install -y \
        ros-melodic-gazebo-ros-pkgs \
        ros-melodic-gazebo-ros-control \
        ros-melodic-joint-trajectory-controller \
        ros-melodic-position-controllers \
        ros-melodic-effort-controllers \
        ros-melodic-velocity-controllers \
        ros-melodic-robot-state-publisher \
        ros-melodic-joint-state-publisher \
        ros-melodic-xacro \
        ros-melodic-controller-manager \
        ros-melodic-tf \
        ros-melodic-tf2-ros \
        ros-melodic-cv-bridge \
        ros-melodic-image-transport \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -ms /bin/bash ros && \
    echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    mkdir -p /home/ros/catkin_ws && \
    chown -R ros:ros /home/ros

USER ros
WORKDIR /home/ros

# Create catkin workspace
RUN mkdir -p /home/ros/catkin_ws/src && \
    /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_init_workspace src"

# Setup bashrc
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    echo "source ~/catkin_ws/devel/setup.bash 2>/dev/null" >> ~/.bashrc && \
    echo "cd ~/catkin_ws" >> ~/.bashrc

WORKDIR /home/ros/catkin_ws

CMD ["/bin/bash"]
```

### Build and Run Docker Container

**1. Allow X11 access (run once on host):**
```bash
xhost +local:docker
```

**2. Build the Docker image:**
```bash
cd /path/to/scout_xarm_complete
docker compose -f docker/compose.yaml build
```

**3. Start the container:**
```bash
docker compose -f docker/compose.yaml up -d
```

**4. Enter the container:**
```bash
docker compose -f docker/compose.yaml exec melodic bash
```

**5. Setup the workspace (on host or inside container):**

**Option A: On host (recommended - files persist):**
```bash
# On your host machine, copy package to catkin workspace
cd /path/to/catkin_ws/src
cp -r /path/to/scout_xarm_complete .

# Install Gazebo plugins
git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git
git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
```

**Option B: Inside container:**
```bash
# Enter container
docker compose -f docker/compose.yaml exec melodic bash

# Inside container, copy package to workspace
cd ~/catkin_ws/src
cp -r /path/to/scout_xarm_complete .  # Adjust path as needed

# Install Gazebo plugins
git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git
git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
```

**6. Build workspace (inside container):**

```bash
# Enter container if not already inside
docker compose -f docker/compose.yaml exec melodic bash

# Build workspace
cd ~/catkin_ws
catkin_make

# Source workspace
source devel/setup.bash
```

**7. Launch simulation:**
```bash
roslaunch scout_xarm_complete scout_with_xarm_empty_world.launch
```

### Docker Management Commands

```bash
# Stop container
docker compose -f docker/compose.yaml down

# View container logs
docker compose -f docker/compose.yaml logs

# Restart container
docker compose -f docker/compose.yaml restart

# Remove container and image
docker compose -f docker/compose.yaml down --rmi all
```

### Docker Notes

- **X11 Forwarding**: The container uses X11 forwarding to display Gazebo and RViz. Make sure your DISPLAY environment variable is set correctly.
- **GPU Support**: The container has GPU access enabled for hardware-accelerated rendering in Gazebo.
- **Network Mode**: Uses `host` networking mode for ROS communication. This allows ROS nodes to communicate directly.
- **Volume Mounts**: The catkin workspace is mounted as a volume, so changes persist between container restarts.
- **User Permissions**: The container runs as user `ros` (not root) for security.

### Troubleshooting Docker Setup

**Gazebo window doesn't appear:**
- Check X11 forwarding: `echo $DISPLAY` should show your display
- Verify xhost permissions: `xhost`
- Try: `xhost +local:docker`

**Container won't start:**
- Check Docker logs: `docker compose -f docker/compose.yaml logs`
- Verify NVIDIA Docker runtime: `docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi`
- Verify ROS_MASTER_URI and ROS_IP are set correctly in compose.yaml (use your device IP, not localhost)

**Permission denied errors:**
- The container runs as user `ros`. Use `sudo` inside container if needed.

## Package Structure

```
scout_xarm_complete/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package manifest
├── README.md               # This file
├── urdf/                   # All URDF/XACRO files
│   ├── scout/             # Scout base URDF
│   ├── xarm6/             # xArm6 arm URDF
│   ├── gripper/           # Gripper URDF
│   ├── camera/            # RealSense camera URDF
│   ├── base/              # Base mounting box URDF
│   └── scout_with_box.xacro  # Main robot URDF (includes everything)
├── meshes/                # All 3D meshes
│   ├── scout/             # Scout base meshes
│   ├── xarm6/             # xArm6 meshes
│   └── gripper/           # Gripper meshes
├── config/                # Configuration files
│   ├── controllers/       # Controller configs
│   └── pid/              # PID gains
├── launch/                # Launch files
│   ├── scout_with_xarm_empty_world.launch  # Main launch file
│   └── spawn_scout_with_box.launch        # Spawn robot
├── scripts/               # Python scripts
│   ├── xarm6_keyboard_teleop.py    # Arm teleoperation
│   ├── xarm6_init_home.py          # Initialize arm position
│   └── gripper_init_position.py    # Initialize gripper
└── worlds/                # Gazebo world files
```

## Usage

### Launch Complete Simulation

```bash
roslaunch scout_xarm_complete scout_with_xarm_empty_world.launch
```

This will:
- Start Gazebo with empty world
- Spawn Scout + xArm6 + Gripper + Camera
- Load all controllers
- Initialize arm to home position
- Start robot state publisher

### Control the Robot

**Arm Teleoperation:**
```bash
rosrun scout_xarm_complete xarm6_keyboard_teleop.py
```

**Base Teleoperation:**
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel
```

### View Topics

```bash
# Check all topics
rostopic list

# Check joint states
rostopic echo /joint_states

# Check camera images
rostopic echo /realsense_gazebo_camera/color/image_raw

# Check arm controller
rostopic echo /xarm6_traj_controller/state
```

## Configuration

### Controller Topics

- **Base:** `/cmd_vel` (Twist)
- **Arm:** `/xarm6_traj_controller/command` (JointTrajectory)
- **Gripper:** `/gripper_traj_controller/command` (JointTrajectory)

### TF Frames

- `base_link` : Scout base
- `odom`  Odometry frame
- `arm_base` : Arm mounting point
- `link_eef` : End effector (gripper attachment)
- `camera_link` : RealSense camera

## Notes

### Path Updates

All paths in this package have been updated to use `scout_xarm_complete` package:
- `$(find scout_xarm_complete)/...` instead of `$(find scout_xarm_description)/...`
- `$(find scout_xarm_complete)/...` instead of `$(find xarm_gripper)/...`
- Mesh paths updated to use `package://scout_xarm_complete/meshes/...`

### Namespace

- Controllers are in **root namespace** (not `/xarm`)
- Joint states published to `/joint_states` (root namespace)
- This matches the Scout base controller setup

### Dependencies

This package is **self-contained** but still requires:
- ROS Melodic
- Gazebo 9
- Standard ROS packages (listed in package.xml)
- RealSense Gazebo plugin (for camera simulation)
- roboticsgroup_gazebo_plugins (for gripper mimic joints)

## Troubleshooting

### Camera not publishing

Install RealSense Gazebo plugin (see step 2 above).

### Gripper "dancing"

Check that `roboticsgroup_gazebo_plugins` is installed and built.

### Controllers not found

Ensure all ROS controller packages are installed:
```bash
sudo apt-get install ros-melodic-joint-trajectory-controller ros-melodic-position-controllers
```

### Mesh files not found

Check that meshes are installed correctly:
```bash
ls ~/catkin_ws/devel/share/scout_xarm_complete/meshes/
```

## SLAM, Navigation, and Exploration

The package includes autonomous navigation and exploration capabilities for Gazebo simulation using SLAM (gmapping), move_base navigation stack, and explore_lite for autonomous exploration.

### Running Navigation in Gazebo

**1. Start Gazebo with robot:**
ash
roslaunch scout_xarm_complete scout_with_xarm_empty_world.launch

**2. Launch navigation and exploration (in another terminal):**
roslaunch scout_xarm_navigation navigation_exploration_docker.launch

This launches:
- **SLAM (gmapping)**: Real-time map building from laser scans
- **Navigation (move_base)**: Path planning with TEB local planner and GlobalPlanner
- **Exploration (explore_lite)**: Automatically sends navigation goals to frontiers for autonomous exploration

### How It Works

When exploration is active, `explore_lite` automatically:
- Detects frontiers (boundaries between known/unknown areas)
- Selects nearby frontiers as exploration goals
- Sends goals to `move_base` to navigate to frontiers
- Repeats until the accessible area is mapped

### Prerequisites

sudo apt-get install -y \
  ros-melodic-navigation \
  ros-melodic-gmapping \
  ros-melodic-teb-local-planner \
  ros-melodic-explore-lite


## Credits

This package consolidates:
- Scout URDF from AgileX Scout description packages
- xArm6 URDF from xArm-Developer/xarm_ros
- RealSense camera from xArm-Developer/xarm_ros
- Integration work by the development team

## License

MIT License - See LICENSE file for details

## Contributing

Feel free to star this repository if you find it useful, or fork it to customize for your own projects. Contributions, issues, and pull requests are welcome!

## Contributor

Solomon Chibuzo Nwafor
