# FAIRINO SimMachine, ROS 2, and RoboPlan

This Compose project runs two containers:

- `fairino-simmachine-v<version>` runs the FAIRINO controller simulator at
  `192.168.58.2`.
- `fairino-ros2-<distro>` provides ROS 2, MoveIt, ros2_control, RViz, RoboPlan
  Core, and the mounted RoboPlan ROS wrappers at `192.168.58.3`.

ROS 2 Humble is the default and currently validated distribution. Jazzy can be
selected through the same build argument, but has intentionally not been tested
yet.

The ROS container runs as the non-root `ros` user with the host user's numeric
UID and GID. The five FAIRINO packages and `roboplan-ros` are mounted read-only
under `/ros2_ws/src`; build, install, and log outputs are stored in named Docker
volumes specific to the selected ROS distribution.

No Cyclone DDS package or `RMW_IMPLEMENTATION` override is configured. The stack
uses the ROS image's default middleware.

## Build and Start the Humble Stack

```bash
cd ~/colcon_ws/src/fairino_ros2/docker
FAIRINO_ROS_DISTRO=humble ./simmachine.sh up
```

`FAIRINO_ROS_DISTRO=humble` is optional because Humble is the default. The first run
downloads SimMachine, builds the ROS image, builds the mounted workspace, and
starts both containers.

Follow the ROS build and inspect the stack with:

```bash
docker compose logs -f ros2
docker compose ps
```

The FAIRINO WebApp is available at <http://192.168.58.2> using `admin` / `123`.

## RoboPlan Layout

RoboPlan Core is cloned and built into `Dockerfile.ros2` at the commit stored in
`roboplan-ros/ROBOPLAN_VERSION` (`ac29aa9` by default). Only the Core packages
needed by the wrappers are included:

- `roboplan`
- `roboplan_simple_ik`

The mounted workspace builds these wrapper packages:

- `roboplan_ros_cpp`
- `roboplan_ros_py`
- `roboplan_ros_visualization`

`roboplan_ros_examples` and `roboplan_ros_franka` are skipped. Consequently,
MuJoCo, the Franka example stack, and their optional dependencies are not
installed.

To intentionally build another pinned Core revision:

```bash
cd ~/colcon_ws/src/fairino_ros2/docker
ROBOPLAN_VERSION=<git-commit> FAIRINO_ROS_DISTRO=humble ./simmachine.sh update
```

## Jazzy Build Option

The Compose and Docker build arguments also accept Jazzy:

```bash
cd ~/colcon_ws/src/fairino_ros2/docker
FAIRINO_ROS_DISTRO=jazzy ./simmachine.sh up
```

This creates `fairino-ros2:jazzy`, the `fairino-ros2-jazzy` container, and
Jazzy-specific workspace volumes. This option is prepared for later use but is
not currently tested or guaranteed to build until the FAIRINO packages are
validated on Jazzy.

## Open a Sourced ROS 2 Shell

```bash
cd ~/colcon_ws/src/fairino_ros2/docker
docker compose exec ros2 bash
```

Every interactive Bash shell automatically sources:

```text
/opt/ros/$ROS_DISTRO/setup.bash
/opt/roboplan_ws/install/setup.bash
/ros2_ws/install/setup.bash
```

You can verify the wrapper packages in that shell with:

```bash
ros2 pkg prefix roboplan_ros_cpp
ros2 pkg prefix roboplan_ros_py
ros2 pkg prefix roboplan_ros_visualization
python3 -c 'import roboplan.core, roboplan.simple_ik, roboplan_ros.cpp'
```

## Run MoveIt with Selectable Hardware

Open a sourced ROS 2 shell and select the hardware backend explicitly:

```bash
# Safe mock hardware
ros2 launch fairino20_v6_moveit2_config demo.launch.py use_mock_hardware:=true

# FAIRINO SDK hardware connected to 192.168.58.2; this is the default
ros2 launch fairino20_v6_moveit2_config demo.launch.py use_mock_hardware:=false
```

When `use_mock_hardware` is `true`, ros2_control loads
`mock_components/GenericSystem`; when it is `false`, it loads
`fairino_hardware/FairinoHardwareInterface` and sends executed trajectories to
the controller.

The optional `ros2_cmd_server` exposes `/fairino_remote_command_service` for
direct string-based FAIRINO SDK commands, so the user can choose it independently
of the MoveIt hardware path.

## Stop or Delete the Stack

```bash
# Stop and remove the containers and network; preserve images and caches
./simmachine.sh down

# Remove all FAIRINO/ROS containers, images, workspace volumes, and caches
./simmachine.sh delete
```
