# FAIRINO SimMachine and ROS 2 Humble Stack

This workspace runs two containers through one Compose project:

- `fairino-simmachine-v<version>` runs the FAIRINO controller simulator at
  `192.168.58.2`.
- `fairino-ros2-humble` provides the minimal ROS 2 Humble, MoveIt, ros2_control,
  controller, and RViz environment at `192.168.58.3`.

The host directory `src/fairino_ros2` is mounted read-only at
`/ros2_ws/src/fairino_ros2`. The ROS container builds all retained packages with
`colcon build --symlink-install` whenever it starts. Build, install, and log
outputs are kept in named Docker volumes.

No Cyclone DDS package or `RMW_IMPLEMENTATION` override is configured. This setup
uses the ROS 2 image's default middleware.

## Build and Start Both Containers

```bash
cd ~/docker
xhost +si:localuser:root
./simmachine.sh up
```

The first ROS 2 build downloads its base image and dependencies. Follow the
container status and build output with:

```bash
docker compose ps
docker compose logs -f ros2
```

The FAIRINO WebApp is available at <http://192.168.58.2> using `admin` / `123`.

## Open a Sourced ROS 2 Shell

```bash
cd ~/colcon_ws/src/fairino_ros2/docker
docker compose exec ros2 bash
```

Every Bash shell automatically sources both:

```text
/opt/ros/humble/setup.bash
/ros2_ws/install/setup.bash
```

## Run MoveIt with Selectable Hardware

Open a sourced ROS 2 shell and choose the hardware backend explicitly:

```bash
cd ~/colcon_ws/src/fairino_ros2/docker
docker compose exec ros2 bash

# Safe mock hardware; this is also the default
ros2 launch fairino20_v6_moveit2_config demo.launch.py use_mock_hardware:=true

# FAIRINO SDK hardware connected to 192.168.58.2
ros2 launch fairino20_v6_moveit2_config demo.launch.py use_mock_hardware:=false
```

When `use_mock_hardware` is `true`, ros2_control loads `mock_components/GenericSystem`; when it is `false`, it loads `fairino_hardware/FairinoHardwareInterface` and sends executed trajectories to the controller.

The optional `ros2_cmd_server` exposes `/fairino_remote_command_service` for direct string-based FAIRINO SDK commands, so the user can choose it independently of the MoveIt hardware path.

## Stop or Delete the Stack

```bash
# Stop and remove the containers and network; preserve images and caches
./simmachine.sh down

# Remove generated containers, images, network, workspace volumes, and caches
./simmachine.sh delete
```
