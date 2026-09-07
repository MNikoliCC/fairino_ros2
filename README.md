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
cd /home/nikolic/colcon_ws/src/fairino_ros2/docker
xhost +local:docker
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
cd /home/nikolic/colcon_ws/src/fairino_ros2/docker
docker compose exec ros2 bash
```

Every Bash shell automatically sources both:

```text
/opt/ros/humble/setup.bash
/ros2_ws/install/setup.bash
```

## Run the FAIRINO Server and MoveIt

Open the first ROS 2 shell and start the FAIRINO command server:

```bash
ros2 run fairino_hardware_v3_9_9 ros2_cmd_server
```

Open a second host terminal and start MoveIt:

```bash
cd /home/nikolic/colcon_ws/src/fairino_ros2/docker
docker compose exec ros2 bash
ros2 launch fairino20_v6_moveit2_config demo.launch.py
```

Important: the retained MoveIt configuration currently uses
`mock_components/GenericSystem`. The demo verifies the ROS 2/MoveIt/controller
stack, but planned trajectories do not yet command SimMachine through the
FAIRINO hardware plugin.

## Stop or Delete the Stack

```bash
# Stop and remove the containers and network; preserve images and caches
./simmachine.sh down

# Remove generated containers, images, network, workspace volumes, and caches
./simmachine.sh delete
```
