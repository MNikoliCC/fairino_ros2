# FAIRINO SimMachine with Docker Compose

This directory downloads the current FAIRINO SimMachine Docker package from the
official FAIRINO download page, extracts `FAIRINOSimMachine.tar`, loads it into
Docker, and starts it together with the ROS 2 environment through `compose.yaml`.

When FAIRINO publishes a newer release and updates its download page, running the
same command again downloads and deploys that version. The SimMachine container
name includes the detected version, for example:

```text
fairino-simmachine-v3.9.9
```

## Requirements

- Linux with Docker Engine and the Docker Compose plugin
- At least 4 GB of RAM, 50 GB of free disk space, and preferably 6 CPU cores
- A user account with permission to run Docker commands
- `curl` and `unzip`

## Start SimMachine and ROS 2 Humble

```bash
cd ~/colcon_ws/src/fairino_ros2/docker
FAIRINO_ROS_DISTRO=humble ./simmachine.sh up
```

Humble is the default, so `FAIRINO_ROS_DISTRO=humble` may be omitted. The first run may
take several minutes because it downloads SimMachine and builds the ROS image,
RoboPlan Core, the FAIRINO packages, and the RoboPlan ROS wrappers.

The SimMachine ZIP is deleted after successful extraction. Only the extracted
TAR remains as a cache so its Docker image can be restored without downloading
the package again.

The WebApp is available at <http://192.168.58.2>. The default username is
`admin`, and the default password is `123`.

## ROS Distribution and RoboPlan

The ROS image uses these Compose build arguments:

- `FAIRINO_ROS_DISTRO`, passed to the Docker `ROS_DISTRO` build argument and defaulting to `humble`; `jazzy` is accepted but untested.
- `ROBOPLAN_VERSION`, defaulting to the pinned RoboPlan Core commit `ac29aa9`.
- `ROS_UID` and `ROS_GID`, populated from the host user by `simmachine.sh`.

RoboPlan Core and `roboplan_simple_ik` are built into the image. The host
`roboplan-ros` directory is mounted at `/ros2_ws/src/roboplan_ros`, and its C++,
Python, and visualization wrappers are built at container startup. The generic
and Franka examples are skipped, so MuJoCo and Franka-specific dependencies are
not installed.

The Jazzy option is available for future validation:

```bash
FAIRINO_ROS_DISTRO=jazzy ./simmachine.sh up
```

## Open the ROS 2 Environment

```bash
docker compose exec ros2 bash
```

The container runs as the non-root `ros` user. The X11 socket and host `DISPLAY`
are passed directly, so no `xhost` command is required for RViz when the host X
session permits the matching host UID.

Every interactive Bash shell sources ROS, the image-baked RoboPlan underlay, and the mounted
workspace overlay.

Choose the MoveIt hardware backend explicitly:

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

## Update and Manage the Stack

```bash
# Check the official SimMachine package and deploy a newer version
./simmachine.sh update

# Show status
./simmachine.sh status

# Follow SimMachine logs
./simmachine.sh logs

# Stop and remove the containers
./simmachine.sh down

# Remove all generated SimMachine and ROS 2 resources
./simmachine.sh delete
```

`down` removes the current containers and network while preserving images,
workspace volumes, and caches. `delete` additionally removes every container and
image created by this stack (including Humble and Jazzy variants), the network,
all matching ROS workspace volumes, `.env`, and cached archives. It preserves
`SimMachine/.gitkeep`.

The generated `.env` records the detected SimMachine version, selected ROS
distribution, RoboPlan revision, display, and host UID/GID for Compose.

## Override the SimMachine Download Source

If FAIRINO changes the structure of its download page, provide a direct Drive
link without editing the script:

```bash
FAIRINO_DRIVE_URL='https://drive.google.com/file/d/DRIVE_ID/view' ./simmachine.sh update
```

The privileged SimMachine container runs as `root` on the `fairino-net` bridge
network (`192.168.58.0/24`), using `192.168.58.2` with gateway `192.168.58.1`.
The ROS container remains non-root at `192.168.58.3`.

## Change the Robot Model

To change the robot model in the SimMachine WebApp:

1. Disable the robot.
2. Open **System → Maintenance**.
3. Enter the maintenance password:

   ```text
   frrts2021
   ```

4. Select and confirm the required robot model.
5. Enable the robot again.
