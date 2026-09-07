# FAIRINO SimMachine with Docker Compose

This directory automatically downloads the current **FAIRINO SimMachine Docker**
package from the official FAIRINO download page, extracts
`FAIRINOSimMachine.tar`, loads it into Docker, and starts it through
`compose.yaml`.

When FAIRINO publishes a
newer release and updates its download page, running the same command again will
download and deploy that version.

The container name includes the detected version:

```text
fairino-simmachine-v3.9.9
```

## Requirements

- Linux with Docker Engine and the Docker Compose plugin
- At least 4 GB of RAM, 50 GB of free disk space, and preferably 6 CPU cores
- A user account with permission to run Docker commands
- `curl` and `unzip`

## Start SimMachine

```bash
cd ~/colcon_ws/src/fairino_ros2/docker
./simmachine.sh up
```

The first run downloads approximately 650 MB and may take several minutes. The
ZIP is deleted after successful extraction. Only the approximately 2.1 GB TAR is
kept as a local cache so the Docker image can be restored without downloading
the package again.

The WebApp is available at <http://192.168.58.2>. The default username is
`admin`, and the default password is `123`.

## Update and manage SimMachine

```bash
# Check the official package and deploy a newer version when available
./simmachine.sh update

# Show status
./simmachine.sh status

# Follow logs
./simmachine.sh logs

# Stop and remove the container
./simmachine.sh down

# Remove all generated SimMachine and ROS 2 resources
./simmachine.sh delete
```

`down` removes both current containers and their network while preserving images,
volumes, and caches. `delete` additionally removes all containers whose names
start with `fairino-simmachine-v`, the ROS 2 container and image, the
`fairino-net` network, ROS workspace volumes, local FAIRINO SimMachine image
tags, `.env`, and all cached archives. It preserves `SimMachine/.gitkeep`.

Extracted TAR archives remain in `SimMachine/`, grouped by version. The generated
`.env` file connects Compose to the currently detected version and image tag.

## Override the download source

If FAIRINO changes the structure of its download page, provide a direct Drive
link without editing the script:

```bash
FAIRINO_DRIVE_URL='https://drive.google.com/file/d/DRIVE_ID/view' ./simmachine.sh update
```

The configuration follows the official deployment instructions: the privileged
container runs as `root` on the `fairino-net` bridge network
(`192.168.58.0/24`), using `192.168.58.2` with gateway `192.168.58.1`.

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

## Start the ROS 2 Environment

`./simmachine.sh up` starts both SimMachine and the ROS 2 Humble container. The
ROS workspace is built automatically from the packages mounted from
the parent `fairino_ros2` repository.

Allow local Docker containers to use the X server before opening RViz:

```bash
xhost +si:localuser:root
```

Open a shell in the ROS 2 container:

```bash
docker compose exec ros2 bash
```

Choose the MoveIt hardware backend explicitly:

```bash
# Safe mock hardware
ros2 launch fairino20_v6_moveit2_config demo.launch.py use_mock_hardware:=true

# FAIRINO SDK hardware connected to 192.168.58.2; this is also the default
ros2 launch fairino20_v6_moveit2_config demo.launch.py use_mock_hardware:=false
```

When `use_mock_hardware` is `true`, ros2_control loads `mock_components/GenericSystem`; when it is `false`, it loads `fairino_hardware/FairinoHardwareInterface` and sends executed trajectories to the controller.

The optional `ros2_cmd_server` exposes `/fairino_remote_command_service` for direct string-based FAIRINO SDK commands, so the user can choose it independently of the MoveIt hardware path.
