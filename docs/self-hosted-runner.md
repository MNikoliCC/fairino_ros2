# FAIRINO self-hosted runner

This connects GitHub Actions to a Linux x64 simulation machine. The agent can
update a branch/PR and read Actions logs and artifacts through the GitHub
connection. It does not open an SSH shell, expose ROS ports, or provide continuous
desktop access. The machine must remain online while jobs run.

## 1. Prepare the machine

Use a dedicated Ubuntu simulation machine or VM with Git, Python 3, Docker Engine
and the Docker Compose v2 plugin. ROS Humble is built inside Docker; the host need
not have ROS installed. GPU and desktop access are not required for these checks.
The runner account must be non-root and able to run these without sudo:

```bash
git --version
python3 --version
docker version
docker compose version
docker info
```

The repository is public. Docker access is powerful, and the optional vendor
SimMachine container uses privileged mode, matching the existing stack. Use a
simulation-only host/VM without physical robot access or personal credentials.
Do not add automatic fork PR execution. Only trusted collaborators should push
to branches that execute on this runner. A YAML condition is not a security
boundary against someone who can change the workflow itself.

## 2. Register the runner

Open:

https://github.com/MNikoliCC/fairino_ros2/settings/actions/runners/new

Choose **Linux / x64** and follow GitHub's current download, checksum and extract
commands in a separate directory such as `~/actions-runner-fairino`, outside the
repository. Use GitHub's displayed `./config.sh --url ... --token ...` command.
Append:

```text
--name fairino-sim-01 --labels fairino-sim --work _work
```

Keep the default `self-hosted`, `Linux`, and `X64` labels. The registration token
is entered only on your machine; do not paste it into chat or commit it.

Start in the foreground first:

```bash
./run.sh
```

Wait for `Listening for Jobs` and an **Idle** runner in GitHub. To run as a
service later, stop the foreground runner with Ctrl+C, then from its directory:

```bash
sudo ./svc.sh install
sudo ./svc.sh start
sudo ./svc.sh status
```

Run `svc.sh install` from the intended non-root runner account using sudo. If
Docker access was newly granted, restart the login session/service before testing.

## 3. Enable and run the workflow

Merge the runner setup PR into `main` so GitHub displays **Run workflow**.
Under Settings > Secrets and variables > Actions > Variables, create:

| Variable | Value |
| --- | --- |
| `FAIRINO_RUNNER_ENABLED` | `true` |
| `FAIRINO_CI_SIMMACHINE_IMAGE` | Optional local simulator tag, e.g. `fairino-simmachine:3.9.9` only if that image exists on the runner |

Open Actions > FAIRINO runner > Run workflow. Select `main` and **diagnostics**
first. It checks Docker and records a small host report without dumping environment
variables or inspecting unrelated containers. Then run **mock**.

| Mode | What happens |
| --- | --- |
| `diagnostics` | Host and Docker checks only |
| `mock` | Build the Humble image and checkout, launch MoveIt without RViz with explicit mock hardware, collect 100 valid six-joint messages |
| `simmachine` | Same mock check plus a separate SimMachine container; check HTTP and receive bytes on status port 8083 |

For `simmachine`, first prepare the vendor image using the existing instructions
in `docker/README.md`, or reuse an already loaded image. Verify its exact tag:

```bash
docker image ls --format '{{.Repository}}:{{.Tag}}' --filter 'reference=fairino-simmachine:*'
```

Set `FAIRINO_CI_SIMMACHINE_IMAGE` to one listed tag. CI does not download or
silently upgrade the simulator. It creates its own container from that image.
The first ROS build can take a long time and needs internet access to image,
package and RoboPlan sources. The job has a 90-minute timeout.

## 4. Iterating through PRs

Pushes to `main`, `ci/**`, and `codex/**` run **mock** when enabled. Use a branch
such as `codex/fix-feedback`, open a PR from it, and each new push tests that
branch's commit. There is deliberately no `pull_request`/`pull_request_target`
trigger. A fork PR is not executed by this workflow. This tests the pushed commit,
not GitHub's synthetic PR merge commit.

For a SimMachine check of a PR, use Run workflow and select its branch, then
`simmachine`. The workflow must already exist on `main`. The current chat's
GitHub tools can read runs, logs and artifacts and retry jobs, but do not expose
a general workflow-dispatch action; use the UI for this manual selection.

Send the agent the run URL or PR number. It can inspect results and update the
branch. GitHub does not automatically resume an idle chat when a job finishes.

## 5. Results and isolation

Each run uploads `fairino-<run-id>-<attempt>` for seven days, including
`runner.log`, `result.txt`, and, when available, `launch.log`, `joint_states.json`,
ROS logs, `containers.log`, `cleanup.log`, and `simmachine.json`. These are public
repository artifacts/logs; keep collected data limited to this simulation.

CI uses `ci/compose.yaml`, a unique Compose project/network and a fresh container
build/install filesystem. It mounts the checked-out repository read-only and
writes only to that run's artifact directory. It does not use the interactive
stack's fixed IPs, named volumes, X11 socket, container names or Docker image tag
for ROS. The base image build cache is reused. Jobs are serialized; GitHub may
replace an older pending run with a newer pending run in the concurrency group.

Containers/networks are removed on normal success, error or handled cancellation.
After a hard host crash, inspect leftover `fairino-ci-*` Compose projects and
remove only the affected CI project. Artifact copies under the runner's temporary
directory follow the runner's temporary-file cleanup lifecycle.

## Scope of this first version

The ROS test verifies startup and valid mock joint feedback, not commanded motion,
planning quality or controller tracking. The SimMachine probe verifies HTTP and
an 8083 byte stream, not decoded protocol correctness. It does not connect the
FAIRINO hardware plugin to the simulator: that plugin currently uses the existing
controller addressing, while CI uses Docker service discovery. A later integration
test should make the controller address configurable and validate a bounded
trajectory against SimMachine before testing motion through that backend.

## Troubleshooting

- **Skipped:** `FAIRINO_RUNNER_ENABLED` must be an Actions repository variable
  with exactly `true` as its value.
- **Queued:** runner must be online with all four labels in the workflow.
- **No Run workflow button:** merge the workflow into the default branch first.
- **Docker permission denied:** verify Docker access as the service account.
- **Mock failure:** inspect build output in `runner.log`, then `launch.log` and
  `ros/`. Empty joint data is a failure, not a passing simulation check.
- **SimMachine failure:** verify the image tag exists locally and inspect
  `containers.log` and `simmachine.json`.
- **Stop future jobs:** set `FAIRINO_RUNNER_ENABLED=false`; cancel an active run
  separately if needed.

References:
- https://docs.github.com/en/actions/how-tos/manage-runners/self-hosted-runners/add-runners
- https://docs.github.com/en/actions/reference/workflows-and-actions/events-that-trigger-workflows
