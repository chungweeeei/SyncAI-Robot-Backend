# Build and run

> Part of the `syncai_backend` reference — see the [documentation index](../README.md#documentation).

Builds run **inside the robot container** — see the workspace `CLAUDE.md`. Workspace-relative paths in these docs (`src/syncai_backend/…`, `.env`) assume this repo is vcs-imported at `<workspace>/src/syncai_backend`.

```bash
colcon build --packages-select syncai_backend --symlink-install
source install/setup.bash
```

Python deps are **not** rosdep-managed (jammy has no reliable key for fastapi);
`requirements.txt` is the single source of truth, used by every Docker stage:

```bash
pip install -r src/syncai_backend/requirements.txt
```

Run it:

```bash
ros2 launch syncai_backend backend.launch.py                     # namespaced from config/system.ini
ros2 launch syncai_backend backend.launch.py system_config:=config/instances/robot.ini
ros2 run syncai_backend backend                                  # no namespace -> default_robot
```

In practice `NodeManager` starts it from **both** session specs,
`config/sessions/start_nav.yaml` and `start_mapping.yaml`, in the `backend`
window (pane 2, behind `robot_state`, `sleep: 2`). It is in the mapping spec
because the operator console *is* the mapping UI — mode switching, teleop, the
live cloud and save-map all go through it. It still hard-requires postgres, which
lives in the infra compose stack and is up regardless of session.

> `setup.py` installs **compiled bytecode only** (`InstallNoSource`): after a
> normal install it byte-compiles the package and deletes the `.py` sources from
> the install space. The step self-disables for symlinked modules, so
> `--symlink-install` builds are unaffected.

## As its own container

`Dockerfile` has four stages — `base` (ROS + pip deps, the expensive one),
`builder` (the colcon install space), `runtime` (the service) and `dev` (the test
image). `docker-compose.yml` runs `runtime` as one service. There is no default
target; name it.

### Step by step

**1. Before the first start.** postgres and temporal (the workspace's infra
stack), syncai_tts (SyncAI-TTS) and the ROS side (the robot container) must
already be up; compose starts none of them.

```bash
# The infra, if it is not up.
docker compose -f ~/SyncAI-Robot-Workspace/docker-compose.yml up -d postgres temporal

# The WebRTC worker, built in SyncAI-WebRTC-Worker and copied in by hand.
# Optional: without it the camera stream 502s and everything else works.
cp /path/to/libsyncai_worker.so lib/

# Stop the byobu-pane backend inside the robot container first (see
# "One backend at a time" below) — two backends on one robot is an outage.
```

**2. Build the image.** The builder stage clones syncai_common from
`interface.repos` (HTTPS, no credentials); nothing needs copying in. `base` is
slow (~20 min on aarch64 the first time) and only rebuilds on a
`requirements.txt` change (or a different `PIP_INDEX_URL`).

Nearly all of that time is pip **downloading**, not building — every wheel is
prebuilt for aarch64. From some networks `files.pythonhosted.org` crawls at
40–70 kB/s (a 30 MB scipy wheel took 10 min), so a build that looks stuck in
`[base 4/4]` usually is just slow. Point it at a mirror instead:

```bash
PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple docker compose build
```

pip's cache is a BuildKit cache mount, so a cancelled build does not throw away
the wheels it already fetched; the next run picks up from there.

```bash
docker compose build                                    # tags syncai-backend, USER_UID=${UID:-1000}
# or, without compose — name the target, there is no default:
docker build --target runtime --build-arg USER_UID=$(id -u) -t syncai-backend .
```

`USER_UID` must own the host's `config/`, `map/` and `record/`, or the container
user cannot write them.

Or skip the build: every push to `dev` publishes the runtime image (arm64 only,
`USER_UID=1000`) through `.github/workflows/cd-dev.yml` as
`ghcr.io/chungweeeei/syncai-backend:dev` and an immutable `:<short-sha>-dev`
to roll back to. Compose still names `syncai-backend`, so retag it:

```bash
docker pull ghcr.io/chungweeeei/syncai-backend:dev
docker tag  ghcr.io/chungweeeei/syncai-backend:dev syncai-backend
docker compose up -d                # without --build, or compose rebuilds it
```

The GHCR package starts private: make it public in its settings, or
`docker login ghcr.io` on the robot with a token that has `read:packages`.

**3. Run it.** Compose is the supported way — host networking, `ipc: host`, the
nvidia runtime, camera devices, mounts and environment, each commented in
`docker-compose.yml`.

```bash
docker compose up -d                # add --build to rebuild first
docker compose logs -f              # rclpy + uvicorn + Temporal worker, one stream
```

Optional knobs, in the shell or a `.env` beside the compose file:

| variable | default | what it moves |
|---|---|---|
| `ROBOT_WS` | `~/SyncAI-Robot-Workspace` | where `config/` and `map/` come from |
| `ROBOT_INSTANCE` | `robot` | which `config/instances/<name>.ini` is mounted as `system.ini` (the workspace's file is `robot.ini` since 2026-10-28, `robot01.ini` before; a name with no file mounts an empty directory, silently) |
| `RECORD_DIR` | `./record` | where bags land on the host |
| `WEBRTC_LIB` | `./lib/libsyncai_worker.so` | the WebRTC worker `.so` |
| `POSTGRES_USER` / `POSTGRES_PASSWORD` | `postgres` / `postgres` | the infra stack's credentials |
| `UID` / `GID` / `VIDEO_GID` | `1000` / `1000` / `44` | container user and the host's `video` group (`getent group video`) |
| `PIP_INDEX_URL` | `https://pypi.org/simple` | build only: the PyPI index the base stage installs from (a mirror when PyPI is slow) |

```bash
ROBOT_INSTANCE=other RECORD_DIR=/mnt/ssd/record docker compose up -d
```

**4. Check it.** `/health` is always 200; the body says `ok` or `degraded`
(e.g. Temporal down). The healthcheck turns healthy within 120 s (postgres is
retried 20 × 5 s).

```bash
docker compose ps                   # STATUS: healthy
curl http://127.0.0.1:3000/health
docker exec syncai_backend /usr/local/bin/entrypoint.sh ros2 node list   # /<robot_id>/... present
```

A node named `default_robot` means the per-robot INI was not mounted; an empty
`ros2 topic list` in here means check `ROS_DOMAIN_ID` / the RMW (below and in the
compose file).

**5. Stop / restart.**

```bash
docker compose restart              # e.g. after changing .env or the WebRTC .so
docker compose down                 # stop and remove the container
```

**Without compose.** A plain `docker run` must reproduce the compose service;
this equivalent drifts the moment the compose file changes, so prefer compose:

```bash
WS=${ROBOT_WS:-$HOME/SyncAI-Robot-Workspace}
R=/home/syncrobotic/robot_ws
docker run -d --name syncai_backend --restart unless-stopped \
    --network host --ipc host --runtime nvidia \
    --user "$(id -u):$(id -g)" --group-add "$(getent group video | cut -d: -f3)" \
    --workdir $R --stop-timeout 30 \
    -e ROS_DOMAIN_ID=1 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=$R/config/cyclonedds.xml -e ROS_LOG_DIR=/tmp/ros_log \
    -e POSTGRES_HOST=127.0.0.1 -e POSTGRES_PORT=5432 \
    -e POSTGRES_USER=postgres -e POSTGRES_PASSWORD=postgres \
    -e TEMPORAL_ADDRESS=127.0.0.1:7233 -e TTS_SERVICE_URL=http://127.0.0.1:9090 \
    -e STUN_SERVERS= -e VIDEO_FORMAT=rtpjpeg -e VIDEO_FRAMERATE=30 \
    -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
    --device /dev/video0 --device /dev/video1 --device /dev/video2 --device /dev/video3 \
    -v /dev/syncai:/dev/syncai:ro \
    -v $WS/config:$R/config \
    -v $WS/config/instances/robot.ini:$R/config/system.ini \
    -v $WS/map:$R/map \
    -v "$PWD/record":$R/record \
    -v "$PWD/lib/libsyncai_worker.so":$R/lib/libsyncai_worker.so:ro \
    --tmpfs /tmp/ros_log:size=128m,mode=1777 \
    syncai-backend
```

Drop `--device` lines for unplugged cameras (`docker run` refuses a missing
device), and `--runtime nvidia` off a Jetson.

The robot's `config/instances/robot.ini` is mounted **over**
`config/system.ini` as a single file (the checked-out `system.ini` is an empty
placeholder), as the workspace's compose does. Miss it and there is no `[system]
robot_id`: namespace, database and task queue all become `default_robot`.

The compose file comments every piece of the service; the ones that bite:

- **`network_mode: host`.** DDS discovery with the nav stack runs over `lo` with
  multicast off (`config/cyclonedds.xml`), so both sides share the host's network
  namespace. Compose service names therefore do not resolve — postgres, temporal
  and syncai_tts are `127.0.0.1:<published port>` — and the API lands on the
  host's `:3000` with no `ports:` mapping.
- **`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`**, installed in the image. On the
  default fastrtps the backend starts cleanly, logs nothing alarming and sees not
  one topic.
- **Three bind mounts from `ROBOT_WS`** carry what this package shares with the
  stack: `config/` (read-write — activating a map rewrites `system.ini` in place;
  `instances/robot.ini` goes over `config/system.ini` as one file) and `map/`
  (must be the directory the nav stack's localizer and Octo3D read). **`record/` and
  `lib/libsyncai_worker.so` are not among them**: nothing else reads a bag or
  dlopens the worker, so both default beside the compose file — moved with
  `RECORD_DIR` and `WEBRTC_LIB`. Both are tracked as empty directories
  (`.gitkeep`; the `.so` is gitignored) so Docker never creates a bind source
  itself: one it creates is `root:root`, unwritable for the uid-1000 user, and a
  missing *single-file* source becomes a directory that dlopen then fails on.
- **`ipc: host`.** pgo hands over the map cloud as a PCD under the host's
  `/dev/shm/syncai_pgo/<robot_id>` (see [*ROS interfaces*](ros-interfaces.md)); the path only names
  the same file if both containers share the host's IPC namespace (the
  workspace's robot service sets it too). Not a bind mount of a `/dev/shm`
  subdirectory, for the `root:root` reason above.
- **One backend at a time.** `NodeManager` starts this process as a byobu pane
  in the robot container. Running the service as well gives two processes on
  `:3000` and, less visibly, two Temporal workers polling the same
  `<robot_id>.ROBOT_TASK_QUEUE`. Take it out of the session spec first.

Knock-on for `switch_mode`: as a byobu pane the process dies with the session and
`NodeManager` restarts it — what the route's "success looks like a dropped
connection" rests on. In its own container it survives the teardown, and
`restart: unless-stopped` only covers a crash, so a mode switch no longer
recycles it.
