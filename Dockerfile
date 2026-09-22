# syncai_backend — one file, four stages, two images that get used.
#
#   base     ROS 2 Humble + this package's pip dependencies. Shared by
#            everything below and by far the most expensive layer (~20 min on
#            aarch64), so nothing that changes often goes in it.
#     ├─ builder  colcon-builds this package and syncai_common into an install
#     │           space. Throwaway: only its /ros2_ws/install survives.
#     ├─ runtime  base + that install space. `ros2 launch`, non-root, no
#     │           compilers, no test tooling. This is what docker-compose runs.
#     └─ dev      base + the ament linters and pytest. Source is bind-mounted,
#                 CMD is pytest. This is the old (and only) image this file
#                 used to build.
#
#   docker build --target runtime -t syncai-backend .   # or: docker compose build
#   docker build --target dev     -t syncai-backend-dev .
#
# There is no default target on purpose — `docker build .` builds the last
# stage, and which stage that is should not be the thing that decides whether
# you get a service or a test harness. Name it.
#
# ── the missing package ──────────────────────────────────────────────────────
#
# This repo imports generated interfaces from two other colcon packages, and
# they arrive by different routes:
#
#   syncai_common  -- `vcs import < interface.repos`, from its own repo, in the
#                     builder stage below. No workspace checkout, no credentials.
#   interface      -- FAST-LIO2's srvs (SaveMaps / ResetMapping / Relocalize /
#                     IsValid). Still inside the private SSH FAST-LIO2 fork, so
#                     naming it in interface.repos would make every import need
#                     credentials. It is NOT in this image.
#
# Missing `interface` used to mean the process could not start at all --
# `gateways/map/map.py` imported `interface.srv` at module level and main.py
# imports that gateway. That import is currently wrapped in a TEMPORARY
# try/except (see the comment on it), so the backend comes up either way and the
# four services it types simply refuse:
#
#   with .interface/     everything works.
#   without              REST, telemetry, point clouds, nav goals, Temporal and
#                        the map catalogue all work; map save, new map, map
#                        switch and relocalize answer with "`interface` is
#                        missing from this image" and the gateway logs a warning
#                        at startup.
#
# The builder stage picks the package up if you drop a copy at `.interface/` in
# the repo root (gitignored):
#
#   cp -r ~/SyncAI-Robot-Workspace/src/third-party/FASTLIO2_ROS2/interface .interface
#
# That is a stopgap, and it is the reason this image still has a string tied to
# a workspace checkout. The real fix is the one syncai_common already had:
# split `interface` into its own repo and add it to interface.repos; the
# try/except goes away with it.
#
# ── dev image usage ──────────────────────────────────────────────────────────
#
# The source is NOT copied into the dev stage. It is bind-mounted, so an edit on
# the host is visible to the next pytest run with no rebuild — the reason to
# rebuild it is a change to requirements.txt, nothing else.
#
#   # Without either package: 284 pass, 6 skip, and 11 files cannot even be
#   # collected because they reach syncai_common through an import rather than
#   # through an importorskip. Useful for the helpers, the repos and the
#   # catalogue; not enough to trust a change to a router or a gateway.
#   docker run --rm -v "$PWD":/ros2_ws/src/syncai_backend syncai-backend-dev
#
#   # With syncai_common only: 625 pass, 2 skip, nothing errors — test_map_gateway
#   # importorskips `interface` and skips as a module, which is the whole of the
#   # difference. vcs clones it into /ros2_ws/src and colcon builds it, a few
#   # seconds; this needs no workspace checkout at all.
#   docker run --rm -v "$PWD":/ros2_ws/src/syncai_backend syncai-backend-dev \
#       bash -lc '
#           cd /ros2_ws
#           vcs import < src/syncai_backend/interface.repos
#           colcon-build --packages-select syncai_common >/dev/null
#           source /ros2_ws/install/setup.bash
#           cd /ros2_ws/src/syncai_backend && python3 -m pytest test/ -q'
#
#   # With both: 659 pass, 1 skips. WS is a SyncAI-Robot-Workspace checkout,
#   # needed for `interface` alone -- mounted read-only, the build writes only
#   # to /ros2_ws. Drop this mount and the map-gateway tests go with it while
#   # everything syncai_common covers still runs.
#   WS=~/SyncAI-Robot-Workspace
#   docker run --rm \
#       -v "$PWD":/ros2_ws/src/syncai_backend \
#       -v "$WS/src/third-party/FASTLIO2_ROS2/interface":/ros2_ws/src/interface:ro \
#       syncai-backend-dev bash -lc '
#           cd /ros2_ws
#           vcs import < src/syncai_backend/interface.repos
#           colcon-build --packages-select syncai_common interface >/dev/null
#           source /ros2_ws/install/setup.bash
#           cd /ros2_ws/src/syncai_backend && python3 -m pytest test/ -q'
#
#   # --rm re-clones on every run. To keep the import AND the build, name both:
#   # `-v syncai-src:/ros2_ws/src -v syncai-ws:/ros2_ws/install`. Or use
#   # `docker run -d --name … sleep infinity` + `docker exec` and import once.
#
#   # A shell to poke around in.
#   docker run --rm -it -v "$PWD":/ros2_ws/src/syncai_backend syncai-backend-dev bash

# =============================================================================
# base — ROS + pip dependencies. The slow, stable half of every image here.
# =============================================================================
#
# ros-base rather than ros-core: rosbag2 (the recording gateway spawns
# `ros2 bag record`) and tf2 come with it, which package.xml's ros2bag and
# tf2_ros exec_depends both want. It also carries colcon, vcstool and a
# compiler, which is why the builder stage installs almost nothing.
FROM ros:humble-ros-base AS base

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Three groups here, and the split is worth keeping straight:
#
#   * ROS interfaces this package imports that ros-base does not carry.
#     nav2_msgs is the one that actually matters (the robot gateway's
#     NavigateToPose action and the map gateway's LoadMap service); the others
#     are cheap and explicit rather than relied on transitively.
#   * rmw_cyclonedds_cpp, which is not optional on this fleet. The rest of the
#     stack runs `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` with a lo-only,
#     multicast-off config (the workspace's config/cyclonedds.xml). A container
#     that falls back to the default fastrtps starts cleanly, logs nothing
#     alarming and sees none of the robot's topics — the worst failure shape
#     there is. Installing it here is half the fix; docker-compose.yml sets the
#     variable and mounts the XML.
#   * open3d's native dependencies. `libGL.so.1` is needed even though nothing
#     here ever renders: open3d links it unconditionally, so without it every
#     `import open3d` fails on a missing shared library while pip insists the
#     package is installed. libgomp is its OpenMP runtime.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        ros-humble-nav2-msgs \
        ros-humble-sensor-msgs-py \
        ros-humble-tf2-ros-py \
        ros-humble-rmw-cyclonedds-cpp \
        libgl1 \
        libgomp1 \
    && rm -rf /var/lib/apt/lists/*

# Copied on its own, ahead of any source, so editing the package does not
# invalidate this layer. requirements.txt is the single source of truth for the
# python deps (it says so itself); nothing is duplicated here.
COPY requirements.txt /tmp/requirements.txt

# open3d is installed separately and is allowed to fail, which is a deliberate
# choice rather than sloppiness. It ships no aarch64 Linux wheel for every
# version in the pinned range, and this image is built on Apple Silicon as often
# as not. The package already treats it as optional at runtime -- helpers/
# traversable.py is imported inside the conversion thread's try and an
# ImportError is recorded as a per-map failure -- so the cost of not having it
# is test_traversable.py skipping and the traversability recipe being
# unavailable. The z-band recipe, which is what every save actually uses, is
# open3d-free by design. Read the warning; do not let it become invisible.
#
# There is no kokoro-onnx step any more, and no alsa-utils above: the speech
# engine and the speaker moved to the syncai_tts container, so this image holds
# neither onnxruntime nor a ~310 MB model. gateways/tts is an httpx client.
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && grep -v '^open3d' /tmp/requirements.txt > /tmp/requirements.core.txt \
    && python3 -m pip install --no-cache-dir -r /tmp/requirements.core.txt \
    && { python3 -m pip install --no-cache-dir "$(grep '^open3d' /tmp/requirements.txt)" \
         || echo "WARNING: open3d unavailable for this platform — test_traversable.py will skip and the traversability recipe is disabled"; }

# =============================================================================
# builder — colcon install space. Nothing but /ros2_ws/install leaves here.
# =============================================================================
FROM base AS builder

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Already present in ros-base; named anyway, because a build that silently
# depends on what the base image happens to ship is a build that breaks on a
# base image bump with an error nobody can place.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-vcstool \
        git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY . /ros2_ws/src/syncai_backend

# Three packages at most, and only one of them is unconditional:
#
#   syncai_common  cloned by vcs from interface.repos (public, no credentials).
#   interface      only if you dropped a copy at .interface/ — see the header.
#                  Without it the image builds and runs, with map save / new map
#                  / map switch refusing. Deliberately not a hard failure here,
#                  so the image stays buildable from a clean clone with no
#                  workspace anywhere.
#   syncai_backend this repo.
#
# No --symlink-install: that flag is what disables setup.py's InstallNoSource,
# and the whole point of the runtime image is that it ships bytecode rather
# than source. Do not add it here to "make rebuilds faster".
RUN source /opt/ros/humble/setup.bash \
    && vcs import < src/syncai_backend/interface.repos \
    && if [ -d src/syncai_backend/.interface ]; then \
           cp -r src/syncai_backend/.interface src/interface; \
       else \
           echo "WARNING: FAST-LIO2 'interface' not in the build context — map save / new map / map switch will refuse at runtime (see the Dockerfile header)"; \
       fi \
    && colcon build --install-base /ros2_ws/install \
    && rm -rf build log src

# =============================================================================
# runtime — what docker-compose runs.
# =============================================================================
FROM base AS runtime

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# HOME is set explicitly and it is load-bearing. Four paths in this package are
# absolute and start with `~` — the system INI, ~/robot_ws/map, ~/robot_ws/record
# and ~/robot_ws/lib/libsyncai_worker.so — and they resolve through
# os.path.expanduser, which reads $HOME first. compose overrides `user:` with a
# numeric uid, and a numeric user with no HOME in the environment would send
# expanduser to the passwd file, or to `/` if the uid has no entry at all. The
# name matches the host account on the robot so paths read the same in both
# places.
ARG USER_UID=1000
ARG USER_NAME=syncrobotic
ENV HOME=/home/${USER_NAME}

# The four mount points are created HERE, owned by the runtime user, rather
# than left to compose. Docker creates a missing bind-mount target as root:root,
# and this process writes to three of them — `ros2 bag record` into record/,
# the gridmap conversion into map/, and the in-place rewrite of system.ini into
# config/. A root-owned mount point turns those into EACCES at the worst moment
# (mid-recording, mid-save) instead of at startup.
RUN useradd --create-home --uid ${USER_UID} ${USER_NAME} \
    && mkdir -p ${HOME}/robot_ws/{config,map,record,lib} \
    && chown -R ${USER_UID}:${USER_UID} ${HOME}

COPY --from=builder --chown=${USER_UID}:${USER_UID} /ros2_ws/install /ros2_ws/install

# Sources ROS, then this package's install space. The stock /ros_entrypoint.sh
# does only the first half, and without the second `ros2 launch syncai_backend`
# finds nothing.
#
# `set -e` without `-u`: ROS's own setup.bash reads AMENT_TRACE_SETUP_FILES
# unguarded, so nounset turns sourcing it into an immediate failure.
RUN printf '%s\n' \
        '#!/bin/bash' \
        'set -e' \
        'source /opt/ros/humble/setup.bash' \
        'source /ros2_ws/install/setup.bash' \
        'exec "$@"' \
        > /usr/local/bin/entrypoint.sh \
    && chmod +x /usr/local/bin/entrypoint.sh

USER ${USER_NAME}
WORKDIR ${HOME}/robot_ws

# Unbuffered so rclpy/uvicorn lines reach `docker logs` promptly rather than
# block-buffering, and uncoloured because these logs are read out of a file as
# often as a terminal. Same pair the workspace's robot services set.
ENV PYTHONUNBUFFERED=1 \
    RCUTILS_COLORIZED_OUTPUT=0

# Documentation only: the service runs with host networking, where EXPOSE does
# nothing. uvicorn binds 0.0.0.0:3000 (server.py).
EXPOSE 3000

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
# The launch file, not `ros2 run`: it reads [system] robot_id out of the INI and
# uses it as the node namespace, and that one value scopes the ROS names, the
# `<robot_id>_db` database and the Temporal task queue. `ros2 run backend` would
# come up as `default_robot` and quietly talk to the wrong database.
CMD ["ros2", "launch", "syncai_backend", "backend.launch.py"]

# =============================================================================
# dev — the test image. Source bind-mounted, CMD is pytest.
# =============================================================================
FROM base AS dev

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# colcon/vcstool as above, plus the three ament linters, because
# test_copyright.py / test_flake8.py / test_pep257.py are part of this suite and
# silently vanish without them — a green run that skipped the linters is exactly
# the kind of false green this image exists to avoid.
#
# python3-pytest comes in anyway as an ament-linter dependency; it is not what
# actually runs the suite (see the pip install of pytest below).
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-vcstool \
        python3-pytest \
        ros-humble-ament-copyright \
        ros-humble-ament-flake8 \
        ros-humble-ament-pep257 \
    && rm -rf /var/lib/apt/lists/*

# `src/` is created HERE rather than being left to the bind mount. Docker
# creates a missing mount parent as root:root at run time, and this image ends
# on `USER dev` -- so `vcs import` writing a sibling package into /ros2_ws/src
# fails with EACCES, while the mount of syncai_backend itself still works and
# hides the cause. The chown further down covers it because it is recursive.
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# pytest from pip rather than the apt python3-pytest that the ament linters drag
# in. Ubuntu 22.04 ships pytest 6.2.5, and anyio -- which arrives with
# fastapi/starlette and registers itself as a pytest11 plugin, so it is loaded
# whether or not a test asks for it -- imports `_pytest.scope`, which did not
# exist before pytest 7. On the apt version the suite does not fail a test, it
# fails to collect at all, with a ModuleNotFoundError out of plugin loading.
#
# pip installs into /usr/local/lib/python3.10/dist-packages, which precedes the
# apt path on sys.path, so `python3 -m pytest` picks this one up.
# Both ends of the range are load-bearing, and each was found by a failing run:
#
#   * the floor, because `_pytest.scope` landed in 7.0. It is also written as a
#     constraint rather than a bare `pytest` because apt's 6.2.5 satisfies an
#     unconstrained requirement -- pip reports "already satisfied", installs
#     nothing, and the collection error survives the fix meant to cure it.
#   * the ceiling, because ROS 2 Humble's own `launch_testing` registers a
#     pytest11 plugin whose `pytest_pycollect_makemodule(path, parent)` uses the
#     `path` argument that pytest 8 removed. It is loaded whether or not this
#     suite uses it, so an unpinned install takes pytest 9 and dies in plugin
#     validation before collecting a single test.
#
# 7.x is the whole of the window where modern anyio and Humble's ROS plugins
# both work.
RUN python3 -m pip install --no-cache-dir "pytest>=7,<8"

# A convenience wrapper rather than something the entrypoint does on its own:
# building takes minutes and most runs of this image do not need it, so it is
# opt-in. --symlink-install because setup.py strips sources on a non-symlink
# install (InstallNoSource) and that would leave the tests importing bytecode —
# the exact opposite of what the runtime stage wants, which is why the two
# builds are separate commands rather than one shared script.
#
# `set -e` without `-u`: ROS's own setup.bash reads AMENT_TRACE_SETUP_FILES
# unguarded, so nounset turns sourcing it into an immediate failure.
RUN printf '%s\n' \
        '#!/bin/bash' \
        'set -eo pipefail' \
        'source /opt/ros/humble/setup.bash' \
        'cd /ros2_ws' \
        'colcon build --symlink-install "$@"' \
        > /usr/local/bin/colcon-build \
    && chmod +x /usr/local/bin/colcon-build

# Sources ROS, then the workspace install space if colcon-build has been run.
# The stock /ros_entrypoint.sh does only the first half, and without the second
# a built syncai_common would be invisible to pytest.
RUN printf '%s\n' \
        '#!/bin/bash' \
        'set -e' \
        'source /opt/ros/humble/setup.bash' \
        'if [ -f /ros2_ws/install/setup.bash ]; then' \
        '  source /ros2_ws/install/setup.bash' \
        'fi' \
        'exec "$@"' \
        > /usr/local/bin/entrypoint.sh \
    && chmod +x /usr/local/bin/entrypoint.sh

# Not root, and this is not hygiene theatre: root bypasses file-permission
# checks entirely, so os.access(path, os.W_OK) answers True even for a mode-0444
# file. test_activate_refuses_when_the_ini_is_not_writable chmods the system INI
# to 0444 and asserts the route refuses the switch -- as root the preflight sees
# a writable INI, the refusal never happens, and the test fails for a reason
# that has nothing to do with the code under test. An image that reports a
# failure the robot does not have is worse than no image.
#
# Override USER_UID at build time if your host uid is not 1000 and bind-mounted
# files come out unwritable.
ARG USER_UID=1000
RUN useradd --create-home --uid ${USER_UID} dev \
    && chown -R dev:dev /ros2_ws
USER dev

# pytest is run as `python3 -m pytest test/` from here, and the `python3 -m`
# half is load-bearing: it puts the repo root on sys.path, which is how
# `import syncai_backend` resolves without the package being installed.
WORKDIR /ros2_ws/src/syncai_backend

# Humble registers two launch-testing plugins as pytest11 entry points, so they
# load whether or not anything here uses them -- and `launch_ros` declares a hook
# that only `launch_testing` provides, so they have to be disabled as a pair or
# pytest dies in plugin validation before collecting anything. Worse, with them
# enabled `launch_testing` eagerly imports every test module looking for launch
# entrypoints, which turns a module that would have been skipped into a fatal
# collection error.
#
# In the environment variable rather than in CMD so it still applies when you
# pass your own pytest arguments, and overridable with `-e PYTEST_ADDOPTS=` if
# you ever do want them.
# --continue-on-collection-errors is the third piece: with neither generated
# interface package present, 11 files reach syncai_common through a plain import
# (not an importorskip) and fail to collect. Left fatal, those 11 abort the run
# and the 284 tests that would have passed never execute. They are still
# reported as errors and the run still exits non-zero -- nothing is hidden, the
# rest just gets to finish. `interface` no longer contributes to that count:
# gateways/map's import of it is guarded (TEMPORARY, see the module), so the
# files that reach it skip rather than error.
ENV PYTEST_ADDOPTS="-p no:launch_testing -p no:launch_ros --continue-on-collection-errors"

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["python3", "-m", "pytest", "test/", "-q"]
