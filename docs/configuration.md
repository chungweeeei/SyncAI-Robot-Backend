# Configuration

> Part of the `syncai_backend` reference — see the [documentation index](../README.md#documentation).

| Env var | Default | Used by |
|---|---|---|
| `TEMPORAL_ADDRESS` | `127.0.0.1:7233` | Temporal client + worker |
| `POSTGRES_HOST` | `localhost` | `database/postgres.py` |
| `POSTGRES_PORT` | `5432` | ditto |
| `POSTGRES_USER` | `syncrobotic` | ditto |
| `POSTGRES_PASSWORD` | `syncrobotic` | ditto |
| `SYNCAI_SYSTEM_INI` | `~/robot_ws/config/system.ini` | `helpers/system_config.py` (per-robot INI reads, e.g. `[map]`) |
| `INTERNAL_API_URL` | `http://127.0.0.1:8090` | `gateways/internal_api` — the robot's internal REST service (`/internal/maps`), told of every map switch |
| `TTS_SERVICE_URL` | `http://syncai_tts:8080` | `gateways/tts` — the syncai_tts container by compose service name; `http://127.0.0.1:8080` with the backend on the host (the compose service uses `http://127.0.0.1:9090`, the port syncai_tts publishes) |
| `LOG_LEVEL` | `info` | `logger.py`. Any stdlib level name; an unknown one logs a warning and keeps `info`. It was unfiltered (`NOTSET`) until 2026-10 |
| `LOG_FORMAT` | `console` | `logger.py`. `json` for one object per line (ISO UTC timestamps, tracebacks as text) for a log collector; `console` is the coloured human one |
| `SYNCAI_WEBRTC_LIB` | `~/robot_ws/lib/libsyncai_worker.so` | `gateways/webrtc` — the Go worker to dlopen. A nonexistent path is the only way to disable WebRTC: a loaded c-shared library cannot be unloaded |
| `VIDEO_RTP_PORT` · `VIDEO_AUDIO_RTP_PORT` · `AUDIO_RTP_PORT` · `STUN_SERVERS` | `5006` · `5007` · `5004` · `stun:stun.l.google.com:19302` | Read by the **worker**, not Python; `gateways/webrtc` only `setdefault`s them right before `InitWorker`, which freezes them for the process. The compose service sets `STUN_SERVERS=""` (LAN robot), `VIDEO_FORMAT=rtpjpeg`, so video comes from the camera feeder's RTP/JPEG on `127.0.0.1:5008` instead of each session opening `/dev/syncai/camera0` itself (which fails "busy" while the feeder holds it), and `VIDEO_FRAMERATE=30`, the feeder's rate with stabilization on, which under `rtpjpeg` only sets the half-second IDR interval. `TURN_SERVERS` / `TURN_USERNAME` / `TURN_PASSWORD` have no default on purpose — `.env` is their only source |

`.env` in the workspace root is loaded via `python-dotenv` at import time.

`[system] robot_id` comes from the system INI (read by the launch file), not the
environment. This package resolves that INI by an **absolute** default
(`~/robot_ws/config/system.ini`) in both `launch/backend.launch.py` and
`helpers/system_config.py`: the old relative `config/system.ini` only worked
because every entrypoint happened to run from the workspace root, and tests and
shells do not. Override with `system_config:=` or `SYNCAI_SYSTEM_INI`.

`set_active_map` (behind `POST /api/v1/maps/{name}/activate`) rewrites that INI
**in place** — it is a single-file bind mount, so a temp file + rename would
leave the container on the old inode — and in-place means truncate, then write.
Before it does, it copies the previous content to **`system.ini.bak`** beside
it (tmp + rename; a sibling file can be atomic). A rewrite that fails its
verifying re-read puts the previous content back itself; a process killed inside
the write (switch_mode kills the byobu session this process is a pane of, and
an OOM kill or a power cut will do) leaves an empty INI, which
`active_map_name` answers from the backup (with a warning) so the next start of
this process keeps working — but every launch file reads `[system] robot_id` from the INI itself,
so **restore it from the `.bak` before the next boot**. The backup is this
package's only INI-shaped file; nothing else reads or writes it.

**The loaded map is read from the INI once, at startup** (`MapCatalogRepo`'s
constructor), and kept in memory; `activate` writes the INI and then that copy
(`MapCatalogRepo.set_active`), and a failed write leaves the copy alone. Every
dispatch, map-lock check and MOVE goal asks which map is loaded, several of them
on the event loop, so none of them touches the disk. `set_active_map` is the
INI's only writer, so nothing is missed — except a hand edit, which, like every
other setting, needs a backend restart.

There are **no ROS parameters** — nothing calls `declare_parameter`. Everything
configurable is the INI, the environment, or a commented constant.

One constant spans two containers: `MapCloudSubscriber`'s `allowed_root`,
`/dev/shm`. pgo writes the map cloud under `/dev/shm/syncai_pgo/<robot_id>` and
names the file in its notice; this side only checks the path is under the root.
Neither end reads it from INI or env — it is a convention, and `ipc: host` on
both compose services makes it the same tmpfs (see [*ROS interfaces*](ros-interfaces.md)). Only tests
override it.

Postgres is retried 20× at 5 s on startup and `<robot_id>_db` is created if
absent, so the backend may come up before the `postgres` container is ready.
After startup the pool runs with `pool_pre_ping` (a `SELECT 1` before each
checkout, so a postgres restart — its own compose stack — costs a reconnect, not
one 500 per pooled connection) and a 5 s libpq `connect_timeout`.

CORS is wide open (`allow_origins=["*"]`).
