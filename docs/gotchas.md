# Gotchas

> Part of the `syncai_backend` reference — see the [documentation index](../README.md#documentation).

- **Configuration is read once, at startup.** The INI (`[map]`, `robot_id`),
  `.env` and the environment are all read during construction — any change needs
  a backend restart. That includes restoring `[map] name` from `system.ini.bak`
  or editing it by hand: only `activate` changes the map this process reports.
- Relative topic names are not optional — see [*robot_id, namespaces, and per-robot isolation*](architecture.md#robot_id-namespaces-and-per-robot-isolation).
- **Exactly two latched subscriptions, both pgo's: `pgo/map_cloud_file` and
  `pgo/mapping_status`.** (`map` and `localizer/map_cloud` went away with the
  file-based map endpoints.) Durability must match pgo's publisher exactly — a
  VOLATILE reader connects and gets nothing replayed. Both exist only under
  `MANUAL`; a silent `pointcloud/map/stream` and `unknown` run state on a
  navigating robot are expected, not a QoS mismatch.
- **Mapping mode comes up idle.** Since 2026-10 pgo banks nothing until
  `POST /api/v1/mapping/start`, and a successful save returns it to idle. A save or
  reset then is a 409 `mapping_idle` from the latched status, not pgo's
  `NO POSES!` as a 502 — and an older console with no Start control cannot map
  against this stack. The status ages out after 5 s (`MAPPING_STATUS_TTL_S`):
  this container outlives the mapping session, and a latched `mapping` from a
  pgo that is gone must not refuse a Start forever.
- **The map cloud is a file in the host's `/dev/shm`; both containers need
  `ipc: host`.** Without it, notices still arrive, every read logs
  `map cloud file unreadable`, and the preview never updates. Do not "fix" the size cliff by
  raising `net.core.rmem_max` and going back to the `PointCloud2` — host state on
  every robot, and 16–45 MB per merge only moves the cliff.
- `map -> pointlio_odom` exists once `syncai_localizer` has its first synced
  `body_cloud` + `lio_odom` pair — **not** once it has converged: it broadcasts
  from that sample on, starting at identity, relocalized or not. A missing
  transform means the localizer (or pointlio under it) is not running; a present
  one proves nothing about the pose — `relocalize_check` is the convergence
  check. Until it exists the live cloud stream is silent; the subscriber logs
  once on the first drop and once on recovery, so check the log if the 3D view
  is empty.
- **Deleting `keepout.json` clears nothing now.** The running 3D planner keeps
  the zones it last read. Clear with `PUT .../keepout` and `[]`, which rewrites
  the file with none and reloads it.
- **A client that drops `name` / `z_min` / `z_max` widens zones.**
  `PUT …/keepout` replaces the whole list, and a zone sent without a band
  covers every floor. Send back what `GET` returned.
- The `sqlalchemy` session convention is per-repo: `init_map_repo` creates the
  schema and builds its own `sessionmaker` from the injected engine.
