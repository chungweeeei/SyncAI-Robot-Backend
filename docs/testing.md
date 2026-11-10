# Tests

> Part of the `syncai_backend` reference — see the [documentation index](../README.md#documentation).

```bash
colcon test --packages-select syncai_backend
colcon test-result --verbose
# or, inside the container, from src/syncai_backend/:
pytest test/
```

**Off the robot**, the `Dockerfile`'s `dev` target (
`docker build --target dev -t syncai-backend-dev .`) supplies ROS 2 Humble and the pip dependencies, with the
source bind-mounted, so edits need no rebuild. Its header comment has the exact
commands. Before reading a result from it:

- **Get the interface package in, or a third of the suite does not run.**
  `vcs import < interface.repos` materialises `syncai_common`; build it with
  `--packages-select syncai_common`. Without it, or with one older than this
  branch needs (e.g. no `NavigateThroughGoals`), every router and gateway test
  file is a **collection error**: they import `syncai_common` plainly rather
  than via `importorskip`. A workspace kept between runs goes stale this way;
  delete it and import again.
- **The image runs as non-root on purpose.** Root bypasses permission checks, so
  `os.access(W_OK)` is `True` on a read-only file and the map router's
  `ini_not_writable` test fails against working code.

`test/` holds ~40 files, roughly one per router / gateway / subscriber / repo /
helper (`ls src/syncai_backend/test/` is the index), and needs `rclpy` /
`nav_msgs` / `syncai_common` / OpenCV importable. The database layer runs
against **in-memory SQLite** (`StaticPool`, one shared connection), so no
PostgreSQL. The kept 2D helpers' tests also need scipy (`test_pcd_to_gridmap.py`
→ `scipy.ndimage`) and open3d (`test_traversable.py` `importorskip`s
`helpers.traversable` per test, so it skips without open3d). The standard ament
linters (`test_copyright`, `test_flake8`, `test_pep257`) run alongside.
