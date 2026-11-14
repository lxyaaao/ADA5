# GitHub packaging verification — 2026-11-14

This branch packages the archived fin301 reconstruction and temporal-cleanup results. It does not represent a new sensor recording or a new localization-accuracy experiment.

- Python: `python -m pytest -q` — **105 passed**.
- Viewer: `node --test tests/viewer_io.test.cjs tests/viewer_renderer.test.cjs` — **22 passed**. The local Windows sandbox run also used Node's `--preserve-symlinks --preserve-symlinks-main` options.
- Portable replay preparation: `prepare_local_validation.py --check-only --scan-cache <original scan cache>` — **all 12 frozen source hashes matched**. The raw cache is intentionally not committed.
- Matched map/frame validation: conservative **47,708 combined points** (47,679 LiDAR + 29 camera); strict **39,643** (39,614 + 29). Both retain 3,526 trajectory samples and consistent YAML/PGM/frame metadata.
- Main README, ROS instructions and generated comparison-page local links resolve.
- New files contain no raw DB3/MCAP recordings, ZIP inputs, Python environment, sensor cache or bytecode.
- The inherited `fin301-map-generation/` tree remains unchanged from base commit `4c4d1884365e85ec7e9471c61aa4d5efaf6d3b1b`.

Source/PCD coordinates and historical evidence are preserved. New documentation and the portable path-preparation helper are the repository-specific changes. Historical manifests retain their original paths and scope. No new browser interaction, ROS runtime execution, physical reference measurement or absolute-accuracy validation was performed for this packaging task.
