# fin301 offline viewer — 2026-11-10

The user selected a double-click offline browser viewer. Build a standalone HTML artifact with embedded real map files, no remote scripts, CDN, server, account, installation, or data transmission. Preserve source maps and existing estimator outputs.

The viewer opens the metric LiDAR PCD in an equal-scale top view. It offers 2D and orbitable 3D, pan/zoom/fit/reset, point size, independent LiDAR/camera/grid/trajectory layers, two-point planar measurement, trajectory playback/scrubbing, coordinate readout, local file import, image export, and original-file downloads. Camera landmarks remain a separate optional layer. Grid position uses YAML origin, yaw, resolution and image row orientation. The main copy is Chinese. Measurements are estimates from this map, not physical ground truth.

Supported imports: ASCII and uncompressed binary XYZ PCD, ROS map YAML with PGM/PNG/JPEG, and project trajectory CSV. Explicitly reject unsupported PCD binary_compressed instead of silently misreading it. Imported single files must not be overlaid on unrelated bundled layers. Malformed imports show recoverable errors. Files are processed locally in the browser.

Source layout: viewer/index.html template, viewer/style.css, viewer/io.js parsers, viewer/renderer.js rendering/camera, viewer/app.js interaction. scripts/build_viewer.py embeds these sources and real files into outputs/fin301/viewer/viewer.html, then packages a convenient deliverables/fin301_viewer.zip with source maps and Chinese instructions. Viewer-specific tests and verification artifacts stay separate from estimator evidence.

Validation: Node parser/math tests for field offsets, binary pixels, frame rotation, timestamp handling and measurement geometry; actual browser checks for default map, toggles, navigation, measurement, playback, imports, export and errors. Compare bundled bytes/hashes against source maps and inspect rendered screenshots. This is a visualization task, not another mapping experiment.
