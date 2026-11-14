# fin301 Offline Map Viewer

Extract the entire ZIP into a folder, then double-click **open_viewer.cmd** or open **viewer.html** directly in your browser. Use an installed browser such as Chrome, Edge, or Firefox. Viewing requires no Python, ROS, installation, internet connection, or local server. Run the launcher from the extracted folder, not from the ZIP preview.

The viewer opens with an equal-scale top view of the LiDAR point cloud. Switch between 2D and 3D, pan, zoom, fit the map to the window, and toggle the LiDAR, camera, occupancy-grid, and trajectory layers independently. Rotate the view in 3D mode. Two-point measurement reports straight-line distance in the map's XY plane, in metres. Play the trajectory or drag its time slider. The page also supports image export, original-file downloads, and local file import. See the in-page help for mouse controls and keyboard shortcuts.

If the package includes `lidar_scan_map_clean_*.pcd`, use **Point cloud file** to compare the filtered clouds with the original `lidar_scan_map.pcd`. The original opens by default. Switching bundled clouds preserves zoom and view position for comparison; press F to fit again. Each cleaned cloud is a subset of observed LiDAR points with their coordinates unchanged. The occupancy grid, trajectory and camera landmarks remain the original layers: filtering a point does not establish that its grid cell is free. Consult the bundled filtering report for parameters and localization validation; a cleaner appearance alone does not show improved accuracy.

## Data and coordinates

- `maps/lidar_scan_map.pcd`: 2D LiDAR scan-plane points at z=0, suitable for 2D scan localization.
- `maps/visual_landmarks.pcd`: a separate, sparse layer of camera-derived 3D candidate landmarks.
- `maps/map.pcd`: combined XYZ points, with LiDAR points first and camera points second. Do not project the camera points into the scan plane as 2D obstacles.
- `maps/map.yaml` and its referenced `maps/map.pgm`: the ROS occupancy grid, resolution, origin, and thresholds.
- `maps/trajectory.csv`: trajectory samples with nanosecond timestamps and separate vehicle-base and LiDAR pose columns.
- `maps/map_metadata.json` and `maps/metrics.json`: original map descriptions and result statistics.

The coordinate origin is the first LiDAR pose. Initially, x points forward, y left, and z up. Units are metres. The z=0 plane is the LiDAR scan plane, not an independently measured floor height. Grid placement uses the YAML translation, yaw, and resolution together; the top image row corresponds to higher local grid y.

Distances measured in the viewer are map estimates, not physical ground truth. Camera points passed six-view in-sample geometric screening; they have not demonstrated an improvement in localization accuracy. Physical-reference validation remains incomplete. Glass reflections, calibration error, and the planar vehicle-motion assumption can still affect the result.

## Local import and offline use

Imported files are processed locally in your browser and are not uploaded. Supported formats are ASCII or uncompressed binary PCD with XYZ fields, ROS trinary map YAML with a PGM/PNG/JPEG image, and this project's trajectory CSV format. `binary_compressed` PCD is unsupported and produces an error. Select the referenced image together with its YAML file. Files without an established common coordinate frame must not be interpreted as aligned with the bundled data; importing a separate dataset removes unrelated bundled layers. Invalid imports do not modify the original map files.

Interaction state is not guaranteed to persist after closing the browser. Your browser may ask where to save an exported image. If WebGL is disabled, enable graphics acceleration or use a browser with WebGL support. If the HTML still does not display, retain the page's error message for troubleshooting. A network service is not required.

## Package contents and verification

`viewer.html` contains all styles, scripts, and map data and can be copied by itself. The `maps/` folder contains original-file copies for use in other software. Editing that folder does not automatically update the data embedded in the HTML. `viewer_manifest.json` lists packaged file sizes and SHA-256 hashes; the manifest does not recursively hash itself. The builder verifies every ZIP member, original map bytes, point counts, component order, coordinate relationships, and the YAML image reference.

`src/viewer/` and `src/scripts/build_viewer.py` contain editable viewer sources. They are needed only to rebuild the page; opening the generated HTML requires no development environment. The original mapping software, previous project ZIP, and existing Chinese viewer outputs are preserved.

## Rebuilding the English viewer

From the original project root, using its existing Python environment:

```powershell
.venv/Scripts/python.exe scripts/build_viewer.py --dataset fin301 --output outputs/fin301/viewer_en --zip deliverables/fin301_viewer_en.zip
```

This creates `outputs/fin301/viewer_en/viewer.html` and `deliverables/fin301_viewer_en.zip`. The explicit English output paths preserve `outputs/fin301/viewer/` and `deliverables/fin301_viewer.zip`.

To validate the source data without requiring viewer assets or writing output:

```powershell
.venv/Scripts/python.exe scripts/build_viewer.py --dataset fin301 --validate-data-only
```

To rebuild from the extracted package root, use a development environment with Python 3.10+ and PyYAML:

```powershell
python src/scripts/build_viewer.py --dataset fin301 --result maps --source src/viewer --output rebuilt_en --zip rebuilt_en.zip
```

To include cleaned variants when rebuilding, append `--filtered-pcd maps/lidar_scan_map_clean_conservative.pcd` (repeat for each variant) and optionally `--filter-report maps/cleanup_summary.json`. The builder checks every retained XYZ against the original LiDAR cloud and bundles the additional files without replacing originals.

The template contains `{{STYLE}}`, `{{IO_JS}}`, `{{RENDERER_JS}}`, `{{APP_JS}}`, and `{{BUNDLE_JSON}}`, each exactly once. The `bundled-files` JSON element contains `{dataset, files: [{name, base64}], metadata: {map_metadata, metrics}}`. Embedded metadata integers outside JavaScript's exact integer range are encoded as decimal strings; the original JSON copies remain unchanged. The JavaScript source files document the parser and renderer interfaces.
