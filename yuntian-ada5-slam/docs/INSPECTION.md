# Recording `fin301`: inspection notes

Generated with `scripts/inspect_bag.py` and `scripts/plot_inspection.py`.
Raw numbers are in `bag_inspection.json`; the overview figure is
`inspection_overview.png`.

## Source and integrity

- Location: FinTech 301, according to the group (`fin301` in the file name).
- Windows archive: `data_project2_2/fin301.zip` (2,054,195,402 bytes).
- VM copy: `/home/charlie/project2_2_data/fin301/` (`fin301_0.db3`, 5,548,421,120 bytes, plus
  `metadata.yaml`). CRC32 of both files matches the ZIP directory
  (`5bb16317`, `e8842524`).
- rosbag2, sqlite3 storage, CDR, one file.
- Recorded 2026-10-29 18:15:41–18:25:34 EDT, 592.72 s, 313,349 messages.

## Streams for estimation

| Stream | Type | Rate | Notes |
|---|---|---|---|
| `/scan/raw` | LaserScan | 29.8 Hz, no gaps | 1440 beams, −180° to +179.75°, 0.25° step, intensities, frame `laser` |
| `/scan` | LaserScan | 29.8 Hz, no gaps | same geometry; fewer valid beams (57.6% vs 60.7%), so it is a filtered copy |
| `/camera/camera/color/image_raw/compressed` | CompressedImage (JPEG) | 10.0 Hz, no gaps | 640×480, frame `camera_link` |
| `/camera/camera/color/image_raw` | Image `bgr8` | 9.97 Hz, 38 gaps up to 0.60 s | incomplete; use the compressed stream |
| `/camera/camera/color/camera_info` | CameraInfo | 10.0 Hz | constant for the whole recording |
| `/tf_static` | TFMessage | 3 messages | sensor mounts, below |

Header stamps have no duplicates or backward steps on any sensor stream.
Receive time minus header stamp: LiDAR median 34.8 ms (about one scan period,
so the header is most likely the scan start), camera median 24.5 ms. Both
are stamped close to the recorder's receive time, so they appear to share one
clock; this is an inference, not a verified hardware synchronization.

## Streams present but not used for estimation

- `/imu/data` (27.7 Hz, frame `imu_link`) and `/magnetometer_data` (178 Hz):
  the assignment requires operation without inertial measurements.
- `/odom` and dynamic `/tf` (`odom -> base_link`, 20 Hz): vehicle odometry, not
  one of the two assigned sensors. Its covariance is all zeros (unspecified).
  Kept for context only. Its path length is 238.9 m over an area of about
  37 m × 23 m, and its start/end gap is 7.40 m; these are odometry values, not
  ground truth.
- `/map` (1 message, OccupancyGrid, 1162×963 at 0.05 m, origin (−20.1, −39.3) m,
  frame `map`): a pre-built map published 1,302 s before the recording started.
  It is not our map and must not be used to estimate poses or as the
  submitted map. It is not a physical reference either.
- Nav2/AMCL/costmap/behavior/teleop topics: vehicle software state. AMCL was
  running but published no pose.

## Calibration as recorded

Camera intrinsics (`camera_info`, plumb_bob):
fx 433.769, fy 430.026, cx 311.076, cy 221.526;
D = [−0.066549, 0.048809, −0.000212, −0.004628, 0.0];
rectified P: fx 424.312, fy 423.102, cx 306.843, cy 220.988.

Mounts (`/tf_static`, parent `base_link`):

| Child | Translation x, y, z (m) | Rotation (roll, pitch, yaw) |
|---|---|---|
| `laser` | −0.0826, −0.0172, +0.0341 | identity |
| `camera_link` | +0.1232, −0.0172, −0.0534 | (−90°, 0°, −90°): optical axes, z forward, x right, y down |
| `imu_link` | +0.0418, −0.0178, −0.0636 | identity (not used) |

Implied camera position relative to the laser: 0.206 m forward and 0.087 m
below the scan plane, laterally aligned. The frame named `camera_link` carries
the optical rotation. These values come from the vehicle's configuration and
still need a physical check (tape measurement on the car and a LiDAR-to-image
projection test).

## Environment, from sample images and scans

The camera sits close to the floor. The route passes an open lounge with
table and chair legs, people, floor-to-ceiling glass, reflective panels, and
plain walls. Expected problems: LiDAR returns through or off glass, moving
people, clutter from furniture legs, motion blur and over-exposure near
windows, and low-texture walls for the camera.

## Contents over time (second pass, 2026-11-06)

Generated with `scripts/describe_bag.py` and `scripts/plot_content.py`.
Outputs are in `../content_fin301/`: figures `timeline.png`,
`route_context.png`, `start_end.png`, `frames.jpg` (one frame every 30 s) and
`scan_filter.png`, numbers in `summary.json`. Odometry and drive commands are
used here only to describe the drive.

### Topics

70 topics are declared and 28 carry data.

| Group | Topics (with data) | MB |
|---|---|---|
| camera | 3 (3) | 4,998, of which `image_raw` 4,828 |
| LiDAR | 2 (2) | 408 |
| navigation stack and UI | 33 (7) | 36 (local costmaps, `/bond`, the one `/map`) |
| inertial (not allowed) | 2 (2) | 18 |
| vehicle state | 4 (4) | 10 (`/odom`, `/tf`, `/tf_static`, `/battery_state`) |
| driving commands and mode | 11 (6) | 2.4 (`/ackermann_cmd`, `/teleop`, `/rc_data`, control-center strings) |
| logging and diagnostics | 4 (2) | 0.5 |
| behavior module | 11 (2) | 0.1 (the sign, cone and light detectors published nothing) |

The uncompressed image stream is 88% of the data and duplicates the
compressed one. The topics this project needs come to about 0.59 GB.

### The drive

- The control center reported `HUMAN_CONTROL` for the whole recording: the car
  was teleoperated through `/teleop`. When no fresh teleop message arrived it
  sent zero (`teleop_stale_zero`, 183 s in total). All ten `/rc_data` channels
  are zero.
- Commands sit mostly at the limits, ±0.8 m/s and ±14.3° steering. From
  odometry: median moving speed 0.70 m/s (maximum 0.86), reversing 16% of the
  time, turning faster than 30°/s 6% of the time (the peaks, about 35°/s, are
  full-lock turns).
- The car stands still for the first 39 s and the last 10 s. There are 13 stops
  of 2 s or more, 169 s in all; the longest is 4:29–5:07.
- Route, by odometry: several loops in the lounge where the group started
  (0–4 min), a short corridor to a smaller room (4–6 min), then back along a
  long corridor, into a room off it (about 7 min) and on to a room in the far
  wing (8–9.5 min), ending at the corridor junction the car passed at about
  6:30. Path 239 m.
- The car did not return to its start: the first and last scans and images show
  different places (`start_end.png`), and odometry puts the end 7.4 m from the
  start.

Candidate revisits (odometry within 1 m after at least 10 m of driving; drift
makes these approximate), as places to test loop closure:

| Later pass | Earlier pass | Where |
|---|---|---|
| 2:15–2:32 | 1:45–1:50 | lounge |
| 3:04–3:45 | 2:17–3:10 | lounge |
| 3:49–3:57 | 0:00–1:19 | start area |
| 6:00–6:11 | 4:02–5:14 | small room |
| 7:33–7:47 | 6:58–7:03 | room off the long corridor |
| 8:31–9:19 | 7:55–8:16 | far wing room |
| 9:39–9:52 | 6:23–6:48 | corridor junction (end) |

### Camera

- Exposure is fine: median grey level 135, no frame with more than 5% of its
  pixels saturated. Daylight was fading (recorded 18:15–18:25) and comes through
  floor-to-ceiling windows.
- Sharpness (variance of the Laplacian): median 198; 299 when stopped, 233
  when driving straight, 124 when turning faster than 30°/s. 13% of frames fall
  below a quarter of the median, in 14 stretches of 1 s or more; the longest are
  1:52–2:00, 7:00–7:06, 8:21–8:27 and the final 11 s facing a wall corner.
  Low values come from motion blur and from plain surfaces.
- Texture: median 353 FAST corners per frame; 15% of frames have fewer than
  100 (plain walls, close-ups of furniture or the floor).
- The camera is a few centimetres above the floor and mostly sees chair and
  table legs, people's legs, windows and the ceiling. People appear in many
  frames, and another small car appears at about 1:30.

### LiDAR

- Field of view: the rear 90° (beyond ±135°) never returns, so the usable view
  is 270°. About 80% of the beams in it return; 61% of all beams do. Invalid
  beams are non-finite; there are no zero ranges.
- The scene is close: per-scan median range 1.76 m (10th–90th percentile
  1.08–3.45 m); 22% of returns are within 1 m in a typical scan, 54% at the
  95th percentile.
- `/scan` is `/scan/raw` minus self-returns. It removes 5.1% of the raw returns,
  98% of them in two fixed sectors, −47.3° to −44.8° and +2.0° to +11.8°, at
  0.1–0.5 m. No other value changes (`scan_filter.png`).
- 0° points forward: placed with odometry, the walls line up
  (`route_context.png`).

### Vehicle software state

- Nav2 localization (map server and AMCL) was active with the pre-built `/map`,
  but no initial pose was set. AMCL warned 298 times that it cannot publish a
  pose, and the global costmap logged 1,206 time-outs waiting for the missing
  `map` frame. Nav2 navigation was inactive (diagnostics: "Nav2 is inactive");
  no goal, plan or `cmd_vel` was published.
- The behavior module stayed in `NORMAL_NAV2`; the Foxglove waypoint tool was
  idle.
- Battery 12.19 V → 11.95 V (77% → 64%).

## Open checks

1. Physical reference measurements from the group. The bag holds none, and the
   start/end gap cannot serve because the car ended elsewhere.
2. Whether `/odom` may be used (the assignment lists only camera and 2D LiDAR).
3. Camera-to-LiDAR projection test of the mounts.

Resolved on 2026-11-06: which returns `/scan` removes, whether 0° points
forward, and whether the route returns to its start (it does not; revisits are
listed above).
