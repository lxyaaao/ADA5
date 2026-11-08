# Fin301 sensor-only occupancy maps

This package contains the active code, both original recordings and only the final **PGM/YAML** results.
Both maps use raw LiDAR through Cartographer 2D. IMU, camera images, recorded odometry,
recorded maps and the previously supplied map screenshot are not used for map generation.

| Recording | Occupancy image | Configuration |
|---|---|---|
| fin301 | [PGM](results/fin301/map.pgm) | [YAML](results/fin301/map.yaml) |
| Fin301-11051740 | [PGM](results/Fin301-11051740/map.pgm) | [YAML](results/Fin301-11051740/map.yaml) |

There are no point clouds, 3D reconstruction, previews, trajectories, reports or comparison
outputs in `results/`. The generation code does not create those products.

## Folder layout

```
run_mapping.sh                   portable entry point
src/                             data restoration, scan replay, SLAM runner and PGM/YAML export
config/cartographer/             Cartographer settings and required Lua includes
config/datasets.json             filenames relative to the data root
results/fin301/map.pgm            first recording's final occupancy map
results/fin301/map.yaml           its matching metadata
results/Fin301-11051740/map.pgm    second recording's final occupancy map
results/Fin301-11051740/map.yaml   its matching metadata
data/                            both original databases as compressed Git LFS chunks
requirements.txt                 non-ROS Python dependencies
```

## Requirements

Use Ubuntu 22.04 / ROS 2 Humble, Python 3.10, the SQLite rosbag2 storage plugin,
`rosbag2_py`, and a compatible Cartographer ROS 2 installation. The installation must provide
`lib/cartographer_ros/cartographer_offline_node` and
`lib/cartographer_ros/cartographer_pbstream_to_ros_map`.
Install the Python packages in `requirements.txt` in an environment that can import ROS.
`MAP_PYTHON` can select the Python interpreter.

Source your ROS setup and Cartographer workspace setup before running. For example, source
`/opt/ros/humble/setup.bash`, then your Cartographer workspace's `install/setup.bash`.
The runtime libraries are dependencies; they are not copied into this repository.

## Run after cloning

With Git LFS installed:

```bash
git clone YOUR_GITHUB_REPOSITORY_URL
cd YOUR_REPOSITORY_FOLDER
git lfs pull
```

After activating the dependencies above:

```bash
bash run_mapping.sh --dataset both
```

The runner automatically restores both bundled databases, verifies their SHA256 checksums,
and writes only `map.pgm` and `map.yaml` under each recording's results folder.
To generate only one recording, use `--dataset fin301` or `--dataset Fin301-11051740`.

Additional options:

```bash
bash run_mapping.sh --help
bash run_mapping.sh --dataset both --rebuild-backend
bash run_mapping.sh --dataset both --data-root /path/to/recordings
bash run_mapping.sh --dataset both --work-root /path/to/scratch --results-root /path/to/output
```

An external data root must contain `fin301/fin301_0.db3` and
`Fin301-11051740/Fin301-11051740_0.db3`. For a custom Cartographer prefix, add
`--cartographer-prefix /path/to/prefix`. That prefix must contain `lib/cartographer_ros/`.
The adapter also supports an extracted Ubuntu-style runtime root.

The entry point works from any current directory and the package can be moved or renamed.
No code depends on the original project paths. Source database and configuration SHA256
signatures invalidate stale backend caches. Logs and intermediate SLAM states remain under
`work/`, which Git ignores. These are not final deliverables.

## Included recordings

Both complete original SQLite databases are stored as lossless gzip chunks of at most
512 MiB each. They total about 4.1 GB compressed and restore to about 11.3 GB.
`data/archives.json` records original sizes and hashes plus the ordered chunk hashes.
These are full byte-identical recordings, not reduced scan subsets. Recorded IMU and camera
topics remain preserved in the source archives but are not supplied to the mapping backend.

The runner restores `data/fin301/fin301_0.db3` and
`data/Fin301-11051740/Fin301-11051740_0.db3` automatically. Restored databases are ignored by Git
so the recording is not uploaded twice. Original source metadata files are not needed.
Allow disk space for compressed archives, restored databases and additional backend caches.
For data restoration alone:

```bash
python3 src/restore_data.py
```

## Mapping method

Only `/scan/raw` is copied into a fresh replay bag as `/scan`. Original databases are read-only.
Cartographer 2D uses correlative scan-to-submap matching, Ceres refinement, overlapping
probability submaps and global loop constraints. Settings remain 0.22–12 m range,
0.025 m grid resolution, 75 insertions per submap and 0.58 minimum loop match score.
IMU, odometry, GNSS and external landmark inputs are disabled.

The optimized SLAM state is rendered into a native occupancy grid. The final three-value
PGM marks native occupied evidence >=65/100 black (0), known evidence <20/100 white (254),
and everything else gray (205). Its YAML uses a relative `image: map.pgm` path and preserves
Cartographer's metric origin and resolution. This matches the occupancy export used before
the point-cloud and camera stages were removed; point-cloud filtering never changed this grid.

There is no screenshot guidance, point-cloud cleanup, camera triangulation, wall thickening,
or forced corner closure in this occupancy-only workflow. Glass, occlusion, noisy objects
and possible doubled walls can remain. No claim of complete denoising or surveyed accuracy
is made. The grid resolution is a sampling scale, not an accuracy guarantee.

Each recording retains its own first-laser map frame. Keep each PGM beside its matching YAML;
do not exchange metadata between recordings. Black means occupied, white observed free space,
and gray unknown. The two coordinate frames are not aligned to each other.

## Upload with Git LFS

The included `.gitattributes` tracks all `data/**/*.gz.part*` files with Git LFS. Use Git LFS
rather than browser drag-and-drop or ordinary Git for these large chunks. Ensure your GitHub
account has sufficient LFS storage and download allowance.

Install Git LFS first, then run in this folder (replace the repository URL):

```bash
git init
git lfs install --local
git add .
git commit -m "Add occupancy mapping code data and PGM YAML results"
git branch -M main
git remote add origin YOUR_GITHUB_REPOSITORY_URL
git push -u origin main
```

Download on another computer with `git clone` and `git lfs pull`. GitHub's Download ZIP may
contain LFS pointers instead of the data; missing chunks or unresolved pointers produce an
explicit restoration error. Upload the entire folder including `data/`, `.gitattributes`
and `.gitignore`. Do not force-add restored `.db3` files or scratch caches.

GitHub LFS documentation:
https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-git-large-file-storage

## Software attribution

Cartographer and Ceres provide SLAM. NumPy, OpenCV and PyYAML provide occupancy export.
Required Cartographer Lua files retain upstream Apache license notices. ROS, Cartographer,
Ceres and Python dependencies retain their own licenses; no runtime binaries are redistributed.
