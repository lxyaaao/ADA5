"""Export camera-constrained occupancy maps and planar occupied-cell PCDs."""
import os
from pathlib import Path
import cv2
import numpy as np
import yaml
from context import SOURCE, DATASET

source_image = cv2.imread(str(SOURCE / 'native_map.pgm'), cv2.IMREAD_GRAYSCALE)
if source_image is None:
    raise RuntimeError('Cartographer native_map.pgm is missing or unreadable')
meta = yaml.safe_load((SOURCE / 'native_map.yaml').read_text())
# Same native occupancy conversion as the previous complete workflow.
probability = np.rint((255 - source_image.astype(float)) / 255 * 100).astype(np.int16)
probability[source_image == 128] = -1
image = np.full(source_image.shape, 205, np.uint8)
image[(probability >= 0) & (probability < 20)] = 254
image[probability >= 65] = 0
destination = Path(os.environ['MAP_RESULTS_DIR'])
destination.mkdir(parents=True, exist_ok=True)
assert cv2.imwrite(str(destination / 'map.pgm'), image)
output_meta = dict(image='map.pgm', resolution=float(meta['resolution']),
                   origin=[float(v) for v in meta['origin']], negate=0,
                   occupied_thresh=0.65, free_thresh=0.196)
(destination / 'map.yaml').write_text(yaml.safe_dump(output_meta, sort_keys=False))
saved = cv2.imread(str(destination / 'map.pgm'), cv2.IMREAD_GRAYSCALE)
assert np.array_equal(saved, image)
assert yaml.safe_load((destination / 'map.yaml').read_text())['image'] == 'map.pgm'
# PGM rows start at the top; map grid y starts at the bottom. Use cell centres.
rows, cols = np.nonzero(image == 0)
resolution = output_meta['resolution']
local_x = (cols + 0.5) * resolution
local_y = (image.shape[0] - rows - 0.5) * resolution
ox, oy, yaw = output_meta['origin']
cloud = np.column_stack((ox + np.cos(yaw)*local_x - np.sin(yaw)*local_y,
                         oy + np.sin(yaw)*local_x + np.cos(yaw)*local_y,
                         np.zeros(len(rows)))).astype('<f4')
name = 'map_xinyao_pcd_old' if DATASET == 'fin301' else 'map_xinyao_pcd_new'
header = ('# .PCD v0.7 - Point Cloud Data file format\n'
          'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
          f'WIDTH {len(cloud)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
          f'POINTS {len(cloud)}\nDATA binary\n')
with (destination / (name + '.pcd')).open('wb') as stream:
    stream.write(header.encode('ascii'))
    stream.write(cloud.tobytes())
pcd_meta = dict(pcd=name+'.pcd', format='PCD 0.7 binary XYZ float32', frame_id='map',
                units='metres', points=len(cloud), source_image='map.pgm',
                occupancy_yaml='map.yaml', resolution=resolution,
                origin=output_meta['origin'], z=0.0,
                representation='One point at each occupied grid cell centre in the 2D laser scan plane; not a 3D reconstruction.',
                coordinates='Points already include the map origin and yaw; do not apply origin again.')
(destination / (name + '.yaml')).write_text(yaml.safe_dump(pcd_meta, sort_keys=False))
print(f'[{DATASET}] Exported map.pgm, map.yaml and {name}.pcd/.yaml ({len(cloud)} points)', flush=True)
