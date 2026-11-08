"""Export only the camera-constrained occupancy PGM and matching portable YAML."""
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
print(f'[{DATASET}] Exported only map.pgm and map.yaml; resolution {meta["resolution"]} m/pixel', flush=True)
