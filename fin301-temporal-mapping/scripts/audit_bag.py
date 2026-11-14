"""CLI for a complete streaming sensor audit and cache extraction."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from indoor_mapping.bag_io import audit_and_extract, export_recorded_map, nearest_time_matches, quaternion_matrix, stamp_ns, static_transform_between, timestamp_audit


def self_test() -> None:
    """Guard timestamp precision, nearest-neighbor boundaries and TF direction."""
    import numpy as np
    from types import SimpleNamespace
    base = 1_792_708_234_000_000_000
    assert stamp_ns(SimpleNamespace(sec=1792708234, nanosec=123456789)) == base + 123456789
    target = np.array([base + 10, base + 20, base + 40], dtype=np.int64)
    source = np.array([base, base + 15, base + 21, base + 60], dtype=np.int64)
    indices, errors = nearest_time_matches(source, target)
    np.testing.assert_array_equal(indices, [0, 0, 1, 2])
    np.testing.assert_array_equal(errors, [10, 5, 1, 20])
    stats = timestamp_audit([base, base + 10, base + 10, base + 50, base + 20], [base] * 5)
    assert stats['duplicate_header_timestamps'] == 1 and stats['header_reversals_in_bag_order'] == 1
    assert stats['gap_count'] == 1 and stats['estimated_missing_frames'] == 2
    np.testing.assert_allclose(quaternion_matrix(0, 0, np.sqrt(.5), np.sqrt(.5)) @ [1, 0, 0], [0, 1, 0], atol=1e-12)
    a = np.eye(4)
    a[0, 3] = 2
    b = np.eye(4)
    b[1, 3] = 3
    transforms = [{'parent': 'base', 'child': 'laser', 'T_parent_child': a.tolist()},
                  {'parent': 'base', 'child': 'camera', 'T_parent_child': b.tolist()}]
    result = static_transform_between(transforms, 'camera', 'laser')
    np.testing.assert_allclose(result[:3, 3], [2, -3, 0])
    assert static_transform_between(transforms, 'unknown', 'laser') is None
    import cv2
    import tempfile
    import yaml
    origin = SimpleNamespace(position=SimpleNamespace(x=-1, y=2), orientation=SimpleNamespace(x=0, y=0, z=0, w=1))
    message = SimpleNamespace(info=SimpleNamespace(height=2, width=2, resolution=.05, origin=origin),
                              data=np.array([0, 100, -1, 0], np.int8),
                              header=SimpleNamespace(frame_id='map', stamp=SimpleNamespace(sec=0, nanosec=0)))
    with tempfile.TemporaryDirectory() as directory:
        output = Path(directory)
        export_recorded_map(message, output, 0)
        pixels = cv2.imread(str(output / 'recorded_baseline_UNVERIFIED.pgm'), cv2.IMREAD_GRAYSCALE)
        np.testing.assert_array_equal(pixels, [[205, 254], [254, 0]])
        metadata = yaml.safe_load((output / 'recorded_baseline_UNVERIFIED.yaml').read_text())
        assert metadata['origin'] == [-1, 2, 0] and metadata['resolution'] == .05
    print('Audit self-tests passed: integer timestamps, nearest matching, dedup/gap statistics, quaternion/TF direction, map row orientation and YAML.')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', default='data/gorehall-10221830/gorehall-10221830_0.db3')
    parser.add_argument('--output', default='outputs')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
    else:
        audit_and_extract(args.bag, args.output)


if __name__ == '__main__':
    main()
