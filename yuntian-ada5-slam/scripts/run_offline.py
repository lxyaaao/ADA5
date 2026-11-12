#!/usr/bin/env python3
"""Run ada5_slam.pipeline from the source tree under `python3 -I`.

-I keeps the recording's folder and the user site off sys.path; this puts
back the ROS installation and this workspace's package.

Usage: python3 -I run_offline.py <bag_dir> <out_dir> [pipeline options]
"""
import os
import sys

# Multi-threaded BLAS/OpenMP is several times slower on the course VM.
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')

sys.path[:0] = [p for p in os.environ.get('PYTHONPATH', '').split(os.pathsep)
                if p.startswith('/opt/ros/')]
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'ada5_slam'))

from ada5_slam.pipeline import main  # noqa: E402

if __name__ == '__main__':
    main(sys.argv[1:])
