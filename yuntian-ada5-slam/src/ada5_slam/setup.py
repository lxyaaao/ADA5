from glob import glob
from setuptools import find_packages, setup

setup(name='ada5_slam', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/ada5_slam']),
                  ('share/ada5_slam', ['package.xml']),
                  ('share/ada5_slam/config', glob('config/*.yaml')),
                  ('share/ada5_slam/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Yuntian Meng', maintainer_email='student@example.invalid',
      description='Indoor camera + 2D LiDAR SLAM for Project 2.2.',
      license='Apache-2.0',
      entry_points={'console_scripts': ['slam_node = ada5_slam.slam_node:main',
                                        'offline = ada5_slam.pipeline:main']})
