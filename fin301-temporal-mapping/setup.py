from glob import glob

from setuptools import find_packages, setup


setup(
    name="indoor_mapping",
    version="0.1.0",
    packages=find_packages(exclude=("tests",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/indoor_mapping"]),
        ("share/indoor_mapping", ["package.xml", "ROS_RUNTIME.md"]),
        ("share/indoor_mapping/launch", glob("launch/*.launch.py")),
        ("share/indoor_mapping/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Intro2AV project contributors",
    maintainer_email="maintainer@example.invalid",
    description="Batch camera-assisted planar LiDAR mapping and ROS 2 artifact publication",
    license="LicenseRef-Proprietary",
    entry_points={"console_scripts": ["batch_mapping = indoor_mapping.ros_node:main"]},
)
