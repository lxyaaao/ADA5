# 技术参考资料

下列为官方文档、维护者源码或原始研究。访问日期：2026-10-27。它们说明消息、坐标、文件格式和方法依据；不能替代本数据集的实测结果，也不表示本项目已经运行了所有列出的库。当前映射核心自行实现，未直接调用 RTAB-Map 或 SLAM Toolbox。

## 当前实现的接口与方法依据

1. ROS 2 common_interfaces. [LaserScan.msg](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg). 首束采集时间、角度方向、量程、逐束时间增量与单位定义。
2. Tully Foote and Mike Purvis. [REP 103: Standard Units of Measure and Coordinate Conventions](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst). 米制单位、车体轴及相机光学坐标轴约定。
3. Wim Meeussen. [REP 105: Coordinate Frames for Mobile Platforms](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst). `base_link`、`odom`、`map` 的语义；本项目 ROS 批处理适配器仅发布历史结果，不提供实时 TF。
4. OpenCV. [Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html). 相机投影、畸变、RANSAC 几何验证；单目 `recoverPose` 的平移只确定方向和未知尺度。本项目不使用它输出米制位移。
5. ROS 2 message_filters. [Documentation](https://github.com/ros2/message_filters/blob/rolling/doc/index.rst). 按消息时间戳同步的语义。当前离线核心自行执行最近时间关联，没有把近似关联宣称为硬件同步。
6. Point Cloud Library. [The PCD file format](https://pointclouds.org/documentation/tutorials/pcd_file_format.html). PCD 0.7 文件头、XYZ 字段、点数和 ASCII 数据布局。
7. Point Cloud Library. [Visualization tutorial](https://github.com/PointCloudLibrary/pcl/blob/master/doc/tutorials/content/visualization.rst). `pcl_viewer` 查看 PCD 的命令接口。
8. CloudCompare. [Official changelog](https://github.com/CloudCompare/CloudCompare/blob/master/CHANGELOG.md) and [Open documentation](https://www.cloudcompare.org/doc/wiki/index.php/Open). File → Open 操作；PCD 支持要求对应 qPCL I/O 插件。

## 后续物理标定验证

9. ROS image_pipeline. [Monocular camera calibration tutorial](https://github.com/ros-perception/image_pipeline/blob/rolling/camera_calibration/doc/tutorial_mono.rst). 已知尺寸标定板、单目内参标定和 ROS 2 工作流。
10. Robert Pless and Qilong Zhang. [Extrinsic Calibration of a Camera and Laser Range Finder](https://openscholarship.wustl.edu/cse_research/1103/). Washington University technical report WUCSE-2003-58, 2003. 使用多姿态平面目标建立相机与二维激光之间的外参约束。引用为未来实验依据，当前输入不含完成该实验所需的独立目标观测。

## 对照方案与设计边界

11. IntRoLab. [RTAB-Map ROS 2 ICP odometry](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_odom/doc/icp_odometry.md). 二维/三维 LiDAR 配准及几何退化问题，尤其走廊方向的可观测性。
12. IntRoLab. [RTAB-Map ROS 2 SLAM input documentation](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_slam/doc/rtabmap.md). 图像外观回访、激光几何以及单目和 RGB-D 输入能力的区别。
13. IntRoLab. [RegistrationIcp.cpp](https://github.com/introlab/rtabmap/blob/master/corelib/src/RegistrationIcp.cpp), [Rtabmap.cpp](https://github.com/introlab/rtabmap/blob/master/corelib/src/Rtabmap.cpp), and [parameter reference](https://introlab.github.io/rtabmap/api/latest/parameters.html). ICP 初值、全局回访初值选项与注册策略；不能将 RGB-D 示例无条件视为单目加二维 LiDAR 配置。
14. Steve Macenski and Ivona Jambrecic. [SLAM Toolbox maintainer documentation](https://github.com/SteveMacenski/slam_toolbox). 二维激光位姿图、回环与地图重建；适合作为 LiDAR 基线方案，本身不能证明本项目相机输入的贡献。

第三方库、文档和录制数据保留各自许可及作者署名。报告应将这里的通用原理、项目实现选择及数据实际支持的结论分别表述。

## fin301 视觉地标补充（2026-11-05 核对）

15. OpenCV. [Object Tracking: calcOpticalFlowPyrLK](https://docs.opencv.org/4.13.0/dc/d6b/group__video__track.html). 当前视觉地标模块使用金字塔 Lucas–Kanade 光流跟踪，并自行检查前后向误差。
16. OpenCV. [Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html). 当前模块使用 `undistortPoints`、`triangulatePoints` 和 `projectPoints`，用首末视图三角化并在四幅图像中检查重投影；米制相机位姿来自 LiDAR 轨迹与录制的安装变换。这些文档说明算法接口，不证明本实验的绝对精度。
