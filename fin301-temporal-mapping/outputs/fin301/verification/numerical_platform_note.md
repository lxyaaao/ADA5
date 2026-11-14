# Cross-platform numerical reproduction

The Windows delivered run contains 49,783 LiDAR points and 29 visual landmarks (49,812 combined). The independent full ROS 2 Jazzy processing run contains 49,784 LiDAR points and 29 visual landmarks (49,813 combined). Both used the selected LiDAR revisit configuration and six-view visual reconstruction.

The maximum absolute difference between their 3,526 corresponding corrected trajectory components is x=8.89314958e-7 m, y=1.15458149e-6 m, yaw=3.00449098e-7 rad. Different NumPy/SciPy/OpenCV builds produce small numerical differences; the one-point voxel count difference is consistent with points near voxel boundaries. The exact responsible voxel has not been isolated. Byte-identical cross-platform PCD reproduction is not claimed.

`ros_processing_final_test.txt` checks processing and message publication of the ROS-created artifacts. `ros_final_artifacts_test.txt` separately checks publication of the exact delivered Windows artifacts, including every PointCloud2 XYZ entry, occupancy cell, pose and timestamp. The one-page report and localization evaluation use the Windows result under `outputs/fin301/result`.
