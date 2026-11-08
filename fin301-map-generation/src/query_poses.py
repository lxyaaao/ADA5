from pathlib import Path
import numpy as np,rclpy,json,time
from cartographer_ros_msgs.srv import TrajectoryQuery
import os
OUT=Path(os.environ['MAP_QUERY_DIR'])
rclpy.init();node=rclpy.create_node('fin301_optimized_pose_export');client=node.create_client(TrajectoryQuery,'/trajectory_query');assert client.wait_for_service(timeout_sec=15);req=TrajectoryQuery.Request();req.trajectory_id=0;future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=30);resp=future.result();assert resp is not None and resp.status.code==0, 'Trajectory query failed';print(resp.status);rows=[]
for m in resp.trajectory:
 q=m.pose.orientation;yaw=np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));rows.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.pose.position.x,m.pose.position.y,yaw])
np.savetxt(OUT/'optimized_poses.csv',rows,delimiter=',',header='timestamp_s,x_m,y_m,yaw_rad',comments='');print('optimized Cartographer nodes',len(rows));node.destroy_node();rclpy.shutdown()
