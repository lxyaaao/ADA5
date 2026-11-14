# fin301 / FinTech301 报告使用说明与尚缺实验

英文小组技术报告由实际 JSON 指标与结果图片生成；PDF 严格为一页，Markdown 可编辑。小组编号保留为 `[enter group ID]`，未填写人员姓名，也未虚构个人贡献或宣称本组亲自完成原始采集。

数据来源：`data/fin301/fin301_0.db3`。本报告的指标目录为 `outputs/fin301/result`，审计记录为 `outputs/fin301/audit/audit.json`。

相机首帧比原始激光首帧晚 3.033 秒；这段初始化时间没有相机观测，不能强行关联未来图像。

当前记录范围：591.8 / 591.9 秒；已接受 3,526 帧用于轨迹估计，导出 49,812 个 PCD 点。时长覆盖不能代替独立精度验证；应同时检查拒绝帧和时序间隔。

## 留出原始扫描定位的开发验证

100 个查询扫描没有用于里程计和地图插入；训练参考排除查询前后 5 秒和 0.5 米以内的扫描，且只含二维 LiDAR 点。每个查询使用 8 组固定初值（平移 0.25/0.50 m，航向 5/10°），所有候选方法使用同一份冻结查询、参考扫描及初值。通过固定配准门槛的次数为 712/800；全部初值通过且两两平移差不超过 0.15 m、航向差不超过 3° 的查询为 71/100。至少两组初值被接受的 92 个查询中，最大两两位置分歧的 p95 为 0.009 m；这不是相对真实位置的误差。仅用至少 30 秒之前的参考扫描时，稳定查询为 11/20。失败初值没有从接受率分母剔除；拟合残差仅统计被接受配准，不能单独用来选优。查询位置和初值来自估计轨迹，协议也受基线轨迹影响；这些查询用于选择改进方案，属于开发验证，不是未参与选型的最终测试集，也不是独立定位精度实验。证据：`outputs/fin301/verification/localization_final/localization_evaluation.json`。

四视图旧方案的额外两帧预测诊断：10/93 个点同时通过两帧 2 px 门槛。当前 6 视图方案已把构建视图用于筛选，因此不能将这些视图再次称作独立留出验证。相机点保留为实验性几何附件，没有证明提高定位精度。诊断来源：`outputs/fin301/verification/visual_landmark_review_heldout.json`。


## 尚缺的实体参照验证

**Physical-reference validation remains incomplete; no reference provided.**

现有输入没有独立测得的墙长、测量控制点或真实车辆轨迹。ICP 残差、图优化残差、首尾距离及相机引起的修正都是内部一致性指标，不是绝对位置误差。不得据此声称厘米级精度或已经满足评分中的实体测量要求。

后续应选取可以在图中明确对应的实体墙段或标记点，独立测量长度/位置并记录量具、测量日期、不确定度和端点定义；比较地图坐标的欧氏距离，报告绝对与相对误差。若要验证定位，应记录独立参考轨迹或多个独立控制点；不能使用该 bag 内由同一 SLAM 系统生成的 /pose、/odom 或 /map 充当真值。提交时附测量表和原始证据，并把结果回填英文报告。

## 当前报告的边界

- 输入来自所提供的实体小车 bag；估计器仅使用原始二维 LiDAR 与单目 RGB，CameraInfo 和静态 TF 用于标定。没有使用 IMU、GNSS、磁力计、轮速/录制里程计或动态 TF。
- `/scan` 重复时间戳为 0 次、同时间戳量程冲突为 0 次（该话题存在情况以审计清单为准）；实际使用 `/scan/raw`。图像解码失败为 2 帧，记录在审计中。
- 相机采用 6 帧 LK 跟踪、记录的 K/D 去畸变，以及 LiDAR 轨迹和相机外参提供的米制基线，对首末两帧进行三角化，再用全部 6 帧做构建样本内筛选；没有进行所有视图的联合优化。深度、视差、基线、正反向光流和多帧重投影检查后保留 29 个稀疏视觉 XYZ 候选点。这些点增加地图的视觉几何内容；它们共享 LiDAR 位姿和已记录外参，并不是独立定位或高度真值。采用世界坐标向上视线筛选结构候选点，没有任意设置世界高度截断值。
- 安装变换和内参来自记录，未独立完成物理标定验证。扫描过程中的运动畸变尚未补偿。
- 主 map.pcd 合并 LiDAR 的 z=0 平面端点与相机三角化的稀疏三维地标。lidar_scan_map.pcd 与 visual_landmarks.pcd 分别保留两种来源。视觉高度相对于初始 LiDAR 扫描平面，依赖车辆平面运动及记录的相机外参；不是完整、稠密或独立验证的三维重建。二维定位使用 lidar_scan_map.pcd 或 LiDAR 占据地图，禁止直接把视觉三维点投影到 XY 后混入定位地图。
- 当前运行结果、图和图像对必须来自同一轮计算；报告生成器 `--strict` 会拒绝检测到的快照不一致。预览文件为 `outputs/fin301/report/report_preview.png`。

相机提出并通过 LiDAR 验证的回访约束为 0 条；最大图优化平面修正为 0.000 m。零回访意味着相机未通过回环修正位姿；视觉地标建图是另一个独立记录的贡献。约束数量、点数和修正幅度都不证明相对真实位置误差一定降低。

另外，经几何验证的 LiDAR 回访约束为 1 条，是否实际应用：True；全部图约束造成的最大平面修正为 1.498 m。这项修正不能归因于相机，也不是相对真值误差降低的幅度。

本数据集的 ROS 验证证据路径：`outputs/fin301/verification/ros_final_artifacts_test.txt`；以该日志明确检查的范围为准。

重新生成本数据集报告：

```text
.venv/Scripts/python.exe scripts/build_report.py --result "outputs/fin301/result" --audit "outputs/fin301/audit/audit.json" --output "outputs/fin301/report" --dataset-name "fin301 / FinTech301" --notes-output "docs/report_notes_fin301_zh.md" --strict --runtime-evidence "outputs/fin301/verification/ros_final_artifacts_test.txt" --localization-evidence "outputs/fin301/verification/localization_final/localization_evaluation.json" --camera-validation-evidence "outputs/fin301/verification/visual_landmark_review_heldout.json"
```

## 个人总结模板（与小组报告分开填写）

姓名 / 学号：[由本人填写]

实际承担的工作：[只写本人完成的任务；列出相应代码、实验或记录]

关键决定及依据：[说明一个真实技术选择、替代方案和证据]

遇到的问题与改进：[记录复现步骤、诊断和最终处理，未解决的问题也应说明]

验证与局限：[本人实际运行过的检查；独立测量仍缺失时明确注明]

协作与引用：[说明合作范围、外部库/文档及辅助工具的使用，遵循课程规定]

这是一份供核对和继续补实验的草稿，不能据此声称已经完成所有评分要求。
