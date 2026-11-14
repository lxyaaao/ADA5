# fin301 / FinTech301：本次数据的运行与交付说明

本次输入文件是 `fin301.zip`，解压后的记录为 `data/fin301/fin301_0.db3`。本文用 **fin301 / FinTech301** 标识用户本次提供的数据；所有新缓存、审计、地图和报告写入 `outputs/fin301/`，与先前 gorehall 结果分开保存。

**二维定位请使用 [lidar_scan_map.pcd](../outputs/fin301/result/lidar_scan_map.pcd) 或 LiDAR 占据地图。** 重点是扫描配准能否从不同初值稳定收敛到一致位姿；地图观感和点数不能证明定位精度。下面的留出扫描实验记录成功接受率与同一查询的多初值分歧，同时保留失败计数。

交付的 [map.pcd](../outputs/fin301/result/map.pcd) 合并 LiDAR 平面扫描点与相机多帧三角化的稀疏 XYZ 候选地标，用于核查两种传感器的建图内容。两类点都使用第一帧 LiDAR 定义的局部米制坐标。`lidar_scan_map.pcd` 的 `z=0`；`visual_landmarks.pcd` 是独立实验附件。**不要将视觉 XYZ 直接投影到 XY 后混入二维定位地图。** 视觉高度依赖平面车辆运动和记录外参，并非独立高度测量或完整稠密三维重建；当前没有证据证明这些点提高定位精度。

运行中的文件可能尚未形成一致快照，最终结论必须结合完成后的 `metrics.json`、`loop_events.json` 和报告清单。最终 `metrics.geometry` 应为 `lidar_scan_plane_plus_triangulated_visual_landmarks`；旧的临时纯激光输出没有此几何标识。

## 在项目根目录复现

以下命令在解压后的项目根目录运行。新机器需先安装 Python 3.11，再创建环境并安装 [requirements.txt](../requirements.txt)。已有可用环境时可跳过创建步骤。这里直接调用虚拟环境解释器，无需修改 PowerShell 执行策略。

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

若系统没有 `py` 启动器，可把 `py -3.11` 换成已安装 Python 3.11 的完整路径。若目录尚无解压后的 bag，将原始 `fin301.zip` 放在项目根目录，再执行下面的解压和处理命令。不要把 Windows 虚拟环境复制到 Linux 使用。

```powershell
# 仅在尚未解压时执行；保留原 ZIP。
Expand-Archive -LiteralPath .\fin301.zip -DestinationPath .\data

.\.venv\Scripts\python.exe scripts\audit_bag.py --bag data/fin301/fin301_0.db3 --output outputs/fin301
.\.venv\Scripts\python.exe -m indoor_mapping.pipeline --cache outputs/fin301/cache --output outputs/fin301/result --lidar-revisits
.\.venv\Scripts\python.exe scripts\evaluate_localization.py --cache outputs/fin301/cache --trajectory outputs/fin301/baseline/trajectory.csv --baseline-trajectory outputs/fin301/baseline/trajectory.csv --protocol outputs/fin301/verification/localization_protocol.json --output outputs/fin301/verification/localization_baseline
.\.venv\Scripts\python.exe scripts\evaluate_localization.py --cache outputs/fin301/cache --trajectory outputs/fin301/result/trajectory.csv --baseline-trajectory outputs/fin301/baseline/trajectory.csv --protocol outputs/fin301/verification/localization_protocol.json --compare outputs/fin301/verification/localization_baseline/localization_evaluation.json --output outputs/fin301/verification/localization_final
.\.venv\Scripts\python.exe scripts\plot_localization_quality.py --evaluation outputs/fin301/verification/localization_final/localization_evaluation.json --pcd outputs/fin301/result/lidar_scan_map.pcd --output outputs/fin301/verification/localization_final/localization_quality.png
.\.venv\Scripts\python.exe scripts\build_report.py --result outputs/fin301/result --audit outputs/fin301/audit/audit.json --output outputs/fin301/report --dataset-name "fin301 / FinTech301" --notes-output docs/report_notes_fin301_zh.md --localization-evidence outputs/fin301/verification/localization_final/localization_evaluation.json --strict
.\.venv\Scripts\python.exe -m pytest -q
```

必须等待映射命令结束再生成报告。`--strict` 会检查事件数量、轨迹数量、图片时间，以及所选定位实验的轨迹/协议散列等一致性条件；它不能替代物理精度测量。若用 `--max-seconds` 做短测试，请指定独立输出目录。`--resume` 只适合缓存和里程计参数不变、需要重新执行相机/位姿图阶段的情况；报告数值来自最后一次成功生成的结果。

交付保留 `outputs/fin301/baseline/trajectory.csv` 作为固定比较基线；`--lidar-revisits` 启用额外的几何 LiDAR 回访约束，默认不加该参数时仍是原始无此约束的方案。若需完全重建基线，请把不带该参数的 pipeline 输出到新目录（例如 `outputs/fin301/reproduced_baseline`），并在评估命令中使用该目录的轨迹；保留历史诊断引用的原 `baseline` 文件。

报告也接受 `--camera-validation-evidence outputs/fin301/verification/visual_landmark_review_heldout.json` 和 `--runtime-evidence outputs/fin301/verification/ros_final_artifacts_test.txt`，用于引用随交付保存的诊断与 ROS 日志；这些参数不会重新执行实验。应确保文件存在并确认它们对应的实验范围。最终报告的完整生成命令和所有来源散列保存在报告及 `report_manifest.json` 中。

## 本次录制已经核实的数据事实

以下来自 [audit.json](../outputs/fin301/audit/audit.json)，属于输入审计，不是算法精度：

| 项目 | fin301 审计结果 |
| --- | --- |
| 原始 LiDAR | 17,628 帧，1,440 束/帧，时间跨度 591.900 秒 |
| RGB | 5,879 个压缩图像消息；成功解码图像尺寸 640×480 |
| 初始化时间差 | 相机首帧晚于原始 LiDAR 首帧约 3.033 秒 |
| 图像解码失败 | 2 帧，审计图像 ID 为 2764、3045 |
| `/scan` 时间戳 | 无重复时间戳、无同时间戳量程冲突 |
| `/scan` 与 `/scan/raw` | 17,570 个共同时间戳对应的量程数组全部不同，不能视为相同输入 |
| 图像到最近原始扫描 | 时间差中位数 8.397 ms、p95 15.852 ms、最大 17.228 ms |

初始约三秒没有相机图像，估计器应在这段时间依靠 LiDAR，不可将后来的图像当作同步观测。上述图像到扫描的统计也不等于“每帧 LiDAR 都有 50 ms 内的图像”；方向不同的关联统计分别保存在审计中。时间戳接近不证明硬件同步。

本次审计中 CameraInfo 与静态 TF 可用；它们是记录内参数，尚没有独立实物标定证据。估计器只使用原始二维 LiDAR、单目 RGB 以及需要的标定元数据，不使用 IMU、GNSS、磁力计、轮速/录制里程计、动态 TF 或已有地图。导出的旧 `/map` 仅是标为 `UNVERIFIED` 的来源记录，不能当作本次地图或真值。

## 如何检查本次输出

| 路径 | 含义 |
| --- | --- |
| [result/map.pcd](../outputs/fin301/result/map.pcd) | 联合 PCD，供两种来源的几何检查 |
| [result/lidar_scan_map.pcd](../outputs/fin301/result/lidar_scan_map.pcd) | 二维定位用 LiDAR 扫描平面点，z=0 |
| [result/visual_landmarks.pcd](../outputs/fin301/result/visual_landmarks.pcd) | 相机构建视图内筛选的实验性稀疏 XYZ 候选地标 |
| [result/pcd_preview.png](../outputs/fin301/result/pcd_preview.png) | 实际 PCD 的平面预览 |
| [result/map_trajectory.png](../outputs/fin301/result/map_trajectory.png) | 占据地图与轨迹 |
| [result/trajectory.csv](../outputs/fin301/result/trajectory.csv) | 原采集时间戳、`x,y,yaw`、LiDAR 位姿及参考坐标系 |
| [result/metrics.json](../outputs/fin301/result/metrics.json) | 当前计算的覆盖范围、约束数、内部残差等 |
| [result/loop_events.json](../outputs/fin301/result/loop_events.json) | 图像候选及 LiDAR 验证结果 |
| [result/camera_landmarks.png](../outputs/fin301/result/camera_landmarks.png) | 视觉地标的图像证据；不是回访示意图 |
| [result/visual_landmark_evidence.csv](../outputs/fin301/result/visual_landmark_evidence.csv) | 每个保留地标的各构建帧像素、时间戳及几何检验结果 |
| [result/visual_landmark_stats.json](../outputs/fin301/result/visual_landmark_stats.json) | 视觉地标统计、筛选参数和假设 |
| [report/group_technical_report.pdf](../outputs/fin301/report/group_technical_report.pdf) | 一页英文报告 |
| [report/group_technical_report.md](../outputs/fin301/report/group_technical_report.md) | 可编辑报告，图片使用相对路径 |
| [report/report_manifest.json](../outputs/fin301/report/report_manifest.json) | 报告来源文件散列和快照检查 |
| [report_notes_fin301_zh.md](report_notes_fin301_zh.md) | 本数据集的中文结果说明与待补实验 |
| [localization_evaluation.json](../outputs/fin301/verification/localization_final/localization_evaluation.json) | 最终轨迹的留出扫描定位统计、逐次结果及基线比较 |
| [localization_quality.png](../outputs/fin301/verification/localization_final/localization_quality.png) | 查询位置与多初值一致性；坐标来自估计轨迹 |
| [localization_protocol.json](../outputs/fin301/verification/localization_protocol.json) | 不随候选方法变化的查询、训练扫描 ID 与初值 |
| [map_selection.md](../outputs/fin301/verification/map_selection.md) | 对照实验、采用/拒绝方案及其依据 |
| [visual_landmark_review_final.json](../outputs/fin301/verification/visual_landmark_review_final.json) | 独立代码复算最终视觉点的构建视图内几何条件；不是独立物理精度 |

相机的两类作用在统计中分别记录：外观检索和 ORB/RANSAC 可以提出回访候选，只有通过 LiDAR 验证后才加入位姿约束；另一条分支使用多帧图像为地图增加视觉地标。`accepted_camera_lidar_loops=0`、`camera_constraints_applied=false` 意味着本次没有通过相机回环修正位姿，不应声称相机提高了定位精度。`visual_point_count` 和 `camera_mapping_contribution` 记录相机是否实际增加了通过几何验证的地图点。

额外的 LiDAR 回访分支独立记录在 `accepted_lidar_revisits` 与 `lidar_revisit_constraints_applied`。它根据几何重叠、双向配准与退化检查决定是否向 SE(2) 位姿图加入约束；若应用，轨迹修正来自 LiDAR，而非相机。`max_graph_correction_m` 是估计轨迹的改变量，不是相对真值误差降低量。

本次最终结果接受 1 条 LiDAR 回访约束，最大图修正约 1.498 m；相机回环修正仍为 0。最终导出 49,783 个 LiDAR 点和 29 个视觉候选点，共 49,812 点。具体数值以同轮运行的 `metrics.json` 为准。

视觉地标分支默认在约 0.3 秒间隔的六帧中进行 LK 特征跟踪；将记录的 CameraInfo `K,D` 用于去畸变，由 LiDAR 位姿插值和 `T_camera_lidar` 计算米制相机姿态，再三角化首末帧并用全部六帧筛选，没有进行六视图联合优化。检查包括首末帧基线 0.12–1.5 m、视差至少 2°、各帧相机深度均为 0.4–12 m、每帧原图重投影误差不超过 2 px，以及 LK 正反向误差不超过 0.8 px。世界坐标中向上的观测射线用于选择结构候选点，没有任意设置世界高度截断值。相机时刻不得外推，LiDAR 插值区间不得超过 0.5 秒。XYZ 用 0.05 m 体素去重，保留实际三角化点。最终生效视图数量、筛选参数与统计以 `metrics.visual_landmarks.parameters` 为准。

六帧都参与构建和筛选，因此它们属于**样本内一致性检查**，不能重复称作独立留出验证。较早四视图方案的 93 个点中，只有 10 个在另两帧中同时满足 2 px 预测门槛；该诊断保存在 [visual_landmark_review_heldout.json](../outputs/fin301/verification/visual_landmark_review_heldout.json)，不适用于宣称最终六视图方案已经通过独立精度测试。

视觉点使用与 LiDAR 相同的轨迹和记录外参，不能拿它们反过来充当 LiDAR 定位的独立真值。占据地图仍由二维 LiDAR 构建；主 PCD 保留两种来源，便于分别核查相机贡献。审计抽查的起始图像含行人与家具腿，末段画面主要为近处低纹理表面；动态目标和缺少纹理可能降低稳定跟踪和候选识别。具体观察见 [独立数据审查](../outputs/fin301/verification/data_review.md)，不能据此推断每个失败候选的原因。

## 定位稳定性的开发验证

基线协议选取 100 个未参与里程计/地图插入的原始扫描；参考地图仅用训练 LiDAR 点，排除距离查询时间 5 秒以内或扫描原点相距 0.5 米以内的参考扫描。每个查询尝试 8 个固定初值，扰动为 0.25/0.50 m 和 5/10°。配准接受门槛为重叠比例至少 0.5、点到线残差不超过 0.10 m、条件数不超过 5000；“稳定查询”还要求全部 8 次被接受，且解两两相差不超过 0.15 m 和 3°。

同一冻结协议的开发验证结果如下。各方法的参考扫描 ID、查询 ID、初值与评分代码一致，只有参考地图使用的估计轨迹/表示发生变化。

| 指标 | 无额外 LiDAR 回访的基线 | 最终采用的 1 条 LiDAR 回访 |
| --- | ---: | ---: |
| 被接受配准 / 全部初值 | 707 / 800 | 712 / 800 |
| 全部初值一致的查询 | 67 / 100 | 71 / 100 |
| 查询内最大位置分歧 p95 | 0.528 m | 0.00910 m |
| 查询内最大位置分歧的最大值 | 1.444 m | 1.046 m |
| 至少两次配准被接受、可计算分歧的查询 | 91 | 92 |
| 仅用至少 30 秒前参考观测的稳定查询 | 11 / 20 | 11 / 20 |

这些指标支持采用当前 LiDAR 回访修正，但仍有 29/100 个查询未达到全部初值一致，较早观测子集也没有增加稳定查询。曾尝试的多扫描支持过滤表示降低了接受率与稳定查询数，因此没有采用。残差只统计被接受配准，失败计数仍保留在接受率分母；不能只凭小残差判断表现良好。完整选择记录见 [map_selection.md](../outputs/fin301/verification/map_selection.md)。

协议固定查询、训练扫描 ID 和基线初值，候选轨迹通过 `--trajectory` 替换，并复用 `--protocol`，再用 `--compare` 指向基线 JSON。估计基线仍影响样本选取和初值；这些查询被用于方案选择，所以是**开发验证集**，不是未参与选型的最终测试集。0.00910 m 表示被接受解之间的 p95 分歧，不是相对真实位置的误差，不支持“厘米级定位”结论。仍需新录制的独立测试数据和实体参照验证。

用支持 PCD 的 CloudCompare 选择 File → Open，或运行 `pcl_viewer outputs/fin301/result/map.pcd -ax 1 -ps 2`。标准 ROS 2 查看已有结果的方式如下，路径应换成 Linux/WSL 中实际的绝对路径：

```bash
ros2 launch indoor_mapping mapping.launch.py \
  process_cache:=false output_dir:=/absolute/path/to/outputs/fin301/result
```

ROS 适配器发布历史 `/map`、`/map_points`、`/trajectory`，不发布实时车辆 TF。只有本数据集对应的运行日志才能作为 fin301 的 ROS 测试证据；不能直接沿用旧数据的测试时长或接收数量。

本次已在 Ubuntu 24.04 / ROS 2 Jazzy 中完成两类实际验证：[直接发布最终 Windows 结果](../outputs/fin301/verification/ros_final_artifacts_test.txt)核对了 49,812 个点、3,526 个历史位姿及每个占据栅格；[在 ROS 节点内重新处理完整缓存](../outputs/fin301/verification/ros_processing_final_test.txt)验证了处理后发布与晚加入订阅者接收。ROS 自行重建得到 49,813 个点，与 Windows 结果存在 1 个 LiDAR 点的差异，因此不宣称跨平台 PCD 逐字节相同；每项发布测试均对照相应来源文件验证。

若尚未构建 ROS 包，可在安装了 ROS 2 的 Linux/WSL 主机上，将本项目放到 `~/mapping_ws/src/indoor_mapping`。下面以 Jazzy 为例，在 `~/mapping_ws` 中执行：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select indoor_mapping --symlink-install
source install/setup.bash
ros2 launch indoor_mapping mapping.launch.py \
  process_cache:=false output_dir:=/absolute/path/to/outputs/fin301/result
```

`process_cache:=false` 读取已在 Windows 或 Linux 离线生成的结果；若要在 ROS 节点内重新运行最终方案，设为 `true`，同时传入 `lidar_revisits:=true` 和 `cache_dir:=/absolute/path/to/outputs/fin301/cache`。该回访开关默认关闭，必须显式开启才能复现最终修正。提取缓存仍需用 `scripts/audit_bag.py`，相关 Python 依赖必须安装在实际执行它的环境中。节点不是对 `ros2 bag play` 的实时订阅器。

在 RViz 中将 Fixed Frame 设为 `map`，分别添加 Map、PointCloud2、Path 显示项。话题使用 reliable、transient-local、depth=1；地图和轨迹消息保留录制的采集时间戳。`/map_points` 传输联合 XYZ 点云，`/map` 仍是 LiDAR 平面占据地图。

目前没有本次场景的独立墙长、检查点或参考轨迹。ICP 残差、首尾估计距离、图优化代价属于内部一致性指标，不能改称厘米级绝对定位精度。最终提交还需要核实小组姓名与个人贡献，并补充课程要求的独立物理验证。
