# Armstrong 遥操作数据集工作流

本文描述从两路相机、Armstrong 右臂和 CTAG2F120 夹爪生成可用于
LeRobot/SmolVLA 训练的数据集。录制器和导出器均不发布机器人控制命令；
机械臂是否运动只由已有遥操作安全栈决定。

## 采用的格式

目标格式是 **LeRobot Dataset v3**（`lerobot>=0.4.0`），不是旧版 v2.1
的“每个 episode 一个 Parquet/MP4”结构。v3 会自动把多个 episode 合并到
文件分片中：

```text
dataset_root/
├── meta/
│   ├── info.json
│   ├── stats.json
│   ├── tasks.jsonl
│   └── episodes/chunk-000/file-000.parquet
├── data/chunk-000/file-000.parquet
└── videos/
    ├── observation.images.head/chunk-000/file-000.mp4
    └── observation.images.wrist_right/chunk-000/file-000.mp4
```

不要手动改这些文件的名称、JSON、Parquet 或 episode index。导出器只通过
LeRobot 官方 `LeRobotDataset.create/resume/add_frame/save_episode/finalize`
API 写入。

## 为什么先录 ROS bag，再导出 LeRobot

采集分为两层：

1. `ros2_episode_recorder.py` 无损保存原始 ROS 话题和任务元数据。
2. `export_rosbag_to_lerobot.py` 离线做时间同步、质量检查和 LeRobot 写入。

这样相机偶发掉帧、话题命名错误或导出参数变化时，原始证据仍在，可以重新
导出，不需要重新让机器人执行任务。

原始 episode 文件夹按本地日期和时间命名，例如：

```text
datasets/20260730_143012_pick_water_bottle/
├── episode_metadata.json
├── rosbag/
└── lerobot_export_report.json
```

日期时间仅用于原始会话和追溯；LeRobot 内部 episode 编号仍由官方 API
自动维护。

## 标准特征

| LeRobot 特征 | 维度 | 来源 |
| --- | ---: | --- |
| `observation.state` | 8 | 7 个真实右臂关节角 + 夹爪二值状态估计 |
| `action` | 8 | 7 个经过限速/限加速度后实际送入 JAKA SDK 的目标 + 夹爪已接受命令 |
| `observation.images.head` | RGB | 头部相机 |
| `observation.images.wrist_right` | RGB | 右腕相机 |
| `observation.gripper_contact` | 1 | 夹爪 torque/contact 状态 |
| `observation.gripper_feedback_valid` | 1 | 夹爪位置回读是否可信 |

关节角单位是弧度。夹爪统一约定为：

- `0.0`：全开命令；
- `1.0`：闭合命令。

当前 CTAG2F120 固件的位置寄存器曾出现无效零值，因此第 8 维 observation
暂时是“最近一次开合命令的状态估计”，不是精确实际开度。该事实通过
`observation.gripper_feedback_valid` 和导出报告明确记录。夹到物体后不要求
电机一定走到闭合端点；`torque_reached/contact` 才表示受力停止。不要为了
追求数值 1.0 而要求夹爪压到底。

## 频率与同步

- JAKA 伺服循环：125 Hz（8 ms），负责平滑执行。
- LeRobot 数据集：固定 30 FPS（每帧约 33.33 ms）。
- 相机：请求 30 FPS。
- 导出：在统一 30 FPS 时间栅格上选择最近的关节/action/相机样本；二值状态
  使用时间上最近的历史值，绝不使用未来命令。

控制循环不需要降成 30 Hz，也不需要相机升到 125 Hz。训练数据只需要在每个
30 FPS 帧时刻取得同步的 observation 和实际 action。导出器默认拒绝：

- 相机时间偏差超过 50 ms；
- 实际关节反馈偏差超过 75 ms；
- 执行 action 偏差超过 20 ms；
- 相机重复帧比例超过 5%；
- 不足 30 帧的 episode。

每次导出都会生成 `lerobot_export_report.json`，包含各话题平均、P95 和最大
时间偏差，以及相机重复帧比例。

## 两路相机节点

独立启动文件：

```bash
ros2 launch one_arm_teleop_bridge dataset_cameras.launch.py
```

它启动两个 `usb_camera_publisher`：

- `/cameras/head/image_raw/compressed`
- `/cameras/wrist/image_raw/compressed`

并发布：

- `/cameras/head/status`
- `/cameras/wrist/status`

状态中包含实际发布 FPS、读帧失败、JPEG 编码失败和定时器掉周期计数。

配置文件是
`one_arm_teleop_bridge/config/dataset_cameras.yaml`。其中两个序列号来自旧体感
遥操作项目：

- 头部：`CP8A9450002M`
- 右腕：`CP8A9450001Z`

这些序列号和 V4L2 兼容性尚未在真机复核。节点会按
`/dev/v4l/by-id` 查找，找不到或不能满足 1280×720@30 FPS 时默认拒绝启动，
不会悄悄降帧。若 Orbbec 设备只支持厂家 ROS 驱动，则保留相同话题名，替换
相机源节点即可，录制和导出部分不需要改。

Ubuntu 依赖：

```bash
sudo apt install python3-opencv
```

## 单个 episode 的操作顺序

以下命令留作回到实验室后的执行手册；当前不需要运行。

### 1. 只检查话题

遥操作栈和两路相机都已启动后：

```bash
cd ~/onearm_teleop/One-Arm-Teleoperation
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 tools/ros2_episode_recorder.py \
  --name preflight \
  --task "把农夫山泉放进大纸箱" \
  --preflight-only
```

LeRobot profile 默认要求实际关节反馈、实际执行 action、夹爪状态和两路相机
均有活跃 publisher。缺任何一个都会拒绝开始。

### 2. 开始原始录制

```bash
python3 tools/ros2_episode_recorder.py \
  --name pick_water_bottle \
  --task "把农夫山泉放进大纸箱" \
  --operator Lucky \
  --fps 30
```

任务完成后按 `Ctrl+C` 结束录制。任务字符串会原样写入元数据，并保存 SHA-256
校验值。工具拒绝首尾空格，但不会擅自改写中文。

### 3. 人工复核成功/失败

```bash
python3 tools/set_episode_outcome.py \
  datasets/20260730_143012_pick_water_bottle \
  --outcome success
```

失败 episode 应标记 `failure` 并保留用于排错；默认导出器不会把失败或未复核
episode 混入行为克隆训练集。

### 4. 先做只验证导出

```bash
python3 tools/export_rosbag_to_lerobot.py \
  --episode-dir datasets/20260730_143012_pick_water_bottle \
  --dataset-root ~/lerobot_datasets/water_bottle \
  --repo-id jason0925pig-rgb/water_bottle \
  --dry-run
```

`--dry-run` 会读取和同步数据、检查图像与时间质量，但不会创建或修改 LeRobot
数据集。

### 5. 正式导出

确认质量报告后去掉 `--dry-run`。第一次创建数据集，后续 episode 自动
`resume` 追加：

```bash
python3 tools/export_rosbag_to_lerobot.py \
  --episode-dir datasets/20260730_143012_pick_water_bottle \
  --dataset-root ~/lerobot_datasets/water_bottle \
  --repo-id jason0925pig-rgb/water_bottle
```

每次必须完成 `finalize()`；工具已自动处理。禁止同时运行两个导出进程写同一个
dataset root。

## 任务表

后续 CSV/Excel 建议至少包含：

```text
task_id,task,planned_episodes,object,scene,notes
```

真正写入 LeRobot 的是 `task` 原文。任务表导入时应进行精确匹配和重复检查，
不自动翻译、不自动去同义词、不自动添加标点。建议一组数据统一一种语言和固定
句式，例如始终使用“把农夫山泉放进大纸箱”，不要在不同 episode 中混用
“拿起水瓶放箱子里”。

## 目前仍需回实验室确认

1. 两个旧序列号是否仍分别对应头部和右腕相机。
2. 相机是否能通过 V4L2 稳定输出 1280×720@30 FPS；否则切换 Orbbec ROS2
   驱动并保持话题名。
3. 两路相机的安装方向、画面遮挡、曝光和白平衡。
4. CTAG2F120 是否有可用的真实位置/力数据寄存器；在确认前不得把估计值宣传
   为真实开度或牛顿力。
5. 真实任务长度下 CPU、磁盘写入、相机重复帧率和 ROS 时间偏差。
