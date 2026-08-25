# LingBot-VA 1.0：新 50 组真机数据的 A800 训练交接

> 目的：让下一位执行者只用**本次新采集的 50 组**右臂摇操数据，基于 LingBot-VA 的公开基础权重启动一次全新的训练。本文不授权控制真机，也不包含部署步骤。

## 0. 必须遵守的范围

- 训练集只能是本次新任务的 **50 个已保存（S）的 episode**。
- 不要读取、合并或续训以下旧数据/旧权重：
  - 旧水瓶放箱子的 `onearm_Tele` 数据；
  - `/ssd/hanbo/TNNLS_2026/data/onearm_Tele/lingbot_va_v1_stage1_tcp_preliminary`；
  - SmolVLA checkpoint、QGF critic 或任何 QGF rollout；
  - 任意已有 LingBot 训练 checkpoint。
- 初始参数必须来自 `Robbyant/lingbot-va-base` 基础模型；`resume_from` 必须为空。
- 先做小规模 smoke test，再开始完整训练。不要把未验证的产物称为可真机部署的 adapter。

## 1. 数据现在在哪里

### Orin：本次新 50 组的唯一源

```text
/home/nvidia/work/telop/lingbot_va_teleop/
├── lerobot_dataset/    # 训练传输的输入：LeRobot v3 Parquet、两路 MP4、meta
└── raw_episodes/       # ROS bag / 原始运行记录，仅用于追溯或排错
```

传输的输入是 `lerobot_dataset/`，不是 `raw_episodes/`。采集器由
[`tools/start_lingbot_va_teleop_collection.ps1`](../tools/start_lingbot_va_teleop_collection.ps1)
启动；正常保存的每轮已经导出为 LeRobot 数据。

本次任务的文字不要靠人工回忆填写。训练前从源数据的 `meta/tasks.parquet` 或
`meta/episodes.jsonl` 读取并确认所有 50 组任务文本一致；任务文本将成为 LingBot
的 action text / text embedding 的来源。

### A800：本次训练专用的固定路径

以下目录由本次交接定义。若不存在，由执行者创建；不要复用旧目录。

```bash
export RUN_ID=20260825_new_task_50
export A800_DATA_ROOT=/ssd/hanbo/TNNLS_2026/data/onearm_Tele/lingbot_va/${RUN_ID}
export A800_SOURCE=${A800_DATA_ROOT}/source_lerobot_v3
export A800_READY=${A800_DATA_ROOT}/lingbot_ready_v2_1
export A800_OUT=/ssd/hanbo/TNNLS_2026/outputs/lingbot_va/${RUN_ID}_full_transformer

export LINGBOT_CODE=/ssd/hanbo/TNNLS_2026/work/lingbot_va_official_localcopy
export LINGBOT_BASE=/ssd/hanbo/TNNLS_2026/models/lingbot-va-base
export LINGBOT_ENV=/ssd/hanbo/TNNLS_2026/work/venvs/lingbot_va_train_py311
```

`LINGBOT_CODE` 是本地留存的 LingBot-VA 官方源码副本，对应本地记录的 upstream
commit `7c6ffa9bfc4b83582cafc860fab4c82cc7deeeeb`。它不是本仓库的一部分。

## 2. 从 Orin 传输到 A800

### 2.1 先在 Orin 做只读预检

```bash
export SRC=/home/nvidia/work/telop/lingbot_va_teleop/lerobot_dataset
test -f "$SRC/meta/info.json"
find "$SRC/data" -name '*.parquet' | wc -l
find "$SRC/videos" -name '*.mp4' | wc -l
du -sh "$SRC"
```

随后用 Python/LeRobot 元数据确认：

1. `episode_index` 恰好有 50 个且连续；
2. 每个 episode 都有两路视频；
3. 没有未完成、丢弃或异常中断的 episode；
4. `observation.state`、`action` 均存在；
5. 两个相机的实际 fps、分辨率和帧数写入 transfer manifest。

如果不是恰好 50 个，先输出 episode 清单让负责人决定；**禁止悄悄混入旧 episode**。

### 2.2 传输命令

优先使用 `rsync --partial --append-verify`，它可以中断后续传。执行端可选：

- 若 Orin 能以 SSH 直连 A800：从 Orin 推送；
- 若 A800 能直连 Orin：从 A800 拉取；
- 两者无法直连时，经已配置密钥的笔记本中转，但仍使用 `rsync` 或 WinSCP 的续传。

示例（从有权限的一端执行；不要把私钥写入 Git）：

```bash
mkdir -p "$A800_SOURCE"
rsync -aH --partial --append-verify --info=progress2 \
  /home/nvidia/work/telop/lingbot_va_teleop/lerobot_dataset/ \
  zwwl_user3@175.102.130.70:"$A800_SOURCE"/
```

实际连接必须使用管理员已配置的 A800 SSH 密钥/别名；不要在命令历史或本文件写密码。

### 2.3 传输完成校验（必须做）

在 Orin 与 A800 各生成一次文件大小清单并比较：

```bash
cd "$SRC"                 # Orin 上
find . -type f -printf '%P\t%s\n' | LC_ALL=C sort > source_file_sizes.tsv

cd "$A800_SOURCE"         # A800 上
find . -type f -printf '%P\t%s\n' | LC_ALL=C sort > received_file_sizes.tsv
```

将 Orin 的 `source_file_sizes.tsv` 传到 A800 后执行：

```bash
cmp -s source_file_sizes.tsv received_file_sizes.tsv \
  && echo TRANSFER_SIZE_MANIFEST_OK \
  || { echo TRANSFER_MISMATCH; exit 1; }
```

对 Parquet 和 MP4 各抽取至少一个 episode 实读；确认两路视频可解码、不是倍速视频。

## 3. LingBot 格式转换：LeRobot v3 -> 官方训练所需 LeRobot v2.1 + latent

公开 LingBot-VA 训练器不是直接吃当前的原始 MP4/Parquet。官方 README 定义的完整链路为：

```text
LeRobot 源数据
  -> 30维 action + action_config 的 LeRobot v2.1 数据
  -> Wan2.2 VAE 视频 latent + 文本 embedding
  -> LingBot-VA trainer
```

### 3.1 30 维 action 约定

官方布局是：

```text
[ 左臂 EEF(7), 右臂 EEF(7), 左臂关节(7), 右臂关节(7), 左夹爪(1), 右夹爪(1) ]
```

对于本项目：未使用的左臂通道为 0；有效右臂数据为 `right EEF[7:14]` 与
`right gripper[29]`。`observation.state` 应保留原始 7 轴实际状态和夹爪状态，供审计；
训练 `action` 使用转换出的右手 TCP 位姿与夹爪命令。

### 3.2 TCP/FK 是硬性前置条件

现有静态标定的临时变换是：

```text
rt -> TCP translation = [-0.00034602, -0.02245812, 0.27561081] m
rt -> TCP quaternion  = [0, 0, 0, 1]  (xyzw)
```

它由 5 个手摆姿势估计，残差 RMS **20.7 mm**、最大两两差 **47.7 mm**。因此它只能用于
数据管道 smoke test，不能作为“已验证的真机 TCP”。最终训练前应完成 CTAG2F120 真实
`rt -> TCP` 标定或至少额外静态复测，并将最终数值写进本次转换 manifest。

注意：`walle_description/urdf/walle.urdf` 的 `rt` 后面是灵巧手，不是 CTAG2F120，不能把
它的手指几何直接用于本夹爪 TCP。

### 3.3 每个 episode 必须生成的内容

在 `$A800_READY` 生成独立副本（或受控硬链接副本）而非改写 `$A800_SOURCE`：

```text
$A800_READY/
├── data/chunk-000/episode_XXXXXX.parquet
├── videos/chunk-000/observation.images.chest/episode_XXXXXX.mp4
├── videos/chunk-000/observation.images.wrist_right/episode_XXXXXX.mp4
├── meta/info.json
├── meta/episodes.jsonl             # 每行有 tasks 和 action_config
├── meta/tasks.parquet
├── empty_emb.pt
└── latents/chunk-000/
    ├── observation.images.chest/episode_XXXXXX_0_END.pth
    └── observation.images.wrist_right/episode_XXXXXX_0_END.pth
```

每个 `episodes.jsonl` 条目至少包含：

```json
{
  "episode_index": 0,
  "tasks": ["从源数据读取的精确任务文本"],
  "length": 0,
  "action_config": [{
    "start_frame": 0,
    "end_frame": 0,
    "action_text": "从源数据读取的精确任务文本"
  }]
}
```

这里的两个 `0` 是示意，实际 `end_frame` 必须等于该 episode 的 `length`。latent 文件名也必须
精确匹配 `episode_{index:06d}_{start_frame}_{end_frame}.pth`。

视频应保留双路 RGB，但 VAE 抽取时按官方建议 resize 到约 256×256，并明确记录最终 fps
（建议从源时间戳选择 15 fps；不要改变动作时间基准而只把视频“加速”）。每个 `.pth` 要至少
包含 `latent`、尺寸/帧数、`text_emb`、`text`、`frame_ids`、`fps` 与 `ori_fps`。

### 3.4 转换完成后的强制 smoke test

在训练前执行下列检查，任何一项失败都不开始训练：

1. `episodes.jsonl` 恰好 50 个 episode；
2. 两路相机每个 episode 都有 MP4 与对应 latent `.pth`；
3. action shape 固定为 `[T, 30]`，NaN/Inf 数为 0；
4. only-right-arm 约定成立：左臂通道全 0，右臂 EEF / 夹爪非空；
5. 逐 episode 检查 `action_config`、latent 文件名和真实长度一致；
6. 用官方 `wan_va.dataset.lerobot_latent_dataset.MultiLatentLeRobotDataset`
   成功加载至少一个 batch；
7. 将源码路径、50 个 episode id、任务文本、TCP 变换、转码 fps、源码/转换后的 SHA256
   写到 `$A800_READY/conversion_manifest.json`。

## 4. A800 环境与基础权重

### 4.1 当前事实

- A800 代码副本：`$LINGBOT_CODE`；
- 仅用于先前数据转换的 Python 环境：
  `/ssd/hanbo/TNNLS_2026/work/lingbot_va_convert_env`；它**不是**已验证的训练环境；
- LingBot 基础权重应放置于 `$LINGBOT_BASE`；此前 Hugging Face 从 A800 直连被拒绝，已确认
  ModelScope 可作为下载源；
- 基础权重尚未确认完整下载，训练开始前必须检查 `transformer/`、`text_encoder/`、`tokenizer/`
  与 `vae/` 都存在且文件大小/哈希完整。

推荐为训练新建隔离环境：

```bash
python3.11 -m venv "$LINGBOT_ENV"
source "$LINGBOT_ENV/bin/activate"
python -m pip install -U pip wheel setuptools
cd "$LINGBOT_CODE"
python -m pip install -r requirements.txt
python -m pip install -e .
```

官方 requirements 锁定 PyTorch 2.9 / CUDA 12.6、`diffusers==0.36.0`、
`transformers==4.55.2`、`lerobot==0.3.3` 与 `flash-attn`。必须先用单卡 import test
验证 `torch.cuda.is_available()`、`flash_attn`、Wan VAE 和 transformer 能加载；不要覆盖 A800
其他项目的环境。

### 4.2 不是 LoRA adapter 的事实

当前固定的官方源码 `wan_va/train.py` 对 transformer 调用 `requires_grad_(True)`，并在
`checkpoint_step_<N>/transformer/diffusion_pytorch_model.safetensors` 保存完整 transformer。
源码中没有现成 LoRA 注入/保存逻辑。

所以执行者必须二选一并如实记录：

1. **按官方代码全参数 post-training**：产物是完整 transformer checkpoint，不应称 LoRA adapter；
2. **另行实现并验证 LoRA**：只有明确添加 target modules、可训练参数清单、load/save 与复现实验后，
   才能称其为单任务 adapter。

本交接默认选项是 1，且只从 `$LINGBOT_BASE` 开始，不从任何任务 checkpoint 恢复。

## 5. 训练配置与产物

不要直接运行官方 `robotwin_train`：它默认三相机、RobotWin action 归一化和
`/path/to/your/dataset`，与本项目双相机单右臂不匹配。需要复制为一个新、可提交的配置，例如
`va_armstrong_right_train.py`，至少显式设置：

```text
dataset_path                    = $A800_READY 的父目录或包含该数据集的根
wan22_pretrained_model_name_or_path = $LINGBOT_BASE
save_root                       = $A800_OUT
resume_from                     = null / 空
obs_cam_keys                    = [chest, wrist_right]（与实际目录名一致）
env_type                        = 非 robotwin_tshape 的单右臂值
action_dim                      = 30
action normalisation            = 仅由本 50 组转换后的训练数据计算
enable_wandb                    = false（除非明确提供项目与密钥）
```

建议的执行顺序：

1. 单 GPU / 少量 step（例如 10）验证数据、latent、FSDP/保存路径；
2. 记录显存、吞吐、loss 是否有限；
3. 决定 GPU 数量和总 steps；
4. 全训练只写入 `$A800_OUT`；
5. 每次 checkpoint 后记录 config、git commit、环境 `pip freeze`、随机种子、数据 manifest；
6. 训练结束用独立的真机回合评估，不能用这 50 组演示本身冒充成功率。

官方启动器形式为：

```bash
cd "$LINGBOT_CODE"
source "$LINGBOT_ENV/bin/activate"
NGPU=<N> CONFIG_NAME=armstrong_right_train \
  bash script/run_va_posttrain.sh
```

这条命令只有在新增的 `armstrong_right_train` 配置通过 smoke test 后才能执行。默认脚本的
`NGPU=8` 不能直接照用；必须按当时空闲卡数设置，且不占用其他人的作业。

## 6. 交付物清单

训练执行者完成后应交付：

```text
$A800_OUT/
├── checkpoints/checkpoint_step_<N>/transformer/
│   ├── diffusion_pytorch_model.safetensors
│   └── config.json
├── training_config_resolved.yaml
├── train.log
├── environment_pip_freeze.txt
├── dataset_manifest_used.json
├── tcp_transform_used.json
└── TRAINING_SUMMARY.md
```

`TRAINING_SUMMARY.md` 必须明确：基础权重来源及哈希、是否全参数训练或 LoRA、只使用的 50 个
episode id、训练卡数/步数、最终 checkpoint、所有已知限制（尤其 TCP 标定状态）。

## 7. 当前阻塞项

1. A800 的 LingBot 基础权重尚需通过 ModelScope 完整下载与校验；
2. 50 组新数据尚需实际传输并在 A800 上确认数量；
3. CTAG2F120 的最终 TCP 标定尚未达到生产级精度；
4. 官方代码需要新增本项目的双相机、单右臂配置与 latent 提取脚本。

这些完成前，不能声称“已经得到新的 LingBot adapter/checkpoint”。

## 8. 训练完成后：回传 Orin 并准备次日运行（全程不许上电/使能）

本节是给训练执行者的交付要求。它的目标是让次日到现场时**软件、权重和只读检查已经齐全**，
而不是让 A800 或 Orin 在无人看护时控制机械臂。

### 8.1 回传的内容和 Orin 目录

完成训练后，不能只复制一个 `diffusion_pytorch_model.safetensors` 就认为可部署。Orin 运行时还需要
与训练完全匹配的 base bundle（`vae/`、`text_encoder/`、`tokenizer/`、配置）以及本次新训练的
transformer。

建议在 Orin 建立一个完全独立的运行目录：

```bash
export ORIN_ROOT=/home/nvidia/work/telop
export ORIN_LINGBOT=${ORIN_ROOT}/lingbot_va_runtime/20260825_new_task_50
export ORIN_BASE=${ORIN_LINGBOT}/base_bundle
export ORIN_CHECKPOINT=${ORIN_LINGBOT}/checkpoint
export ORIN_RUNTIME=${ORIN_LINGBOT}/runtime
mkdir -p "$ORIN_BASE" "$ORIN_CHECKPOINT" "$ORIN_RUNTIME"
```

回传后目录必须满足：

```text
$ORIN_LINGBOT/
├── base_bundle/
│   ├── vae/
│   ├── text_encoder/
│   ├── tokenizer/
│   └── （基础模型所需的其余只读配置）
├── checkpoint/
│   └── transformer/
│       ├── diffusion_pytorch_model.safetensors
│       └── config.json
├── runtime/
│   ├── resolved_runtime_config.yaml
│   ├── model_manifest.json
│   └── README_RUNBOOK.md
└── SHA256SUMS
```

`checkpoint/transformer/` 必须来自本次 `$A800_OUT` 的最终 checkpoint，其他内容来自**同一个**
`$LINGBOT_BASE` 基础模型。不要把旧 SmolVLA checkpoint、QGF critic 或其他任务的 LingBot
checkpoint 放入该目录。

### 8.2 允许笔记本中转

若 A800 和 Orin 之间没有可用的 SSH 路由，可以由 Windows 笔记本中转；这完全可行。
使用可恢复传输，不要用 ZIP 后再解压来掩盖文件缺失。

PowerShell 示例（路径和私钥按现场实际配置替换）：

```powershell
# 1) A800 -> Windows：下载最终 checkpoint 与运行所需 base bundle。
$stage = "$env:USERPROFILE\Downloads\lingbot_va_20260825_new_task_50"
New-Item -ItemType Directory -Force -Path $stage | Out-Null
scp -r -i "$env:USERPROFILE\.ssh\a800_zwwl_user3_ed25519" `
  zwwl_user3@175.102.130.70:/ssd/hanbo/TNNLS_2026/outputs/lingbot_va/20260825_new_task_50_full_transformer `
  $stage

# 2) Windows -> Orin：上传到一个新目录；不要覆盖已有模型。
scp -r -i "$env:USERPROFILE\.ssh\one_arm_teleop_ed25519" `
  $stage\* nvidia@192.168.2.170:/home/nvidia/work/telop/lingbot_va_runtime/20260825_new_task_50/
```

大文件建议使用 WinSCP 的 SFTP 队列/续传，或把上述 `scp` 替换为 `rsync --partial --append-verify`。
回传后，在 A800、Windows（如中转）和 Orin 对最终 checkpoint 计算 SHA256；三者必须一致。

### 8.3 Orin 只读验收：不连控制、不上电、不使能

以下测试只允许加载文件、解码录像、运行模型前向或检查 ROS topic 类型；不得调用：

```text
/right_arm/set_powered_on=true
/right_arm/set_robot_enabled=true
/right_arm/set_motion_enabled=true
/right_arm/set_gripper_enabled=true
任何 servo_j / servo mode / gripper motion 请求
```

必须完成并保存日志：

1. `sha256sum -c SHA256SUMS`；
2. Python 能 import LingBot runtime、加载 VAE / text encoder / 新 transformer；
3. 用一个保存的双相机 observation 做**离线**前向，检查动作 tensor 全为有限数，且 action 维度与
   本项目转换约定匹配；
4. 用录制 MP4 验证 chest 与 wrist 的时间戳、解码 fps、色彩顺序和 resize/crop 与训练一致；
5. 若需要 ROS 集成，只能启动 `dry_run=true` 且全部 `hardware_*_authorized=false` 的进程，确认
   其不发布控制消息；
6. 在 `runtime/README_RUNBOOK.md` 写出检查结果、模型 SHA256、实际推理延迟和所有失败原因。

没有现场人员明确执行 `ARM` / `MOVE`，也没有实体急停在手边时，禁止突破上述边界。

## 9. LingBot 的 15 Hz 异步推理与夹爪状态机要求

### 9.1 不要把 SmolVLA 客户端原样当成 LingBot 客户端

上一次可运行的 SmolVLA 异步运行参考代码在另一个仓库：

```text
E:/AAA__Github_Project/SmolVLA-with-QGF/
├── tools/smolvla_orin_env.sh
├── tools/run_smolvla_orin.sh
├── tools/start_smolvla_policy_server.sh
├── tools/start_smolvla_ros_client.sh
├── lerobot_robot_armstrong_ros2/src/lerobot_robot_armstrong_ros2/armstrongros2.py
├── lerobot_robot_armstrong_ros2/src/lerobot_robot_armstrong_ros2/configuration_armstrong_ros2.py
└── lerobot_robot_armstrong_ros2/src/lerobot_robot_armstrong_ros2/smolvla_guard.py
```

这些代码可复用**系统结构和保护逻辑**，不能直接把 SmolVLA gRPC / action-chunk 协议接到
LingBot。LingBot runtime 必须单独实现一个适配层，并保留与上面引用文件相同的可观测日志：
时间戳、图像新鲜度、当前实际关节、原始模型动作、处理后动作、已执行动作、队列长度与停止原因。

建议把新代码放在 Orin 的独立运行目录（或对应的独立 Git 分支）下，例如：

```text
/home/nvidia/work/telop/lingbot_va_runtime/20260825_new_task_50/runtime/
```

不要修改 `SmolVLA-with-QGF` 的已有 rollout 脚本，也不要让 LingBot 使用 QGF 代码路径。

### 9.2 固定频率和异步队列

次日的初始运行配置应为：

```text
相机采集：30 FPS（chest 与 right-wrist 保持原始同步流）
LingBot observation / action 消费：15 Hz，周期 66.667 ms
底层 JAKA 目标平滑：125 Hz / 8 ms（仅在现场授权运动后启用）
```

15 Hz 不表示把 30 FPS 视频变成两倍速：每个 15 Hz tick 只选择一对最新且时间戳匹配的 RGB 帧，
保留真实时间轴。推理 worker 必须异步工作：

1. 首次 observation 触发第一个 action chunk 推理；**第一个 chunk 尚未准备好时不得允许运动**；
2. 开始执行 chunk 的第一个动作后，立即用最新 observation 在后台请求下一 chunk；
3. 执行线程每 66.667 ms 消费一个已验证动作；推理线程永远不能阻塞它；
4. 新 chunk 到达后，只在清楚记录其 observation 时间戳、动作索引和替换点的情况下切换；
5. 若队列耗尽、图像/关节状态过期或推理异常，关闭**软件 action gate**，保持当前目标；不要用旧动作
   无限追赶，也不要自动上电/使能来“恢复”。

SmolVLA 过去是 50 action / chunk、15 Hz（约 3.33 秒覆盖）；LingBot 的 action chunk 长度必须从
新模型实际输出确定。初始值不得盲目复制 50：先记录 warm-up 后 p50/p95 推理延迟，再选择能覆盖
`p95 延迟 × 15 Hz + 安全余量` 的 chunk；将该值、预取时点和实际队列低水位写入
`resolved_runtime_config.yaml`。

### 9.3 夹爪语义（必须与训练数据一致）

统一规定：模型归一化输出 `0 = open`，`1 = closed`。处理规则必须为：

```text
raw <= 0.15  -> 请求 OPEN
0.15 < raw < 0.85 -> 保持上一已确认状态
raw >= 0.85  -> 请求 CLOSE
```

为防止 chunk 边界抖动，沿用已验证的时间状态机参数：

```text
confirmation_frames       = 10 个连续的 15 Hz action（约 0.67 s）
min_state_dwell_seconds   = 2.0 s
contact_hold_seconds      = 3.0 s
```

这套实现可参考：

```text
E:/AAA__Github_Project/SmolVLA-with-QGF/lerobot_robot_armstrong_ros2/
  src/lerobot_robot_armstrong_ros2/configuration_armstrong_ros2.py
  src/lerobot_robot_armstrong_ros2/smolvla_guard.py
```

注意 ROS 夹爪服务的布尔值历史上表示 `requested_open`，而模型内部表示的是 `gripper_closed`；
适配层必须只在一个位置做显式反转，并为每一次状态转换打印：原始值、候选状态、确认计数、最终
`requested_open`、接触保持原因和时间戳。不得通过“忽略 alarm”绕过夹爪硬件故障。

### 9.4 明天之前必须存在的交付物

训练执行者在交付前至少给出：

```text
1. A800 50 episode 传输/转换 manifest（含 TCP 状态）；
2. 最终 checkpoint 与 SHA256；
3. Orin 完整 base + checkpoint 文件清单；
4. LingBot 独立 runtime 代码、15 Hz 异步队列日志、离线前向测试；
5. 固定 0.15 / 0.85 夹爪阈值与 10 帧确认的单元测试；
6. 明确证明没有在交付过程中调用 power / enable / servo / gripper motion；
7. 现场首次授权运动前的人工 checklist。
```

只有上述项目都完成，才可称为“明天可由人在实体急停旁进行首次受控运行”。

## 参考

- [LingBot-VA upstream repository](https://github.com/robbyant/lingbot-va)
- [LingBot-VA base model（ModelScope）](https://www.modelscope.cn/models/Robbyant/lingbot-va-base)
- [LingBot-VA paper](https://arxiv.org/abs/2601.21998)
- [本项目的 LingBot 摇操采集说明](LINGBOT_VA_TELEOP_COLLECTION.md)
