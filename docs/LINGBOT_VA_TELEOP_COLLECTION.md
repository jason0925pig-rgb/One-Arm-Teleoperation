# LingBot-VA：右臂摇操数据采集与 adapter 准备

这套采集器与 `tools/start_full_teleop.cmd` 独立。原启动器保留其单轮、结束即完整下电的行为；本采集器用于**同一任务连续多轮**收集 LingBot-VA 的源数据。

## 运行命令（Windows PowerShell）

```powershell
cd E:\AAA__Github_Project\One-Arm-Teleoperation
.\tools\start_lingbot_va_teleop_collection.cmd `
  -Task "把矿泉水放进纸箱里。"
```

默认连接新 Humble/Orin 主机，要求 Windows 有线地址为 `192.168.2.130`，并将数据独立写到：

```text
/home/nvidia/work/telop/lingbot_va_teleop/
  raw_episodes/       # 原始 ROS bag、逐轮元数据、运行日志
  lerobot_dataset/    # 已导出的 LeRobot v3：Parquet + 两路 MP4 + meta
```

如需改端口、任务或目录，使用 PowerShell 参数，例如 `-ComPort COM10`、`-UbuntuDatasetDataRoot /path/to/data`。开始前必须确认实体急停可用、右臂附近无人、两路相机和 ZLink2 都已连接。

## 每轮交互

1. 脚本只启动相机与 ROS 节点；机器人此时不动。
2. 新弹出的 ZLink2 窗口中摆好主臂，按 **Enter** 记录本轮起始偏移。
3. 第一轮会执行：上电、使能、从臂夹爪打开、进入伺服。后续轮只重新进入伺服，不重复上下电。
4. 出现 `REMOTE ROUND READY` 后：第一次按 **Space** 开始摇操；第二次按 **Space** 正常结束本轮。
5. 本轮结束会立即关闭映射并**退出伺服模式**，但保留：ROS 节点、两台相机、机器人上电与使能状态。
6. 输入 `S` 保存或 `D` 删除本轮的原始数据、LeRobot 数据和 Windows 主臂 CSV。
7. 按 **Enter** 进入下一轮（重新基线、重新按 Space）；输入 `Q` 才会执行完整安全退出：退出伺服、禁用、下电，并关闭节点与相机。

物理急停、保护停、发送器异常或启动异常会按安全路径删除当前轮；不能将异常中断的轮当作训练样本。

## `walle_description` 结论

`walle_description/urdf/walle.urdf` 是完整双臂展示 URDF：右臂链为

```text
base_link -> base_link_jaka_right -> r1 ... -> r7 -> rt
            r-j1 ... r-j7                   r-t (fixed)
```

但 `rt` 后固定连接 `R_hand_base_link`，并包含拇指、食指、中指、无名指、小指的 `R_hand_*` 多指关节和 STL 网格。它描述的是**灵巧手**，不是现场正在使用的 `CTAG2F120` 平行夹爪。因此：

- 可暂时用 `r-j1`～`r-j7` 的右臂链检查关节命名和可视化；
- 不可把该灵巧手的关节、尺寸或末端变换当作 CTAG2F120 的真实几何；
- 训练/部署 LingBot-VA 前仍要提供或测量 CTAG2F120 的真实安装位姿（`rt -> TCP` 固定变换）及夹爪开闭语义。

## LingBot-VA adapter 还需要的转换步骤

本采集器输出的是可追溯的 **LeRobot v3 源数据**：两路同步 RGB 视频、实际执行的 7 轴关节动作、关节状态、二值夹爪状态和任务文本。它不会伪造不存在的末端位姿。

LingBot-VA 的公开训练代码使用规范的 30 维 action 布局，并以末端执行器位姿为主；对当前单右臂，必须在离线转换中：

1. 用已验证的右臂 FK 与真实 `rt -> CTAG2F120 TCP` 得到右手 7D 位姿；
2. 填充右臂 EEF/夹爪通道，未使用的左臂通道置零；
3. 保持相机顺序为外部（chest）在前、右腕在后；
4. 按所选 LingBot 代码版本生成 `action_config` 与 Wan VAE latents；
5. 用成功演示训练 LoRA adapter，并在独立保留回合上验证。

所以，当前采集格式是正确且不丢信息的训练源；但 adapter 训练前不能跳过 TCP/FK 校验和这一转换步骤。

参考： [LingBot-VA paper](https://arxiv.org/abs/2601.21998)、[LeRobot LingBot-VA documentation](https://github.com/huggingface/lerobot/blob/main/docs/source/lingbot_va.mdx)、[upstream implementation](https://github.com/robbyant/lingbot-va)。
