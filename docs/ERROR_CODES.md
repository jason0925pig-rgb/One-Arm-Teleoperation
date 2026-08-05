# One-Arm Teleoperation error codes

这些 code 分两层：

- PowerShell 里看到的 `SSH command failed with exit code N`：表示远端 Ubuntu 脚本返回了 `N`。
- 最后 PowerShell 自己退出 `1`：表示本次完整流程没有正常保存为一次成功 episode，通常只是包装层结果，不一定说明机器人已经动过。

## `tools/ubuntu_full_teleop_stack.sh`

这是机器人运动栈脚本，也就是启动/停止右臂、夹爪和 UDP 桥接器的脚本。

| Code | 含义 |
| --- | --- |
| 0 | 成功。 |
| 1 | 通用脚本失败。原则上应尽量避免直接出现；如果出现，看它前面的 `ERROR:` 行。 |
| 2 | 环境或参数问题，例如 ROS setup / workspace setup / 配置文件不存在。 |
| 3 | 本 launcher 的 arm/gripper/bridge 栈已经在运行，需要先 stop。 |
| 4 | Armstrong 控制器 `192.168.2.226` ping 不通。 |
| 5 | ROS2 服务没有按时出现，通常是 arm/gripper/bridge 某个节点启动失败。 |
| 6 | 已进入硬件上电/使能阶段后失败，例如 power on、enable、gripper open、motion enable、servo-ready 检查失败。 |
| 7 | 停止/关机流程没有完全确认，需要人工检查并按实体急停。 |
| 8 | Windows 主臂 UDP 预览没有被 Ubuntu 桥接器接受；机器人尚未上电/使能。常见原因是源 IP 不对、UDP 没到、时间戳/包格式被拒、桥接器没收到连续安全包。 |

## `tools/ubuntu_dataset_episode.sh`

这是相机、ROS bag 录制、LeRobot 导出的脚本。

| Code | 含义 |
| --- | --- |
| 0 | 成功。 |
| 1 | 通用脚本失败。看前面的 `ERROR:` / Python traceback。 |
| 2 | 用法、参数或基础路径错误。 |
| 3 | 已有相机/录制/同名进程或 topic publisher 冲突。 |
| 8 | 存储或相机 preflight 检查失败。 |
| 9 | 相机启动、相机 topic、30 FPS 检查或预览服务失败。 |
| 10 | 录制器启动失败、episode 路径缺失、或待处理 episode 不存在。 |
| 11 | ROS bag 停止不干净。 |
| 12 | 另一个 LeRobot exporter 正在写同一个数据集。 |
| 13 | 正在录制/导出时拒绝 discard，避免删错正在写的数据。 |
| 14 | discard 路径安全检查失败，避免删除 raw root 外的目录。 |

## `tools/right_arm_manual_mode.sh`

这是单独右臂手动拖动/上电使能脚本。

| Code | 含义 |
| --- | --- |
| 0 | 成功退出，已执行 drag off / disable / power off。 |
| 2 | ROS setup、workspace setup 或配置文件不存在。 |
| 3 | 手动模式已经在运行。 |
| 4 | Armstrong 控制器 ping 不通。 |
| 5 | 节点/服务启动失败或存在冲突。 |
| 6 | 上电/使能/拖动模式前置条件失败。 |
| 7 | 手动模式运行中节点异常退出。 |

## Windows sender / ZLink2

| Code | 含义 |
| --- | --- |
| 0 | 正常退出。 |
| 1 | PowerShell wrapper 认为完整流程失败；通常要看上面的 SSH code 或 sender log。 |
| 2 | Python sender / probe 参数或运行时错误，例如串口打不开、基线捕获失败、启动检查失败。 |
| 130 | 用户 Ctrl+C 中断 Python 工具。 |

## 本次看到的 `code1`

这次本机 sender 日志显示：

- COM10 正常打开；
- 8/8 个 ZLink2 ID 都回复；
- baseline 已捕获；
- 后续一直在发 `teleop=WAITING_FOR_SPACE` 预览帧；
- `joint_3/id2` 读数约为 2330，不是 0。

远端日志显示：

- arm/gripper/bridge 已启动；
- 右臂真实角度能读到；
- 机器人未上电、未使能；
- bridge 只进入监听，没有进入 mapping enabled。

所以这次失败不是 ID2 掉线，也不是相机 code9；它是“远端没有确认收到并接受连续 UDP 预览包”。后续版本会把这类失败显示为 `exit code 8`，并打印 bridge 的拒包细节。
