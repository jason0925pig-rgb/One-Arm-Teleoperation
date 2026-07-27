# 上游内容与清理范围

本项目基于：

https://github.com/MINT-SJTU/LeRobot-Anything-U-Arm

当前仓库只服务于：

```text
Windows ZLink2 主臂 → UDP → Ubuntu ROS2 → Armstrong 右臂
```

因此本地和 GitHub 仓库均已删除以下未使用内容：

- 原始 ROS1/Catkin 工作区和通用 U-Arm 节点；
- LeRobot、SO100、XLeRobot、Dobot、ARX 和 xArm 示例；
- 内嵌 xArm Python SDK；
- ManiSkill 仿真镜像；
- 仿真脚本和演示 GIF；
- ROS1/仿真专用依赖清单；
- 编辑器生成的 `copy/bak/error` 备份。

继续保留：

- `mechanical/` 中全部 108 个 STL/STEP 文件；
- ZLink2 读取、ID 映射、标定 JSON 和 CSV 工具；
- Windows→Ubuntu ROS2 网络桥接；
- Armstrong/JAKA 右臂安全执行端；
- 知行 CTAG2F120 夹爪驱动和只读反馈探针；
- 老师提供的执行端正式参考源码。

如果将来需要恢复 ROS1、其他品牌机械臂示例或 ManiSkill，请从上述上游仓库
单独获取，不要直接混入真机部署分支。
