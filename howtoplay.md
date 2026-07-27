# 当前 Windows → Armstrong 右臂流程

请从 [FAST_START.md](FAST_START.md) 开始。它按 Windows/Ubuntu 分开列出命令、
暂停点、需要回传的信息和真机安全边界。

# 🔧 System Setup

## Windows Leader Recording (One-Arm-Teleoperation)

The Windows leader-side tools do not require ROS 1. They use PlatformIO's
Python environment and read the ZLink2 adapter directly.

Validate the eight-channel physical mapping and list available serial ports:

```powershell
.\tools\run_zlink2_recorder.cmd --validate-only
```

Record one leader-arm session:

```powershell
.\tools\run_zlink2_recorder.cmd --session-name pick_cup `
  --task "pick up the cup and place it on the tray"
```

Place the leader in the desired starting pose and press Enter. That pose becomes
the offset origin for the session. Press `Ctrl+C` to stop cleanly.

Each session is written under `recordings/` with:

- `frames.csv`: raw pulses, relative start offsets, calibrated leader-gripper
  state, timestamps, and frame quality.
- `metadata.json`: mapping snapshot, baseline, actual rate, missing replies, and
  the stop reason.

Normal recording sends only the read-only `PRAD` query. It does not command
motion. Raw pulses are preserved because joint directions, limits, scale, and
zero offsets still need to be calibrated against the Armstrong robot. The
leader gripper uses the calibrated hysteresis parameters in
`gripper_calibration.json`.

See [the detailed Windows recorder guide](tools/README_ZLINK2_LEADER_RECORDER.md).
The complete safety-gated Windows-to-ROS2 workflow is documented in
[docs/TELEOP_FULL_FLOW.md](docs/TELEOP_FULL_FLOW.md).

> The current recorder is the leader action source, not yet a complete
> LeRobot/Armstrong dataset. The future Linux ROS 2 adapter will add the
> Armstrong state, executed targets, follower gripper/contact state, limits,
> and camera frame indices using the same timestamps.

---

## Original Upstream ROS 1 Workflow

## Prerequisites

- **Ubuntu 20.04**
- [**ROS Noetic**](https://wiki.ros.org/noetic/Installation/Ubuntu)
- **Python 3.9+**

---

## Step-by-Step Setup

1. **Install Python Dependencies**

   ```sh
   # install both ros1 and simulation requirements
   pip install -r overall_requirements.txt 
   ```
   
   If your system doesn't support ROS1, you can install the dependencies without ROS1 with the following command which supports simulation teleoperation and check [this note](https://github.com/MINT-SJTU/Lerobot-Anything-U-arm/blob/main/src/simulation/README.md) . 
   ```sh
   pip install -r requirements.txt
   ```

3. **Build Catkin Workspace**

   ```sh
   catkin_make
   source devel/setup.bash
   ```

4. **Verify Installation**

   ```sh
   # Test if ROS can find the package
   rospack find uarm
   ```

---

# 🤖 Plug-and-Play with Real Robot with ROS1
> Zhonglin servo version
## 1. Start ROS Core

Open a terminal and run:

```sh
roscore
```

## 2. Verify Teleop Arm Output

In a new terminal, check servo readings:

```sh
rosrun uarm servo_zero.py
```

This will display real-time angles from all servos. You should check whether `SERIAL_PORT` is available on your device and modify the variable if necessary. 

## 3. Publish Teleop Data

Still in the second terminal, start the teleop publisher:

```sh
rosrun uarm servo_reader.py
```

Your teleop arm now publishes to the `/servo_angles` topic.

## 4. Control the Follower Arm

Choose your robot and run the corresponding script:

- **For Dobot CR5:**
  ```sh
  rosrun uarm scripts/Follower_Arm/Dobot/servo2Dobot.py
  ```

- **For xArm:**
  ```sh
  rosrun uarm scripts/Follower_Arm/xarm/servo2xarm.py
  ```

---

> Feetech servo version (Global Version)
## 1. Start ROS Core

Open a terminal and run:

```sh
roscore
```

## 2. Verify Teleop Arm Output

In a new terminal, check servo readings:

```sh
rosrun uarm feetech_servo_zero.py
```

This will display real-time angles from all servos. You should check whether `SERIAL_PORT` is available on your device and modify the variable if necessary. You may find all servos' angles are `2047` since servo's position is set as `2047` (0~4095 for $360^\circ$) when this code starts running.

## 3. Publish Teleop Data

Still in the second terminal, start the teleop publisher:

```sh
rosrun uarm feetech_servo_reader.py
```

Servo's position is set again as `2047` when this code starts running . **Please return UARM to initial position before starting this script.** Your teleop arm now publishes to the `/servo_angles` topic.

## 4. Control the Follower Arm

Choose your robot and run the corresponding script:

- **For Dobot CR5:**
  ```sh
  rosrun uarm scripts/Follower_Arm/Dobot/servo2Dobot.py
  ```

- **For xArm:**
  ```sh
  rosrun uarm scripts/Follower_Arm/xarm/servo2xarm.py
  ```
---

# 🖥️ Try It Out in Simulation

If you do not have robot hardware, you can try teleoperation in simulation.  
See detailed guidance [here](https://github.com/MINT-SJTU/Lerobot-Anything-U-arm/blob/feat/simulation/src/simulation/README.md).
