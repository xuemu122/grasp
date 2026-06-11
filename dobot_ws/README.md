# dobot_ws — DOBOT ROS2 工作空间

基于 ROS2 Humble 的 DOBOT Nova5 六轴机械臂控制工作空间, 含 bringup、MoveIt、Gazebo、点位录制、灵巧手联动等。

## 编译

```bash
cd /home/nvidia/grasp/dobot_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 环境变量

```bash
export IP_address=192.168.1.6        # 有线: 192.168.5.1, 无线: 192.168.1.6
export DOBOT_TYPE=cr5                # CR3/CR5/CR7/CR10/CR12/CR16/CR20/E6/ME6/NOVA2/NOVA5
```

## 功能包

| 包 | 说明 |
|------|------|
| `dobot_bringup_v4` | TCP 直连机械臂, 发布关节状态, 提供服务接口 |
| `dobot_msgs_v4` | 自定义消息和服务定义 (EnableRobot, StartDrag, BrakeControl 等) |
| `servo_action` | **关节路点录制 + 轨迹播放 + 灵巧手联动** |
| `dobot_demo` | 演示: 视觉检测、移动归零、视觉抓取 |
| `dobot_moveit` | MoveIt 运动规划 |
| `dobot_gazebo` | Gazebo 仿真 |
| `dobot_rviz` | RViz 可视化 |
| `*_moveit` | 各型号 MoveIt 配置包 (CR3-CR30, ME6, NOVA2/5) |

## 自定义程序 (servo_action)

| 文件 | 说明 |
|------|------|
| `record_waypoints.py` | **路点录制**: 按 `r` 记录当前关节位姿, `q` 保存 YAML |
| `play_trajectory.py` | **轨迹播放**: 回放录好的 YAML 轨迹 + 灵巧手联动 |
| `action_move_client.py` | Action 客户端: 发送关节目标 |
| `Joint_Position.py` | 关节位置控制 |
| `j6_rotate_demo.py` | J6 旋转演示 |

### record_waypoints.py

关节轨迹录制工具。需要先启动 bringup。

```bash
# 终端1: 启动 bringup (连接机械臂)
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py

# 终端2: 使能 + 拖动示教
ros2 service call /dobot_bringup_ros2/srv/EnableRobot dobot_msgs_v4/srv/EnableRobot
ros2 service call /dobot_bringup_ros2/srv/StartDrag dobot_msgs_v4/srv/StartDrag

# 终端3: 启动录制
ros2 run servo_action record_waypoints
```

操作:
- 拖动机械臂到目标位姿
- 按 `r` 记录当前位姿
- 重复直到录完所有路点
- 按 `q` 保存 YAML 到 `servo_action/servo_action/trajectories/`

### play_trajectory.py

回放录好的轨迹，支持灵巧手联动。

```bash
# 方式1: 指定 sequence 序列 (数字对应 YAML 里路点的 id)
ros2 run servo_action play_trajectory --ros-args -p sequence:=[3,2,1,2,3] -p dwell:=3.0

# 方式2: 指定 trajectory YAML 文件
ros2 run servo_action play_trajectory --ros-args -p trajectory_file:=trajectories/trajectory_xxx.yaml

# 单独编译
colcon build --packages-select servo_action --symlink-install
```

参数:
- `sequence`: 路点 id 序列, 如 `[3,2,1,2,3]`
- `dwell`: 每段等待时间 (秒)

## DOBOT TCP 服务 (cr_robot_ros2)

bringup 只有一个节点 `cr_robot_ros2_node`（源码 `cr_robot_ros2`），通过 TCP 端口 29999 与机械臂控制器通信，提供以下 Service 和 Topic。

### 话题 (Topic)

| 话题 | 类型 | 功能 |
|------|------|------|
| `/dobot_bringup_ros2/feed_back_info` | `std_msgs/msg/String` | 机械臂实时状态 (JSON: 关节角、笛卡尔位姿、IO、模式、碰撞态等), 100Hz |
| `/joint_states_robot` | `sensor_msgs/msg/JointState` | 6 关节角度 (由 feed_info 中提取发布) |
| `/dobot_msgs_v4/msg/ToolVectorActual` | 自定义消息 | 末端实际笛卡尔位姿 |

### 服务 (Service) — 全部挂在 `/dobot_bringup_ros2/srv/` 下

#### 使能与控制

| 服务 | 功能 |
|------|------|
| `EnableRobot` | **使能机械臂** (释放抱闸) |
| `DisableRobot` | **下使能** |
| `PowerOn` | 上电 |
| `ResetRobot` | 复位 |
| `ClearError` | 清除错误 |
| `EmergencyStop` | **急停** |
| `Stop` | 停止运动 |
| `Pause` | 暂停 |
| `Continue` | 继续 |
| `RobotMode` | 切换机器人模式 |
| `RunScript` | 运行脚本 |
| `RequestControl` | 请求控制权 |

#### 拖动示教

| 服务 | 功能 |
|------|------|
| `StartDrag` | **进入拖动示教 (零重力)** |
| `StopDrag` | **退出拖动示教** |
| `DragSensivity` | 设置拖动灵敏度 |
| `BrakeControl` | 抱闸控制 (axis_id + value) |

#### 运动控制

| 服务 | 功能 |
|------|------|
| `MovJ` | **关节空间点到点运动** |
| `MovL` | **直线运动** |
| `MovLIO` | 直线运动 + IO 触发 |
| `MovJIO` | 关节运动 + IO 触发 |
| `Arc` | 圆弧运动 |
| `Circle` | 整圆运动 |
| `MoveJog` | 点动开始 |
| `StopMoveJog` | 点动停止 |
| `RelMovJTool` | 工具坐标系相对关节运动 |
| `RelMovLTool` | 工具坐标系相对直线运动 |
| `RelMovJUser` | 用户坐标系相对关节运动 |
| `RelMovLUser` | 用户坐标系相对直线运动 |
| `RelJointMovJ` | 相对关节运动 |
| `ServoJ` | **关节伺服模式** (位置控制) |
| `ServoP` | **笛卡尔伺服模式** |
| `RunTo` | 移动到指定位姿 |
| `StartPath` | 路径开始 |
| `GetStartPose` | 获取起始位姿 |

#### 运动参数

| 服务 | 功能 |
|------|------|
| `SpeedFactor` | 全局速度因子 |
| `AccJ` | 关节加速度 |
| `AccL` | 直线加速度 |
| `VelJ` | 关节速度 |
| `VelL` | 直线速度 |
| `CP` | 连续路径模式 |
| `GetCurrentCommandId` | 获取当前指令 ID |

#### 位姿获取与运动学

| 服务 | 功能 |
|------|------|
| `GetPose` | **获取当前笛卡尔位姿 (x,y,z,rx,ry,rz)** |
| `GetAngle` | **获取当前关节角度 (J1~J6 度)** |
| `PositiveKin` | 正运动学 |
| `InverseKin` | 逆运动学 |
| `CheckOddMovL` | 直线运动奇异性检查 |
| `CheckOddMovJ` | 关节运动奇异性检查 |
| `CheckOddMovC` | 圆弧运动奇异性检查 |

#### 负载与工具

| 服务 | 功能 |
|------|------|
| `SetPayload` | **设置负载 (质量 + 重心)** |
| `Tool` | 选择工具坐标系 |
| `User` | 选择用户坐标系 |
| `SetUser` | 设置用户坐标系参数 |
| `SetTool` | 设置工具坐标系参数 |
| `CalcUser` | 计算用户坐标系 |
| `CalcTool` | 计算工具坐标系 |

#### IO 控制

| 服务 | 功能 |
|------|------|
| `DO` | 设置数字输出 |
| `DOInstant` | 即时数字输出 |
| `DOGroup` | 数字输出组 |
| `DOGroupDEC` | 数字输出组 (DEC) |
| `ToolDO` | 工具数字输出 |
| `ToolDOInstant` | 工具即时数字输出 |
| `AO` | 模拟输出 |
| `AOInstant` | 即时模拟输出 |
| `DI` | 读取数字输入 |
| `DIGroup` | 数字输入组 |
| `DIGroupDEC` | 数字输入组 (DEC) |
| `ToolDI` | 工具数字输入 |
| `AI` | 模拟输入 |
| `ToolAI` | 工具模拟输入 |
| `GetDO` / `GetDOGroup` / `GetDOGroupDEC` | 查询 DO 状态 |
| `GetAO` | 查询 AO 状态 |
| `GetToolDO` | 查询工具 DO |

#### 高级 IO 寄存器

| 服务 | 功能 |
|------|------|
| `GetInBits` / `GetInRegs` | 读取 Modbus 输入位/寄存器 |
| `GetCoils` / `SetCoils` | 读写线圈 |
| `GetHoldRegs` / `SetHoldRegs` | 读写保持寄存器 |
| `ModbusRTUCreate` / `ModbusCreate` / `ModbusClose` | Modbus 配置 |
| `SetTool485` | 工具 485 参数 |

#### 安全

| 服务 | 功能 |
|------|------|
| `SetCollisionLevel` | 碰撞检测等级 |
| `SetSafeWallEnable` | 安全墙开关 |
| `SetWorkZoneEnable` | 工作区域限制 |
| `SetBackDistance` | 碰撞后回退距离 |
| `SetPostCollisionMode` | 碰撞后行为 |
| `EnableSafeSkin` | 安全皮肤开关 |
| `SetSafeSkin` | 安全皮肤参数 |

#### 力控

| 服务 | 功能 |
|------|------|
| `EnableFTSensor` | 六维力传感器使能 |
| `SixForceHome` | 力传感器归零 |
| `GetForce` | 读取力数据 |
| `ForceDriveMode` / `ForceDriveSpeed` | 力牵引模式 |
| `FCForceMode` | 力控模式 |
| `FCSetForce` / `FCSetDeviation` / `FCSetForceLimit` | 力控参数 |
| `FCSetMass` / `FCSetStiffness` / `FCSetDamping` | 力控动力学参数 |
| `FCSetForceSpeedLimit` / `FCOff` | 力控速度/关闭 |
| `SetFCCollision` / `FCCollisionSwitch` | 力控碰撞 |

#### 自定义寄存器 IO (V3 拓展)

| 服务 | 功能 |
|------|------|
| `GetInputBool` / `GetInputInt` / `GetInputFloat` | 读取寄存器输入 (bool/int/float) |
| `GetOutputBool` / `GetOutputInt` / `GetOutputFloat` | 读取寄存器输出 |
| `SetOutputBool` / `SetOutputInt` / `SetOutputFloat` | 写入寄存器输出 |
| `StartRTOffset` / `EndRTOffset` | 实时偏移指令 |

#### 其他

| 服务 | 功能 |
|------|------|
| `GetErrorID` / `GetError` | 读取错误码 |

### 可执行节点 (其他包)

| 包 | 节点 | 功能 |
|------|------|------|
| `servo_action` | `record_waypoints` | 关节路点录制 (按 r 记录, q 保存 YAML) |
| `servo_action` | `play_trajectory` | 轨迹播放 (sequence 或 YAML 文件) |
| `servo_action` | `action_move_client` | Action 关节控制客户端 |
| `servo_action` | `Joint_Position` | 关节位置直接控制 |
| `servo_action` | `j6_rotate_demo` | J6 轴旋转演示 |
| `dobot_demo` | `demo` | 综合演示 |
| `dobot_demo` | `move_and_home` | 移动 & 回零 |
| `dobot_demo` | `vision_detector` | 视觉检测 |
| `dobot_demo` | `vision_pick` | 视觉抓取 |
| `dobot_moveit` | `action_move_server` | MoveIt Action 服务端 |
| `dobot_moveit` | `joint_states` | 关节状态发布 |

## 仿真

```bash
# RViz 模型
ros2 launch dobot_rviz dobot_rviz.launch.py

# MoveIt 虚拟演示
ros2 launch dobot_moveit moveit_demo.launch.py

# Gazebo 仿真
ros2 launch dobot_gazebo dobot_gazebo.launch.py
```

## 控制真实机械臂

```bash
# 1. 启动 bringup
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py

# 2. 使能
ros2 service call /dobot_bringup_ros2/srv/EnableRobot dobot_msgs_v4/srv/EnableRobot

# 3. MoveIt 控制
ros2 launch dobot_moveit dobot_moveit.launch.py
```
