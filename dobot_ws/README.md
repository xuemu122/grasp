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

## DOBOT TCP 服务

bringup 注册的服务 (通过 `ros2 service list` 查看):

| 服务 | 功能 |
|------|------|
| `/dobot_bringup_ros2/srv/EnableRobot` | 使能机械臂 |
| `/dobot_bringup_ros2/srv/DisableRobot` | 下使能 |
| `/dobot_bringup_ros2/srv/StartDrag` | 进入拖动示教 |
| `/dobot_bringup_ros2/srv/StopDrag` | 退出拖动示教 |
| `/dobot_bringup_ros2/srv/BrakeControl` | 抱闸控制 |
| `/dobot_bringup_ros2/srv/DragSensivity` | 拖动灵敏度 |

话题:
- `/joint_states_robot` — 关节状态 (sensor_msgs/JointState)
- `/dobot_msgs_v4/msg/ToolVectorActual` — 笛卡尔位姿

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
