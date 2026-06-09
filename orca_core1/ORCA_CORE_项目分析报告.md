# ORCA Core 项目详细分析报告

## 1. 项目概述

**ORCA Core** 是 [ORCA Hand](https://orcahand.com) 灵巧机器手的核心控制软件包，版本 `0.2.1`，采用 MIT 许可证。该项目抽象了底层硬件交互，提供了关节空间的高层控制 API，使用户能够通过简单的 Python 接口控制一个拥有 17 个自由度的仿人灵巧手。

- **论文**: [arXiv:2504.04259](https://arxiv.org/abs/2504.04259)
- **官网**: [orcahand.com](https://orcahand.com)
- **仓库**: [github.com/orcahand/orca_core](https://github.com/orcahand/orca_core)

---

## 2. 项目结构

```
orca_core/
├── orca_core/                    # 核心包
│   ├── __init__.py               # 公共 API 导出
│   ├── version.py                # 版本信息 (LATEST_VERSION = "v2")
│   ├── constants.py              # 全局常量定义
│   ├── joint_position.py         # OrcaJointPositions 不可变数据容器
│   ├── hand_config.py            # 配置数据类 (BaseHandConfig, OrcaHandConfig)
│   ├── calibration.py            # CalibrationResult 校准结果数据类
│   ├── base_hand.py              # BaseHand 抽象基类
│   ├── hardware_hand.py          # OrcaHand 硬件实现 + MockOrcaHand
│   ├── core.py                   # 向后兼容的入口 (已弃用)
│   ├── api/                      # FastAPI REST 接口
│   │   └── api.py                # HTTP API 服务器
│   ├── hardware/                 # 硬件驱动层
│   │   ├── motor_client.py       # MotorClient 抽象基类
│   │   ├── dynamixel_client.py   # Dynamixel 电机驱动
│   │   ├── feetech_client.py     # Feetech 电机驱动
│   │   ├── mock_dynamixel_client.py  # 模拟电机驱动(用于测试)
│   │   └── feetech/              # Feetech SDK 子模块(9个文件)
│   ├── utils/                    # 工具函数
│   │   └── utils.py              # YAML读写、路径解析、端口检测、插值
│   └── models/                   # 手型配置文件
│       ├── v1/
│       │   ├── orcahand_left/
│       │   └── orcahand_right/
│       └── v2/
│           ├── orcahand_left/
│           ├── orcahand_right/
│           ├── orcahand_touch_left/
│           └── orcahand_touch_right/
├── scripts/                      # 22 个 CLI 实用脚本
├── tests/                        # 单元测试
├── docs/                         # MkDocs 文档站
├── pyproject.toml                # 项目元数据与依赖
└── LICENSE                       # MIT 许可证
```

---

## 3. 架构设计

### 3.1 类继承体系

```
MotorClient (ABC)                    # 电机通信抽象接口
├── DynamixelClient                  # ROBOTIS Dynamixel 电机 (Protocol 2.0)
├── FeetechClient                    # 飞特 Feetech SCServo 电机
└── MockDynamixelClient              # 内存模拟电机 (测试用)

BaseHand (ABC)                       # 手部抽象基类 (关节空间接口)
└── OrcaHand                         # 物理手 (连接管理 + 力矩 + 校准 + 后台任务)
    └── MockOrcaHand                 # 模拟手 (基于 MockDynamixelClient)

BaseHandConfig (frozen dataclass)    # 基础配置 (关节ID/ROM/中性位)
└── OrcaHandConfig                   # 硬件配置 (电机映射/电流/波特率/校准参数)

CalibrationResult (frozen dataclass) # 校准结果快照 (不可变)

OrcaJointPositions (frozen dataclass)# 关节位置容器 (不可变, 线程安全)
```

### 3.2 核心设计模式

| 模式 | 体现 |
|------|------|
| **策略模式** | `MotorClient` 抽象接口，`OrcaHand` 通过 `_create_motor_client()` 工厂方法在 Dynamixel/Feetech 之间切换 |
| **模板方法** | `BaseHand` 定义 `set_joint_positions()` 的插值逻辑，子类实现 `_get_joint_positions()` 和 `_set_joint_positions()` |
| **不可变数据** | `OrcaJointPositions`、`CalibrationResult`、`OrcaHandConfig` 均为 `frozen=True` 的 dataclass |
| **工厂方法** | `_create_motor_client()` 根据 `config.motor_type` 动态创建对应的电机客户端 |
| **线程安全** | `_motor_lock` (RLock) 保护串口总线访问，`_task_thread` + `_task_stop_event` 管理后台任务 |

---

## 4. 核心模块详解

### 4.1 `BaseHand` — 抽象基类

定义了所有手部后端共享的关节空间接口：

- **`set_joint_positions()`**: 核心控制方法，支持 `OrcaJointPositions` / `dict` / `np.ndarray` 三种输入格式，自动进行 ROM 裁剪和线性插值
- **`get_joint_position()`**: 读取当前关节位置
- **`set_neutral_position()`** / **`set_zero_position()`**: 移动到预定义位置
- **`register_position()`** / **`set_named_position()`**: 命名位置录制与回放
- **`_linear_waypoints_to()`**: 从当前位置到目标位置的线性插值路径点生成

### 4.2 `OrcaHand` — 物理手实现

在 `BaseHand` 基础上增加了完整的物理手生命周期管理：

#### 连接管理
- `connect()`: 三级端口发现策略 → 配置端口 → USB VID 自动检测 → 交互式终端选择
- `disconnect()`: 安全断开（先禁用力矩再关闭串口）
- `is_connected()`: 连接状态查询

#### 力矩与电流控制
- `enable_torque()` / `disable_torque()`: 电机使能/失能
- `set_max_current()`: 设置电机最大电流限制
- `set_control_mode()`: 切换控制模式，手腕电机自动使用 `multi_turn_position` 模式

#### 校准系统
- `calibrate()`: 自动校准流程，按照 `config.yaml` 中的 `calibration_sequence` 逐步驱动关节到达机械限位
- `is_calibrated()`: 校准状态检查
- 校准结果实时持久化到 `calibration.yaml`，中断不会丢失进度

#### 辅助功能
- `tension()`: 保持电机力矩供手动张紧肌腱
- `jitter()`: 正弦微振动帮助肌腱就位
- `get_motor_pos()` / `get_motor_current()` / `get_motor_temp()`: 传感器数据读取

#### 关节-电机坐标转换
- `_motor_to_joint_pos()`: 电机坐标 → 关节坐标（含编码器回绕补偿）
- `_joint_to_motor_pos()`: 关节坐标 → 电机坐标
- `_compute_wrap_offsets_dict()`: 检测并修正编码器 2π 回绕偏移

### 4.3 `OrcaJointPositions` — 关节位置容器

不可变、线程安全的关节位置存储：

- 支持 `dict`、`np.ndarray`、自身实例三种构造方式
- `register_joint_names()`: 注册全局默认关节排序
- `from_dict()`: 自动过滤 `None` 值
- `from_ndarray()`: 自动过滤 `NaN` 值
- `as_dict()` / `as_array()` / `as_list()`: 多种输出格式

### 4.4 `OrcaHandConfig` — 配置系统

两层配置架构：

| 配置类 | 职责 |
|--------|------|
| `BaseHandConfig` | 通用关节空间配置：关节ID、ROM、中性位 |
| `OrcaHandConfig` | 硬件专属配置：电机ID、映射、电流、波特率、校准参数 |

关键特性：
- YAML 驱动，支持多版本 (`v1`/`v2`) 和多型号 (`left`/`right`/`touch`)
- 负数电机ID表示反转关节方向（如 `ring_mcp: -7`）
- 启动时自动校验配置完整性

### 4.5 `CalibrationResult` — 校准结果

不可变数据快照，包含：
- `motor_limits_dict`: 电机硬限位 `[lower, upper]` (弧度)
- `joint_to_motor_ratios_dict`: 电机/关节传动比
- `calibrated`: 是否全部校准完成
- `wrist_calibrated`: 手腕是否校准完成

### 4.6 硬件驱动层

#### `MotorClient` (抽象基类)
定义统一的电机通信接口：`connect()`、`disconnect()`、`set_torque_enabled()`、`set_operating_mode()`、`read_pos_vel_cur()`、`read_temperature()`、`write_desired_pos()`、`write_desired_current()`、`calibrate_offset()`

#### `DynamixelClient`
- 基于 ROBOTIS DynamixelSDK Protocol 2.0
- 支持 XC330-T288-T 和 XC430-T240BB-T 电机
- 高级特性：
  - `_AlertCaptureBulkRead`: 自定义 BulkRead 封装，捕获 Alert 位实现**响应式过载检测**
  - `check_overload_and_reboot()`: 自动检测过载并重启恢复
  - 支持 ID 变更、波特率变更、电机扫描
  - 低延迟模式 (Linux FTDI) 可达 ~500Hz 通信

#### `FeetechClient`
- 基于飞特 SCServo 协议
- 支持 Servo 模式(位置控制)和 Wheel 模式(速度控制)
- `requires_offset_calibration = True`: 需要偏移校准 (`reOfsCal` 指令)
- 电流控制映射为扭矩限制 (0-1000)

#### `MockDynamixelClient`
- 内存模拟，无需硬件即可运行
- 用于 `MockOrcaHand` 和单元测试

---

## 5. 配置文件体系

### 5.1 `config.yaml` 结构 (以 v2/orcahand_right 为例)

```yaml
port: /dev/cu.usbmodem3101     # 串口路径
version: 0.2.1                  # 配置版本
baudrate: 1000000               # 波特率
max_current: 300                # 最大电流 (mA)
type: right                     # 左/右手
control_mode: current_based_position  # 控制模式

motor_ids: [1, 2, ..., 17]     # 17个电机ID

joint_ids:                      # 17个关节名
  - wrist
  - thumb_cmc / thumb_abd / thumb_mcp / thumb_dip
  - index_abd / index_mcp / index_pip
  - middle_abd / middle_mcp / middle_pip
  - ring_abd / ring_mcp / ring_pip
  - pinky_abd / pinky_mcp / pinky_pip

joint_to_motor_map:             # 关节→电机映射 (负数=反转)
  ring_mcp: -7
  wrist: -1

joint_roms:                     # 关节活动范围 (度)
  wrist: [-65, 35]
  thumb_mcp: [-25, 100]

neutral_position:               # 中性位 (度)
  wrist: -20
  thumb_abd: 50

calibration_sequence:            # 校准步骤序列 (16步)
  - step: 1
    joints:
      thumb_cmc: flex
```

### 5.2 支持的手型模型

| 版本 | 型号 | 说明 |
|------|------|------|
| v1 | orcahand_left / orcahand_right | 第一代设计 |
| v2 | orcahand_left / orcahand_right | 第二代设计（默认） |
| v2 | orcahand_touch_left / orcahand_touch_right | 触觉传感版 |

---

## 6. 电机控制模式

| 模式 | 值 | 说明 | 适用场景 |
|------|---|------|----------|
| `current` | 0 | 电流控制 | 力控场景 |
| `velocity` | 1 | 速度控制 | 连续旋转 |
| `position` | 3 | 位置控制 | 精确定位 |
| `multi_turn_position` | 4 | 多圈位置 | 手腕关节(大范围旋转) |
| `current_based_position` | 5 | 带电流反馈的位置控制 | **推荐**：兼顾精度与安全 |

> 手腕关节在 `current_based_position` 和 `current` 模式下会自动切换为 `multi_turn_position` (模式4)。

---

## 7. 校准流程

校准是 ORCA Hand 使用的**关键前置步骤**，流程如下：

```
1. 设置为 current_based_position 模式
2. 设置校准电流
3. 对 calibration_sequence 中每个步骤:
   a. 禁用力矩
   b. 使能当前步骤涉及的电机
   c. 按方向(-flex/extend)逐步驱动电机
   d. 检测位置是否稳定 (calibration_num_stable 次读数差值 < calibration_threshold)
   e. 到达机械限位后记录电机位置
   f. 计算关节-电机传动比 = Δmotor / Δjoint
   g. 实时持久化到 calibration.yaml
4. 恢复正常电流
```

特殊处理：
- 手腕校准使用更高电流 (`wrist_calibration_current`)
- 已校准的手腕默认跳过（除非 `force_wrist=True`）
- Feetech 电机需要额外的偏移校准 (`calibrate_offset`)

---

## 8. CLI 脚本工具

`scripts/` 目录提供 22 个实用脚本，统一通过 `common.py` 的 `create_hand()` 创建手实例：

### 核心工作流脚本

| 脚本 | 功能 |
|------|------|
| `tension.py` | 肌腱张紧（保持电机力矩供手动调整） |
| `calibrate.py` | 运行自动校准 |
| `neutral.py` | 移动到中性位 |
| `setup.py` | 初始化设置 |

### 交互式控制

| 脚本 | 功能 |
|------|------|
| `slider_joint.py` | 滑块控制关节位置 (matplotlib) |
| `slider_motor.py` | 滑块控制电机位置 |
| `main_demo.py` | 基础演示 |
| `main_demo_abduction.py` | 外展关节演示 |

### 录制与回放

| 脚本 | 功能 |
|------|------|
| `record_angles.py` | 录制关节角度序列 |
| `replay_angles.py` | 回放录制的关节角度 |
| `record_continuous.py` | 连续录制 |
| `replay_continuous.py` | 连续回放 |

### 诊断与测试

| 脚本 | 功能 |
|------|------|
| `check_motor.py` | 检查电机状态 |
| `jitter.py` | 正弦微振动（肌腱就位） |
| `stress_test.py` | 压力测试 |
| `test_motor_latency.py` | 通信延迟测试 |
| `test_overload.py` / `debug_overload.py` | 过载测试/调试 |
| `overload_demo.py` | 过载恢复演示 |

### 配置工具

| 脚本 | 功能 |
|------|------|
| `configure_motor_chain.py` | 配置电机链 |
| `zero.py` | 归零操作 |

所有脚本支持 `--mock` 标志，可在无硬件环境下运行。

---

## 9. REST API

`orca_core/api/api.py` 基于 FastAPI 提供 HTTP 接口，默认运行在 `0.0.0.0:8000`：

| 方法 | 路径 | 功能 |
|------|------|------|
| POST | `/config` | 设置手部配置 |
| POST | `/connect` | 连接硬件 |
| POST | `/disconnect` | 断开硬件 |
| GET | `/status` | 获取连接和校准状态 |
| POST | `/torque/enable` | 使能力矩 |
| POST | `/torque/disable` | 禁用力矩 |
| POST | `/current/max` | 设置最大电流 |
| GET | `/motors/position` | 读取电机位置 |
| GET | `/motors/current` | 读取电机电流 |
| GET | `/motors/temperature` | 读取电机温度 |
| GET | `/joints/position` | 读取关节位置 |
| POST | `/joints/position` | 设置关节位置 |
| GET | `/calibrate/status` | 校准状态 |
| POST | `/calibrate` | 执行自动校准 |

---

## 10. 依赖项

### 运行时依赖
| 包 | 版本 | 用途 |
|----|------|------|
| `dynamixel-sdk` | ≥3.7.31 | Dynamixel 电机通信 |
| `pyyaml` | ≥6.0.2 | YAML 配置读写 |
| `fastapi` | ≥0.115.12 | REST API 框架 |
| `uvicorn` | ≥0.34.2 | ASGI 服务器 |
| `numpy` | ≥2.2.6 | 数值计算 |

### 开发依赖
| 包 | 用途 |
|----|------|
| `pytest` | 单元测试 |
| `matplotlib` | 滑块可视化 |

### 文档依赖
MkDocs + mkdocs-material + mkdocstrings 构建 API 文档站。

---

## 11. 典型使用流程

```python
from orca_core import OrcaHand, OrcaJointPositions

# 1. 创建手实例（默认使用 v2/orcahand_right 配置）
hand = OrcaHand()

# 2. 连接硬件
hand.connect()

# 3. 初始化关节（使能力矩 + 校准 + 中性位）
hand.init_joints()

# 4. 控制关节
hand.set_joint_positions(
    OrcaJointPositions({"index_mcp": 0.5, "thumb_mcp": 1.2}),
    num_steps=50,    # 插值步数
    step_size=0.01,  # 步间延时(秒)
)

# 5. 读取状态
pos = hand.get_joint_position()
motor_temp = hand.get_motor_temp(as_dict=True)

# 6. 断开连接
hand.disconnect()
```

无硬件测试：
```python
from orca_core.hardware_hand import MockOrcaHand

hand = MockOrcaHand()  # 自动使用模拟电机客户端
```

---

## 12. 安全机制

| 机制 | 实现 |
|------|------|
| **ROM 裁剪** | `set_joint_positions()` 自动将目标位置限制在配置的关节活动范围内 |
| **电流限制** | `set_max_current()` 限制电机最大电流，防止过热/过载 |
| **过载恢复** | Dynamixel 客户端响应式检测 Alert 位，自动重启过载电机并恢复工作模式 |
| **编码器回绕** | `_compute_wrap_offsets_dict()` 检测并修正 2π 回绕 |
| **校准增量保存** | 每步校准结果实时写入 YAML，中断不丢失 |
| **手腕特殊处理** | 不兼容的控制模式自动降级为 `multi_turn_position` |
| **Jitter 幅度限制** | 最大 10° 防止意外大动作 |

---

## 13. 测试体系

```
tests/
├── conftest.py                  # 测试固件
├── test_base_hand_contract.py   # BaseHand 接口契约测试
├── test_calibration.py          # 校准逻辑测试
├── test_calibration_reference.py # 校准参考测试
├── test_core.py                 # OrcaHand 集成测试
├── test_hardware_hand.py        # 硬件手测试
├── test_jitter.py               # Jitter 功能测试
├── test_model_dispatch.py       # 型号分发测试
├── test_tension.py              # 张紧功能测试
├── test_yaml.py                 # YAML 配置测试
└── reference/                   # 参考数据
```

运行测试：`uv run pytest tests/`

---

## 14. 关节命名约定

格式：`{finger}_{joint_type}`

| 手指 | 关节类型 |
|------|----------|
| `thumb` | `cmc`, `abd`, `mcp`, `dip` |
| `index` / `middle` / `ring` / `pinky` | `abd`, `mcp`, `pip` |
| `wrist` | (无后缀) |

- `abd` = abduction (外展)
- `mcp` = metacarpophalangeal (掌指关节)
- `pip` = proximal interphalangeal (近端指间关节)
- `dip` = distal interphalangeal (远端指间关节)
- `cmc` = carpometacarpal (腕掌关节)

---

## 15. 总结

### 核心优势

1. **清晰的架构分层**: 抽象基类 → 硬件实现 → 模拟后端，层次分明
2. **多电机生态支持**: 统一接口同时支持 Dynamixel 和 Feetech 两种电机
3. **安全性优先**: ROM 裁剪、电流限制、过载自动恢复、编码器回绕补偿
4. **开箱即用**: 丰富的 CLI 脚本、REST API、Mock 模式，降低使用门槛
5. **可扩展配置**: YAML 驱动的多版本/多型号配置系统
6. **线程安全设计**: 不可变数据结构 + RLock 保护共享资源

### 适用场景

- 灵巧手运动控制研究
- 机器人抓取与操作实验
- 手部康复机器人开发
- 远程遥操作系统集成（通过 REST API）
- 仿人手运动学/动力学仿真验证
