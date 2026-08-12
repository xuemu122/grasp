# LHandPro RS485 安装、标定与测试说明

本文用于 DH116/DH116S 一类 LHandPro RS485 灵巧手的现场安装和验收，覆盖：

- SDK/串口通信检查
- 回零与张开测试
- 新灵巧手空载电流曲线录制
- 柔性抓取独立测试
- 接入 `shelf_grasp.py`
- 末端负载、碰撞箱和深度护栏更新

## 1. 当前实现边界

当前 `LHandProGripper` 使用厂家 Linux SDK，并要求操作系统中存在串口设备，例如
`/dev/ttyUSB0` 或 `/dev/serial/by-id/...`。

Nova 末端 485 文档目前明确提供的是 Modbus RTU 接口，没有确认可发送 SDK 所需的
原始 RS485 字节帧。因此第一次测试使用 Jetson 的 USB-RS485 转换器。若只把手接到
Nova 末端 485 口，Linux 不会出现 `/dev/ttyUSB*`，当前 SDK 后端无法直接连接。

确认 DOBOT 支持原始字节透传后，只需要替换传输层；本文的空载曲线和柔性抓取逻辑
可以继续使用。

## 2. 安装前检查

1. 确认购买的是 RS485/CANFD 型号，不是仅 EtherCAT 型号。
2. 断电完成接线，确认 VDC、GND、RS485+、RS485-，不要带电插拔。
3. 灵巧手供电按实物铭牌和手册配置。不要在未核实电压、电流能力前直接使用 Nova
   末端工具电源。
4. 使用独立电源时，按设备手册确认通信地连接和隔离要求。
5. 灵巧手、转接板、相机和线缆不能与机械臂、货架或台面干涉。
6. 第一次使能和回零时，机械臂保持静止，手指范围内不得有物体。

## 3. 软件和串口

SDK 默认目录：

```text
/home/nvidia/grasp/LHandProLib-API-Linux
```

当前 SDK 是 `aarch64` 版本，程序会直接加载包内 `lib/libLHandProLib.so`，不要求复制到
`/usr/local/lib`。Python 依赖只有 `pyserial`；如果新系统没有安装：

```bash
python3 -m pip install --user -r /home/nvidia/grasp/LHandProLib-API-Linux/share/LHandProLib/examples/RS485_python/requirements.txt
```

查找稳定串口名：

```bash
ls -l /dev/serial/by-id/
```

优先使用 `/dev/serial/by-id/...`，避免重启后 `ttyUSB0` 变成 `ttyUSB1`。如果没有串口权限：

```bash
sudo usermod -aG dialout $USER
```

重新登录后生效。默认通信参数为 `500000 baud`、`8N1`、`NodeID=1`。

以下示例用 `/dev/ttyUSB0`，现场替换成实际串口。

## 4. 通信、回零和张开

进入工作目录：

```bash
cd /home/nvidia/grasp/dipan
```

先做只读通信检查，不使能电机、不回零：

```bash
python3 test_lhandpro_flexible_grasp.py --port /dev/ttyUSB0 info
```

正常结果应显示 6 个主动轴，每个轴都有 `enable/status/alarm/position/current`。如果出现
超时或数据格式错误，按以下顺序检查：

1. A/B 极性、GND、供电和急停状态。
   特别确认转接盒的 DC 主电源已经接入；USB-RS485 能读到 DOF 不代表电机母线已有电。
   应按手册在手端 VDC/GND 实测电压，并确认电源具备要求的持续电流能力。
2. 串口是否被其他进程占用。
3. 波特率和 NodeID 是否与手内参数一致。
4. `alarm` 是否非零。
5. RS485 线缆、终端匹配和屏蔽是否符合厂家要求。

通信正常后，在手指周围完全无障碍的情况下执行：

```bash
python3 test_lhandpro_flexible_grasp.py --port /dev/ttyUSB0 --debug home
python3 test_lhandpro_flexible_grasp.py --port /dev/ttyUSB0 open
```

`home` 只清报警、使能和回零，不发送位置命令；`--debug` 每秒打印六轴状态。`open`
会在回零后运动到默认张开位置。默认回零超时为 45 秒，可用
`--home-timeout 90` 临时延长。若实际运动方向相反，不继续录曲线，
先确认型号、固件、回零方向和轴映射。

## 5. 录制新手空载曲线

必须为新灵巧手重新录曲线。Aero 曲线的单位是 mA，LHandPro 电流反馈单位是额定电流
千分比（‰），两者不能共用。以下情况也必须重录：

- 更换灵巧手或电机
- 修改回零、轴方向、最大位置、速度或电流限制
- 修改固件、传动机构或手指组件
- 空载电流相对原曲线出现持续漂移

第一次保守录到 `7000`：

```bash
python3 test_lhandpro_flexible_grasp.py \
  --port /dev/ttyUSB0 record --max 7000 --step 250
```

程序要求输入 `RECORD` 后才运动。录制过程会让 6 个主动轴同步从 `0` 扫到最大位置，
每个位置采集 3 次电流。任一轴达到 `750‰` 会立即中止，并且不会覆盖已有曲线。

默认输出：

```text
/home/nvidia/grasp/dipan/runtime/lhandpro_empty.csv
```

校验文件：

```bash
python3 test_lhandpro_flexible_grasp.py check-profile
```

柔性闭合会自动截断到曲线中已录制的最大位置，所以先录到 `7000` 时不会继续闭到
`9000`。确认方向、行程和空载电流正常后，才能逐步扩大录制上限。

## 6. 独立柔性抓取测试

使用海绵、空纸盒等软质测试物。不要第一次就抓玻璃瓶、硬金属件或尺寸过大的物体。

```bash
python3 test_lhandpro_flexible_grasp.py --port /dev/ttyUSB0 flex
```

把物体放入掌心后输入 `GRASP`。此脚本只控制灵巧手，不会移动机械臂。

默认逻辑：

- 1～6 号轴参与闭合。
- 3～6 号轴作为四根手指独立判断接触。
- 同位置电流比空载包络增加 `80‰`，连续 2 次才确认接触。
- 接触后额外闭合 `150` 位置单位并锁定该轴。
- 至少 2 根手指确认接触才算成功。
- 控制最大电流默认 `600‰`；实测电流达到 `750‰` 安全停止。
- 曲线缺失时拒绝柔性抓取。

常用调参：

```text
LHAND_FLEX_PROFILE_DELTA       接触电流增量阈值，默认 80‰
LHAND_FLEX_STEP                每步位置增量，默认 250
LHAND_FLEX_EXTRA_CLOSE         接触后补紧量，默认 150
LHAND_FLEX_REQUIRED_AXES       必须判断的手指轴，默认 3,4,5,6
LHAND_FLEX_MIN_CONTACTS        成功所需最少接触轴，默认 2
LHAND_FLEX_MAX_CURRENT         SDK 下发的最大电流，默认 600‰
LHAND_FLEX_HARD_CURRENT        实测硬保护阈值，默认 750‰
LHAND_FLEX_MAX_POSITIONS       各轴最大位置，默认 9000；会受曲线范围限制
```

调参原则：先减小步长和最大位置，再调整接触阈值。不要通过提高硬保护阈值解决误触发。

## 7. 更新末端负载

称量以下所有随机械臂运动的部件：

- 灵巧手
- 法兰转接板和紧固件
- 腕部相机及支架
- 随动线缆中由末端承担的部分

测量组合件相对法兰坐标系的重心。程序使用米，DOBOT 指令发送时会转换为毫米：

```bash
export HAND_WEIGHT_KG=0.xxx
export HAND_COG_X_M=0.xxx
export HAND_COG_Y_M=0.xxx
export HAND_COG_Z_M=0.xxx
```

不要只填写灵巧手产品手册中的净重。重心不确定时应先完成实际称量和简化平衡测量，
不能沿用 Aero 的默认重心。

## 8. 更新碰撞箱

### 8.1 坐标定义

当前碰撞模型以 Nova `Link6` 法兰坐标系为基准：

- 灵巧手长度沿法兰 `+Z`
- 掌体厚度沿法兰 `X`
- 掌宽和手指张开宽度沿法兰 `Y`

如果新安装方式不是沿 `+Z`，或者手相对法兰存在明显横向偏置/旋转，不能只放大宽度
凑合，需要修改 `tool_geometry.py` 支持安装变换后再运行机械臂。

### 8.2 必须实测的尺寸

全部单位为米：

| 环境变量 | 实测定义 |
|---|---|
| `TOOL_HAND_LENGTH_Z_M` | 法兰安装面到张开状态最长指尖的最大 `+Z` 距离 |
| `HAND_PALM_LENGTH_M` | 法兰安装面到掌心前端的 `+Z` 距离 |
| `TOOL_HAND_BODY_DEPTH_X_M` | 掌体、转接件在 `X` 方向的最大总厚度 |
| `TOOL_HAND_PALM_WIDTH_Y_M` | 掌体、转接件在 `Y` 方向的最大总宽度 |
| `TOOL_HAND_OPEN_WIDTH_Y_M` | 完全张开时最外侧手指之间的最大 `Y` 宽度 |
| `TOOL_HAND_MOUNT_EXCLUSION_Z_M` | 法兰到主手体起点；新转接件不确定时先设 `0`，让手碰撞箱从法兰开始覆盖 |
| `HAND_FINGER_THICKNESS_M` | 手指/指尖在路径检查中使用的最大厚度 |

测量值应包含安装误差和保护余量。不要使用产品外形图的单一视图估算张开宽度。

示例模板，所有 `0.xxx` 必须替换成实测值：

```bash
export TOOL_HAND_LENGTH_Z_M=0.xxx
export HAND_PALM_LENGTH_M=0.xxx
export TOOL_HAND_BODY_DEPTH_X_M=0.xxx
export TOOL_HAND_PALM_WIDTH_Y_M=0.xxx
export TOOL_HAND_OPEN_WIDTH_Y_M=0.xxx
export TOOL_HAND_MOUNT_EXCLUSION_Z_M=0.000
export HAND_FINGER_THICKNESS_M=0.xxx
```

这些变量必须在启动 Python 进程前设置；程序导入模块时读取，修改后需要重启进程。

### 8.3 两套护栏的关系

程序同时使用两套末端保护：

1. `self_collision_guard.py`：把掌体和张开手指包络转换为多根胶囊，检查末端与 Nova
   自身连杆的碰撞。
2. `shelf_grasp.py` 深度护栏：根据当前深度点云检查手掌/手指沿抓取路径是否撞到货架，
   并从点云里剔除相机看到的灵巧手自身。

设置上述手尺寸后，以下深度护栏参数会自动跟随：

- `DEPTH_GUARD_HAND_RADIUS_M` 默认是掌宽一半加安全余量。
- `DEPTH_GUARD_FINGER_RADIUS_M` 默认是手指厚度一半加安全余量。
- `DEPTH_GUARD_SELF_MASK_LENGTH_M` 默认等于法兰到指尖长度。
- 自手部剔除半径默认比实际路径碰撞半径更大。

只有出现“自己的手仍被识别成障碍”或“误删近处层板”时，才单独调整：

```text
DEPTH_GUARD_SELF_MASK_RADIUS_M
DEPTH_GUARD_SELF_MASK_FINGER_RADIUS_M
DEPTH_GUARD_SELF_MASK_LENGTH_M
DEPTH_GUARD_SELF_MASK_BACKOFF_M
```

自手部参数只影响深度点剔除，不可代替真实碰撞箱尺寸。不得为通过路径检查而关闭
`SELF_COLLISION_ENABLE`、`SELF_COLLISION_FAIL_CLOSED` 或 `DEPTH_GUARD_ENABLE`。

### 8.4 无动作检查

打印当前加载的末端尺寸，不连接机械臂：

```bash
cd /home/nvidia/grasp/dipan
PYTHONPATH=. python3 -c 'from tool_geometry import tool_size_label_mm; print(tool_size_label_mm())'
```

运行碰撞模型测试：

```bash
PYTHONPATH=. python3 -m unittest discover -s tests -p 'test_tool_collision_geometry.py'
```

启动主程序后，自碰撞初始化日志也会打印类似：

```text
tool=hand=...mm/palm=...mm/open=...mm, camera=...mm
```

必须核对该日志与实测值一致。若腕部相机、相机支架或安装方向也发生变化，还需要重新
设置 `TOOL_CAMERA_*` 参数并重新进行手眼标定。

## 9. 接入货架主程序

### 9.1 通过机械臂末端 RS485 通信（当前接法）

如果灵巧手直接插在 Nova 机械臂末端的 RS485/航插上，Linux 上通常不会出现
`/dev/ttyUSB0`。不要再用 USB 串口命令。程序先通过 Dashboard 设置末端供电和
485 格式，再让厂商 SDK 通过 Nova 的 `192.168.5.1:60000` 原始透传端口发送和接收
完整 RS485 帧。代码保留原来的 `serial` 后端，同时新增 `dobot` 后端，不会影响
旧 USB 接法。

先在机械臂已经上电、急停可用、手指周围无障碍的情况下执行只读检查：

```bash
cd /home/nvidia/grasp/dipan
PYTHONPATH=. python3 test_lhandpro_flexible_grasp.py \
  --transport dobot \
  --robot-ip 192.168.5.1 \
  --baud 500000 --node 1 \
  info
```

这个命令只会设置末端供电/485格式、连接 60000 透传端口，然后由 SDK 读取六轴的位置、
状态、电流和报警，不会使能、回零或发送位置命令。输出正常后再执行：

如果从第一条 `60 01 40 ...` 开始就没有任何 `[RX]`，说明返回链路没有工作，不是轴使能或控制字问题。请先停止旧进程，按下面顺序重新配置末端：

```text
SetToolPower(0)
SetToolMode(1,0,1)
SetTool485(500000,"N",1,1)
SetToolPower(1)
```

等待 2~3 秒后再运行 `info`。手端波特率必须与 `SetTool485` 一致，不要使用
`600000`。同时检查手端 VDC/GND、RS485 A/B 和通信地；只有看到 `[RX] 58 ...`或 `[RX] 48 ...`
才可继续执行 `home` 和运动测试。

如需将手端从当前 `500000` 迁移到技术支持要求的 `115200`，先使用当前波特率执行一次：

```bash
python3 test_lhandpro_flexible_grasp.py \
  --transport dobot --robot-ip 192.168.5.1 \
  --baud 500000 --node 1 \
  set-baud --new-baud 115200
```

按提示输入 `SETBAUD`。完成后对 Nova 末端和灵巧手完整断电重启，再使用 `--baud 115200`。
如果不执行迁移，则继续使用 `--baud 500000`，不能只改 Nova 一侧。

Nova V4.6.2 的 `ModbusRTUCreate` 会拒绝 `500000`，但末端透传资料确认控制器端口
`60000` 直接对应末端 485。Jetson 已实测可连接该端口，因此程序不再使用
`ModbusRTUCreate/GetHoldRegs` 猜测手端状态寄存器，而是复用 LHandPro SDK 自带的
RS485 发送回调和数据解码接口。这样可以继续使用灵巧手原来的 `500000`，不需要拆线
以上仅适用于手端仍使用 `500000` 的情况。技术支持示例使用 `115200`，需先在现有
`500000` 链路上迁移手端波特率，再同时切换 Nova 和程序。

### 9.2 Nova 透传与短帧控制

LHandPro SDK 的状态和 SDO 请求只有 6/12 字节，而使能、回零和运动控制帧为
68 字节。若调试日志出现下面的状态 `02`，表示手端报告 CRC 错误，不是
机械臂未使能：

```text
[LHandPro/TCP485][协议错误] 手端报告CRC错误: type=0x81
```

实测对 68 字节帧做 `60+8` TCP 分片仍不能改变手端的 CRC 拒绝，
因此程序不再默认分片。协议表中的同一批运动对象可以用短 SDO 帧访问（读取 12 字节，写入 14 字节），
并已经过只读实机验证：`F100/F120/F140/F160` 的子索引 `1` 均有有效响应。
所以 `dobot` 后端现在默认改用技术支持示例中的短自定义帧，不再使用该 68 字节控制路径：

```text
[LHandPro/TCP485] 电机控制使用厂家短帧协议
```

短帧顺序为：使能 `0x81` → 回零 `0x81` → 重新使能 `0x81` → 电流 `0x84` → 速度 `0x83` → 位置 `0x82`。
六轴帧数据顺序为 `[mode/control]` 、`[current]` 、`[velocity]` 、`[position]`：

```text
50 01 81 06 20 01 20 01 20 01 20 01 20 01 20 01 CRC
50 01 81 06 25 04 25 04 25 04 25 04 25 04 25 04 CRC
50 01 84 06 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 CRC
50 01 83 06 D0 07 D0 07 D0 07 D0 07 D0 07 D0 07 CRC
50 01 82 06 88 13 88 13 88 13 88 13 88 13 88 13 CRC
```

对象地址映射（仅用于 SDO 诊断模式）为：`F100:轴号` 模式/控制、`F120:轴号`
电流限制、`F140:轴号` 速度、`F160:轴号` 位置。仅用于对比诊断时，
可临时切回原 SDK 的 68 字节 PDO 控制：
速度、`F160:轴号` 位置。状态、位置和电流反馈仍交给 SDK 解码。仅用于对比诊断时，
可临时切回原 SDK 的 68 字节 PDO 控制：

`F100` 的 SDO 数据必须保持协议字节顺序 `[模式, 控制]`：使能为 `20 01`、
断使能为 `20 02`、清报警为 `20 03`、负向回零为 `25 04`。SDO 的整数在线上
以低字节先传输，因此代码打包值时必须将控制字节放在高 8 位。

```bash
LHAND_DOBOT_CONTROL=pdo \
PYTHONPATH=. python3 test_lhandpro_flexible_grasp.py \
  --transport dobot --robot-ip 192.168.5.1 \
  --baud 500000 --node 1 --hex-debug --home-timeout 90 home
```

不建议在正常抓取中使用 `pdo`；手端一旦明确返回 CRC 错误，程序会立即报错，
不再连续重发控制帧。

### 9.3 末端供电限制（必须确认）

只有当日志不再报 CRC/指令错误，但六轴仍全部 `en=0` 时，才继续检查供电。
机械臂关节未使能不等于手端电机不能使能；但 Leadshine DH116 类 11 DoF/6 主动轴手的
规格约为 `12~48V、2.5A`；Nova 末端 IO 规格为 `24V、总最大 2A、单通道最大 0.5A`。
因此逻辑板可以正常回 RS485 帧，但六个电机仍可能拒绝使能。

在继续回零前，用万用表在灵巧手 VDC/GND 处测量：空闲和执行使能命令瞬间都应满足
手册的电压要求，不能只测机械臂端口空载电压。若电压下陷，需要按灵巧手手册接入
独立、限流合适的 24V 电源，同时保留 RS485 A/B 和通信地；不能把外部 24V 与 Nova
24V 直接并联，除非设备厂商明确提供电源 OR-ing 方案。

```bash
# 第一次只做回零，现场有人按急停监护
PYTHONPATH=. python3 test_lhandpro_flexible_grasp.py \
  --transport dobot --robot-ip 192.168.5.1 \
  --baud 500000 --node 1 --debug --home-timeout 90 home

# 回零确认无误后张开
PYTHONPATH=. python3 test_lhandpro_flexible_grasp.py \
  --transport dobot --robot-ip 192.168.5.1 \
  --baud 500000 --node 1 open
```

回零失败时先回到 `info`，记录 `alarm/status/position/current`，不要连续重复
`home`。确认方向和限位后，按原流程在空手状态录制曲线：

```bash
PYTHONPATH=. python3 test_lhandpro_flexible_grasp.py \
  --transport dobot --robot-ip 192.168.5.1 \
  --baud 500000 --node 1 --home-timeout 90 \
  record --start 0 --max 9000 --step 250 --yes
```

货架主程序使用同一条机械臂 Dashboard 连接，设置：

```bash
HAND_BACKEND=lhandpro_dobot HAND_GRASP_MODE=flex \
LHAND_DOBOT_BAUD=500000 LHAND_RS485_NODE_ID=1 \
python3 shelf_grasp.py
```

不要同时运行两个程序创建 Modbus 主站；测试脚本退出后再启动货架程序。
`LHAND_RS485_PORT` 只属于旧的 USB `serial` 后端，在机械臂接法下不需要设置。

只有以下项目全部通过后才能运行机械臂抓取：

- 只读通信正常，无报警
- 回零和张开方向正确
- 空载曲线有效
- 独立柔性抓取能识别接触、空抓和安全停止
- 负载重量和重心已更新
- 碰撞箱尺寸已更新并通过无动作检查
- 新手/转接板没有改变相机外参；若改变，已重新完成手眼标定
- 旧 Aero 抓取偏移已经重新做低速对齐验证

USB-RS485 直连的旧启动模板：

```bash
cd /home/nvidia/grasp/dipan

HAND_BACKEND=lhandpro \
HAND_GRASP_MODE=flex \
LHAND_RS485_PORT=/dev/ttyUSB0 \
HAND_WEIGHT_KG=0.xxx \
HAND_COG_X_M=0.xxx HAND_COG_Y_M=0.xxx HAND_COG_Z_M=0.xxx \
TOOL_HAND_LENGTH_Z_M=0.xxx \
HAND_PALM_LENGTH_M=0.xxx \
TOOL_HAND_BODY_DEPTH_X_M=0.xxx \
TOOL_HAND_PALM_WIDTH_Y_M=0.xxx \
TOOL_HAND_OPEN_WIDTH_Y_M=0.xxx \
TOOL_HAND_MOUNT_EXCLUSION_Z_M=0.000 \
HAND_FINGER_THICKNESS_M=0.xxx \
DOBOT_IP=192.168.5.1 \
python3 shelf_grasp.py
```

机械臂末端 RS485 启动模板：

```bash
HAND_BACKEND=lhandpro_dobot \
HAND_GRASP_MODE=flex \
LHAND_RS485_BAUD=500000 \
LHAND_RS485_NODE_ID=1 \
HAND_WEIGHT_KG=0.xxx \
HAND_COG_X_M=0.xxx HAND_COG_Y_M=0.xxx HAND_COG_Z_M=0.xxx \
TOOL_HAND_LENGTH_Z_M=0.xxx \
HAND_PALM_LENGTH_M=0.xxx \
TOOL_HAND_BODY_DEPTH_X_M=0.xxx \
TOOL_HAND_PALM_WIDTH_Y_M=0.xxx \
TOOL_HAND_OPEN_WIDTH_Y_M=0.xxx \
TOOL_HAND_MOUNT_EXCLUSION_Z_M=0.000 \
HAND_FINGER_THICKNESS_M=0.xxx \
DOBOT_IP=192.168.5.1 \
python3 shelf_grasp.py
```

第一次联机只在空旷位置、低速、有人监护且急停可达的条件下进行。新手长度更新后，
法兰抓取点会随 `TOOL_HAND_LENGTH_Z_M` 改变，但 `SIDE_GRASP_OFFSET_M`、
`TOP_GRASP_OFFSET_M` 仍是旧手现场参数，必须重新做抓取对齐和小步验证，不能直接批量抓取。

## 10. 回退旧手

默认后端仍是 Aero。需要回退时不设置 `HAND_BACKEND`，或明确设置：

```bash
HAND_BACKEND=aero HAND_GRASP_MODE=flex python3 shelf_grasp.py
```

LHandPro 曲线保存在独立文件，不会覆盖 Aero 的 `runtime/aero_empty.csv`。
