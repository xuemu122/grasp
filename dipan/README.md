# dipan — 机械臂抓取、手眼标定与底盘控制

本目录包含 DOBOT Nova5 货架抓取主程序、Aero Hand 控制、手眼标定、底盘 TCP 移动控制和调试工具。

## 文件

| 文件 | 说明 |
|------|------|
| `shelf_grasp.py` | 货架抓取主程序：识别盒子、自动判定侧抓/上抓、执行抓取和放置 |
| `hand_eye_calib.py` | ArUco 手眼标定，生成 `calib_eye_in_hand.json` |
| `move_robot.py` | 底盘 TCP 控制，移动到药架/充电点/取消任务，并返回底盘电量 |
| `shelf_layers.json` | 货架层观察位姿，程序内 `l` 菜单维护 |
| `place_point.json` | 放置点位姿，程序内 `p` 菜单维护 |
| `place_transition.json` | 放置过渡点位姿，程序内 `p` 菜单维护 |
| `stow_pose.json` | 收起姿态，程序内 `u` 菜单维护 |
| `capture_one.py` | 单帧 RGB-D 采集工具 |
| `test_dobot.py` | DOBOT 连接测试 |

## 抓取主程序

启动：

```bash
cd /home/nvidia/grasp/dipan
DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

动作节奏可用环境变量现场微调：

```bash
MOTION_SETTLE_S=0.15 GRASP_SETTLE_S=0.3 HAND_CLOSE_WAIT_S=2.5 ROBOT_POLL_S=0.15 PLACE_TRANSITION_CP=60 \
  DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

含义：

- `MOTION_SETTLE_S`：普通运动完成后的短缓冲，默认 `0.2` 秒。
- `GRASP_SETTLE_S`：抓取前进、闭手前、后退后的缓冲，默认 `0.4` 秒。
- `HAND_CLOSE_WAIT_S`：灵巧手闭合后等待物体稳定的时间，默认 `3.0` 秒。
- `ROBOT_POLL_S`：等待 DOBOT 运动完成时的状态轮询间隔，默认 `0.2` 秒。
- `PLACE_TRANSITION_CP`：放置路径经过过渡点时的平滑过渡比例，默认 `60`；调大更顺滑，但会更明显地绕过中间点。

代码里的 `speed` 参数会随 `MovL`/`MovJ` 下发为 DOBOT `v=1..100` 百分比速度；超过 100 的值会自动夹到 100。记录放置过渡点后，前往最终放置点和放置后回层都会连续下发路径，过渡点不会再停顿。

启动后程序会：

1. 连接相机和 DOBOT。
2. 先回 HOME。
3. 加载 `shelf_layers.json`、`place_point.json`、`place_transition.json` 和 `stow_pose.json`。
4. 固定默认回到第 2 层观察位，忽略上次退出时的 `current`，避免从第 3/4 层启动造成碰撞。

## 主菜单

```text
Enter  抓取
l      货架层管理
p      放置点管理
u      收起姿态管理
b      切换底盘放置流程
v      预览
j      点动
q      退出
```

## 货架层管理

进入 `l` 菜单后：

```text
<数字>    切换并移动到该层
s         保存当前法兰位姿为新层
s <数字>  覆盖指定层
d <数字>  删除层
q         返回
```

层位姿保存在 `shelf_layers.json`。当前配置中第 1-4 层分别是货架不同高度，第 4 层在基座下方，因此代码里只对第 4 层允许低/负 Z。

## 放置点管理

进入 `p` 菜单后，把机械臂拖到放置位置，输入 `s` 保存。放置点保存在 `place_point.json`。

如果最终放置点距离较远，可以再拖动机械臂到中间安全位置，输入 `ts` 保存放置过渡点。过渡点保存在 `place_transition.json`。

常用操作：

```text
s   记录当前法兰位姿为放置点
g   移动到放置点；如果有过渡点，会先经过渡点
d   删除放置点
ts  记录当前法兰位姿为放置过渡点
tg  移动到放置过渡点
td  删除放置过渡点
q   返回
```

抓取成功后，如果放置点存在，程序会自动前往放置点并张手；如果放置点不可达或确认距离过大，会停止在当前位置，不继续回层。

记录了放置过渡点时，放置路径会变为：

```text
当前层/中转层 → 平滑经过放置过渡点 → 最终放置点 → 张手 → 平滑经过放置过渡点 → 回层
```

## 收起姿态管理

进入 `u` 菜单后，把机械臂拖到安全收起姿态，输入 `s` 保存。收起姿态保存在 `stow_pose.json`。

收起姿态用于两类场景，执行收起前都会先回第 2 层观察位避碰：

- 退出程序前会尝试先回第 2 层，再回到收起姿态，最后断开 DOBOT 和相机。
- 底盘放置流程开启时，调用底盘移动前必须先回第 2 层并回到收起姿态，防止底盘移动时机械臂伸出造成碰撞。

如果未记录收起姿态，退出时会跳过；底盘放置流程开启时则会停止，不会调用底盘。

## 抓取模式

`plan_edge_grasp()` 会根据检测框宽度自动判断：

- 盒宽 `<= 8cm`：侧面抓，使用 `SIDE_GRASP_OFFSET_M`
- 盒宽 `> 8cm`：上面抓，使用 `TOP_GRASP_OFFSET_M`，抓取前 J6 转到 `TOP_J6_DEG`

机械臂路径仍按侧抓/上抓自动判定。灵巧手侧面抓当前默认使用瓶子抓取手势：

```python
GRASP_BOTTLE = [0, 82, 65, 76, 55, 67, 63]
```

启动时默认：

```bash
HAND_GRASP_MODE=bottle
```

上端抓取不使用瓶子手势，仍使用 `GRASP_TOP`。如果后续不是瓶子，想让侧面抓也恢复原来的普通手势，可以这样启动：

```bash
HAND_GRASP_MODE=auto DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

目标选择在 `pick_target_box()` 内完成。当前规则：

- 先按画面纵向 ROI 过滤上下层物品，默认只保留 `y=85~430` 的候选。
- 在 ROI 内按深度分层，选最前层。
- 在最前层从右往左选目标，始终从右侧边缘抓取。

如果仍然识别到上下层，可以按现场画面调 ROI：

```bash
TARGET_ROI_Y_MIN=170 TARGET_ROI_Y_MAX=320 DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

检测截图 `/tmp/shelf_detection.jpg` 会画蓝色 ROI 边界线。

每次抓取前会进入微调界面，显示当前模式、偏移参数和固化建议。

常用微调键：

```text
wasd  微调 X/Y
e/r   微调 Z
p     预览悬停位
f     手动输入 x,y,z 偏移
F5    重新拍照
Enter 执行抓取
Q     跳过/退出
```

## 3/4 层放置路径

第 1/2 层：抓完后直接去放置点，放完再回当前层。

第 3 层：抓完后先回第 3 层观察位，再直接上到第 1 层，之后去放置点；放完后回第 1 层，再安全下降回第 3 层。

第 4 层：抓完后先回第 4 层观察位，再直接上撤到第 3 层，然后到第 1 层，之后去放置点；放完后回第 1 层，再安全下降回第 4 层。

下降回第 3/4 层时使用 `go_to_layer_via_clearance()`，上撤时使用直接 `go_to_layer()`。

## 底盘放置流程

主菜单输入 `b` 可以切换底盘放置流程，提示里会显示 `b 底盘:开/关`。

默认关闭，方便只调机械臂抓取和放置点。也可以启动前用环境变量默认开启：

```bash
USE_CHASSIS_PLACE=1 MARKER_PLACE=place_point DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

开启后抓取完成流程会变为：

```text
抓取后退
→ 按当前层逻辑上撤
→ 回第 2 层
→ 回到收起姿态
→ 调用 move_robot.py place
→ 等底盘到达放置固定点
→ 机械臂前往 place_point.json 的放置点
→ 张手放置
→ 回层
```

`MARKER_PLACE` 是底盘地图里的固定点名称，默认值是 `place_point`。

## 第 4 层低位设置

第 4 层允许低/负 Z：

```python
LOW_Z_LAYER_IDXS = {4}
```

第 4 层抓取时使用更保守的低位参数：

```python
LOW_Z_GRASP_ADVANCE_M
LOW_Z_APPROACH_SPEED
LOW_Z_GRASP_SPEED
```

其他层仍保留 `Z < 50mm` 本地拒绝保护。

## 点动

主菜单输入 `j`：

```text
wasd  XY
e/r   Z
z/x   J6 -/+
=/+   放大步长
-     缩小步长
q     退出点动
```

点动模式会按当前层判断是否允许低 Z；第 4 层可低位点动，其他层仍保护。

## 手眼标定

```bash
cd /home/nvidia/grasp/dipan
DOBOT_IP=192.168.5.1 python3 hand_eye_calib.py
```

流程：

1. 打印或显示 ArUco marker。
2. 确认 `MARKER_SIZE_MM` 与实际 marker 边长一致。
3. 拖动机械臂到多角度、多位置观察 marker，采集 10-15 组。
4. 求解后保存 `calib_eye_in_hand.json`。

## 底盘控制

```bash
cd /home/nvidia/grasp/dipan
python3 move_robot.py           # 交互模式
python3 move_robot.py yaojia1   # 去药架，默认等待到达
python3 move_robot.py charge    # 去充电点，默认等待到达
python3 move_robot.py place     # 去放置固定点，默认等待到达
python3 move_robot.py place --no-wait
python3 move_robot.py cancel    # 取消移动
```

底盘地址：

```text
192.168.124.90:31001
```

每次移动/取消后，脚本会额外调用：

```text
/api/get_power_status
```

返回值会附加到原结果的 `power_status` 字段，并打印电量、电压、电流和充电状态。

移动命令默认还会轮询：

```text
/api/robot_status
```

直到 `move_status=succeeded` 才返回成功；如果 `failed`、`canceled` 或超时，会返回非 0，抓取主程序不会继续机械臂放置。
