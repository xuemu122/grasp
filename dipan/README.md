# dipan — 机械臂抓取、手眼标定与底盘控制

本目录包含 DOBOT Nova5 货架抓取主程序、Aero Hand 控制、手眼标定、底盘 TCP 移动控制和调试工具。

## 文件

| 文件 | 说明 |
|------|------|
| `shelf_grasp.py` | 统一入口和流程编排：菜单、识别、抓取、放置、底盘调用 |
| `dobot_arm.py` | DOBOT Nova5 TCP 控制：连接、使能、MovL/MovJ、报警恢复、关节安全过渡 |
| `aero_gripper.py` | Aero Hand 控制：张开、普通抓、上端抓、瓶子抓 |
| `hand_eye.py` | 手眼坐标转换：像素/深度到机器人基座坐标，含远距透视补偿 |
| `grasp_planner.py` | 抓取目标选择和抓取点规划：ROI 过滤、深度分层、右侧优先、侧抓/上抓判定 |
| `target_selection.py` | 检测结果融合、目标关键词匹配、类别列表显示、目标深度重采样 |
| `depth_collision_guard.py` | 深度护栏：把当前深度图投到机器人坐标系，发动作前检查灵巧手路径是否扫到障碍 |
| `self_collision_guard.py` | Nova5 自碰撞护栏：读取 URDF/SRDF，用胶囊近似在 MovL/MovJ 前检查自身连杆碰撞 |
| `hand_eye_calib.py` | ArUco 手眼标定，生成 `calib_eye_in_hand.json` |
| `move_robot.py` | 底盘 TCP 控制，移动到药架/充电点/取消任务，并返回底盘电量 |
| `test_aero_current_grasp.py` | Aero Hand 电流夹紧测试：0号固定，其余关节渐闭，电流上升后停止 |
| `shelf_layers.json` | 货架层观察位姿，程序内 `l` 菜单维护 |
| `place_point.json` | 放置点位姿，程序内 `p` 菜单维护 |
| `place_transition.json` | 放置过渡点位姿，程序内 `p` 菜单维护 |
| `stow_pose.json` | 收起姿态，程序内 `u` 菜单维护 |
| `capture_one.py` | 单帧 RGB-D 采集工具 |
| `test_dobot.py` | DOBOT 连接测试 |

## 抓取主程序

统一仍然从 `shelf_grasp.py` 启动；其他模块由入口脚本导入复用，现场不用分别运行。

启动：

```bash
cd /home/nvidia/grasp/dipan
DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

识别阈值和动作节奏可用环境变量现场微调：

```bash
DETECT_CONF_THRESHOLD=0.7 WATER_CONF_THRESHOLD=0.52 MOTION_SETTLE_S=0.15 GRASP_SETTLE_S=0.3 HAND_CLOSE_WAIT_S=2.5 ROBOT_POLL_S=0.15 PLACE_RETURN_CLEARANCE_MM=140 PLACE_RETURN_MAX_Z_MM=820 \
  DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

含义：

- `DETECT_CONF_THRESHOLD`：抓取主程序 YOLO 显示/输出阈值，默认 `0.7`。
- `WATER_CONF_THRESHOLD`：饮料/矿泉水类专用阈值，默认 `0.52`；只放宽 `drink` 类，避免所有类别一起降阈值造成误检。
- `DETECTION_STABILITY_IOU`：抓取前多帧检测融合的同目标 IoU 阈值，默认 `0.35`；矿泉水识别闪烁时会按多帧稳定候选融合。
- `TARGET_DEPTH_REFINE_RADIUS_PX`：最终参考帧上重新采样目标中心深度的半径，默认 `10` px；用于让抓取点深度与实际用于抓取的图像帧一致。
- `TARGET_DEPTH_REFINE_MAX_DELTA_M`：目标深度重采样允许偏离融合深度的最大值，默认 `0.08` m；超过则认为可能采到背景并跳过校正。
- `MOTION_SETTLE_S`：普通运动完成后的短缓冲，默认 `0.2` 秒。
- `GRASP_SETTLE_S`：抓取前进、闭手前、后退后的缓冲，默认 `0.4` 秒。
- `HAND_CLOSE_WAIT_S`：灵巧手闭合后等待物体稳定的时间，默认 `3.0` 秒。
- `ROBOT_POLL_S`：等待 DOBOT 运动完成时的状态轮询间隔，默认 `0.2` 秒。
- `PLACE_TRANSITION_CP`：放置路径经过过渡点时的平滑过渡比例，默认 `60`；调大更顺滑，但会更明显地绕过中间点。
- `PLACE_SMOOTH_TRANSITION`：放置段是否启用 `MovL cp` 平滑过渡，默认关闭。
- `PLACE_RETURN_CLEARANCE_MM`：放置后回层时先向上抬高的安全量，默认 `140` mm；如果回第 2 层仍擦碰，可以继续调大。
- `PLACE_RETURN_MAX_Z_MM`：放置后回层抬高的默认上限，默认 `820` mm；用于避免在靠外放置点抬得过高触发机械臂报警。
- `PERSPECTIVE_COMP_ENABLE`：开启远距透视补偿，默认开启；只压纵向，不收左右。
- `PERSPECTIVE_COMP_Z_REF_M`：透视补偿参考距离，默认 `0.32` m。
- `PERSPECTIVE_COMP_GAIN`：透视补偿强度，默认 `2.0`。
- `PERSPECTIVE_COMP_MIN_SCALE`：透视补偿最小缩放，默认 `0.65`。
- `GRASP_APPROACH_BACKOFF_M`：抓取前固定退距，默认 `0.15` m；对齐位会放在抓取位后方这个距离，避免从观察位保留过长探入路径。
- `SIDE_GRASP_MIN_HEIGHT_M`：侧面抓取点离物体底部的最小高度，默认 `0.06` m；物体高度不够时自动改为上面抓。
- `SIDE_GRASP_MAX_HEIGHT_M`：侧面抓取点离物体底部的最大高度，默认 `0.11` m；防止抓手位置过高导致相机与前一层层板碰撞。
- `LAYER_Z_TOLERANCE_M`：当前层世界 Z 过滤容差，默认 `0.16` m；如果误识别上一层会被剔除，现场层高/标定偏差大时可调大。
- `MAX_GRASP_DEPTH_M`：最大抓取深度，默认 `0.52` m；超过此深度的物品不抓取，避免机械臂伸太深导致货架层板碰撞相机。货架更浅时可调小。
- `ALLOW_NO_DEPTH_GRASP`：无可信深度时是否仍按最右侧框兜底抓取，默认关闭；只建议离线调试时临时设为 `1`。
- `TARGET_PRODUCT_QUERY`：目标商品/类别关键词，默认不指定；可匹配任意模型类别或检测身份字段，包括 `product_name`、`medicine_name`、`class_name`、`category`、`package_type`、`product_id`、`class_id`、`ocr_text`。
- `SELF_COLLISION_ENABLE`：Nova5 自碰撞护栏开关，默认开启；基于 URDF/SRDF 做 FK，并用保守胶囊近似检查 MovL/MovJ 的关节路径。
- `SELF_COLLISION_MARGIN_M`：自碰撞安全余量，默认 `0.015` m；误拦可调小，担心擦碰可调大。
- `SELF_COLLISION_PATH_SAMPLES`：自碰撞路径采样次数，默认 `8`；关节运动跨度大时可调大。
- `SELF_COLLISION_REQUIRE_IK`：MovL 无法求近邻逆解时是否中止，默认关闭；开启后更保守，但 IK 返回异常会导致 MovL 被拒绝。
- `SELF_COLLISION_VERBOSE`：自碰撞护栏通过时是否打印最近余量，默认关闭。
- `DOBOT_IK_VERBOSE`：是否打印自碰撞检查用的 `InverseKin` 请求/响应，默认关闭；排查 IK 失败时可开启。
- `DEPTH_GUARD_ENABLE`：深度护栏开关，默认开启；会排除当前画面中所有检测框，把其余深度点当成货架/障碍，检查对齐和探入路径。
- `DEPTH_GUARD_CLEARANCE_M`：深度护栏额外安全余量，默认 `0.005` m；默认手部半径按 `HAND_W_Y/2 + 余量` 计算。
- `DEPTH_GUARD_HAND_RADIUS_M`：灵巧手安全半径，默认约 `0.0525` m；等于 95mm 手宽的一半再加 5mm 余量。误拦截可调小到 `0.0475`，仍擦碰可调大到 `0.06`。
- `DEPTH_GUARD_APPROACH_LENGTH_M`：对齐路径使用的手部检查长度，默认 `0.08` m；对齐阶段只检查法兰/手掌根部附近，避免把准备抓的货物前方误判成碰撞。最后探入仍使用完整手长。
- `DEPTH_GUARD_STRIDE`：深度点采样步长，默认 `8`；调小更密但更慢，调大更快但可能漏细杆/边缘。
- `DEPTH_GUARD_PATH_SAMPLES`：路径采样次数，默认 `8`；运动路径长或拐得多时可调大。
- `DEPTH_GUARD_MAX_DEPTH_M`：纳入障碍点的最大深度，默认 `1.20` m；货架背景太远造成误拦截时可调小。
- `DEPTH_GUARD_EXCLUDE_MARGIN_PX`：检测框外扩排除范围，默认 `32` px；如果货物边缘被误当货架可继续调大。
- `DEPTH_GUARD_SELF_MASK_ENABLE`：自手部深度点剔除开关，默认开启；用于过滤深度相机看到的当前灵巧手/手指，避免把自己的手当障碍。
- `DEPTH_GUARD_SELF_MASK_RADIUS_M`：自手部剔除半径，默认约 `0.0725` m；如果红点仍贴在手指上可略调大，如果误删近处层板可调小。
- `DEPTH_GUARD_SELF_MASK_LENGTH_M`：自手部剔除长度，默认 `0.25` m；通常等于法兰到指尖长度。
- `DEPTH_GUARD_SELF_MASK_BACKOFF_M`：自手部剔除从法兰向后覆盖的长度，默认 `0.03` m；用于覆盖手掌根部/安装件附近的深度点。
- `DEPTH_GUARD_DEPTH_ALIGN_ENABLE`：相机深度一致性过滤开关，默认开启；碰撞点除了要靠近手部胶囊，还要和路径最近点在相机深度方向上对齐，用于过滤透视造成的近邻误判。
- `DEPTH_GUARD_DEPTH_ALIGN_TOLERANCE_M`：相机深度一致性容差，默认 `0.035` m；如果层板确实有距离但仍被红点拦截可略调小/保持，若真实障碍漏检则调大或关闭深度对齐。
- `DEPTH_GUARD_AXIS_WINDOW_ENABLE`：探入路径轴向窗口开关，默认开启；只检查法兰附近到指尖前方一点的障碍，忽略指尖/目标后方的层板后缘或背景点。
- `DEPTH_GUARD_FINGERTIP_FRONT_MARGIN_M`：指尖前方仍纳入检查的长度，默认 `0.02` m；红点在目标后方/层板后方时可调小，担心指尖前方障碍漏检时调大。
- `DEPTH_GUARD_FLANGE_BACK_MARGIN_M`：法兰后方仍纳入检查的长度，默认 `0.03` m；用于覆盖手掌根部附近。
- `DEPTH_GUARD_BYPASS_ENABLE`：对齐路径自动绕行开关，默认开启；直达被深度护栏挡住时会尝试上抬/侧绕中间点。
- `DEPTH_GUARD_BYPASS_LIFTS_MM`：绕行上抬候选，默认 `60,100,140` mm。
- `DEPTH_GUARD_BYPASS_SIDE_MM`：绕行侧向候选，默认 `60,-60` mm。

深度护栏拦截动作时会自动保存调试图到 `/tmp/depth_guard_*.jpg`：灰点为参与判断的深度障碍点，红点为命中碰撞半径的点，绿框为检测排除框/当前目标框，青线为手部检查线。

代码里的 `speed` 参数会随 `MovL`/`MovJ` 下发为 DOBOT `v=1..100` 百分比速度；超过 100 的值会自动夹到 100。放置段可选 `MovL cp` 平滑过渡；回层会先原地抬高，再高位回层。

启动后程序会：

1. 连接相机和 DOBOT。
2. 先回 HOME。
3. 加载 `shelf_layers.json`、`place_point.json`、`place_transition.json` 和 `stow_pose.json`。
4. 固定默认回到第 2 层观察位，忽略上次退出时的 `current`，避免从第 3/4 层启动造成碰撞。

## 主菜单

```text
Enter  抓取
c      设置目标商品/类别/id
?      查看模型类别列表
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

如果最终放置点距离较远，可以再拖动机械臂到中间安全位置，输入 `ts` 保存放置过渡点。

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
当前层/中转层 → 关节运动到放置过渡点 → 最终放置点 → 张手 → 原地抬高 → 回层
```

如果开启 `PLACE_SMOOTH_TRANSITION=1`，最终移动到放置点时会使用 `PLACE_TRANSITION_CP` 作为 `MovL cp` 平滑过渡比例。

如果菜单里显示过渡点 `[无joints]`，建议拖动机械臂到该过渡点后重新输入 `ts` 记录一次。大范围换姿态时程序会优先使用记录关节角 `MovJ`，避免 `MovL` 直线插补导致腕部翻转和机械臂自碰撞。

## 收起姿态管理

进入 `u` 菜单后，把机械臂拖到安全收起姿态，输入 `s` 保存。收起姿态保存在 `stow_pose.json`。

收起姿态用于退出程序时的主动收尾：

- 按 `q` 退出时会回到收起姿态，再断开 DOBOT 和相机。
- 放置流程内部默认不再自动收起，避免放置前流程自行回撤。

如果未记录收起姿态，按 `q` 退出时会跳过。

## 抓取模式

`plan_edge_grasp()` 会根据检测框宽度自动判断：

- 盒宽 `<= 8cm`：侧面抓，使用 `SIDE_GRASP_OFFSET_M`
- 盒宽 `> 8cm`：上面抓，使用 `TOP_GRASP_OFFSET_M`，抓取前 J6 转到 `TOP_J6_DEG`

侧面抓取点不会低于物体底部上方 `6cm`。如果检测框估算出的物体高度小于等于 `6cm`，会自动改成上面抓取，避免手掌或手指低位撞到货架/物体。

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

## Aero 电流夹紧测试

单独测试灵巧手“电流变大即停止”：

```bash
cd /home/nvidia/grasp/dipan
python3 test_aero_current_grasp.py
```

默认流程：

- 0 号关节固定 `80`
- 1-6 号关节从 `0` 开始，每步增加 `2`
- 每步读取 7 路电机电流，单位 mA
- 1-6 号任意一路满足 `相对基线增加 >= 180mA` 或 `绝对电流 >= 650mA` 时停止
- 低于 `--min-cmd` 的早期尖峰只记录不停止，默认 `5`
- 默认需要连续 `2` 步超过阈值才停止，避免刚启动的瞬时电流误触发
- 停止后保持 5 秒，不主动张开；加 `--release-after` 可测试结束后张开

常用调参：

```bash
python3 test_aero_current_grasp.py --step 1 --delta-ma 120 --abs-ma 300 --min-cmd 8 --consecutive 3 --settle 0.3 --samples 5
python3 test_aero_current_grasp.py --step 1 --delta-ma 120 --abs-ma 300 --ignore-actuators 5
python3 test_aero_current_grasp.py --active-joints 3 --max 45 --release-after  # 只测食指
python3 test_aero_current_grasp.py --active-joints 4 --max 45 --release-after  # 只测中指
python3 test_aero_current_grasp.py --active-joints 5 --max 45 --release-after  # 只测无名指
python3 test_aero_current_grasp.py --active-joints 6 --max 45 --release-after  # 只测小指
python3 test_aero_current_grasp.py --torque-limit 250 --step 1 --delta-ma 120 --abs-ma 300 --min-cmd 8
python3 test_aero_current_grasp.py --max 70 --hold-seconds 10
python3 test_aero_current_grasp.py --release-after
```

每次会把日志写到 `/tmp/aero_current_grasp_*.csv`，里面有每步命令角度、电流、相对基线增量和触发原因。

如果空载弯曲本身就会让电流上升，先录一条空载曲线，再按同角度差值判断：

```bash
python3 test_aero_current_grasp.py --active-joints 1,2,3,4,5,6 --max 60 --step 1 --disable-trigger --release-after --log /tmp/aero_empty.csv
python3 test_aero_current_grasp.py --active-joints 1,2,3,4,5,6 --max 60 --step 1 --profile /tmp/aero_empty.csv --profile-delta-ma 180 --min-cmd 15 --consecutive 2 --release-after
```

这会把“正常弯曲/预紧造成的电流”从判断里扣掉，只在比空载曲线额外高出阈值时才停止。

目标选择支持两层规则：先按目标关键词过滤，再由 `pick_target_box()` 做几何选择。主菜单里输入 `?` 可查看当前模型的全部 `class_id:class_name`；输入 `c` 可以设置任意目标商品/类别/id；也可以直接输入目标名开始抓取，例如 `雪碧`、`褪黑素`、`sprite`、`ths`、`class_id:63`。输入 `all` 或 `clear` 会清空目标，恢复任意候选抓取。

也可以启动时指定：

```bash
TARGET_PRODUCT_QUERY=雪碧 DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
TARGET_PRODUCT_QUERY=褪黑素 DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

目标过滤命中后，`pick_target_box()` 当前规则：

- 先按画面纵向 ROI 过滤上下层物品，默认只保留 `y=85~430` 的候选。
- 再按当前层记录位姿的世界 Z 过滤上下层，默认容差 `±16cm`。
- 在 ROI 内按深度分层，选最前层。
- 在最前层从右往左选目标，始终从右侧边缘抓取。

如果仍然识别到上下层，可以按现场画面调 ROI：

```bash
TARGET_ROI_Y_MIN=170 TARGET_ROI_Y_MAX=320 DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

如果当前层目标也被层高过滤误杀，可以临时调大或关闭：

```bash
LAYER_Z_TOLERANCE_M=0.22 DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
LAYER_Z_FILTER_ENABLE=0 DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

检测截图 `/tmp/shelf_detection.jpg` 会画蓝色 ROI 边界线。

每次抓取识别后还会保存 `/tmp/shelf_yolo_candidates.jpg`：绿色框表示达到 `DETECT_CONF_THRESHOLD`，橙色框表示 YOLO 有原始候选但置信度低于阈值；图左上角会写 raw/显示/低置信数量和光照评分。

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
