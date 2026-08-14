# dipan — 机械臂抓取、手眼标定与底盘控制

本目录包含 DOBOT Nova5 货架抓取主程序、Aero/LHandPro 灵巧手控制、手眼标定、底盘 TCP 移动控制和调试工具。

## 文件

| 文件 | 说明 |
|------|------|
| `shelf_grasp.py` | 统一入口和流程编排：菜单、识别、抓取、放置、底盘调用 |
| `dobot_arm.py` | DOBOT Nova5 TCP 控制：连接、使能、MovL/MovJ、报警恢复、关节安全过渡 |
| `aero_gripper.py` | Aero Hand 控制：张开、普通抓、上端抓、瓶子抓 |
| `lhandpro_gripper.py` | LHandPro RS485 控制：张开、固定位置抓取、空载曲线和逐轴电流柔性抓取 |
| `test_lhandpro_flexible_grasp.py` | 新灵巧手独立测试：通信状态、回零张开、录空载曲线、单次柔性抓取 |
| `LHANDPRO_RS485_安装标定与测试说明.md` | 新灵巧手现场说明：接线、空载曲线、柔性测试、负载和碰撞箱更新 |
| `hand_eye.py` | 手眼坐标转换：像素/深度到机器人基座坐标，含远距透视补偿 |
| `grasp_planner.py` | 抓取目标选择和抓取点规划：动态层板过滤、深度分层、右侧优先、侧抓/上抓判定 |
| `target_selection.py` | 检测结果融合、目标关键词匹配、类别列表显示、目标深度重采样 |
| `depth_collision_guard.py` | 深度护栏：把当前深度图投到机器人坐标系，发动作前检查灵巧手路径是否扫到障碍 |
| `self_collision_guard.py` | Nova5 自碰撞护栏：读取 URDF/SRDF，用胶囊近似在 MovL/MovJ 前检查自身连杆碰撞 |
| `tool_geometry.py` | 末端共享几何：法兰坐标系下的灵巧手附属相机尺寸与碰撞近似 |
| `hand_eye_calib.py` | ArUco 手眼标定，生成 `calib_eye_in_hand.json` |
| `move_robot.py` | 底盘 TCP 控制，移动到药架/充电点/取消任务，并返回底盘电量 |
| `test_aero_current_grasp.py` | Aero Hand 电流夹紧测试：0号固定，其余关节渐闭，电流上升后停止 |
| `demo_aero_contact_states.py` | Aero Hand 接触状态识别 demo：空载、浅层接触、持续接触、饱和堆积、挤压溢出 |
| `shelf_layers.json` | 货架层观察位姿，程序内 `l` 菜单维护 |
| `place_point.json` | 放置点位姿，程序内 `p` 菜单维护 |
| `place_transition.json` | 放置过渡点位姿，程序内 `p` 菜单维护 |
| `stow_pose.json` | 收起姿态，程序内 `u` 菜单维护 |
| `capture_one.py` | 单帧 RGB-D 采集工具 |
| `test_dobot.py` | DOBOT 连接测试 |

### 历史脚本

`box_grasp.py` 是早期 Orca 灵巧手方案的独立实验程序，不属于当前
`shelf_grasp.py` + Aero Hand 主链路，也没有被运行入口引用。现场只使用上表中的
当前模块；保留该文件是为了兼容历史实验记录，后续可在确认不再需要 Orca 方案后单独归档或删除。

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
- `DETECTION_CAPTURE_FRAMES`：抓取前最多执行的 YOLO 帧数，默认 `10`。
- `DETECTION_WARMUP_FRAMES`：相机自动曝光预热丢弃帧数，默认 `2`。
- `DETECTION_EARLY_STOP_FRAMES`：至少采集多少有效帧后允许稳定目标提前结束，默认 `6`；设为 `0` 关闭提前结束。
- `DETECTION_EARLY_STOP_SUPPORT`：提前结束所需的同目标稳定轨迹支持帧数，默认 `5`。
- `SAVE_DEBUG_IMAGES`：是否保存识别候选图和抓取标注图，默认 `0`；需要现场排查时设为 `1`，失败原图仍会单独保存。
- `TARGET_DEPTH_REFINE_RADIUS_PX`：最终参考帧上重新采样目标中心深度的半径，默认 `10` px；用于让抓取点深度与实际用于抓取的图像帧一致。
- `TARGET_DEPTH_REFINE_MAX_DELTA_M`：目标深度重采样允许偏离融合深度的最大值，默认 `0.08` m；超过则认为可能采到背景并跳过校正。
- `MOTION_SETTLE_S`：普通运动完成后的短缓冲，默认 `0.2` 秒。
- `GRASP_SETTLE_S`：抓取前进、闭手前、后退后的缓冲，默认 `0.4` 秒。
- `HAND_CLOSE_WAIT_S`：灵巧手闭合后等待物体稳定的时间，默认 `1.0` 秒。
- `HAND_BACKEND`：灵巧手后端，默认 `lhandpro_dobot`；通过 Nova 末端 RS485 使用新手。旧 Aero 硬件可显式设为 `aero`回退。
- `LHAND_DOBOT_BAUD`：Nova 末端 RS485 波特率，默认 `115200`。
- `LHAND_RS485_NODE_ID`：灵巧手 RS485 节点号，默认 `1`。
- `LHAND_DOBOT_CONTROL`：控制帧类型，默认 `compact`，执行“使能→回零→再使能→电流→速度→位置”。
- `HAND_GRASP_MODE`：灵巧手抓取模式，默认 `flex`，使用电流反馈柔性抓取；可设 `bottle` 使用旧瓶子固定手势，或设 `auto` 使用普通/上抓固定手势。
- 直接运行 `shelf_grasp.py` 时按 `Ctrl+C` 会优先发送 `DisableRobot()` 让机械臂下使能，
  然后断开灵巧手和相机；不会发送 `PowerOff()`，控制柜仍保持上电。
- `FORCE_SIDE_GRASP`：是否临时强制所有目标按侧面抓取点规划，默认 `0`，即宽盒/矮盒会自动上抓；设为 `1` 可临时全侧抓。
- `AERO_FLEX_PROFILE`：柔性抓取空载曲线 CSV，默认 `/home/nvidia/grasp/dipan/runtime/aero_empty.csv`；建议先用 `test_aero_current_grasp.py --disable-trigger` 录制。
- `AERO_FLEX_ACTIVE_JOINTS`：柔性抓取参与闭合的 compact 关节，默认 `1,2,3,4,5,6`。
- `AERO_FLEX_REQUIRED_FINGERS`：柔性抓取必须完成的手指，默认取 active 中的 `3..6`；如果只用两指抓可设为 `3,4`。
- `AERO_FLEX_PROFILE_DELTA_MA`：相对空载曲线额外增加多少 mA 判定接触，默认 `180`。
- `AERO_FLEX_JOINT1_STEP`：一号 compact 关节的柔性闭合步长，默认 `2°`；保持阶段默认同步缩小，可用 `AERO_HOLD_JOINT1_STEP` 单独覆盖。
- `AERO_FLEX_JOINT2_STEP`：二号 compact 关节的柔性闭合步长，默认 `5°`；其他关节沿用 `AERO_FLEX_STEP` 的默认 `3°`。保持阶段默认同步放大，可用 `AERO_HOLD_JOINT2_STEP` 单独覆盖。
- `AERO_FLEX_PROFILE_WINDOW_DEG`：读取空载曲线时取附近角度电流上包络，默认 `6` 度；用于抑制某个角度空载电流异常偏低导致误触发。
- `AERO_FLEX_EXTRA_CLOSE`：单根手指检测到接触后继续闭合多少度再锁住，默认 `8`。
- `AERO_FLEX_MIN_CONTACTS`：柔性抓取判定成功所需的最少接触手指数，默认 `2`；仅到达最大闭合角度、不满足接触数时判定为空抓并让订单失败。
- `AERO_HOLD_TIGHTEN_ENABLE`：柔性抓取保持期间是否持续检测接触电流并自动补紧，默认开启。
- `AERO_HOLD_CHECK_INTERVAL_S` / `AERO_HOLD_TIGHTEN_CONSECUTIVE`：保持检测周期和连续低电流次数，默认 `0.2` 秒、`3` 次。
- `AERO_HOLD_MIN_EXTRA_MA` / `AERO_HOLD_TIGHTEN_STEP` / `AERO_HOLD_TIGHTEN_MAX`：保持电流余量、每次补紧角度和补紧上限；默认余量为抓取阈值的 55%（至少 `60mA`）、每次 `1°`、上限沿用 `AERO_FLEX_MAX`。
- `AERO_HOLD_HZ`：抓取成功后重复发送最终闭合目标的频率，默认 `100Hz`，与 Aero 官方 power-grasp 示例的 10ms 循环一致；一直保持到放置位调用张开，防止运输期间手指缓慢回开。
- `AERO_HOLD_MAX_ERRORS`：保持期间允许的连续发送失败次数，默认 `5`；短暂串口抖动恢复后继续保持，连续失败才禁用灵巧手。
- `AERO_HOLD_LOG_INTERVAL_S`：保持心跳日志间隔，默认 `10` 秒；设为 `0` 可关闭。心跳会显示实际发送数和失败数。
- `AERO_FLEX_MAX_TEMPERATURE_C` / `AERO_HOLD_MAX_TEMPERATURE_C`：柔性闭合/保持期间的温度提示阈值，默认 `65°C`；超限只后台打印，不改变抓取或张手动作。
- `AERO_FLEX_TEMPERATURE_EVERY` / `AERO_HOLD_TEMPERATURE_INTERVAL_S`：闭合阶段按步数、保持阶段按秒读取温度，默认 `10` 步、`1` 秒。
- `AERO_HOLD_TEMPERATURE_LOG_INTERVAL_S`：保持温度心跳日志间隔，默认 `10` 秒，会分别显示 0–6 号电机温度。
- `AERO_RESPONSE_TIMEOUT_S`：等待灵巧手状态响应的单次超时，默认 `0.2` 秒。
- `AERO_RESPONSE_RETRIES`：灵巧手状态查询失败后的总尝试次数，默认 `3`；读取时会自动跳过串口中的 `[BOOT]` 等非协议文本。
- `ROBOT_POLL_S`：等待 DOBOT 运动完成时的状态轮询间隔，默认 `0.2` 秒。
- `PLACE_TRANSITION_CP`：放置路径经过过渡点时的平滑过渡比例，默认 `60`；调大更顺滑，但会更明显地绕过中间点。
- `PLACE_SMOOTH_TRANSITION`：放置段是否启用 `MovL cp` 平滑过渡，默认关闭。
- `PLACE_RETURN_CLEARANCE_MM`：放置后前往下一目标层时先向上抬高的安全量，默认 `140` mm；如果从放置点去货架仍擦碰，可以继续调大。
- `PLACE_RETURN_MAX_Z_MM`：放置后前往下一目标层的抬高上限，默认 `820` mm；用于避免在靠外放置点抬得过高触发机械臂报警。
- `PERSPECTIVE_COMP_ENABLE`：开启远距透视补偿，默认开启；只压纵向，不收左右。
- `PERSPECTIVE_COMP_Z_REF_M`：透视补偿参考距离，默认 `0.32` m。
- `PERSPECTIVE_COMP_GAIN`：透视补偿强度，默认 `2.0`。
- `PERSPECTIVE_COMP_MIN_SCALE`：透视补偿最小缩放，默认 `0.65`。
- `GRASP_APPROACH_BACKOFF_M`：抓取前固定退距，默认 `0.15` m；对齐位会放在抓取位后方这个距离，避免从观察位保留过长探入路径。
- `SIDE_GRASP_MIN_HEIGHT_M` / `SIDE_GRASP_MAX_HEIGHT_M`：侧面抓取点离物体底部的允许区间，默认 `0.065..0.11` m。
- `SIDE_GRASP_LOWER_SHIFT_M`：在原侧抓选点基础上向瓶底方向下移的距离，默认 `0.01` m；下移后仍夹紧在上述区间内。
- `LAYER_Z_TOLERANCE_M`：当前层世界 Z 过滤容差，默认 `0.16` m；如果误识别上一层会被剔除，现场层高/标定偏差大时可调大。
- `SHELF_BOARD_FILTER_ENABLE`：是否启用基于当前深度帧的层板识别过滤，默认开启。识别不到可靠上下层板时自动回退旧的 ROI/Z 过滤。
- `SHELF_BOARD_SAMPLE_STRIDE`：层板识别的深度采样步长，默认 `4` px；调小可提高稀疏/断裂层板的识别率，但会增加计算量。
- `SHELF_BOARD_MIN_POINTS`：单个图像行形成层板候选所需的最少有效点数，默认 `24`。
- `SHELF_BOARD_MIN_EDGE_POINTS`：深度边缘型层板候选所需的最少连续边缘点数，默认 `12`。
- `SHELF_BOARD_MIN_X_SPAN_M`：层板候选在机器人 X 方向的最小横向跨度，默认 `0.18` m；用于排除商品表面和小块背景。
- `SHELF_BOARD_MAX_Z_SPREAD_M`：同一图像行允许的层板 Z 高度离散范围，默认 `0.045` m；深度噪声较大时可适当调大。
- `SHELF_BOARD_MIN_DEPTH_EDGE_M`：相邻采样行的最小深度跳变，默认 `0.035` m；用于识别被商品遮挡、没有完整平面点的层板前沿。
- `SHELF_BOARD_CLUSTER_Z_M`：相邻采样行合并为同一层板的 Z 距离阈值，默认 `0.055` m。
- `SHELF_BOARD_ROW_GAP`：相邻采样行合并为同一层板的最大行间距，默认 `40` px；用于把同一块层板的上沿、前沿等多条边缘合并起来。
- `SHELF_BOARD_ROW_EXCLUDE_MARGIN_PX`：检测框外扩后屏蔽框内深度点的边距，默认 `6` px；只屏蔽商品附近的局部像素，商品左右仍可用于识别同一行的层板。
- `SHELF_BOARD_MIN_ROW_SPAN_PX`：上下层板候选的最小图像行间距，默认 `100` px；用于排除相邻背景横线或同一平面产生的窄范围假边界。
- `SHELF_BOARD_EXPECTED_Z_SPAN_M`：现场上下层板的标准机器人 Z 间距，默认 `0.38` m。
- `SHELF_BOARD_MIN_Z_SPAN_M` / `SHELF_BOARD_MAX_Z_SPAN_M`：动态上下板配对允许的间距范围，默认 `0.30..0.46` m；范围外不采用动态结果。
- `SHELF_BOARD_MAX_REFERENCE_DISTANCE_M`：候选层板距离当前层参考 Z 的最大距离，默认 `0.75` m。
- `SHELF_BOARD_OBJECT_MARGIN_M`：上下层板内侧留给商品的 Z 安全边距，默认 `0.018` m。
- `SHELF_BOARD_ROW_MARGIN_PX`：上下层板图像行内侧留给商品的像素边距，默认 `8` px。
- `MAX_GRASP_DEPTH_M`：最大抓取深度，默认 `0.52` m；超过此深度的物品不抓取，避免机械臂伸太深导致货架层板碰撞相机。货架更浅时可调小。
- `ALLOW_NO_DEPTH_GRASP`：无可信深度时是否仍按最右侧框兜底抓取，默认关闭；只建议离线调试时临时设为 `1`。
- `TARGET_PRODUCT_QUERY`：目标商品/类别关键词，默认不指定；可匹配任意模型类别或检测身份字段，包括 `product_name`、`medicine_name`、`class_name`、`category`、`package_type`、`product_id`、`class_id`、`ocr_text`。
- `SELF_COLLISION_ENABLE`：Nova5 自碰撞护栏开关，默认开启；基于 URDF/SRDF 做 FK，并用保守胶囊近似检查连杆、灵巧手和相机的 MovL/MovJ 关节路径。
- `SELF_COLLISION_MARGIN_M`：自碰撞安全余量，默认 `0.015` m；误拦可调小，担心擦碰可调大。
- `SELF_COLLISION_PATH_SAMPLES`：自碰撞路径采样次数，默认 `8`；关节运动跨度大时可调大。
- `SELF_COLLISION_REQUIRE_IK`：MovL 无法求近邻逆解时是否中止，默认关闭；开启后更保守，但 IK 返回异常会导致 MovL 被拒绝。
- `SELF_COLLISION_VERBOSE`：自碰撞护栏通过时是否打印最近余量，默认关闭。
- `DOBOT_IK_VERBOSE`：是否打印自碰撞检查用的 `InverseKin` 请求/响应，默认关闭；排查 IK 失败时可开启。
- `DEPTH_GUARD_ENABLE`：深度护栏开关，默认开启；会排除当前画面中所有检测框，把其余深度点当成货架/障碍，检查对齐和探入路径。
- `DEPTH_GUARD_CAMERA_ENABLE`：相机碰撞盒开关，当前默认关闭，末端模型只考虑连接件和 DH116。以后安装相机并实测外壳位置后可重新开启。
- `TOOL_CAMERA_LONG_SIDE_Y_M` / `TOOL_CAMERA_BODY_DEPTH_X_M` / `TOOL_CAMERA_HEIGHT_Z_M`：相机外壳尺寸，默认分别为 `0.145/0.060/0.025` m；旧的 `TOOL_CAMERA_LENGTH_X_M` / `TOOL_CAMERA_WIDTH_Y_M` 仍作为兼容配置读取。
- `TOOL_CAMERA_CENTER_X_M` / `TOOL_CAMERA_CENTER_Y_M`：相机外壳中心在法兰坐标系的位置，默认 `-0.120/0.0` m。
- `TOOL_CAMERA_Z_MIN_M`：相机盒在新连接件托板上方的法兰 `Z` 起点，默认 `0.020` m；实测安装位置有偏移时可调整。
- `DEPTH_GUARD_CAMERA_CLEARANCE_M`：深度护栏对相机盒增加的安全余量，默认等于 `DEPTH_GUARD_CLEARANCE_M`，即 `0.002` m。
- `DEPTH_GUARD_SELF_MASK_CAMERA_PADDING_M`：从深度点云剔除相机自身时的外扩量，默认 `0.01` m。
- `HAND_PALM_LENGTH_M`：Nova 法兰经连接件到掌心前端的长度，默认约 `0.1555` m；之后到指尖的部分按手指建模。
- `HAND_FINGER_THICKNESS_M`：手指厚度，默认 `0.03` m。
- `DEPTH_GUARD_CLEARANCE_M`：深度护栏额外安全余量，默认 `0.002` m。
- `DEPTH_GUARD_HAND_RADIUS_M`：掌心段安全半径，默认约 `0.045` m；等于 86mm 掌心宽度的一半再加 2mm 余量。偏置拇指、连接件和相机托板由独立盒体覆盖。
- `DEPTH_GUARD_FINGER_RADIUS_M`：手指段安全半径，默认约 `0.017` m；等于 30mm 手指厚度的一半再加 2mm 余量。
- `DEPTH_GUARD_SELF_MASK_RADIUS_M` / `DEPTH_GUARD_SELF_MASK_FINGER_RADIUS_M`：从深度障碍点里排除相机看到的自己手，默认约 `0.07` m；只影响“删自己手”，不影响路径碰撞半径。
- `DEPTH_GUARD_APPROACH_LENGTH_M`：对齐路径使用的手部检查长度，默认 `0.08` m；对齐阶段只检查法兰/手掌根部附近，避免把准备抓的货物前方误判成碰撞。最后探入仍使用完整手长。
- `DEPTH_GUARD_STRIDE`：深度点采样步长，默认 `8`；调小更密但更慢，调大更快但可能漏细杆/边缘。
- `DEPTH_GUARD_PATH_SAMPLES`：路径采样次数，默认 `8`；运动路径长或拐得多时可调大。
- `DEPTH_GUARD_MIN_HIT_POINTS`：手部路径触发中止所需的独立深度点数，默认 `4`；少量层板边缘/深度跳变点不会误报，连续障碍点簇仍会拦截。需要更保守时可调回 `2`，现场仍有边缘误报时可调到 `5` 或 `6`。
- `DEPTH_GUARD_HAND_BOX_MIN_HIT_POINTS`：DH116 偏置拇指/手指和连接件外接盒触发中止所需的独立深度点数，默认 `12`；外接盒比中心胶囊保守，因此要求更完整的障碍点簇。调试图中这类命中显示为橙点。
- `DEPTH_GUARD_MAX_DEPTH_M`：纳入障碍点的最大深度，默认 `1.20` m；货架背景太远造成误拦截时可调小。
- `DEPTH_GUARD_TARGET_DEPTH_MARGIN_M`：相对当前目标深度允许纳入护栏的后方余量，默认 `0.06` m；更远的后层板/背板点会被忽略。
- `EDGE_MIDPOINT_GRASP_ENABLE`：是否使用所选检测框侧边竖线的中点作为抓取点，默认关闭；当前恢复使用原侧抓选点，即侧边中点向瓶底方向下移并限制安全高度。设为 `1` 可使用严格的边线中点。
- `DEPTH_GUARD_EXCLUDE_MARGIN_PX`：检测框外扩排除范围，默认 `32` px；如果货物边缘被误当货架可继续调大。
- `DEPTH_GUARD_SELF_MASK_ENABLE`：自手部深度点剔除开关，默认开启；用于过滤深度相机看到的当前灵巧手/手指，避免把自己的手当障碍。
- `DEPTH_GUARD_SELF_MASK_RADIUS_M`：自手部剔除半径，默认约 `0.0725` m；如果红点仍贴在手指上可略调大，如果误删近处层板可调小。
- `DEPTH_GUARD_SELF_MASK_LENGTH_M`：自手部剔除长度，默认 `0.25` m；通常等于法兰到指尖长度。
- `DEPTH_GUARD_SELF_MASK_BACKOFF_M`：自手部剔除从法兰向后覆盖的长度，默认 `0.03` m；用于覆盖手掌根部/安装件附近的深度点。
- `DEPTH_GUARD_DEPTH_ALIGN_ENABLE`：相机深度一致性过滤开关，默认开启；碰撞点除了要靠近手部胶囊，还要和路径最近点在相机深度方向上对齐，用于过滤透视造成的近邻误判。
- `DEPTH_GUARD_DEPTH_ALIGN_TOLERANCE_M`：相机深度一致性容差，默认 `0.035` m；如果层板确实有距离但仍被红点拦截可略调小/保持，若真实障碍漏检则调大或关闭深度对齐。
- `DEPTH_GUARD_AXIS_WINDOW_ENABLE`：探入路径轴向窗口开关，默认开启；只检查法兰附近到指尖前方一点的障碍，忽略指尖/目标后方的层板后缘或背景点。
- `DEPTH_GUARD_FINGERTIP_FRONT_MARGIN_M`：指尖前方仍纳入检查的长度，默认 `0.005` m；红点在目标后方/层板后方时可调小到 `0`，担心指尖前方障碍漏检时调大。
- `DEPTH_GUARD_FLANGE_BACK_MARGIN_M`：法兰后方仍纳入检查的长度，默认 `0.03` m；用于覆盖手掌根部附近。
- `DEPTH_GUARD_BYPASS_ENABLE`：对齐路径自动绕行开关，默认开启；直达被深度护栏挡住时会尝试上抬/侧绕中间点。
- `DEPTH_GUARD_BYPASS_LIFTS_MM`：绕行上抬候选，默认 `60,100,140` mm。
- `DEPTH_GUARD_BYPASS_SIDE_MM`：绕行侧向候选，默认 `60,-60` mm。

LHandPro 抓取保持参数：

- `LHAND_HOLD_HZ`：抓取姿态后台重发频率，默认 `20` Hz；用于维持位置控制。
- `LHAND_HOLD_TIGHTEN_INTERVAL_S`：保持期间检测低负载并补紧的周期，默认 `0.12` s。
- `LHAND_HOLD_TIGHTEN_STEP`：每次补紧的位置步长，默认 `100`；物体较软或抓后变松时可调大，出现抖动时调小。
- `LHAND_HOLD_MIN_DELTA`：相对空载电流低于该值才补紧，默认 `80`；与初次接触阈值保持一致，避免抓住后夹持力偏低。
- `LHAND_FLEX_STEP`：柔性闭合每轮位置增量，默认 `500`；减小会更细腻但更慢，增大可更快但不宜超过现场空载曲线采样跨度太多。
- `LHAND_FLEX_CONTINUOUS`：连续柔性闭合模式，默认开启；一次下发闭合上限并在运动中监测电流，接触后立即锁定该轴，避免逐步闭合的卡顿。设为 `0` 可恢复离散步进模式。
- `LHAND_FLEX_SETTLE_S`：柔性闭合每步等待，默认 `0.015` s；现场通信不稳定时可适当调大。
- `LHAND_FLEX_CONSECUTIVE`：接触确认连续次数，默认 `1`；现场电流噪声较大时可设为 `2`。
- `LHAND_FLEX_TIMEOUT_ACCEPT_DELTA`：抓取检测超时/到位后进行二次夹持确认的最小空载电流差，默认 `50`；达到最少接触轴数时直接进入保持，避免已经抓牢却按失败张开。
- `LHAND_FLEX_START_WAIT_S`：闭合前张开姿态稳定等待，默认 `0.03` s。
- `LHAND_PROFILE_SWEEP_AXES`：录制空载曲线时参与扫位的轴，默认 `2,3,4,5,6`；轴1保持 `LHAND_OPEN_POSITIONS` 的张开位置不动。
- `HAND_CLOSE_WAIT_S`：闭合成功后确认保持状态的等待时间，默认 `0.25` s；需要兼容较慢控制器时可调回 `1.0` s。

深度护栏拦截动作时会自动保存调试图到 `/tmp/depth_guard_*.jpg`：灰点为参与判断的深度障碍点，红点为命中碰撞半径的点，绿框为检测排除框/当前目标框，青线为手部检查线。

代码里的 `speed` 参数会随 `MovL`/`MovJ` 下发为 DOBOT `v=1..100` 百分比速度；超过 100 的值会自动夹到 100。放置段可选 `MovL cp` 平滑过渡；回层会先原地抬高，再高位回层。

启动后程序会：

1. 连接相机和 DOBOT。
2. 先回 HOME。
3. 加载 `shelf_layers.json`、`place_point.json`、`place_transition.json` 和 `stow_pose.json`。
4. 固定默认回到第 2 层观察位，忽略上次退出时的 `current`，避免从第 3/4 层启动造成碰撞。

## 代码测试

层板过滤的合成深度图测试位于 `tests/test_grasp_planner.py`。盒子端安装好 `numpy` 后运行：

```bash
cd /home/nvidia/grasp
python3 -m unittest dipan.tests.test_grasp_planner
```

测试覆盖上下层板识别、当前层边界生成和上下层候选过滤；不会连接相机、DOBOT 或驱动真实机械臂。

## 层板过滤预览接口

程序提供只读的层板过滤预览，不会移动机械臂、闭合灵巧手或执行抓取。机械臂停在目标层观察位时：

- 交互模式输入 `f`，保存一帧预览。
- 常驻 worker 从 stdin 写入 `__filter_preview__`（兼容别名 `__inspect_filter__`）。

默认输出：

```text
/tmp/shelf_filter_preview.jpg
/tmp/shelf_filter_preview.jpg.json
```

图片中蓝线是当前过滤行范围，绿色框是严格过滤后保留的候选，红色框是被过滤的候选；JSON 报告包含当前层编号、上下层板 `boards`、动态 `row/z` 边界、每个检测框的机器人 Z 和过滤原因。输出路径可通过以下变量修改：

```bash
SHELF_FILTER_PREVIEW_IMAGE=/tmp/filter.jpg \
SHELF_FILTER_PREVIEW_REPORT=/tmp/filter.json \
SHELF_GRASP_WORKER=1 DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

启动后向该进程 stdin 写入：

```text
__filter_preview__
```

终端会返回类似 `[过滤预览][完成] image=... report=... mode=dynamic boards=... accepted=.../...` 的结果。`mode=dynamic` 表示使用当前帧上下层板；`mode=fallback_roi` 表示本帧没有形成可靠上下层板，使用旧 ROI/Z 回退规则。

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

抓取成功后，如果放置点存在，程序会自动前往放置点并张手，然后停在放置点；如果放置点不可达或确认距离过大，会停止在当前位置，不继续后续订单。

记录了放置过渡点时，放置路径会变为：

```text
当前层/必要中转层 → 关节运动到放置过渡点 → 最终放置点 → 张手 → 停在放置点
```

如果开启 `PLACE_SMOOTH_TRANSITION=1`，最终移动到放置点时会使用 `PLACE_TRANSITION_CP` 作为 `MovL cp` 平滑过渡比例。

如果菜单里显示过渡点 `[无joints]`，建议拖动机械臂到该过渡点后重新输入 `ts` 记录一次。大范围换姿态时程序会优先使用记录关节角 `MovJ`，避免 `MovL` 直线插补导致腕部翻转和机械臂自碰撞。

## 收起姿态管理

进入 `u` 菜单后，把机械臂拖到安全收起姿态，输入 `s` 保存。收起姿态保存在 `stow_pose.json`。如果记录了放置过渡点，放置区收起时会优先经该点使用 `MovJ` 回收，避免直接横向 `MovL` 到第 2 层。

收起姿态用于退出程序时的主动收尾：

- 按 `q` 退出时会回到收起姿态，再断开 DOBOT 和相机。
- 放置流程内部默认不再自动收起，避免放置前流程自行回撤。

如果未记录收起姿态，按 `q` 退出时会跳过。

## 抓取模式

当前默认 `FORCE_SIDE_GRASP=0`，`plan_edge_grasp()` 会根据检测框宽度自动判断：

- 盒宽 `<= 8cm`：侧面抓，使用 `SIDE_GRASP_OFFSET_M`
- 盒宽 `> 8cm`：上面抓，使用 `TOP_GRASP_OFFSET_M`，抓取前 J6 转到 `TOP_J6_DEG`

如果现场需要临时让所有目标都按侧面抓取点规划：

```bash
FORCE_SIDE_GRASP=1 DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

侧面抓取点按检测框底边计算，允许区间为 `6.5–11cm`。规划点低于区间时会自动抬到 `6.8cm`，高于区间时会自动压到 `11cm`；如果检测框本身不足 `6.5cm`，会改成上面抓取。

机械臂路径仍按侧抓/上抓自动判定。灵巧手默认使用电流反馈柔性抓取：

```bash
HAND_GRASP_MODE=flex
```

建议先录制空载电流曲线：

```bash
cd /home/nvidia/grasp/dipan
python3 test_aero_current_grasp.py --active-joints 1,2,3,4,5,6 --max 80 --step 2 --fast --disable-trigger --release-after --log /home/nvidia/grasp/dipan/runtime/aero_empty.csv
```

如果要恢复旧的侧面瓶子固定手势：

```python
GRASP_BOTTLE = [100, 92, 75, 76, 75, 67, 69]
```

启动时默认：

```bash
HAND_GRASP_MODE=bottle DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
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
python3 test_aero_current_grasp.py --active-joints 1,2,3,4,5,6 --max 60 --step 2 --fast --disable-trigger --release-after --log /home/nvidia/grasp/dipan/runtime/aero_empty.csv
python3 test_aero_current_grasp.py --active-joints 1,2,3,4,5,6 --max 60 --step 1 --profile /home/nvidia/grasp/dipan/runtime/aero_empty.csv --profile-delta-ma 180 --profile-window-deg 6 --min-cmd 15 --consecutive 2 --release-after
python3 test_aero_current_grasp.py --active-joints 1,2,3,4,5,6 --max 80 --step 2 --fast --profile /home/nvidia/grasp/dipan/runtime/aero_empty.csv --profile-delta-ma 180 --profile-window-deg 6 --min-cmd 15 --consecutive 2 --per-finger --extra-close 8 --thumb-follow --release-after
```

这会把“正常弯曲/预紧造成的电流”从判断里扣掉，只在比空载曲线额外高出阈值时才停止。
`--per-finger` 会让 active-joints 里的 3-6 号手指分开判断；某根手指触发后会再闭合 `--extra-close` 度并锁住，直到这些手指都抓紧/到上限才退出。`--thumb-follow` 让 1/2 号拇指继续随步进闭合，但不作为“四指都抓紧”的退出条件。需要强制指定等待哪些手指时可加 `--required-fingers 3,4`。

## LHandPro RS485 柔性抓取

新后端默认使用 `500000 baud`、`8N1`、`NodeID=1`，SDK 路径为
`/home/nvidia/grasp/LHandProLib-API-Linux`。电流反馈单位是额定电流的千分比（‰），
不能复用 Aero 的 mA 空载曲线。Python 串口依赖为 `pyserial`。

先确认通信，不使能、不回零：

```bash
cd /home/nvidia/grasp/dipan
python3 test_lhandpro_flexible_grasp.py --port /dev/ttyUSB0 info
```

空手录制新曲线。命令会要求输入 `RECORD`，随后回零并让 6 个主动轴从 `0` 扫到
`9000`；任何轴空载电流达到 `750‰` 会中止且不覆盖旧曲线：

```bash
python3 test_lhandpro_flexible_grasp.py --port /dev/ttyUSB0 record
python3 test_lhandpro_flexible_grasp.py check-profile
```

曲线默认保存到 `runtime/lhandpro_empty.csv`。若 `9000` 接近机械限位，应先保守录到
`7000`，确认方向和行程后再逐步增加；柔性闭合上限会自动截断到已录曲线的最大位置：

```bash
python3 test_lhandpro_flexible_grasp.py --port /dev/ttyUSB0 record --max 7000 --step 250
```

录制成功后，将软质测试物放在掌心，独立测试一次柔性闭合。该脚本不会移动机械臂：

```bash
python3 test_lhandpro_flexible_grasp.py --port /dev/ttyUSB0 flex
```

1 号拇指侧摆默认保持张开位置不动；2～6 号轴参与独立判断并等待接触或到上限，
其中 2 号为拇指弯曲、3～6 号为四指弯曲。至少两个轴相对同位置空载包络增加
`80‰` 才判定抓取成功。曲线缺失时默认拒绝柔性抓取。常用参数为
`LHAND_FLEX_PROFILE_DELTA`、`LHAND_FLEX_STEP`、`LHAND_FLEX_EXTRA_CLOSE`、
`LHAND_FLEX_REQUIRED_AXES` 和 `LHAND_FLEX_MIN_CONTACTS`。

DH116 默认张开位置为 `9000,0,0,0,0,0`：1 号拇指侧摆到已录曲线覆盖的最下端，
柔性闭合期间保持不动。默认快速反馈参数为 `LHAND_FLEX_SETTLE_S=0.05`、
`LHAND_FLEX_SAMPLES=1`、`LHAND_FLEX_SAMPLE_DELAY_S=0`；末端短帧在限流和速度不变时
只重复下发位置帧。现场需要恢复更保守节奏时可显式设为 `0.08`、`2`、`0.01`。

独立测试通过并重新实测末端几何参数后，才启动完整货架流程：

```bash
HAND_BACKEND=lhandpro HAND_GRASP_MODE=flex \
LHAND_RS485_PORT=/dev/ttyUSB0 DOBOT_IP=192.168.5.1 \
python3 shelf_grasp.py
```

机械臂末端 485 若只提供 Modbus RTU 而不提供原始字节透传，不能直接承载 SDK 的
RS485 回调帧；此时先用 Jetson 的 USB-RS485 测试，不要把 `ModbusRTUCreate` 当作
SDK 串口。更换手和安装座后，还必须重新实测并设置 `TOOL_HAND_LENGTH_Z_M`、
`HAND_PALM_LENGTH_M`、`TOOL_HAND_BODY_DEPTH_X_M`、`TOOL_HAND_PALM_WIDTH_Y_M`、
`TOOL_HAND_OPEN_WIDTH_Y_M` 和 `HAND_WEIGHT_KG`，再允许运行机械臂抓取路径。

## Aero 接触状态识别 demo

`demo_aero_contact_states.py` 是独立的灵巧手实验程序，不会调用机械臂、订单服务或 `shelf_grasp.py`。它让选定关节逐步闭合，同时读取电流、实际位置和温度；默认用 `runtime/aero_empty.csv` 的同角度空载电流包络扣除绳腱/机构本身的负载。

状态优先级为：`空载`、`浅层接触`、`持续接触`、`饱和堆积`、`挤压溢出`。普通状态默认连续 2 步确认，溢出立即停止；温度超限只后台打印，不改变手部动作。结束时默认安全张开，可用 `--no-release-after` 禁止。默认不修改当前力矩上限，必须让空载曲线采集时的力矩设置和实验保持一致。

首次建议从较小范围开始，并让手指周围保持空旷：

```bash
cd /home/nvidia/grasp/dipan
python3 demo_aero_contact_states.py --max 35 --step 1 --release-after
```

确认空载运行正常后，用完整 0-90° 行程测试接触状态（90°也是默认上限）：

```bash
python3 demo_aero_contact_states.py --max 90 --step 1 --release-after
```

单指观察可以缩小驱动和感知范围：

```bash
python3 demo_aero_contact_states.py --drive-joints 3 --sense-joints 3 --max 45 --step 1
```

如需在识别到饱和堆积时停止，可使用 `--stop-on saturation`。默认在挤压溢出或硬过流时提前停止，温度只记录和提示。每次运行会在 `/tmp/aero_contact_states_*.csv` 写逐步数据，并生成同名 `.summary.json`，其中包含状态切换、最大电流/残差和停止原因。阈值可通过 `--contact-extra-ma`、`--saturation-depth-deg`、`--overflow-extra-ma` 等参数调节。

目标选择支持两层规则：先按目标关键词过滤，再由 `pick_target_box()` 做几何选择。主菜单里输入 `?` 可查看当前模型的全部 `class_id:class_name`；输入 `c` 可以设置任意目标商品/类别/id；也可以直接输入目标名开始抓取，例如 `雪碧`、`褪黑素`、`sprite`、`ths`、`class_id:63`。输入 `all` 或 `clear` 会清空目标，恢复任意候选抓取。

也可以启动时指定：

```bash
TARGET_PRODUCT_QUERY=雪碧 DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
TARGET_PRODUCT_QUERY=褪黑素 DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

目标过滤命中后，`pick_target_box()` 当前规则：

- 优先从本次最佳深度帧识别横向跨度大、机器人 Z 高度稳定的层板行，并选出当前层的下板和上板。
- 用上下板之间的图像行范围和机器人 Z 范围过滤上下层物品；层板本身不参与抓取目标生成。
- 如果当前帧没有形成可靠的上下板边界，回退到固定画面 ROI（默认 `y=85~430`）和当前层世界 Z 容差（默认 `±16cm`）。
- 在 ROI 内按深度分层，选最前层。
- 在最前层从右往左选目标，始终从右侧边缘抓取。

识别日志示例：

```text
层板识别: 下板 Z=...mm(row=...), 上板 Z=...mm(row=...)
层板行过滤: 跳过上下层/边缘候选 ...
层板高度过滤: 跳过非当前层候选 ...
```

如果日志显示“未形成当前层上下边界”或完全没有“层板识别”，说明本帧已回退旧过滤。可以先保存 `/tmp/shelf_detection.jpg` 检查蓝线：动态识别时标注为“动态层板范围”，回退时标注为“回退ROI”。

现场调参建议：

```bash
# 层板较窄或深度有断点时，提高采样密度并放宽点数门槛
SHELF_BOARD_SAMPLE_STRIDE=2 SHELF_BOARD_MIN_POINTS=16 DOBOT_IP=192.168.5.1 python3 shelf_grasp.py

# 深度噪声较大时放宽同一层板的 Z 离散范围
SHELF_BOARD_MAX_Z_SPREAD_M=0.06 DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

若要临时关闭动态层板过滤，可使用：

```bash
SHELF_BOARD_FILTER_ENABLE=0 DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

只有在现场确认层板识别稳定后才建议恢复真实抓取。若仍使用旧规则，可以按现场画面调 ROI：

```bash
TARGET_ROI_Y_MIN=170 TARGET_ROI_Y_MAX=320 DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

如果当前层目标也被层高过滤误杀，可以临时调大或关闭：

```bash
LAYER_Z_TOLERANCE_M=0.22 DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
LAYER_Z_FILTER_ENABLE=0 DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

检测截图 `/tmp/shelf_detection.jpg` 会画蓝色动态层板/回退 ROI 边界线。

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

第 1/2 层：抓完后直接去放置点，放完停在放置点，下一件再按目标层移动。

第 3 层：抓完后直接安全上撤到第 2 层，之后去放置点并停留；下一件再按目标层移动。

第 4 层：抓完后先安全上撤到第 3 层，再到第 2 层，之后去放置点并停留；下一件再按目标层移动。

从放置点或未知位姿前往下一层使用 `go_to_layer_from_non_shelf_pose()`；从收起位进层会先严格反向经过放置过渡点，再抬高、横移和垂直下落；抓取后的上撤使用 `go_to_layer_via_clearance()`。任何安全路径无法读取当前位姿时都会停止，不再回退为直接 `MovL`。

自碰撞护栏使用 Nova5 连杆胶囊和完整末端包络共同检查：DH116、转接法兰和连接件长托板都会参与关节路径采样，暂不包含相机实体。默认尺寸可通过 `TOOL_ADAPTER_*`、`TOOL_HAND_MOUNT_OFFSET_Z_M`、`TOOL_HAND_LENGTH_Z_M`、`TOOL_HAND_BODY_DEPTH_X_M`、`TOOL_HAND_PALM_WIDTH_Y_M` 和 `TOOL_HAND_OPEN_WIDTH_Y_M` 按实测调整。碰撞模型、当前关节角、逆解或检查过程不可用时默认拒绝运动；只有明确设置 `SELF_COLLISION_ENABLE=0` 或 `SELF_COLLISION_FAIL_CLOSED=0` 才会关闭相应门禁。

## 底盘放置流程

主菜单输入 `b` 可以切换底盘放置流程，提示里会显示 `b 底盘:开/关`。

默认关闭，方便只调机械臂抓取和放置点。也可以启动前用环境变量默认开启：

```bash
USE_CHASSIS_PLACE=1 MARKER_PLACE=fangzhi DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

开启后抓取完成流程会变为：

```text
抓取后退
→ 调用 move_robot.py place
→ 等底盘到达放置固定点
→ 机械臂前往 place_point.json 的放置点
→ 张手放置并停在放置点
```

`MARKER_PLACE` 是底盘地图里的固定点名称，当前默认值是 `fangzhi`。

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
python3 move_robot.py huojia1   # 去货架 1，默认等待到达
python3 move_robot.py charge    # 去充电点，默认等待到达
python3 move_robot.py place     # 去放置固定点，默认等待到达
python3 move_robot.py place --no-wait
python3 move_robot.py cancel    # 取消移动
```

底盘地址：

```text
192.168.10.10:31001
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

普通移动直到本次目标点的 `move_status=succeeded` 才返回成功；如果刚下发后读到上一轮残留的 `succeeded target=旧点位`，会忽略并继续等待。如果本次任务 `failed`、`canceled` 或超时，会返回非 0，抓取主程序不会继续机械臂放置。

`charge` 比较特殊：到底充电桩点位后，底盘会进入 `running_status=dock_to_charging_pile` 自动对接，API 可能很久才把 `move_status` 置为 `succeeded`。脚本默认看到本次目标进入 `dock_to_charging_pile` 就返回成功；如果要一直等到底盘最终 `succeeded`，可这样跑：

```bash
CHARGE_RETURN_ON_DOCKING=0 python3 move_robot.py charge
```
