"""
摄像头手部跟踪 → OrcaHand 控制
使用 MediaPipe Tasks API 检测人手姿态，映射到机械手电机位置，实现实时跟随。
电机控制在独立线程中运行，不阻塞UI。

依赖安装：
    pip install mediapipe opencv-python

用法：
    python mine/hand_tracking.py
    python mine/hand_tracking.py --config orca_core/models/v2/orcahand_touch_left/config.yaml
    python mine/hand_tracking.py --camera 1
    python mine/hand_tracking.py --smoothing 0.6
    python mine/hand_tracking.py --no-grip
"""

import argparse
import logging
import queue
import sys
import threading
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

try:
    import cv2
    import mediapipe as mp
    from mediapipe.tasks.python.vision import HandLandmarker, HandLandmarkerOptions
    from mediapipe.tasks.python import vision
except ImportError:
    print("请先安装依赖: pip install mediapipe opencv-python")
    sys.exit(1)

from orca_core import OrcaHand

MODEL_PATH = str(Path(__file__).resolve().parent / "hand_landmarker.task")

HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16)   ,
    (0, 17), (17, 18), (18, 19), (19, 20),
    (5, 9), (9, 13), (13, 17)
]

# 默认电机位置（中性/握拳），运行时从 OrcaHand config 动态覆盖
# 仅在不使用关节空间映射时作为 fallback
NEUTRAL_MOTOR_POS: dict[int, float] = {}
FIST_MOTOR_POS: dict[int, float] = {}

# config.yaml 中用负号标记的反转电机ID
INVERTED_MOTOR_IDS: set[int] = set()


def compute_finger_flex(landmarks, finger_indices) -> float:
    def pt(idx):
        return np.array([landmarks[idx].x, landmarks[idx].y, landmarks[idx].z])

    if finger_indices[0] in (1, 2):
        mcp = pt(finger_indices[0])
        ip = pt(finger_indices[1])
        tip = pt(finger_indices[2])
        v1 = mcp - ip
        v2 = tip - ip
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
        cos_angle = np.clip(cos_angle, -1, 1)
        return np.clip(np.arccos(cos_angle) / np.pi, 0, 1)

    mcp = pt(finger_indices[0])
    pip = pt(finger_indices[1])
    dip = pt(finger_indices[2])
    tip = pt(finger_indices[3])

    v1 = mcp - pip; v2 = dip - pip
    cos1 = np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6), -1, 1)
    v3 = pip - dip; v4 = tip - dip
    cos2 = np.clip(np.dot(v3, v4) / (np.linalg.norm(v3) * np.linalg.norm(v4) + 1e-6), -1, 1)

    return np.clip((np.arccos(cos1) + np.arccos(cos2)) / 2 / np.pi, 0, 1)


def compute_abduction(landmarks, mcp_idx_1, mcp_idx_2, wrist_idx=0) -> float:
    wrist = np.array([landmarks[wrist_idx].x, landmarks[wrist_idx].y])
    mcp1 = np.array([landmarks[mcp_idx_1].x, landmarks[mcp_idx_1].y])
    mcp2 = np.array([landmarks[mcp_idx_2].x, landmarks[mcp_idx_2].y])
    spread = np.linalg.norm(mcp1 - mcp2)
    hand_size = np.linalg.norm(mcp1 - wrist) + np.linalg.norm(mcp2 - wrist) + 1e-6
    return np.clip(spread / hand_size * 3.0 - 0.5, 0, 1)


def compute_wrist_angle(landmarks) -> float:
    wrist = np.array([landmarks[0].x, landmarks[0].y])
    middle_mcp = np.array([landmarks[9].x, landmarks[9].y])
    return np.clip((middle_mcp[0] - wrist[0]) * 5, -1, 1)


class HandTracker:
    def __init__(self, smoothing: float = 0.5, hand=None):
        options = HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=MODEL_PATH),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.7,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.detector = HandLandmarker.create_from_options(options)
        self.smoothing = smoothing
        self._timestamp_ms = 0

        # 核心改进：使用 OrcaHand 的关节空间 API
        # flex → joint_angle → _joint_to_motor_pos() → motor_pos
        # 这样反转逻辑由 OrcaHand 内部处理，不需要手动处理 INVERTED_MOTOR_IDS
        self._hand = hand
        self._use_joint_api = hand is not None

        if self._use_joint_api:
            self._load_joint_config(hand)
        else:
            # 无 hand 对象时使用硬编码默认值（旧行为，不保证方向正确）
            self._init_hardcoded_defaults()

        self.prev_motor_pos = dict(NEUTRAL_MOTOR_POS)

    def _load_joint_config(self, hand):
        """从 OrcaHand 读取关节配置，构造关节角度映射表。

        核心思路：flex ∈ [0,1] 映射到关节角度 ∈ [ROM_min, ROM_max]，
        然后通过 hand._joint_to_motor_pos() 转换为电机位置。
        这样反转逻辑由 OrcaHand 内部的 joint_inversion_dict 自动处理。
        """
        global NEUTRAL_MOTOR_POS, FIST_MOTOR_POS, INVERTED_MOTOR_IDS

        config = hand.config
        roms = config.joint_roms_dict        # {joint_name: [min_deg, max_deg]}
        neutral = config.neutral_position    # {joint_name: float}
        inv = config.joint_inversion_dict    # {joint_name: bool}

        # 记录反转信息（仅用于日志）
        INVERTED_MOTOR_IDS = set()
        for joint_name, motor_id in config.joint_to_motor_map.items():
            if inv.get(joint_name, False):
                INVERTED_MOTOR_IDS.add(motor_id)

        # 关节ROM映射表：flex=0 → ROM_min(张开), flex=1 → ROM_max(握拳)
        # 对于外展关节：abd=0 → 手指并拢(ROM_min), abd=1 → 手指张开(ROM_max)
        # 注意: config 中 ROM 单位是度，_joint_to_motor_pos 需要的也是度
        self._joint_roms = roms
        self._joint_to_motor = config.joint_to_motor_map

        # 构造 NEUTRAL 和 FIST 的电机位置（用于 UI 面板显示和 smoothing fallback）
        # NEUTRAL = flex=0 时的电机位置, FIST = flex=1 时的电机位置
        NEUTRAL_MOTOR_POS = {}
        FIST_MOTOR_POS = {}
        for joint_name, motor_id in config.joint_to_motor_map.items():
            rom = roms.get(joint_name, [0, 100])
            try:
                neutral_motor = hand._joint_to_motor_pos({joint_name: rom[0]})
                fist_motor = hand._joint_to_motor_pos({joint_name: rom[1]})
                n_val = neutral_motor[config.motor_id_to_idx_dict[motor_id]]
                f_val = fist_motor[config.motor_id_to_idx_dict[motor_id]]
                if n_val is not None:
                    NEUTRAL_MOTOR_POS[motor_id] = float(n_val)
                if f_val is not None:
                    FIST_MOTOR_POS[motor_id] = float(f_val)
            except Exception:
                # 标定数据不完整时用中性位置的当前电机值
                NEUTRAL_MOTOR_POS[motor_id] = 0.0
                FIST_MOTOR_POS[motor_id] = 0.0

    def _init_hardcoded_defaults(self):
        """无 OrcaHand 时的硬编码默认值（旧行为）。"""
        global NEUTRAL_MOTOR_POS, FIST_MOTOR_POS, INVERTED_MOTOR_IDS
        NEUTRAL_MOTOR_POS = {
            1: 3.7322, 2: 0.2930, 3: 6.6682, 4: 5.0238,
            5: 4.6848, 6: 0.1749, 7: 2.3669, 8: 1.6720,
            9: 2.8885, 10: 0.2378, 11: 6.4366, 12: 2.4329,
            13: 2.7596, 14: 1.8224, 15: 4.1157, 16: 3.9024,
            17: 0.4602,
        }
        FIST_MOTOR_POS = {
            1: 4.2430, 2: 2.0693, 3: 1.0140, 4: 5.5085,
            5: 4.4041, 6: 1.6137, 7: 2.7811, 8: 1.7810,
            9: 4.2277, 10: 2.1169, 11: 1.0048, 12: 2.7013,
            13: 4.4746, 14: 2.5541, 15: 4.4025, 16: 4.2829,
            17: 0.4832,
        }
        INVERTED_MOTOR_IDS = set()
        self._joint_roms = None
        self._joint_to_motor = None

    def process_frame(self, frame_rgb):
        self._timestamp_ms += 33  # ~30fps 递增
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
        return self.detector.detect_for_video(mp_image, self._timestamp_ms)

    def get_motor_commands(self, detection_result) -> dict[int, float] | None:
        if not detection_result.hand_landmarks:
            return None
        lm = detection_result.hand_landmarks[0]

        thumb_flex = compute_finger_flex(lm, [1, 2, 3, 4])
        index_flex = compute_finger_flex(lm, [5, 6, 7, 8])
        middle_flex = compute_finger_flex(lm, [9, 10, 11, 12])
        ring_flex = compute_finger_flex(lm, [13, 14, 15, 16])
        pinky_flex = compute_finger_flex(lm, [17, 18, 19, 20])

        index_abd = compute_abduction(lm, 5, 9)
        middle_abd = compute_abduction(lm, 9, 13)
        ring_abd = compute_abduction(lm, 13, 17)
        pinky_abd = compute_abduction(lm, 17, 5)
        thumb_abd = compute_abduction(lm, 2, 5)

        wrist_angle = compute_wrist_angle(lm)

        # 存储原始 flex/abd 值，用于日志
        self._last_raw = {
            "thumb": {"flex": thumb_flex, "abd": thumb_abd},
            "index": {"flex": index_flex, "abd": index_abd},
            "middle": {"flex": middle_flex, "abd": middle_abd},
            "ring": {"flex": ring_flex, "abd": ring_abd},
            "pinky": {"flex": pinky_flex, "abd": pinky_abd},
            "wrist_angle": wrist_angle,
        }

        def flex_to_motor(mid, f):
            """线性插值: f=0→NEUTRAL, f=1→FIST。
            _load_config 已保证: 正常电机 FIST>NEUTRAL, 反转电机 FIST<NEUTRAL。
            """
            n = NEUTRAL_MOTOR_POS.get(mid, 0)
            fi = FIST_MOTOR_POS.get(mid, 0)
            return n + f * (fi - n)

        # 使用 config 中的关节→电机映射（如果有），否则用硬编码映射
        if hasattr(self, '_joint_to_motor') and self._joint_to_motor:
            j2m = self._joint_to_motor
            cmd = {}
            # 食指
            if "index_abd" in j2m:
                cmd[j2m["index_abd"]] = flex_to_motor(j2m["index_abd"], index_abd)
            if "index_mcp" in j2m:
                cmd[j2m["index_mcp"]] = flex_to_motor(j2m["index_mcp"], index_flex)
            if "index_pip" in j2m:
                cmd[j2m["index_pip"]] = flex_to_motor(j2m["index_pip"], index_flex)
            # 中指
            if "middle_abd" in j2m:
                cmd[j2m["middle_abd"]] = flex_to_motor(j2m["middle_abd"], middle_abd)
            if "middle_mcp" in j2m:
                cmd[j2m["middle_mcp"]] = flex_to_motor(j2m["middle_mcp"], middle_flex)
            if "middle_pip" in j2m:
                cmd[j2m["middle_pip"]] = flex_to_motor(j2m["middle_pip"], middle_flex)
            # 无名指
            if "ring_abd" in j2m:
                cmd[j2m["ring_abd"]] = flex_to_motor(j2m["ring_abd"], ring_abd)
            if "ring_mcp" in j2m:
                cmd[j2m["ring_mcp"]] = flex_to_motor(j2m["ring_mcp"], ring_flex)
            if "ring_pip" in j2m:
                cmd[j2m["ring_pip"]] = flex_to_motor(j2m["ring_pip"], ring_flex)
            # 小指
            if "pinky_abd" in j2m:
                cmd[j2m["pinky_abd"]] = flex_to_motor(j2m["pinky_abd"], pinky_abd)
            if "pinky_mcp" in j2m:
                cmd[j2m["pinky_mcp"]] = flex_to_motor(j2m["pinky_mcp"], pinky_flex)
            if "pinky_pip" in j2m:
                cmd[j2m["pinky_pip"]] = flex_to_motor(j2m["pinky_pip"], pinky_flex)
            # 拇指
            if "thumb_cmc" in j2m:
                cmd[j2m["thumb_cmc"]] = flex_to_motor(j2m["thumb_cmc"], thumb_flex)
            if "thumb_abd" in j2m:
                cmd[j2m["thumb_abd"]] = flex_to_motor(j2m["thumb_abd"], thumb_abd)
            if "thumb_mcp" in j2m:
                cmd[j2m["thumb_mcp"]] = flex_to_motor(j2m["thumb_mcp"], thumb_flex)
            if "thumb_dip" in j2m:
                cmd[j2m["thumb_dip"]] = flex_to_motor(j2m["thumb_dip"], thumb_flex)
            # 手腕
            if "wrist" in j2m:
                wn = NEUTRAL_MOTOR_POS.get(j2m["wrist"], 0)
                wf = FIST_MOTOR_POS.get(j2m["wrist"], 0)
                cmd[j2m["wrist"]] = wn + wrist_angle * (wf - wn) * 0.3
        else:
            # 无 config 时的硬编码映射（旧行为）
            cmd = {}
            cmd[1] = flex_to_motor(1, index_abd)
            cmd[2] = flex_to_motor(2, index_flex)
            cmd[3] = flex_to_motor(3, index_flex)
            cmd[5] = flex_to_motor(5, middle_abd)
            cmd[9] = flex_to_motor(9, middle_flex)
            cmd[4] = flex_to_motor(4, middle_flex)
            cmd[11] = flex_to_motor(11, ring_abd)
            cmd[10] = flex_to_motor(10, ring_flex)
            cmd[12] = flex_to_motor(12, ring_flex)
            cmd[7] = flex_to_motor(7, pinky_abd)
            cmd[6] = flex_to_motor(6, pinky_flex)
            cmd[8] = flex_to_motor(8, pinky_flex)
            cmd[14] = flex_to_motor(14, thumb_flex)
            cmd[13] = flex_to_motor(13, thumb_abd)
            cmd[15] = flex_to_motor(15, thumb_flex)
            cmd[16] = flex_to_motor(16, thumb_flex)
            wn = NEUTRAL_MOTOR_POS[17]; wf = FIST_MOTOR_POS[17]
            cmd[17] = wn + wrist_angle * (wf - wn) * 0.3

        for mid in cmd:
            if mid in self.prev_motor_pos:
                cmd[mid] = self.smoothing * self.prev_motor_pos[mid] + (1 - self.smoothing) * cmd[mid]
        self.prev_motor_pos = dict(cmd)
        return cmd

    def get_avg_flex(self, detection_result) -> float | None:
        if not detection_result.hand_landmarks:
            return None
        lm = detection_result.hand_landmarks[0]
        return sum([
            compute_finger_flex(lm, [5, 6, 7, 8]),
            compute_finger_flex(lm, [9, 10, 11, 12]),
            compute_finger_flex(lm, [13, 14, 15, 16]),
            compute_finger_flex(lm, [17, 18, 19, 20]),
        ]) / 4

    def draw_landmarks(self, frame, detection_result):
        if not detection_result.hand_landmarks:
            return frame
        h, w, _ = frame.shape
        lm = detection_result.hand_landmarks[0]
        for i, j in HAND_CONNECTIONS:
            p1 = (int(lm[i].x * w), int(lm[i].y * h))
            p2 = (int(lm[j].x * w), int(lm[j].y * h))
            cv2.line(frame, p1, p2, (0, 255, 0), 2)
        for idx in range(21):
            cx, cy = int(lm[idx].x * w), int(lm[idx].y * h)
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
        return frame

    def release(self):
        self.detector.close()


def motor_worker(hand, cmd_queue, all_motors, args, stop_event):
    """独立线程：从队列中取电机指令并发送，限制发送频率。"""
    logger = logging.getLogger("motor")
    is_gripping = False
    last_send_time = 0
    last_log_time = 0
    min_interval = 0.1  # 最小发送间隔100ms（~10Hz）
    log_interval = 2.0  # 日志输出间隔2秒

    while not stop_event.is_set():
        try:
            item = cmd_queue.get(timeout=0.05)
        except queue.Empty:
            continue

        if item is None:
            break

        cmd_type, data = item

        if cmd_type == "motor_pos":
            if is_gripping:
                continue

            # 提取日志元数据（不修改 data，用单独变量保存）
            avg_flex = data.get("avg_flex", None)
            raw = data.get("_raw", None)

            now = time.time()
            if now - last_send_time < min_interval:
                continue
            last_send_time = now

            # 构造纯电机位置字典，过滤掉非整数键（_raw, avg_flex等）
            motor_data = {k: v for k, v in data.items() if isinstance(k, int)}
            try:
                hand._set_motor_pos(motor_data)
            except Exception as e:
                logger.error(f"电机写入错误: {e}")

            # 定期输出详细日志
            if now - last_log_time >= log_interval:
                motor_to_joint = hand.config.motor_to_joint_dict
                # 原始 flex/abd 值
                if raw:
                    logger.info(
                        f"[RAW] thumb(flex={raw['thumb']['flex']:.2f},abd={raw['thumb']['abd']:.2f}) "
                        f"index(flex={raw['index']['flex']:.2f},abd={raw['index']['abd']:.2f}) "
                        f"middle(flex={raw['middle']['flex']:.2f},abd={raw['middle']['abd']:.2f}) "
                        f"ring(flex={raw['ring']['flex']:.2f},abd={raw['ring']['abd']:.2f}) "
                        f"pinky(flex={raw['pinky']['flex']:.2f},abd={raw['pinky']['abd']:.2f}) "
                        f"wrist_angle={raw['wrist_angle']:.2f}"
                    )
                # 电机原始位置和归一化位置
                pos_detail = []
                for mid in all_motors:
                    jname = motor_to_joint.get(mid, "?")
                    val = motor_data.get(mid, 0)
                    n = NEUTRAL_MOTOR_POS.get(mid, 0)
                    f = FIST_MOTOR_POS.get(mid, 0)
                    norm = (val - n) / (f - n) if abs(f - n) > 0.01 else 0
                    pos_detail.append(f"{jname}={val:.3f}(n={norm:.2f})")
                logger.info(f"[MOTOR] flex={avg_flex:.2f} | " + " ".join(pos_detail))
                last_log_time = now

        elif cmd_type == "reset":
            is_gripping = False
            logger.info("[RESET] 重置到中性位置")
            try:
                hand.set_control_mode("current_based_position")
                hand.set_max_current(args.max_current)
                hand._set_motor_pos(NEUTRAL_MOTOR_POS)
            except Exception as e:
                logger.error(f"重置错误: {e}")

        elif cmd_type == "grip_on":
            if not is_gripping:
                is_gripping = True
                logger.info(f"[GRIP ON] 电流={args.grip_current}mA")
                wrist_motor = hand.config.joint_to_motor_map.get("wrist")
                grip_motors = [mid for mid in all_motors if mid != wrist_motor]
                try:
                    hand.set_control_mode("current")
                    hand._motor_client.write_desired_current(
                        grip_motors, args.grip_current * np.ones(len(grip_motors))
                    )
                except Exception as e:
                    logger.error(f"握紧错误: {e}")

        elif cmd_type == "grip_off":
            if is_gripping:
                is_gripping = False
                logger.info("[GRIP OFF] 切回位置控制模式")
                try:
                    hand.set_control_mode("current_based_position")
                    hand.set_max_current(args.max_current)
                except Exception as e:
                    logger.error(f"释放错误: {e}")

        elif cmd_type == "grip_toggle":
            is_gripping = not is_gripping
            try:
                if is_gripping:
                    wrist_motor = hand.config.joint_to_motor_map.get("wrist")
                    grip_motors = [mid for mid in all_motors if mid != wrist_motor]
                    hand.set_control_mode("current")
                    hand._motor_client.write_desired_current(
                        grip_motors, args.grip_current * np.ones(len(grip_motors))
                    )
                else:
                    hand.set_control_mode("current_based_position")
                    hand.set_max_current(args.max_current)
            except Exception as e:
                print(f"握紧切换错误: {e}")


def main():
    parser = argparse.ArgumentParser(description="摄像头手部跟踪 → OrcaHand 控制")
    parser.add_argument("--config", type=str, default=None, help="config.yaml 路径")
    parser.add_argument("--camera", type=int, default=0, help="摄像头设备号")
    parser.add_argument("--smoothing", type=float, default=0.5, help="平滑系数(0~1, 越大越平滑)")
    parser.add_argument("--max-current", type=int, default=300, help="最大电流(mA)")
    parser.add_argument("--no-grip", action="store_true", help="不启用握紧电流保持")
    parser.add_argument("--grip-current", type=int, default=120, help="握紧保持电流(mA)")
    parser.add_argument("--grip-threshold", type=float, default=0.7, help="握紧触发的平均屈曲阈值(0~1)")
    parser.add_argument("--log-file", type=str, default=None, help="日志输出文件路径，不指定则输出到控制台")
    args = parser.parse_args()

    # ── 初始化日志 ──
    log_handlers = [logging.StreamHandler(sys.stdout)]
    if args.log_file:
        log_handlers.append(logging.FileHandler(args.log_file, encoding="utf-8"))
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
        handlers=log_handlers,
    )
    logger = logging.getLogger("main")

    # ── 初始化摄像头 ──
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        logger.error(f"无法打开摄像头 {args.camera}")
        return 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    for _ in range(5):
        cap.read()
    logger.info(f"摄像头已打开 (设备{args.camera})")

    # 先连接机械手，获取 config 信息
    hand = OrcaHand(config_path=args.config)
    success, msg = hand.connect()
    logger.info(f"机械手连接: success={success}, message={msg}")
    if not success:
        cap.release()
        return 1

    # 用机械手 config 初始化 tracker（含反转信息和电机映射）
    tracker = HandTracker(smoothing=args.smoothing, hand=hand)
    logger.info(f"HandTracker 初始化完成 (smoothing={args.smoothing})")
    logger.info(f"反转电机: {INVERTED_MOTOR_IDS}")
    logger.info(f"NEUTRAL_MOTOR_POS: {NEUTRAL_MOTOR_POS}")
    logger.info(f"FIST_MOTOR_POS: {FIST_MOTOR_POS}")

    hand.enable_torque()
    hand.set_control_mode("current_based_position")
    hand.set_max_current(args.max_current)
    all_motors = hand.config.motor_ids
    logger.info(f"电机已启用, 模式=current_based_position, 电流={args.max_current}mA")
    logger.info(f"电机ID列表: {all_motors}")

    cmd_queue = queue.Queue(maxsize=1)
    stop_event = threading.Event()
    motor_thread = threading.Thread(
        target=motor_worker,
        args=(hand, cmd_queue, all_motors, args, stop_event),
        daemon=True,
    )
    motor_thread.start()
    logger.info("电机控制线程已启动")

    print("\n=== 手部跟踪已启动 ===")
    print("按 'q' 退出 | 按 'r' 重置 | 按 'g' 握紧/释放")
    print("========================\n")

    fps_time = time.time()
    frame_count = 0
    is_gripping = False
    last_motor_cmds = None
    last_detect_log_time = 0
    no_hand_count = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            frame = cv2.flip(frame, 1)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            detection_result = tracker.process_frame(frame_rgb)
            motor_cmds = tracker.get_motor_commands(detection_result)

            if motor_cmds is not None:
                last_motor_cmds = dict(motor_cmds)
                frame = tracker.draw_landmarks(frame, detection_result)
                avg_flex = tracker.get_avg_flex(detection_result)

                if not is_gripping:
                    try:
                        cmd_queue.get_nowait()
                    except queue.Empty:
                        pass
                    motor_cmds["avg_flex"] = avg_flex
                    # 附带原始 flex/abd 值用于日志
                    if hasattr(tracker, '_last_raw'):
                        motor_cmds["_raw"] = tracker._last_raw
                    cmd_queue.put(("motor_pos", motor_cmds))

                cv2.putText(frame, f"Flex: {avg_flex:.2f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                mode_text = "GRIP" if is_gripping else "TRACK"
                mode_color = (0, 0, 255) if is_gripping else (0, 255, 0)
                cv2.putText(frame, f"Mode: {mode_text}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2)
            else:
                no_hand_count += 1
                if no_hand_count == 30:
                    logger.warning("未检测到手部关键点")
                    no_hand_count = 0
                cv2.putText(frame, "No hand detected", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # FPS
            frame_count += 1
            elapsed = time.time() - fps_time
            if elapsed > 1.0:
                fps = frame_count / elapsed
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                frame_count = 0
                fps_time = time.time()

            # ── 右侧面板：显示电机指令状态 ──
            panel_w = 220
            panel = np.zeros((frame.shape[0], panel_w, 3), dtype=np.uint8)
            motor_to_joint = hand.config.motor_to_joint_dict

            if last_motor_cmds:
                y = 10
                cv2.putText(panel, "Motor Commands", (5, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
                y += 20
                for mid in hand.config.motor_ids:
                    jname = motor_to_joint.get(mid, "?")
                    val = last_motor_cmds.get(mid, 0)
                    # 计算归一化位置 (0=张手, 1=握拳)
                    n = NEUTRAL_MOTOR_POS.get(mid, 0)
                    f = FIST_MOTOR_POS.get(mid, 0)
                    norm = (val - n) / (f - n) if abs(f - n) > 0.01 else 0
                    norm = np.clip(norm, 0, 1)
                    # 画进度条
                    bar_w = int(norm * 100)
                    color = (0, int(255 * (1 - norm)), int(255 * norm))
                    cv2.putText(panel, f"{mid:>2}:{jname[:8]}", (5, y + 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.33, (180, 180, 180), 1)
                    cv2.rectangle(panel, (105, y + 2), (105 + 100, y + 14), (40, 40, 40), -1)
                    cv2.rectangle(panel, (105, y + 2), (105 + bar_w, y + 14), color, -1)
                    y += 18

            frame = np.hstack([frame, panel])

            cv2.imshow("Hand Tracking", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                logger.info("用户按q退出")
                break
            elif key == ord('r'):
                cmd_queue.put(("reset", None))
            elif key == ord('g'):
                is_gripping = not is_gripping
                logger.info(f"用户按g切换模式: {'GRIP' if is_gripping else 'TRACK'}")
                if is_gripping:
                    cmd_queue.put(("grip_on", None))
                else:
                    cmd_queue.put(("grip_off", None))

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        stop_event.set()
        cmd_queue.put(None)
        motor_thread.join(timeout=2.0)
        cap.release()
        cv2.destroyAllWindows()
        tracker.release()
        success, msg = hand.disconnect()
        print(f"断开: success={success}, message={msg}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
