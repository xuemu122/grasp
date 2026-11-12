"""奥利奥单次全流程（云端识别）：收起 -> 底盘去 huojia1 -> 第2层 -> 抓取 -> fangzhi 放置 -> 回充电桩。

复制 shelf_grasp 主流程的识别后 xyz 微调菜单、抓取/放置参数与顺序，但识别只用云端 /detect。
手会参与抓取；底盘会移动；执行前请确认现场安全。

用法:
    HAND_BACKEND=aero TOOL_HAND_MODEL=aero python3 oreo_cloud_flow.py
    ... --target "奥利奥" --layer 2
    ... --no-tune        # 不弹微调菜单，直接抓
"""
import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DIPAN = os.environ.get("GRASP_DIPAN_DIR", "/home/nvidia/grasp/dipan")
RECOG = os.environ.get("GRASP_RECOGNITION_DIR", "/home/nvidia/grasp/识别")
for path in (HERE, RECOG, DIPAN):
    if path not in sys.path:
        sys.path.insert(0, path)

from orbbec_detect import RealSenseCamera, DetectionResult, MedicineLocatorSystem  # noqa: E402
from dobot_arm import DobotArm  # noqa: E402
from hand_eye import HandEyeTransformer  # noqa: E402
from grasp_planner import pick_target_box, plan_edge_grasp  # noqa: E402
from servo_joint import ServoJoint  # noqa: E402
from target_selection import filter_detections_by_target_query  # noqa: E402
import shelf_grasp as sg  # noqa: E402
from call_service import detect_rgb  # noqa: E402
from http_client import make_local_detection  # noqa: E402

LIVE_CHASSIS_SETTINGS = "/var/lib/grasp-qianduan/chassis_settings.json"


def parse_args():
    parser = argparse.ArgumentParser(description="奥利奥单次抓取全流程（云端识别）")
    parser.add_argument("--target", default="奥利奥")
    parser.add_argument("--layer", type=int, default=2)
    parser.add_argument("--frames", type=int, default=8)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--outdir", default="/tmp/oreo_cloud_flow")
    parser.add_argument("--cloud-timeout", type=float, default=120.0)
    parser.add_argument("--chassis-host", default=os.getenv("CHASSIS_HOST", "192.168.10.10"))
    parser.add_argument("--chassis-port", type=int, default=int(os.getenv("CHASSIS_PORT", "31001")))
    parser.add_argument("--shelf-marker", default=os.getenv("MARKER_SHELF", "huojia1"))
    parser.add_argument("--place-marker", default="")
    parser.add_argument("--charge-marker", default="")
    parser.add_argument("--no-tune", action="store_true", help="跳过 xyz 微调菜单，直接抓取")
    parser.add_argument("--no-local", action="store_true", help="不跑本地模型参考，只做云端识别")
    return parser.parse_args()


def chassis_env(args):
    env = os.environ.copy()
    env["CHASSIS_HOST"] = str(args.chassis_host)
    env["CHASSIS_PORT"] = str(args.chassis_port)
    return env


def move_chassis(args, target, label):
    print("\n[底盘] 前往 %s (%s) ..." % (target, label), flush=True)
    result = subprocess.run(
        [sys.executable, sg.MOVE_ROBOT_SCRIPT, target],
        cwd=os.path.dirname(sg.MOVE_ROBOT_SCRIPT),
        env=chassis_env(args),
        text=True,
        check=False,
    )
    ok = result.returncode == 0
    print("[底盘] %s %s" % (label, "到位" if ok else "失败返回 %d" % result.returncode), flush=True)
    return ok


def load_live_settings():
    try:
        with open(LIVE_CHASSIS_SETTINGS, encoding="utf-8") as handle:
            return json.load(handle)
    except Exception:
        return {}


def offset_for(target, mode):
    name = str(getattr(target, "class_name", "") or "").strip()
    group = sg.grasp_group_for(target)
    if mode.endswith("_side"):
        if name in sg.MINISO_GRASP_NAMES:
            return sg.MINISO_GRASP_OFFSET_M
        if name in sg.BAICHUN_GRASP_NAMES:
            return sg.BAICHUN_GRASP_OFFSET_M
        if name in sg.BUCKET_GRASP_NAMES:
            return sg.BUCKET_GRASP_OFFSET_M
        if name in sg.OREO_GRASP_NAMES:
            return sg.OREO_GRASP_OFFSET_M
        if group == "chip":
            return sg.CHIP_GRASP_OFFSET_M
        if sg.is_non_drink(target):
            return sg.NECK_GRASP_OFFSET_M if group == "neck" else sg.NON_DRINK_SIDE_GRASP_OFFSET_M
        return sg.SIDE_GRASP_OFFSET_M
    return sg.TOP_GRASP_OFFSET_M


def build_local_detector(camera):
    """本地双模型（best1 主 + best 辅助），只做参考，不参与实际抓取。"""
    model_path = os.path.join(RECOG, "best1.engine")
    if not os.path.exists(model_path):
        model_path = os.path.join(RECOG, "best.engine")
    aux = os.path.join(RECOG, "best.engine")
    locator = MedicineLocatorSystem(
        model_path=model_path,
        aux_model_path=aux if os.path.abspath(aux) != os.path.abspath(model_path) else None,
        aux_class_min_id=19,
        mapping_path=os.path.join(RECOG, "shelf_mapping.yaml"),
        conf_threshold=float(os.getenv("LOCAL_REF_CONF", "0.55")),
        use_ocr=False,
        enhance_mode=os.getenv("ENHANCE_MODE", "force"),
        clahe_clip=float(os.getenv("ENHANCE_CLAHE_CLIP", "2.0")),
        clahe_tile=int(os.getenv("ENHANCE_CLAHE_TILE", "8")),
        enhance_gamma=float(os.getenv("ENHANCE_GAMMA", "1.0")),
        enhance_awb=os.getenv("ENHANCE_AWB", "0").strip().lower() in ("1", "true", "yes", "on"),
    )
    locator.init_camera = lambda: None
    locator.camera = camera
    return locator


def capture_and_plan(args, camera, arm, hand, local_locator=None):
    try:
        camera.tune_exposure_before_capture()
    except AttributeError:
        pass
    best = None
    for _ in range(max(1, args.frames)):
        color, depth = camera.read()
        if color is None:
            continue
        score = float(color.mean())
        if best is None or score > best[0]:
            best = (score, color, depth)
    if best is None:
        raise RuntimeError("没有读到相机帧")
    _, color, depth = best
    frame_id = "oreo_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    cv2.imwrite(os.path.join(args.outdir, frame_id + "_frame.jpg"), color)
    if depth is not None:
        cv2.imwrite(os.path.join(args.outdir, frame_id + "_depth.png"), depth.astype("uint16"))

    ok, encoded = cv2.imencode(".jpg", color, [cv2.IMWRITE_JPEG_QUALITY, 92])
    if not ok:
        raise RuntimeError("JPEG 编码失败")
    started = time.time()
    payload = detect_rgb(encoded.tobytes(), frame_id, frame_id + ".jpg")
    elapsed_ms = (time.time() - started) * 1000.0
    print("[云端] %d 个候选, 往返 %.0fms inference_ms=%s model=%s" % (
        len(payload.get("detections", [])), elapsed_ms,
        payload.get("inference_ms"), payload.get("model_version")), flush=True)

    detections = []
    for row in payload.get("detections", []):
        det = make_local_detection(
            row, DetectionResult,
            depth_at_box=lambda cx, cy, box: camera.get_depth_at(depth, cx, cy, box),
            pixel_to_camera_3d=camera.pixel_to_camera_3d,
        )
        if det is not None:
            detections.append(det)
    detections = filter_detections_by_target_query(detections, args.target)
    print("[目标] 关键词 %r -> %d 个候选" % (args.target, len(detections)), flush=True)
    if not detections:
        raise RuntimeError("云端没有匹配的抓取目标")

    flange_pose = arm.get_pose()
    transformer = HandEyeTransformer(fx=camera.fx, fy=camera.fy, cx=camera.cx, cy=camera.cy)
    transformer.set_flange(flange_pose)
    target, edge = pick_target_box(detections, transformer=transformer, flange_pose=flange_pose)
    if target is None:
        raise RuntimeError("pick_target_box 失败")
    edge = "left"
    wx, wy, wz, grip_mode, p_cam, grip_pixel = plan_edge_grasp(
        target, edge, transformer, flange_pose, fx=camera.fx, camera=camera, depth_img=depth
    )
    print("[目标] %s id=%s conf=%.2f bbox=%s depth=%.3fm edge=%s mode=%s" % (
        target.class_name, target.class_id, target.confidence, list(target.bbox),
        target.depth_m, edge, grip_mode), flush=True)

    local_ms = 0.0
    local_dets = []
    local_target = None
    if local_locator is not None:
        local_started = time.time()
        try:
            local_raw = local_locator.process_frame(color, depth) or []
        except Exception as exc:
            print("[本地参考][警告] 本地模型识别失败: %s" % exc, flush=True)
            local_raw = []
        local_ms = (time.time() - local_started) * 1000.0
        local_dets = filter_detections_by_target_query(local_raw, args.target)
        print("[本地参考] %d 个候选, %.0fms" % (len(local_dets), local_ms), flush=True)
        for det in local_dets[:8]:
            print("  - %s id=%s conf=%.2f bbox=%s depth=%s" % (
                det.class_name, det.class_id, det.confidence, list(det.bbox),
                ("%.3fm" % det.depth_m) if det.depth_m else "-"), flush=True)
        if local_dets:
            try:
                local_target, _ = pick_target_box(
                    local_dets, transformer=transformer, flange_pose=flange_pose)
            except Exception as exc:
                print("[本地参考][警告] 选择本地目标失败: %s" % exc, flush=True)
                local_target = None
            if local_target is not None:
                print("[本地参考] 目标 %s id=%s conf=%.2f bbox=%s" % (
                    local_target.class_name, local_target.class_id, local_target.confidence,
                    list(local_target.bbox)), flush=True)

    if grip_mode.endswith("_side"):
        if sg.is_non_drink(target):
            if sg.grasp_group_for(target) == "chip":
                band = (sg.CHIP_SIDE_GRASP_MIN_HEIGHT_M, sg.CHIP_SIDE_GRASP_MAX_HEIGHT_M,
                        sg.CHIP_SIDE_GRASP_LOW_TARGET_HEIGHT_M)
            else:
                band = (sg.NON_DRINK_SIDE_GRASP_MIN_HEIGHT_M, sg.NON_DRINK_SIDE_GRASP_MAX_HEIGHT_M,
                        sg.NON_DRINK_SIDE_GRASP_LOW_TARGET_HEIGHT_M)
            corrected, _, _ = sg.clamp_side_grasp_pixel_to_safe_clearance(
                target, grip_pixel, camera.fy, band[0], band[1], band[2])
        else:
            corrected, _, _ = sg.clamp_side_grasp_pixel_to_safe_clearance(target, grip_pixel, camera.fy)
        if corrected is not None:
            grip_pixel = corrected
            p_cam = transformer.pixel_to_camera(grip_pixel[0], grip_pixel[1], target.depth_m)
            p_robot = transformer.camera_to_robot(p_cam, flange_pose)
            wx, wy, wz = float(p_robot[0]), float(p_robot[1]), float(p_robot[2])

    is_side_grasp = "side" in grip_mode
    grasp_offset = offset_for(target, grip_mode)
    R_f = transformer._rpy_to_rot(flange_pose[3], flange_pose[4], flange_pose[5])
    fz_dir = R_f[:, 2]
    allow_low_z = args.layer in getattr(sg, "LOW_Z_LAYER_IDXS", set())
    grasp_advance_m = sg.LOW_Z_GRASP_ADVANCE_M if allow_low_z else sg.GRASP_ADVANCE_M
    fingertip_w = np.array([wx, wy, wz]) + np.array(grasp_offset)
    flange_pt = fingertip_w - sg.HAND_LENGTH * fz_dir
    grasp_pt = flange_pt + grasp_advance_m * fz_dir
    approach_pt = sg.axis_aligned_approach_m(
        np.array(flange_pose[:3]) / 1000.0, grasp_pt, fz_dir, sg.GRASP_APPROACH_BACKOFF_M
    ) + np.array([0.0, 0.0, sg.APPROACH_LIFT_M])
    # 保存带框和抓取点的图片/JSON
    annotated = color.copy()
    for det in detections:
        x1, y1, x2, y2 = det.bbox
        is_target = det is target
        col = (255, 128, 0) if is_target else (0, 200, 0)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), col, 3 if is_target else 1)
        cv2.putText(annotated, "%s %.2f" % (det.class_id, det.confidence),
                    (x1, max(16, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
    gx0, gy0 = int(grip_pixel[0]), int(grip_pixel[1])
    cv2.drawMarker(annotated, (gx0, gy0), (0, 0, 255), cv2.MARKER_CROSS, 26, 2)
    cv2.circle(annotated, (gx0, gy0), 7, (0, 0, 255), 2)
    cv2.putText(annotated, "GRASP (%d,%d)" % (gx0, gy0), (max(0, gx0 - 70), max(18, gy0 - 14)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    cv2.imwrite(os.path.join(args.outdir, frame_id + "_plan.jpg"), annotated)
    if local_locator is not None:
        local_viz = color.copy()
        for det in local_dets:
            x1, y1, x2, y2 = det.bbox
            is_target = det is local_target
            col = (255, 128, 0) if is_target else (0, 200, 0)
            cv2.rectangle(local_viz, (x1, y1), (x2, y2), col, 3 if is_target else 1)
            cv2.putText(local_viz, "L %s %.2f" % (det.class_id, det.confidence),
                        (x1, max(16, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        cv2.putText(local_viz, "LOCAL REF", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 0), 2)
        cv2.imwrite(os.path.join(args.outdir, frame_id + "_local.jpg"), local_viz)
    with open(os.path.join(args.outdir, frame_id + "_plan.json"), "w", encoding="utf-8") as handle:
        json.dump({
            "frame_id": frame_id,
            "target": {"class_id": target.class_id, "class_name": target.class_name,
                       "confidence": target.confidence, "bbox": list(target.bbox),
                       "depth_m": target.depth_m, "edge": edge, "mode": grip_mode},
            "grasp_pixel": [gx0, gy0],
            "detections": [{"class_id": d.class_id, "class_name": d.class_name,
                            "confidence": d.confidence, "bbox": list(d.bbox), "depth_m": d.depth_m}
                           for d in detections],
            "local_reference": {
                "elapsed_ms": round(local_ms, 1),
                "target": ({"class_id": local_target.class_id, "class_name": local_target.class_name,
                            "confidence": local_target.confidence, "bbox": list(local_target.bbox),
                            "depth_m": local_target.depth_m} if local_target is not None else None),
                "detections": [{"class_id": d.class_id, "class_name": d.class_name,
                                "confidence": d.confidence, "bbox": list(d.bbox), "depth_m": d.depth_m}
                               for d in local_dets],
            },
        }, handle, ensure_ascii=False, indent=2)

    return {
        "frame_id": frame_id,
        "color": color,
        "depth": depth,
        "target": target,
        "edge": edge,
        "grip_mode": grip_mode,
        "grip_pixel": tuple(int(v) for v in grip_pixel),
        "is_side_grasp": is_side_grasp,
        "grasp_offset": grasp_offset,
        "transformer": transformer,
        "flange_pose": flange_pose,
        "fz_dir": fz_dir,
        "allow_low_z": allow_low_z,
        "grasp_advance_m": grasp_advance_m,
        "ax": float(approach_pt[0] * 1000),
        "ay": float(approach_pt[1] * 1000),
        "az": float(approach_pt[2] * 1000),
        "gx": float(grasp_pt[0] * 1000),
        "gy": float(grasp_pt[1] * 1000),
        "gz": float(grasp_pt[2] * 1000),
        "wx": wx, "wy": wy, "wz": wz,
    }


def tune_offsets(plan):
    """复制主流程的 xyz 微调菜单。返回 (offset_x, offset_y, offset_z) 或 None(重拍) / 抛异常取消。"""
    ox = oy = oz = 0.0
    step = 5.0
    target = plan["target"]
    ax, ay, az = plan["ax"], plan["ay"], plan["az"]
    gx, gy, gz = plan["gx"], plan["gy"], plan["gz"]
    while True:
        tuned = np.array(plan["grasp_offset"]) + np.array([ox, oy, oz]) / 1000.0
        print("\n── 目标位姿 (可微调) ──")
        print("  模式: %s (%s)" % ("侧面抓" if plan["is_side_grasp"] else "上面抓", plan["grip_mode"]))
        print("  当前参数: offset=(%.0f,%.0f,%.0f)mm" % (
            plan["grasp_offset"][0] * 1000, plan["grasp_offset"][1] * 1000, plan["grasp_offset"][2] * 1000))
        print("  固化建议: %s_GRASP_OFFSET_M=(%+.3f, %+.3f, %+.3f)" % (
            "SIDE" if plan["is_side_grasp"] else "TOP", tuned[0], tuned[1], tuned[2]))
        print("  对齐→抓取直线: %.0fmm, 过抓前进: %.0fmm, 入掌后左移: %.0fmm" % (
            np.linalg.norm(np.array([gx - ax, gy - ay, gz - az])),
            plan["grasp_advance_m"] * 1000, sg.PRE_CLOSE_LEFT_SHIFT_M * 1000))
        print("  指尖世界: (%.0f, %.0f, %.0f)mm" % (plan["wx"] * 1000, plan["wy"] * 1000, plan["wz"] * 1000))
        print("  法兰(对齐): X=%+.0f(%+.0f)  Y=%+.0f(%+.0f)  Z=%+.0f(%+.0f) mm" % (
            ax + ox, ox, ay + oy, oy, az + oz, oz))
        print("  法兰(抓取): X=%+.0f  Y=%+.0f  Z=%+.0f mm" % (gx + ox, gy + oy, gz + oz))
        print("\n  操作: Enter=执行抓取 | p=预览对齐位 | wasd=X/Y(±%.0fmm) | e/r=Z | =/-=步长 | f=手动偏移 | F5=重拍 | Q=取消" % step)
        choice = input("  > ").strip()
        if choice == "":
            return ox, oy, oz
        if choice in ("Q", "q"):
            raise RuntimeError("用户取消")
        if choice.lower() == "f5":
            return None
        if choice == "p":
            print("  预览对齐位: (%.0f, %.0f, %.0f)mm" % (ax + ox, ay + oy, az + oz))
            arm = plan.get("arm")
            if arm is not None:
                arm.movel(ax + ox, ay + oy, az + oz, speed=100, allow_low_z=plan["allow_low_z"])
                time.sleep(sg.GRASP_SETTLE_S)
            continue
        if choice in ("w", "s", "a", "d"):
            ox += {"a": -step, "d": +step}.get(choice, 0)
            oy += {"w": -step, "s": +step}.get(choice, 0)
        elif choice == "e":
            oz += step
        elif choice == "r":
            oz -= step
        elif choice in ("-", "=", "+"):
            step = max(1, step / 1.5) if choice == "-" else min(50, step * 1.5)
            print("  步长 → %.0fmm" % step)
        elif choice == "f":
            val = input("  输入偏移 (x,y,z mm, 如 10,-5,0): ").strip()
            try:
                parts = [float(v.strip()) for v in val.split(",")]
                if len(parts) == 3:
                    ox, oy, oz = parts
                else:
                    print("  格式错误, 需要 x,y,z")
            except ValueError:
                print("  格式错误")
        else:
            print("  未知按键: %r" % choice)


def main():
    args = parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    os.environ["CHASSIS_HOST"] = str(args.chassis_host)
    os.environ["CHASSIS_PORT"] = str(args.chassis_port)
    live = load_live_settings()
    place_marker = args.place_marker or live.get("place_marker") or "fangzhi"
    charge_marker = args.charge_marker or live.get("charge_marker") or "charge"

    print("[配置] 货架点位=%s 放置点=%s 充电桩=%s" % (args.shelf_marker, place_marker, charge_marker), flush=True)

    arm = None
    hand = None
    servo = None
    camera = None
    placed = False
    try:
        arm = DobotArm()
        print("[机械臂] 连接/上电 ...", flush=True)
        arm.connect()

        sg.load_shelf_layers()
        sg.load_place_point()
        sg.load_place_transition()
        sg.load_place_transition_low()
        sg.load_stow_pose()

        print("[机械臂] 前往收起位 ...", flush=True)
        if not sg.go_to_stow_pose(arm):
            raise RuntimeError("前往收起位失败")

        if not move_chassis(args, args.shelf_marker, "货架"):
            raise RuntimeError("底盘前往货架失败")

        print("[机械臂] 从收起位前往第%s层 ..." % args.layer, flush=True)
        if not sg.go_to_layer_from_stow(arm, args.layer):
            raise RuntimeError("前往第%s层失败" % args.layer)

        servo = ServoJoint()
        if not servo.connect():
            raise RuntimeError("手腕舵机连接失败")
        sg._servo = servo
        sg.servo_grasp()

        camera = RealSenseCamera(width=args.width, height=args.height, fps=args.fps)
        local_locator = None
        if not args.no_local:
            try:
                print("[本地参考] 加载本地双模型 ...", flush=True)
                local_locator = build_local_detector(camera)
            except Exception as exc:
                print("[本地参考][警告] 本地模型加载失败, 仅用云端: %s" % exc, flush=True)
        hand = sg._create_hand(arm)
        sg._connect_and_open_hand(hand)
        hand.open()

        while True:
            plan = capture_and_plan(args, camera, arm, hand, local_locator)
            plan["arm"] = arm
            print("[抓取点位] 抓取像素=%s 指尖=(%.0f,%.0f,%.0f)mm 对齐=(%.0f,%.0f,%.0f)mm 探入=(%.0f,%.0f,%.0f)mm" % (
                plan["grip_pixel"], plan["wx"] * 1000, plan["wy"] * 1000, plan["wz"] * 1000,
                plan["ax"], plan["ay"], plan["az"], plan["gx"], plan["gy"], plan["gz"]), flush=True)

            if args.no_tune:
                ox = oy = oz = 0.0
                break
            offsets = tune_offsets(plan)
            if offsets is None:
                print("[微调] F5 重新拍照识别 ...", flush=True)
                continue
            ox, oy, oz = offsets
            break

        ax, ay, az = plan["ax"] + ox, plan["ay"] + oy, plan["az"] + oz
        gx, gy, gz = plan["gx"] + ox, plan["gy"] + oy, plan["gz"] + oz
        target = plan["target"]
        target_name = str(getattr(target, "class_name", "") or "").strip()
        target_group = sg.grasp_group_for(target)
        allow_low_z = plan["allow_low_z"]
        approach_speed = sg.LOW_Z_APPROACH_SPEED if allow_low_z else 150
        grasp_speed = sg.LOW_Z_GRASP_SPEED if allow_low_z else 50

        print("\n[执行] 对齐位 (%.0f, %.0f, %.0f)mm ..." % (ax, ay, az), flush=True)
        if not arm.movel(ax, ay, az, speed=approach_speed, allow_low_z=allow_low_z):
            raise RuntimeError("对齐位运动失败")
        time.sleep(sg.GRASP_SETTLE_S)
        print("[执行] 探入位 (%.0f, %.0f, %.0f)mm ..." % (gx, gy, gz), flush=True)
        if not arm.movel(gx, gy, gz, speed=grasp_speed, allow_low_z=allow_low_z):
            raise RuntimeError("探入运动失败")
        time.sleep(sg.GRASP_SETTLE_S)

        if sg.PRE_CLOSE_LEFT_SHIFT_M > 0:
            R_f = plan["transformer"]._rpy_to_rot(*plan["flange_pose"][3:6])
            camera_x_dir = R_f @ plan["transformer"].T_flange_to_cam[:3, 0]
            camera_left_dir = -camera_x_dir / max(float(np.linalg.norm(camera_x_dir)), 1e-9)
            pre_close = np.array([gx, gy, gz], dtype=np.float32) + camera_left_dir * (sg.PRE_CLOSE_LEFT_SHIFT_M * 1000.0)
            if not arm.movel(float(pre_close[0]), float(pre_close[1]), float(pre_close[2]),
                             speed=grasp_speed, allow_low_z=allow_low_z):
                raise RuntimeError("抓前左移失败")
            time.sleep(sg.GRASP_SETTLE_S)

        if sg.HAND_GRASP_MODE in ("flex", "flexible", "current"):
            flex_env = {
                "chip": sg.CHIP_AERO_FLEX_ENV,
                "puff": sg.PUFF_AERO_FLEX_ENV,
                "neck": sg.NECK_AERO_FLEX_ENV,
            }.get(target_group)
            saved_env = {}
            if flex_env:
                saved_env = {key: os.environ.get(key) for key in flex_env}
                os.environ.update(flex_env)
            try:
                grasp_ok = hand.flexible_close()
            finally:
                if flex_env:
                    for key, old_value in saved_env.items():
                        if old_value is None:
                            os.environ.pop(key, None)
                        else:
                            os.environ[key] = old_value
        else:
            grasp_mode = "bottle" if sg.HAND_GRASP_MODE == "bottle" and plan["is_side_grasp"] else None
            grasp_ok = hand.close(top=not plan["is_side_grasp"], mode=grasp_mode)

        if grasp_ok:
            time.sleep(sg.HAND_CLOSE_WAIT_S)
            if not hand.is_holding():
                grasp_ok = False
        if not grasp_ok:
            raise RuntimeError("灵巧手闭合/保持失败")

        lift_mm = float(os.getenv("GRASP_LIFT_MM", "85"))
        if target_name in sg.BUCKET_GRASP_NAMES:
            lift_mm += sg.BUCKET_GRASP_LIFT_EXTRA_MM
        elif not sg.is_non_drink(target):
            lift_mm += sg.DRINK_GRASP_LIFT_EXTRA_MM
        lift_base = arm.get_pose()
        print("[抓后抬升] +%.1fmm" % lift_mm, flush=True)
        if not arm.movel(lift_base[0], lift_base[1], lift_base[2] + lift_mm,
                         lift_base[3], lift_base[4], lift_base[5],
                         speed=int(os.getenv("GRASP_LIFT_SPEED", "60")), allow_low_z=allow_low_z):
            raise RuntimeError("抓后抬升失败")
        time.sleep(sg.GRASP_SETTLE_S)

        print("[后退] 回对齐位 ...", flush=True)
        if not arm.movel(ax, ay, az, speed=100, allow_low_z=allow_low_z):
            raise RuntimeError("后退失败")
        time.sleep(sg.GRASP_SETTLE_S)

        sg.USE_CHASSIS_PLACE = True
        sg.DEFAULT_CHASSIS_PLACE_MARKER = place_marker
        os.environ["MARKER_PLACE"] = place_marker
        print("[放置] 前往 %s ..." % place_marker, flush=True)
        if not sg.finish_pick_cycle(arm, hand, args.layer):
            raise RuntimeError("放置流程失败")
        placed = True

        move_chassis(args, charge_marker, "充电桩")
        print("[完成] 全流程结束", flush=True)
        print("[等待] 程序保持运行，手腕保持通电/保持角；按 Ctrl-C 手动退出（退出时先断机械臂，再断舵机）", flush=True)
        while True:
            time.sleep(1.0)
    finally:
        if camera is not None:
            camera.release()
        if not placed and arm is not None:
            try:
                sg.go_to_stow_pose(arm)
            except Exception as exc:
                print("[机械臂][警告] 安全收起失败: %s" % exc, flush=True)
        if servo is not None:
            try:
                print("[舵机] 回到保持角 %s° ..." % sg.SERVO_HOLD_ANGLE, flush=True)
                servo.set_angle(float(sg.SERVO_HOLD_ANGLE))
                servo.wait_until_reached(float(sg.SERVO_HOLD_ANGLE), timeout=sg.SERVO_WAIT_TIMEOUT)
            except Exception as exc:
                print("[舵机][警告] 回保持角失败: %s" % exc, flush=True)
        if arm is not None:
            try:
                arm.close()
            except Exception:
                pass
        if servo is not None:
            try:
                print("[舵机] 机械臂已断开, 断开舵机 ...", flush=True)
                servo.close()
            except Exception:
                pass
        if hand is not None:
            try:
                hand.disconnect()
            except Exception:
                pass


if __name__ == "__main__":
    main()
