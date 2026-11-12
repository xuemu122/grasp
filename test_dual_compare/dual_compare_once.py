"""一次对比测试：收起位 -> 第2层 -> 手腕抓取角 -> 拍照 -> 云端+本地同时识别 -> 各自抓取点。

只做这一轮，手不动作，不动底盘。云端返回（或失败）后手腕回保持角，再回收起位。

用法:
    python3 dual_compare_once.py
    python3 dual_compare_once.py --layer 2 --frames 8 --query 乐事薯片（番茄味）
"""
import argparse
import json
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
DIPAN = os.environ.get("GRASP_DIPAN_DIR", "/home/nvidia/grasp/dipan")
RECOG = os.environ.get("GRASP_RECOGNITION_DIR", "/home/nvidia/grasp/识别")
for path in (HERE, RECOG, DIPAN):
    if path not in sys.path:
        sys.path.insert(0, path)


def _preset_camera_env():
    """固定本脚本的相机参数：手动曝光默认 4000us，可用 --exposure-us 覆盖。"""
    exposure = "4000"
    argv = sys.argv
    for index, item in enumerate(argv):
        if item == "--exposure-us" and index + 1 < len(argv):
            exposure = argv[index + 1]
        elif item.startswith("--exposure-us="):
            exposure = item.split("=", 1)[1]
    os.environ["CAMERA_AUTO_EXPOSURE"] = "0"
    os.environ["CAMERA_AUTO_WB"] = "0"
    os.environ.setdefault("CAMERA_GAIN", "50")
    os.environ["CAMERA_EXPOSURE_US"] = str(exposure)


_preset_camera_env()

from orbbec_detect import (  # noqa: E402
    MedicineLocatorSystem,
    RealSenseCamera,
    DetectionResult,
    _fast_light_stats,
    _enhance_low_light,
    CAMERA_AUTO_EXPOSURE,
    CAMERA_EXPOSURE_US,
    CAMERA_AUTO_WB,
    CAMERA_GAIN,
)
from dobot_arm import DobotArm  # noqa: E402
from hand_eye import HandEyeTransformer  # noqa: E402
from grasp_planner import (  # noqa: E402
    pick_target_box,
    plan_edge_grasp,
    clamp_side_grasp_pixel_to_safe_clearance,
)
from servo_joint import ServoJoint  # noqa: E402
import shelf_grasp as sg  # noqa: E402
from call_service import detect_rgb  # noqa: E402
from http_client import make_local_detection  # noqa: E402
from target_selection import filter_detections_by_target_query  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(description="云端/本地识别 + 抓取点对比（单次）")
    parser.add_argument("--layer", type=int, default=2)
    parser.add_argument("--frames", type=int, default=8)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--exposure-us", type=float, default=4000.0, help="手动曝光(微秒)，默认 4000")
    parser.add_argument("--query", default="", help="目标关键词；不填选最外侧")
    parser.add_argument("--outdir", default="/tmp/dual_compare")
    parser.add_argument("--cloud-timeout", type=float, default=120.0)
    parser.add_argument("--skip-move", action="store_true", help="只拍照识别，不移动机械臂/手腕")
    parser.add_argument("--flange", default="", help="skip-move 时的手动法兰位姿 x y z rx ry rz")
    return parser.parse_args()


def load_json(path):
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def move_stow(arm):
    sg.load_shelf_layers()
    sg.load_place_point()
    sg.load_place_transition()
    sg.load_stow_pose()
    print("[机械臂] 前往收起位 ...", flush=True)
    ok = sg.go_to_stow_pose(arm)
    print("[机械臂] 收起位 %s" % ("到位" if ok else "失败"), flush=True)
    return ok


def move_layer(arm, layer):
    print("[机械臂] 从收起位前往第%s层观察位 ..." % layer, flush=True)
    ok = sg.go_to_layer_from_stow(arm, layer)
    print("[机械臂] 第%s层 %s" % (layer, "到位" if ok else "失败"), flush=True)
    return ok


ENHANCE_MODE = os.getenv("ENHANCE_MODE", "force").strip().lower()
ENHANCE_CLIP = float(os.getenv("ENHANCE_CLAHE_CLIP", "2.0"))
ENHANCE_TILE = int(os.getenv("ENHANCE_CLAHE_TILE", "8"))
ENHANCE_GAMMA = float(os.getenv("ENHANCE_GAMMA", "1.0"))
ENHANCE_AWB = os.getenv("ENHANCE_AWB", "0").strip().lower() in ("1", "true", "yes", "on")
ENHANCE_LIGHT_THRESHOLD = float(os.getenv("ENHANCE_LIGHT_THRESHOLD", "90"))


def enhance_for_upload(color):
    """和本地 MedicineLocatorSystem 相同的反光/弱光增强，用于上传云端的那张图。"""
    mean_v, std_v = _fast_light_stats(color)
    need_dark = mean_v < ENHANCE_LIGHT_THRESHOLD
    need_low_contrast = std_v < 12.0 and mean_v < 130.0
    if ENHANCE_MODE == "off":
        do_enhance, reason = False, "off"
    elif ENHANCE_MODE == "force":
        do_enhance, reason = True, "force"
    elif need_dark:
        do_enhance, reason = True, "dark"
    elif need_low_contrast:
        do_enhance, reason = True, "low_contrast"
    else:
        do_enhance, reason = False, "off"
    info = {"mode": ENHANCE_MODE, "applied": do_enhance, "reason": reason,
            "clip": ENHANCE_CLIP, "tile": ENHANCE_TILE, "gamma": ENHANCE_GAMMA, "awb": ENHANCE_AWB,
            "before": [round(mean_v, 1), round(std_v, 1)]}
    if not do_enhance:
        return color, info
    out = _enhance_low_light(
        color, mean_v=mean_v, std_v=std_v,
        clahe_clip=ENHANCE_CLIP, clahe_tile=ENHANCE_TILE,
        gamma=ENHANCE_GAMMA, awb=ENHANCE_AWB,
    )
    after_mean, after_std = _fast_light_stats(out)
    info["after"] = [round(after_mean, 1), round(after_std, 1)]
    return out, info


def build_local_detector(camera):
    model_path = os.path.join(RECOG, "best1.engine")
    if not os.path.exists(model_path):
        model_path = os.path.join(RECOG, "best.engine")
    aux = os.path.join(RECOG, "best.engine")
    locator = MedicineLocatorSystem(
        model_path=model_path,
        aux_model_path=aux if os.path.abspath(aux) != os.path.abspath(model_path) else None,
        aux_class_min_id=19,
        mapping_path=os.path.join(RECOG, "shelf_mapping.yaml"),
        conf_threshold=0.55,
        use_ocr=False,
        enhance_mode=ENHANCE_MODE,
        clahe_clip=ENHANCE_CLIP,
        clahe_tile=ENHANCE_TILE,
        enhance_gamma=ENHANCE_GAMMA,
        enhance_awb=ENHANCE_AWB,
    )
    locator.init_camera = lambda: None
    locator.camera = camera
    return locator


# 对比测试固定使用最开始（抓取点取中心时）的偏移，不受主流程后续调整影响。
ORIGINAL_SIDE_GRASP_OFFSET_M = (-0.130, -0.1505, 0.008)
ORIGINAL_NON_DRINK_GRASP_OFFSET_M = (-0.130, -0.142, 0.008)
ORIGINAL_CHIP_GRASP_OFFSET_M = (-0.130, -0.137, 0.008)
ORIGINAL_MINISO_GRASP_OFFSET_M = (-0.132, -0.137, 0.008)
ORIGINAL_BAICHUN_GRASP_OFFSET_M = (-0.1195, -0.142, 0.008)
ORIGINAL_BUCKET_GRASP_OFFSET_M = (-0.143, -0.127, 0.008)
ORIGINAL_TOP_GRASP_OFFSET_M = (-0.12, 0.025, 0.035)


def offset_for(target, mode):
    name = str(getattr(target, "class_name", "") or "").strip()
    group = sg.grasp_group_for(target)
    if mode.endswith("_side"):
        if name in sg.MINISO_GRASP_NAMES:
            return ORIGINAL_MINISO_GRASP_OFFSET_M
        if name in sg.BAICHUN_GRASP_NAMES:
            return ORIGINAL_BAICHUN_GRASP_OFFSET_M
        if name in sg.BUCKET_GRASP_NAMES:
            return ORIGINAL_BUCKET_GRASP_OFFSET_M
        if group == "chip":
            return ORIGINAL_CHIP_GRASP_OFFSET_M
        if sg.is_non_drink(target):
            return ORIGINAL_NON_DRINK_GRASP_OFFSET_M
        return ORIGINAL_SIDE_GRASP_OFFSET_M
    return ORIGINAL_TOP_GRASP_OFFSET_M


def grasp_for(dets, camera, depth, transformer, flange_pose):
    if not dets:
        return {"ok": False, "reason": "no detections"}
    target, edge = pick_target_box(dets, transformer=transformer, flange_pose=flange_pose)
    if target is None:
        return {"ok": False, "reason": "pick_target_box failed"}
    edge = "left"
    wx, wy, wz, mode, p_cam, pixel = plan_edge_grasp(
        target, edge, transformer, flange_pose, fx=camera.fx, camera=camera, depth_img=depth
    )

    try:
        if mode.endswith("_side"):
            if sg.is_non_drink(target):
                if sg.grasp_group_for(target) == "chip":
                    band = (sg.CHIP_SIDE_GRASP_MIN_HEIGHT_M, sg.CHIP_SIDE_GRASP_MAX_HEIGHT_M,
                            sg.CHIP_SIDE_GRASP_LOW_TARGET_HEIGHT_M)
                else:
                    band = (sg.NON_DRINK_SIDE_GRASP_MIN_HEIGHT_M, sg.NON_DRINK_SIDE_GRASP_MAX_HEIGHT_M,
                            sg.NON_DRINK_SIDE_GRASP_LOW_TARGET_HEIGHT_M)
                corrected, _, _ = clamp_side_grasp_pixel_to_safe_clearance(
                    target, pixel, camera.fy, band[0], band[1], band[2])
            else:
                corrected, _, _ = clamp_side_grasp_pixel_to_safe_clearance(target, pixel, camera.fy)
            if corrected is not None:
                pixel = corrected
                p_cam = transformer.pixel_to_camera(pixel[0], pixel[1], target.depth_m)
                p_robot = transformer.camera_to_robot(p_cam, flange_pose)
                wx, wy, wz = float(p_robot[0]), float(p_robot[1]), float(p_robot[2])
    except Exception as exc:
        print("[抓取点][警告] 纠偏失败: %s" % exc, flush=True)

    off = offset_for(target, mode)
    fingertip = (wx + off[0], wy + off[1], wz + off[2])
    return {
        "ok": True,
        "target": {
            "class_id": target.class_id,
            "class_name": target.class_name,
            "confidence": target.confidence,
            "bbox": list(target.bbox),
            "depth_m": target.depth_m,
            "edge": edge,
        },
        "grasp": {
            "mode": mode,
            "pixel": [int(pixel[0]), int(pixel[1])],
            "robot_mm": [round(wx * 1000, 1), round(wy * 1000, 1), round(wz * 1000, 1)],
            "offset_m": [round(float(v), 4) for v in off],
            "fingertip_mm": [round(v * 1000, 1) for v in fingertip],
            "camera_m": [round(float(v), 4) for v in p_cam],
        },
    }


def draw(image, dets, grasp, title):
    out = image.copy()
    for det in dets:
        x1, y1, x2, y2 = det.bbox
        is_target = bool(grasp.get("ok")) and det.class_name == grasp["target"]["class_name"] \
            and list(det.bbox) == grasp["target"]["bbox"]
        color = (255, 128, 0) if is_target else (0, 200, 0)
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 3 if is_target else 1)
        cv2.putText(out, "%s %.2f" % (det.class_id, det.confidence), (x1, max(16, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    if grasp.get("ok"):
        gx, gy = grasp["grasp"]["pixel"]
        cv2.drawMarker(out, (gx, gy), (0, 0, 255), cv2.MARKER_CROSS, 26, 2)
        cv2.circle(out, (gx, gy), 7, (0, 0, 255), 2)
        cv2.putText(out, "GRASP (%d,%d)" % (gx, gy), (max(0, gx - 70), max(18, gy - 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    cv2.putText(out, title, (10, out.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    return out


def main():
    args = parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    frame_id = "dual_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]

    arm = None
    camera = None
    servo = None
    moved = False
    try:
        flange_pose = None
        if args.skip_move:
            if str(args.flange or "").strip():
                flange_pose = [float(v) for v in args.flange.replace(",", " ").split()]
                if len(flange_pose) != 6:
                    raise RuntimeError("--flange 需要 6 个数")
            print("[机械臂] skip-move，不连接机械臂", flush=True)
        else:
            arm = DobotArm()
            print("[机械臂] 连接/上电 ...", flush=True)
            arm.connect()

            if not move_stow(arm):
                raise RuntimeError("前往收起位失败")
            if not move_layer(arm, args.layer):
                raise RuntimeError("前往第%s层失败" % args.layer)
            moved = True

            servo = ServoJoint()
            if servo.connect():
                if str(sg.SERVO_GRASP_ANGLE).strip():
                    servo.set_angle(float(sg.SERVO_GRASP_ANGLE))
                    servo.wait_until_reached(float(sg.SERVO_GRASP_ANGLE), timeout=sg.SERVO_WAIT_TIMEOUT)
                    print("[舵机] 抓取角 %s°" % sg.SERVO_GRASP_ANGLE, flush=True)
            else:
                servo = None

            flange_pose = arm.get_pose()
            print("[位姿] 法兰: (%0.1f, %0.1f, %0.1f)mm rpy=(%0.1f, %0.1f, %0.1f)" % tuple(flange_pose), flush=True)

        camera = RealSenseCamera(width=args.width, height=args.height, fps=args.fps)
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

        frame_path = os.path.join(args.outdir, frame_id + "_frame.jpg")
        depth_path = os.path.join(args.outdir, frame_id + "_depth.png")
        cv2.imwrite(frame_path, color)
        if depth is not None:
            cv2.imwrite(depth_path, depth.astype("uint16"))
        print("[采集] frame_id=%s mean_v=%.1f rgb=%s" % (frame_id, color.mean(), frame_path), flush=True)

        # 上传云端的图做和本地相同的反光/弱光增强（LAB CLAHE，不改变像素位置）
        upload_img, enhance_info = enhance_for_upload(color)
        enhanced_path = os.path.join(args.outdir, frame_id + "_frame_enhanced.jpg")
        cv2.imwrite(enhanced_path, upload_img)
        print("[增强] %s clip=%.1f tile=%d gamma=%.1f awb=%s before=%s after=%s -> %s" % (
            "已应用" if enhance_info["applied"] else "未应用",
            enhance_info["clip"], enhance_info["tile"], enhance_info["gamma"], enhance_info["awb"],
            enhance_info["before"], enhance_info.get("after"), enhanced_path), flush=True)

        transformer = None
        if flange_pose is not None:
            transformer = HandEyeTransformer(fx=camera.fx, fy=camera.fy, cx=camera.cx, cy=camera.cy)
            transformer.set_flange(flange_pose)

        ok, encoded = cv2.imencode(".jpg", upload_img, [cv2.IMWRITE_JPEG_QUALITY, 92])
        if not ok:
            raise RuntimeError("JPEG 编码失败")

        cloud_started = time.time()
        executor = ThreadPoolExecutor(max_workers=1)
        cloud_future = executor.submit(detect_rgb, encoded.tobytes(), frame_id, frame_id + ".jpg")
        cloud_error = None
        cloud_payload = None

        local_started = time.time()
        locator = build_local_detector(camera)
        local_dets = locator.process_frame(color, depth) or []
        local_ms = (time.time() - local_started) * 1000.0
        print("[本地] %d 个候选, %.0fms" % (len(local_dets), local_ms), flush=True)

        try:
            cloud_payload = cloud_future.result(timeout=args.cloud_timeout)
        except Exception as exc:
            cloud_error = str(exc)
            print("[云端][失败] %s" % cloud_error, flush=True)
        cloud_ms = (time.time() - cloud_started) * 1000.0
        executor.shutdown(wait=False)

        cloud_dets = []
        cloud_rejected = []
        if cloud_payload:
            for row in cloud_payload.get("detections", []):
                det = make_local_detection(
                    row, DetectionResult,
                    depth_at_box=lambda cx, cy, box: camera.get_depth_at(depth, cx, cy, box),
                    pixel_to_camera_3d=camera.pixel_to_camera_3d,
                )
                if det is None:
                    cloud_rejected.append(row)
                else:
                    cloud_dets.append(det)
            print("[云端] %d 个候选 (丢弃无深度 %d), 往返 %.0fms inference_ms=%s model=%s" % (
                len(cloud_dets), len(cloud_rejected), cloud_ms,
                cloud_payload.get("inference_ms"), cloud_payload.get("model_version")), flush=True)

        if args.query:
            local_dets = filter_detections_by_target_query(local_dets, args.query)
            cloud_dets = filter_detections_by_target_query(cloud_dets, args.query)
            print("[过滤] 目标关键词: %s -> 本地 %d / 云端 %d" % (
                args.query, len(local_dets), len(cloud_dets)), flush=True)

        if transformer is None:
            local_grasp = {"ok": False, "reason": "no flange pose (skip-move without --flange)"}
            cloud_grasp = {"ok": False, "reason": "no flange pose (skip-move without --flange)"}
        else:
            local_grasp = grasp_for(local_dets, camera, depth, transformer, flange_pose)
            cloud_grasp = grasp_for(cloud_dets, camera, depth, transformer, flange_pose)

        cv2.imwrite(os.path.join(args.outdir, frame_id + "_local.jpg"),
                    draw(color, local_dets, local_grasp, "LOCAL %s" % frame_id))
        cv2.imwrite(os.path.join(args.outdir, frame_id + "_cloud.jpg"),
                    draw(color, cloud_dets, cloud_grasp, "CLOUD %s" % frame_id))

        result = {
            "frame_id": frame_id,
            "layer": args.layer,
            "flange_pose": flange_pose,
            "camera": {
                "auto_exposure": CAMERA_AUTO_EXPOSURE,
                "exposure_us": CAMERA_EXPOSURE_US,
                "auto_white_balance": CAMERA_AUTO_WB,
                "gain": CAMERA_GAIN,
            },
            "enhance": enhance_info,
            "cloud": {
                "elapsed_ms": round(cloud_ms, 1),
                "error": cloud_error,
                "payload": cloud_payload,
                "detections": [
                    {"class_id": d.class_id, "class_name": d.class_name,
                     "confidence": d.confidence, "bbox": list(d.bbox), "depth_m": d.depth_m}
                    for d in cloud_dets
                ],
                "rejected": cloud_rejected,
                "grasp": cloud_grasp,
            },
            "local": {
                "elapsed_ms": round(local_ms, 1),
                "detections": [
                    {"class_id": d.class_id, "class_name": d.class_name,
                     "confidence": d.confidence, "bbox": list(d.bbox), "depth_m": d.depth_m}
                    for d in local_dets
                ],
                "grasp": local_grasp,
            },
        }
        summary_path = os.path.join(args.outdir, frame_id + "_summary.json")
        with open(summary_path, "w", encoding="utf-8") as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2)

        print("\n=== 对比 ===", flush=True)
        for name, grasp in (("本地", local_grasp), ("云端", cloud_grasp)):
            if grasp.get("ok"):
                t, g = grasp["target"], grasp["grasp"]
                print("%s: %s id=%s conf=%.2f bbox=%s depth=%s -> 像素=%s 机械臂=%smm 模式=%s" % (
                    name, t["class_name"], t["class_id"], t["confidence"], t["bbox"],
                    "%.3fm" % t["depth_m"] if t["depth_m"] else "N/A", g["pixel"], g["robot_mm"], g["mode"]), flush=True)
            else:
                print("%s: 未得到抓取点 (%s)" % (name, grasp.get("reason")), flush=True)
        if local_grasp.get("ok") and cloud_grasp.get("ok"):
            lp, cp = local_grasp["grasp"]["pixel"], cloud_grasp["grasp"]["pixel"]
            lr, cr = local_grasp["grasp"]["robot_mm"], cloud_grasp["grasp"]["robot_mm"]
            print("像素差: (%d,%d)  机械臂差: (%.1f, %.1f, %.1f)mm" % (
                cp[0] - lp[0], cp[1] - lp[1], cr[0] - lr[0], cr[1] - lr[1], cr[2] - lr[2]), flush=True)
        print("图片: %s_local.jpg / %s_cloud.jpg" % (frame_id, frame_id), flush=True)
        print("JSON: %s" % summary_path, flush=True)
    finally:
        if camera is not None:
            camera.release()
        # 先回手腕保持角，再收起机械臂，等机械臂断开后才断开舵机
        if servo is not None:
            try:
                print("[舵机] 回到保持角 %s° ..." % sg.SERVO_HOLD_ANGLE, flush=True)
                servo.set_angle(float(sg.SERVO_HOLD_ANGLE))
                servo.wait_until_reached(float(sg.SERVO_HOLD_ANGLE), timeout=sg.SERVO_WAIT_TIMEOUT)
            except Exception as exc:
                print("[舵机][警告] 回保持角失败: %s" % exc, flush=True)
        if arm is not None:
            if moved:
                try:
                    move_stow(arm)
                except Exception as exc:
                    print("[机械臂][警告] 收起失败: %s" % exc, flush=True)
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


if __name__ == "__main__":
    main()
