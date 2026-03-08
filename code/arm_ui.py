"""
arm_ui.py — 機械手臂 Web UI（Flask，port 5001）
提供 RS485 設備掃描、電機控制、繼電器控制、狀態讀取 API
"""
from __future__ import annotations
import sys
import os
import struct
import threading
import time
from flask import Flask, request, jsonify, render_template, Response
from flask_cors import CORS
import numpy as np

# 讓 Python 找到同目錄的 arm_driver
sys.path.insert(0, os.path.dirname(__file__))
from arm_driver import RobotArm, _fc03_frame, _fc06_frame, _crc16

app = Flask(__name__)
CORS(app)

# ── 初始化手臂 ──────────────────────────────────────────────────────────────
arm = RobotArm("/dev/ttyACM0", 9600)
arm.set_speed_all(speed_original=150, b_rpm=60)
_lock = threading.Lock()

JOINTS = {
    "A": arm.a,
    "B": arm.b,
    "C": arm.c,
    "D": arm.d,
    "E": arm.e,
}

# 繼電器 Slave ID
RELAY_SLAVE = 11
RELAY_CHANNELS = {
    0: "B軸煞車",
    1: "夾爪",
}

# 軸資訊
AXIS_INFO = {
    "A": {"slave": 2, "name": "底座 J1",   "brand": "英鵬飛"},
    "B": {"slave": 1, "name": "肩關節 J2", "brand": "另品牌"},
    "C": {"slave": 3, "name": "肘關節 J3", "brand": "英鵬飛"},
    "D": {"slave": 4, "name": "腕部 J4",   "brand": "英鵬飛"},
    "E": {"slave": 5, "name": "末端 J5",   "brand": "英鵬飛"},
}

# ── 路由 ──────────────────────────────────────────────────────────────────────

@app.route("/")
def index():
    from flask import make_response, redirect
    return redirect("/v2")

@app.route("/v2")
def index_v2():
    from flask import make_response
    resp = make_response(render_template("index.html"))
    resp.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    resp.headers["Pragma"] = "no-cache"
    return resp


@app.route("/api/jog", methods=["POST"])
def api_jog():
    data = request.get_json(force=True) or {}
    axis = str(data.get("axis", "")).upper()
    direction = str(data.get("direction", "")).lower()

    if axis not in JOINTS:
        return jsonify({"ok": False, "error": f"unknown axis: {axis}"}), 400
    if direction not in ("forward", "reverse"):
        return jsonify({"ok": False, "error": f"unknown direction: {direction}"}), 400

    joint = JOINTS[axis]
    with _lock:
        print(f"[JOG] {axis} {direction}", flush=True)
        if direction == "forward":
            joint.jog_forward()
        else:
            joint.jog_reverse()

    return jsonify({"ok": True, "axis": axis, "direction": direction})


@app.route("/api/stop", methods=["POST"])
def api_stop():
    data = request.get_json(force=True) or {}
    axis = str(data.get("axis", "")).upper()

    if axis not in JOINTS:
        return jsonify({"ok": False, "error": f"unknown axis: {axis}"}), 400

    with _lock:
        print(f"[STOP] {axis}", flush=True)
        joint = JOINTS[axis]
        if hasattr(joint, 'pause'):
            joint.pause()   # B 軸用 pause（不清行程）
        else:
            joint.stop()

    return jsonify({"ok": True, "axis": axis})


@app.route("/api/estop", methods=["POST"])
def api_estop():
    with _lock:
        arm.estop_all()
    return jsonify({"ok": True, "action": "estop_all"})


@app.route("/api/hall")
def api_hall():
    with _lock:
        val = arm.e.read_hall()
    return jsonify({"E": val})


# ── RS485 設備掃描 ────────────────────────────────────────────────────────────

def _read_reg_raw(slave: int, reg: int, count: int = 1) -> list[int] | None:
    """低層寄存器讀取（需在 _lock 內呼叫）"""
    arm.port.reset_input_buffer()
    arm.port.write(_fc03_frame(slave, reg, count))
    time.sleep(0.15)
    raw = arm.port.read(5 + count * 2)
    if len(raw) < 5 + count * 2:
        return None
    if _crc16(raw[:-2]) != raw[-2:]:
        return None
    words = []
    for i in range(count):
        words.append(struct.unpack('>H', raw[3 + i * 2:5 + i * 2])[0])
    return words


def _read_coils(slave: int, start: int, count: int) -> int | None:
    """FC01 讀線圈狀態"""
    arm.port.reset_input_buffer()
    frame = struct.pack('>BBHH', slave, 0x01, start, count)
    frame += _crc16(frame)
    arm.port.write(frame)
    time.sleep(0.15)
    raw = arm.port.read(64)
    byte_count = (count + 7) // 8
    if len(raw) < 3 + byte_count + 2:
        return None
    if _crc16(raw[:-2]) != raw[-2:]:
        return None
    result = 0
    for i in range(byte_count):
        result |= raw[3 + i] << (8 * i)
    return result


@app.route("/api/scan")
def api_scan():
    """掃描 RS485 總線所有設備狀態"""
    devices = []
    with _lock:
        # 掃描電機軸
        for axis_key, info in AXIS_INFO.items():
            sid = info["slave"]
            dev = {
                "slave_id": sid,
                "axis": axis_key,
                "name": info["name"],
                "brand": info["brand"],
                "type": "motor",
                "online": False,
                "details": {},
            }
            # 嘗試讀取
            if axis_key == "B":
                # B 軸：讀速度 (0x04)
                w = _read_reg_raw(sid, 0x04, 1)
                if w is not None:
                    dev["online"] = True
                    dev["details"]["speed_rpm"] = w[0]
                    # 讀限位狀態
                    w2 = _read_reg_raw(sid, 0x17, 1)
                    if w2:
                        dev["details"]["limit_status"] = w2[0]
                    # 讀角度
                    w3 = _read_reg_raw(sid, 0x16, 1)
                    if w3:
                        v = w3[0] if w3[0] < 0x8000 else w3[0] - 0x10000
                        dev["details"]["angle"] = v
            else:
                # 原品牌軸：讀速度 (0x009A)
                w = _read_reg_raw(sid, 0x009A, 1)
                if w is not None:
                    dev["online"] = True
                    dev["details"]["speed"] = w[0]
                    # 讀告警碼
                    w2 = _read_reg_raw(sid, 0x0082, 1)
                    if w2:
                        dev["details"]["alarm"] = w2[0]
                    # 讀 IO 輸入
                    w3 = _read_reg_raw(sid, 0x0006, 1)
                    if w3:
                        dev["details"]["io_input"] = w3[0]
                        dev["details"]["hall"] = bool(w3[0] & 1)
            devices.append(dev)

        # 掃描繼電器模塊
        relay_dev = {
            "slave_id": RELAY_SLAVE,
            "name": "繼電器模塊",
            "type": "relay",
            "online": False,
            "channels": {},
        }
        coils = _read_coils(RELAY_SLAVE, 0x0000, 16)
        if coils is not None:
            relay_dev["online"] = True
            for ch in range(16):
                state = bool(coils & (1 << ch))
                label = RELAY_CHANNELS.get(ch, f"CH{ch}")
                relay_dev["channels"][str(ch)] = {
                    "label": label,
                    "state": state,
                }
        devices.append(relay_dev)

    return jsonify({"ok": True, "devices": devices})


# ── 繼電器控制 ────────────────────────────────────────────────────────────────

@app.route("/api/relay", methods=["POST"])
def api_relay():
    data = request.get_json(force=True) or {}
    channel = data.get("channel")
    state = data.get("state")  # true/false
    if channel is None or state is None:
        return jsonify({"ok": False, "error": "need channel and state"}), 400
    channel = int(channel)
    if channel < 0 or channel > 15:
        return jsonify({"ok": False, "error": "channel 0~15"}), 400

    val = 0xFF00 if state else 0x0000
    with _lock:
        arm.port.reset_input_buffer()
        frame = struct.pack('>BBHH', RELAY_SLAVE, 0x05, channel, val)
        frame += _crc16(frame)
        arm.port.write(frame)
        time.sleep(0.15)
        arm.port.read(64)

    label = RELAY_CHANNELS.get(channel, f"CH{channel}")
    action = "ON" if state else "OFF"
    print(f"[RELAY] CH{channel}({label}) → {action}", flush=True)
    return jsonify({"ok": True, "channel": channel, "state": state, "label": label})


# ── 速度設定 ──────────────────────────────────────────────────────────────────

@app.route("/api/speed", methods=["POST"])
def api_speed():
    data = request.get_json(force=True) or {}
    axis = str(data.get("axis", "")).upper()
    speed = data.get("speed")
    if axis not in JOINTS or speed is None:
        return jsonify({"ok": False, "error": "need axis and speed"}), 400
    speed = int(speed)
    with _lock:
        if axis == "B":
            arm.b.set_speed(speed)
        else:
            JOINTS[axis].set_speed(speed)
    print(f"[SPEED] {axis} → {speed}", flush=True)
    return jsonify({"ok": True, "axis": axis, "speed": speed})


# ── 相對位置移動 ──────────────────────────────────────────────────────────────

@app.route("/api/move", methods=["POST"])
def api_move():
    data = request.get_json(force=True) or {}
    axis = str(data.get("axis", "")).upper()
    pulses = data.get("pulses")
    if axis not in JOINTS or pulses is None:
        return jsonify({"ok": False, "error": "need axis and pulses"}), 400
    pulses = int(pulses)
    with _lock:
        if axis == "B":
            forward = pulses > 0
            arm.b.move_angle(abs(pulses), forward=forward)
        else:
            JOINTS[axis].move_relative(pulses)
    print(f"[MOVE] {axis} → {pulses}", flush=True)
    return jsonify({"ok": True, "axis": axis, "pulses": pulses})


# ── RealSense 鏡頭 ────────────────────────────────────────────────────────────

_cam_lock = threading.Lock()
_cam_pipeline = None
_cam_align = None
_cam_intr = None
_cam_running = False

def _cam_init():
    """初始化 RealSense（延遲載入）"""
    global _cam_pipeline, _cam_align, _cam_intr, _cam_running
    if _cam_running:
        return True
    try:
        import pyrealsense2 as rs
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
        _cam_align = rs.align(rs.stream.color)
        profile = pipe.start(cfg)
        _cam_intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        # 暖機
        for _ in range(5):
            pipe.wait_for_frames(timeout_ms=3000)
        _cam_pipeline = pipe
        _cam_running = True
        print("[CAM] RealSense 初始化完成", flush=True)
        return True
    except Exception as e:
        print(f"[CAM] RealSense 初始化失敗: {e}", flush=True)
        _cam_running = False
        return False


def _cam_capture():
    """擷取一幀 RGB+Depth，回傳 (color_img, depth_img) 或 (None, None)"""
    import cv2
    global _cam_pipeline, _cam_align, _cam_running
    if not _cam_running:
        if not _cam_init():
            return None, None
    try:
        frames = _cam_pipeline.wait_for_frames(timeout_ms=3000)
        aligned = _cam_align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame:
            return None, None
        color_img = np.asanyarray(color_frame.get_data())
        depth_img = np.asanyarray(depth_frame.get_data()) if depth_frame else None

        # 在畫面上標注深度資訊
        h, w = color_img.shape[:2]
        if depth_img is not None:
            # 中心點深度
            cz = float(depth_img[h//2, w//2]) / 1000.0
            cv2.putText(color_img, f"Center: {cz:.2f}m", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            # 有效深度比
            valid = np.count_nonzero(depth_img)
            pct = valid / depth_img.size * 100
            cv2.putText(color_img, f"Depth: {pct:.0f}%", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            # 偵測最近物體並標記
            obj = _detect_nearest_object(depth_img)
            if obj:
                ox, oy, oz = obj
                dist_cm = oz / 10.0
                # 圓圈 + 十字
                cv2.circle(color_img, (ox, oy), 20, (0, 0, 255), 2)
                cv2.line(color_img, (ox - 15, oy), (ox + 15, oy), (0, 0, 255), 2)
                cv2.line(color_img, (ox, oy - 15), (ox, oy + 15), (0, 0, 255), 2)
                # 距離標籤
                label = f"{dist_cm:.1f}cm"
                cv2.putText(color_img, label, (ox + 25, oy + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return color_img, depth_img
    except Exception as e:
        print(f"[CAM] 擷取失敗: {e}", flush=True)
        _cam_running = False
        return None, None


@app.route("/api/camera/snapshot")
def api_camera_snapshot():
    """回傳一張 JPEG 快照"""
    import cv2
    with _cam_lock:
        color, _ = _cam_capture()
    if color is None:
        return Response(b"", status=503, headers={"X-Error": "camera offline"})
    _, buf = cv2.imencode(".jpg", color, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return Response(buf.tobytes(), mimetype="image/jpeg")


@app.route("/api/camera/stream")
def api_camera_stream():
    """MJPEG 串流"""
    import cv2
    def gen():
        while True:
            with _cam_lock:
                color, _ = _cam_capture()
            if color is None:
                time.sleep(0.5)
                continue
            _, buf = cv2.imencode(".jpg", color, [cv2.IMWRITE_JPEG_QUALITY, 80])
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n"
            time.sleep(0.15)  # ~6 fps
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/camera/status")
def api_camera_status():
    """鏡頭狀態"""
    return jsonify({
        "ok": True,
        "streaming": _cam_running,
        "type": "RealSense D435I",
        "wrist_running": _wrist_running,
    })


# ── USB 腕部鏡頭（近眼）─────────────────────────────────────────────────────

_wrist_lock = threading.Lock()
_wrist_cap = None
_wrist_running = False
WRIST_DEV = 4  # /dev/video4

def _wrist_init():
    """初始化腕部 USB 鏡頭"""
    global _wrist_cap, _wrist_running
    if _wrist_running and _wrist_cap is not None:
        return True
    try:
        import cv2
        cap = cv2.VideoCapture(WRIST_DEV)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # 暖機
        for _ in range(5):
            cap.read()
        if cap.isOpened():
            _wrist_cap = cap
            _wrist_running = True
            print("[WRIST] USB 鏡頭初始化完成", flush=True)
            return True
        print("[WRIST] USB 鏡頭無法開啟", flush=True)
        return False
    except Exception as e:
        print(f"[WRIST] USB 鏡頭初始化失敗: {e}", flush=True)
        _wrist_running = False
        return False


def _wrist_capture():
    """擷取一幀"""
    import cv2
    global _wrist_cap, _wrist_running
    if not _wrist_running:
        if not _wrist_init():
            return None
    try:
        ret, frame = _wrist_cap.read()
        if ret:
            cv2.putText(frame, "Wrist Cam", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
            return frame
        _wrist_running = False
        return None
    except Exception as e:
        print(f"[WRIST] 擷取失敗: {e}", flush=True)
        _wrist_running = False
        return None


@app.route("/api/camera/wrist/snapshot")
def api_wrist_snapshot():
    """腕部鏡頭 JPEG 快照"""
    import cv2
    with _wrist_lock:
        frame = _wrist_capture()
    if frame is None:
        return Response(b"", status=503, headers={"X-Error": "wrist cam offline"})
    _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return Response(buf.tobytes(), mimetype="image/jpeg")


# ── 物體偵測 ──────────────────────────────────────────────────────────────────

def _detect_nearest_object(depth_img, min_mm=200, max_mm=1500):
    """從深度圖找最近物體的像素中心和深度
    回傳 (cx, cy, depth_mm) 或 None
    """
    if depth_img is None:
        return None
    mask = (depth_img > min_mm) & (depth_img < max_mm)
    if mask.sum() < 200:
        return None
    # 取最近的 5% 像素作為物體
    depths = depth_img[mask]
    threshold = np.percentile(depths, 5)
    close_mask = mask & (depth_img <= threshold)
    if close_mask.sum() < 50:
        return None
    ys, xs = np.where(close_mask)
    cx = int(xs.mean())
    cy = int(ys.mean())
    cz = int(np.median(depth_img[close_mask]))
    return cx, cy, cz


@app.route("/api/detect")
def api_detect():
    """偵測天眼視野中最近的物體，回傳位置資訊"""
    import pyrealsense2 as rs
    with _cam_lock:
        if not _cam_running:
            _cam_init()
        if not _cam_running:
            return jsonify({"ok": False, "error": "鏡頭未連線"}), 503

        frames = _cam_pipeline.wait_for_frames(timeout_ms=3000)
        aligned = _cam_align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()

        if not depth_frame or not color_frame:
            return jsonify({"ok": False, "error": "無法擷取畫面"}), 500

        depth_img = np.asanyarray(depth_frame.get_data())
        result = _detect_nearest_object(depth_img)

    if result is None:
        return jsonify({"ok": True, "found": False, "message": "未偵測到物體"})

    cx, cy, cz_mm = result
    h, w = depth_img.shape
    # 像素→3D 相機座標
    cz_m = cz_mm / 1000.0
    pt = rs.rs2_deproject_pixel_to_point(_cam_intr, [cx, cy], cz_m)

    return jsonify({
        "ok": True,
        "found": True,
        "pixel": [cx, cy],
        "depth_mm": cz_mm,
        "camera_xyz": [round(pt[0], 4), round(pt[1], 4), round(pt[2], 4)],
        "frame_size": [w, h],
        "offset_x": round((cx - w / 2) / (w / 2), 3),   # -1~+1, 負=左 正=右
        "offset_y": round((cy - h / 2) / (h / 2), 3),   # -1~+1, 負=上 正=下
    })


# ── 自動靠近 ──────────────────────────────────────────────────────────────────

_approach_state = {
    "running": False,
    "step": 0,
    "log": [],
    "target": None,
}


def _approach_log(msg):
    _approach_state["log"].append(msg)
    if len(_approach_state["log"]) > 50:
        _approach_state["log"] = _approach_state["log"][-50:]
    print(f"[APPROACH] {msg}", flush=True)


def _approach_sense():
    """從鏡頭取一幀深度圖"""
    with _cam_lock:
        if not _cam_running:
            return None
        try:
            frames = _cam_pipeline.wait_for_frames(timeout_ms=3000)
            aligned = _cam_align.process(frames)
            df = aligned.get_depth_frame()
            return np.asanyarray(df.get_data()) if df else None
        except Exception:
            return None


def _approach_detect():
    """偵測最近物體，回傳 (cx, cy, depth_mm) 或 None"""
    d = _approach_sense()
    if d is None:
        return None
    obj = _detect_nearest_object(d)
    return obj  # (cx, cy, depth_mm) or None


def _b_jog(forward, duration=0.5):
    """B 軸 jog 移動指定秒數（move_angle 位移太小，改用 jog+stop）"""
    with _lock:
        if forward:
            arm.b.jog_forward()
        else:
            arm.b.jog_reverse()
    time.sleep(duration)
    with _lock:
        if hasattr(arm.b, 'pause'):
            arm.b.pause()
        else:
            arm.b.stop()
    time.sleep(0.3)  # 等穩定


def _approach_worker():
    """極簡靠近：B 軸 jog 降低 depth

    用 jog+stop 控制 B 軸（move_angle 位移太小不實用）
    每步 jog 0.5s → 偵測 depth 變化 → 自動決定方向
    禁用 A 軸（避免繞線）
    """
    state = _approach_state
    state["step"] = 0

    _approach_log("=== 極簡靠近（B軸 jog）===")

    with _lock:
        arm.b.set_speed(20)
    time.sleep(0.3)

    # 暖機
    for _ in range(3):
        if _approach_sense() is not None:
            break
        time.sleep(0.3)

    # 確認煞車已釋放
    _approach_log("確認煞車...")
    # (使用者需手動確認 CH0=ON)

    # 初始偵測
    obj = _approach_detect()
    if obj is None:
        _approach_log("找不到物體，停止")
        state["running"] = False
        return

    prev_depth = obj[2]
    _approach_log(f"初始: ({obj[0]},{obj[1]}) d={prev_depth}mm")

    # B reverse = 降低手臂 = 靠近桌面物體（已實測確認）
    b_forward = False
    _approach_log("方向: B reverse（降低靠近）")

    # ── 主循環 ────
    stall = 0
    jog_time = 0.5  # 每步 jog 時間

    for i in range(50):
        if not state["running"]:
            _approach_log("手動停止")
            break

        state["step"] = i + 1
        obj = _approach_detect()
        if obj is None:
            _approach_log(f"步驟{state['step']}: 丟失")
            # 回退一步
            _b_jog(forward=not b_forward, duration=jog_time)
            break

        cx, cy, cz = obj
        _approach_log(f"步驟{state['step']}: ({cx},{cy}) d={cz}mm")

        if cz < 300:
            _approach_log(f"✓ 到達! d={cz}mm")
            state["target"] = {"pixel": [cx, cy], "depth_mm": cz}
            break

        # jog 一步
        _b_jog(forward=b_forward, duration=jog_time)

        # 檢查
        obj2 = _approach_detect()
        if obj2 is None:
            _approach_log("移動後丟失，回退")
            _b_jog(forward=not b_forward, duration=jog_time)
            break

        delta = obj2[2] - prev_depth

        if delta < -5:
            _approach_log(f"  {prev_depth}→{obj2[2]} ({delta:+d}mm) ✓")
            prev_depth = obj2[2]
            stall = 0
        elif delta > 30:
            _approach_log(f"  {prev_depth}→{obj2[2]} ({delta:+d}mm) 反轉!")
            b_forward = not b_forward
            prev_depth = obj2[2]
            stall = 0
        else:
            _approach_log(f"  {prev_depth}→{obj2[2]} ({delta:+d}mm) ~")
            prev_depth = obj2[2]
            stall += 1
            if stall >= 4:
                # 嘗試增加 jog 時間或換軸
                if jog_time < 1.0:
                    jog_time += 0.3
                    _approach_log(f"  增加步幅: jog={jog_time:.1f}s")
                    stall = 0
                else:
                    _approach_log("步幅已最大仍無進展，停止")
                    break

    state["running"] = False
    _approach_log("=== 靠近結束 ===")


@app.route("/api/approach/start", methods=["POST"])
def api_approach_start():
    """啟動自動靠近物體"""
    if _approach_state["running"]:
        return jsonify({"ok": False, "error": "已在執行中"})

    _approach_state["running"] = True
    _approach_state["log"] = []
    _approach_state["target"] = None
    t = threading.Thread(target=_approach_worker, daemon=True)
    t.start()
    _approach_log("開始自動靠近")
    return jsonify({"ok": True, "message": "開始靠近物體"})


@app.route("/api/approach/stop", methods=["POST"])
def api_approach_stop():
    """停止自動靠近"""
    _approach_state["running"] = False
    with _lock:
        arm.estop_all()
    _approach_log("手動急停")
    return jsonify({"ok": True})


@app.route("/api/approach/status")
def api_approach_status():
    """查詢靠近狀態"""
    return jsonify({
        "ok": True,
        "running": _approach_state["running"],
        "step": _approach_state["step"],
        "log": _approach_state["log"][-15:],
        "target": _approach_state["target"],
    })


# ── 啟動 ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("機械手臂 Web UI 啟動於 http://0.0.0.0:5001")
    app.run(host="0.0.0.0", port=5001, debug=False, threaded=True)
