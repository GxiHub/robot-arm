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


def _approach_detect(depth_img):
    """偵測物體並回傳 (ox, oy, depth_mm) 或 None"""
    obj = _detect_nearest_object(depth_img)
    if obj is None:
        return None
    cx, cy, cz_mm = obj
    h, w = depth_img.shape
    ox = (cx - w / 2) / (w / 2)
    oy = (cy - h / 2) / (h / 2)
    return ox, oy, cz_mm, cx, cy


def _approach_sense():
    """從鏡頭取一幀深度圖，回傳 depth_img 或 None"""
    with _cam_lock:
        if not _cam_running:
            return None
        try:
            frames = _cam_pipeline.wait_for_frames(timeout_ms=3000)
            aligned = _cam_align.process(frames)
            depth_frame = aligned.get_depth_frame()
            if not depth_frame:
                return None
            return np.asanyarray(depth_frame.get_data())
        except Exception:
            return None


def _approach_probe(axis, amount, state):
    """試探法：移動一小步，觀察 ox/oy/depth 變化，回傳 (d_ox, d_oy, d_depth)"""
    # 移動前偵測
    depth_img = _approach_sense()
    if depth_img is None:
        return None
    before = _approach_detect(depth_img)
    if before is None:
        return None

    # 執行移動
    with _lock:
        if axis == "B":
            arm.b.move_angle(abs(amount), forward=(amount > 0))
        else:
            JOINTS[axis].move_relative(amount)
    time.sleep(1.0)

    # 移動後偵測
    depth_img = _approach_sense()
    if depth_img is None:
        return None
    after = _approach_detect(depth_img)
    if after is None:
        return None

    d_ox = after[0] - before[0]
    d_oy = after[1] - before[1]
    d_depth = after[2] - before[2]
    return d_ox, d_oy, d_depth


def _approach_worker():
    """背景執行的靠近邏輯（試探式 — 禁用 A 軸，每次移動前即時試探）

    策略：eye-in-hand 的 Jacobian 隨姿態劇烈變化，
    不預存固定映射，而是每輪迭代都先試探再修正。
    只用 B 軸（肩）做主要靠近，D/C 僅輔助。
    """
    state = _approach_state
    state["step"] = 0

    _approach_log("=== 安全靠近（即時試探）===")
    _approach_log("禁用A軸，主用B軸")

    # 設定慢速
    with _lock:
        for j in [arm.c, arm.d, arm.e]:
            j.set_speed(60)
        arm.b.set_speed(20)
    time.sleep(0.3)

    # 暖機：先讀幾幀確保鏡頭穩定
    for _ in range(3):
        d = _approach_sense()
        if d is not None:
            break
        time.sleep(0.5)

    # 初始偵測確認
    depth_img = _approach_sense()
    if depth_img is not None:
        det0 = _approach_detect(depth_img)
        if det0:
            _approach_log(f"初始: ({det0[3]},{det0[4]}) d={det0[2]}mm ox={det0[0]:+.2f} oy={det0[1]:+.2f}")
        else:
            _approach_log("初始偵測無物體")
    time.sleep(0.3)

    lost_count = 0

    for iteration in range(40):
        if not state["running"]:
            _approach_log("已手動停止")
            break

        state["step"] = iteration + 1

        # ── 偵測當前狀態 ────
        depth_img = _approach_sense()
        if depth_img is None:
            _approach_log("鏡頭錯誤，停止")
            break

        det = _approach_detect(depth_img)
        if det is None:
            lost_count += 1
            _approach_log(f"步驟 {state['step']}: 丟失 ({lost_count}/3)")
            if lost_count >= 3:
                break
            time.sleep(0.5)
            continue
        lost_count = 0

        ox, oy, cz_mm, cx, cy = det
        _approach_log(
            f"步驟 {state['step']}: ({cx},{cy}) d={cz_mm}mm "
            f"ox={ox:+.2f} oy={oy:+.2f}"
        )

        # ── 到達判定 ────
        if abs(ox) < 0.20 and abs(oy) < 0.20 and cz_mm < 350:
            _approach_log(f"✓ 到達! depth={cz_mm}mm ox={ox:+.2f} oy={oy:+.2f}")
            state["target"] = {"pixel": [cx, cy], "depth_mm": cz_mm}
            break

        # ── 決定修正任務（步幅隨深度縮放）────
        # 越近步幅越小，避免物體飛出畫面
        scale = max(0.3, min(1.0, cz_mm / 600))  # 300mm→0.5, 600mm→1.0
        b_step = max(1, int(2 * scale))           # 1~2°
        d_step = max(10, int(25 * scale))          # 10~25 脈衝

        tasks = []
        # 水平偏差
        if abs(ox) > 0.15:
            tasks.append(("ox", "D", d_step))
        # 深度
        if cz_mm > 400:
            tasks.append(("depth", "B", b_step))
        # 垂直偏差（只在深度夠近時處理）
        elif abs(oy) > 0.15:
            tasks.append(("oy", "B", b_step))

        if not tasks:
            _approach_log(f"  微步 B+1° 靠近")
            with _lock:
                arm.b.move_angle(1, forward=True)
            time.sleep(0.8)
            continue

        for action, axis, probe_amt in tasks:
            if not state["running"]:
                break

            # ── 試探正方向 ────
            _approach_log(f"  [{action}] 試探 {axis}+{probe_amt}")
            with _lock:
                if axis == "B":
                    arm.b.move_angle(probe_amt, forward=True)
                else:
                    JOINTS[axis].move_relative(probe_amt)
            time.sleep(0.8)

            depth_img2 = _approach_sense()
            det2 = _approach_detect(depth_img2) if depth_img2 is not None else None

            if det2 is None:
                _approach_log(f"  [{action}] 試探後丟失，回復")
                with _lock:
                    if axis == "B":
                        arm.b.move_angle(probe_amt, forward=False)
                    else:
                        JOINTS[axis].move_relative(-probe_amt)
                time.sleep(0.8)
                continue

            ox2, oy2, cz2, _, _ = det2

            # 評估改善
            if action == "depth":
                improved = (cz2 < cz_mm - 5)
                _approach_log(f"  [{action}] d: {cz_mm}→{cz2} ({cz2-cz_mm:+d}mm)")
            elif action == "oy":
                improved = (abs(oy2) < abs(oy) - 0.03)
                _approach_log(f"  [{action}] oy: {oy:+.2f}→{oy2:+.2f}")
            else:
                improved = (abs(ox2) < abs(ox) - 0.03)
                _approach_log(f"  [{action}] ox: {ox:+.2f}→{ox2:+.2f}")

            if improved:
                _approach_log(f"  [{action}] ✓ 正向有效，保持")
                # 保持在新位置，不額外推進
            else:
                # 回復到原位，嘗試反方向
                _approach_log(f"  [{action}] 回復，試反向")
                with _lock:
                    if axis == "B":
                        arm.b.move_angle(probe_amt * 2, forward=False)
                    else:
                        JOINTS[axis].move_relative(-probe_amt * 2)
                time.sleep(0.8)

                # 反向偵測
                depth_img3 = _approach_sense()
                det3 = _approach_detect(depth_img3) if depth_img3 is not None else None
                if det3:
                    ox3, oy3, cz3, _, _ = det3
                    if action == "depth":
                        rev_ok = (cz3 < cz_mm - 5)
                        _approach_log(f"  [{action}] 反向 d: {cz_mm}→{cz3}")
                    elif action == "oy":
                        rev_ok = (abs(oy3) < abs(oy) - 0.03)
                        _approach_log(f"  [{action}] 反向 oy: {oy:+.2f}→{oy3:+.2f}")
                    else:
                        rev_ok = (abs(ox3) < abs(ox) - 0.03)
                        _approach_log(f"  [{action}] 反向 ox: {ox:+.2f}→{ox3:+.2f}")

                    if rev_ok:
                        _approach_log(f"  [{action}] ✓ 反向有效")
                    else:
                        # 兩方向都無效，回復到原點
                        _approach_log(f"  [{action}] 兩向皆無效，回原點")
                        with _lock:
                            if axis == "B":
                                arm.b.move_angle(probe_amt, forward=True)
                            else:
                                JOINTS[axis].move_relative(probe_amt)
                        time.sleep(0.5)

        time.sleep(0.3)

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
