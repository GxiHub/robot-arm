#!/usr/bin/env python3
"""
arm_vision.py — YOLOv8 + RealSense D435i 視覺偵測模組
功能：
  - D435i RGB + 深度對齊串流
  - YOLOv8-nano 物件偵測
  - 每個物件：bounding box 中心像素 → 查深度 → 算 3D 相機座標 (X, Y, Z) 公尺
  - Flask web UI：http://<pi>:5003
  - REST API：GET /detections  → JSON 回傳當前所有偵測物件含 3D 座標

用法：
  python3 arm_vision.py
  python3 arm_vision.py --classes 39 41 45  # 只偵測特定 COCO class（39=bottle, 41=cup, 45=bowl）
  python3 arm_vision.py --conf 0.5          # 信心值門檻（預設 0.4）

未來整合：
  from arm_vision import get_detections     # 取得當前偵測結果（含 3D 座標）
"""

import sys
import time
import threading
import argparse
from typing import Optional

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from flask import Flask, Response, render_template_string, jsonify, request

# ─────────────────────────────────────────────────────────
# 全域狀態
# ─────────────────────────────────────────────────────────
_lock = threading.Lock()
_annotated_frame: Optional[bytes] = None   # JPEG bytes for streaming
_detections: list[dict] = []               # 最新偵測結果
_stats = {"fps": 0.0, "det_count": 0, "streaming": False, "error": None}
_depth_arr: Optional[np.ndarray] = None    # 最新深度陣列 (HxW, mm 單位)
_intr = None                               # RealSense color stream intrinsics

# 咖啡色 HSV 範圍（偵測 AirPods 盒）
BROWN_HSV_LOWER = np.array([5,  80, 25])
BROWN_HSV_UPPER = np.array([28, 230, 160])
BROWN_MIN_AREA  = 500   # 最小面積（像素）過濾雜訊

app = Flask(__name__)

# ─────────────────────────────────────────────────────────
# COCO class 名稱（繁中 / 英 對照，只列常見廚房物件）
# ─────────────────────────────────────────────────────────
COCO_ZH = {
    39: "瓶子",    40: "酒杯",    41: "杯子",    42: "叉子",
    43: "刀子",    44: "湯匙",    45: "碗",      46: "香蕉",
    47: "蘋果",    48: "三明治",  49: "柳橙",    50: "花椰菜",
    51: "胡蘿蔔",  52: "熱狗",    53: "披薩",    54: "甜甜圈",
    55: "蛋糕",    56: "椅子",    57: "沙發",    58: "盆栽",
    67: "手機",    73: "筆電",    76: "剪刀",    84: "書",
}

def class_label(cls_id: int, cls_name: str) -> str:
    zh = COCO_ZH.get(cls_id)
    return f"{zh}({cls_name})" if zh else cls_name


# ─────────────────────────────────────────────────────────
# RealSense + YOLO 主迴圈
# ─────────────────────────────────────────────────────────
def vision_loop(filter_classes: Optional[list[int]], conf_thresh: float):
    global _annotated_frame, _detections, _stats

    print("[Vision] 載入 YOLOv8n 模型...")
    model = YOLO("yolov8n.pt")
    print("[Vision] 模型載入完成")

    while True:
        # ── 初始化 RealSense ──────────────────────────────
        try:
            pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
            cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
            align = rs.align(rs.stream.color)

            profile = pipeline.start(cfg)
            color_profile = (profile.get_stream(rs.stream.color)
                             .as_video_stream_profile())
            intr = color_profile.get_intrinsics()
            with _lock:
                global _intr
                _intr = intr

            # 穩定化暖機
            for _ in range(10):
                pipeline.wait_for_frames(timeout_ms=3000)

            # 深度後處理濾波器
            spatial   = rs.spatial_filter()
            temporal  = rs.temporal_filter()
            hole_fill = rs.hole_filling_filter()

            _stats["streaming"] = True
            _stats["error"] = None
            print("[Vision] RealSense 啟動完成")

        except Exception as e:
            _stats["streaming"] = False
            _stats["error"] = str(e)
            print(f"[Vision] RealSense 啟動失敗：{e}")
            time.sleep(3)
            continue

        # ── 主推理迴圈 ───────────────────────────────────
        t0 = time.time()
        frame_count = 0

        try:
            while True:
                frames = align.process(pipeline.wait_for_frames(timeout_ms=5000))
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                color_img = np.asanyarray(color_frame.get_data())   # BGR HxW x 3

                # 深度平滑
                df = hole_fill.process(temporal.process(spatial.process(depth_frame))).as_depth_frame()

                # 儲存深度陣列（供 /depth API 使用）
                with _lock:
                    global _depth_arr
                    _depth_arr = np.asanyarray(df.get_data()).copy()

                # ── 咖啡色偵測（AirPods）─────────────────
                hsv_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
                brown_mask = cv2.inRange(hsv_img, BROWN_HSV_LOWER, BROWN_HSV_UPPER)
                brown_mask = cv2.morphologyEx(brown_mask, cv2.MORPH_OPEN,
                                              np.ones((5,5), np.uint8))
                brown_cnts, _ = cv2.findContours(brown_mask, cv2.RETR_EXTERNAL,
                                                  cv2.CHAIN_APPROX_SIMPLE)
                for cnt in brown_cnts:
                    area = cv2.contourArea(cnt)
                    if area < BROWN_MIN_AREA:
                        continue
                    x, y, bw, bh = cv2.boundingRect(cnt)
                    cx, cy = x + bw // 2, y + bh // 2

                    # 取深度
                    patch = []
                    for dy in range(-2, 3):
                        for dx in range(-2, 3):
                            px, py = cx + dx, cy + dy
                            if 0 <= px < 848 and 0 <= py < 480:
                                d = df.get_distance(px, py)
                                if d > 0:
                                    patch.append(d)
                    z = float(np.median(patch)) if patch else 0.0
                    if z > 0:
                        pt = rs.rs2_deproject_pixel_to_point(intr, [cx, cy], z)
                    else:
                        pt = [None, None, None]

                    xyz_txt = (f"X:{pt[0]:+.3f} Y:{pt[1]:+.3f} Z:{pt[2]:.3f}m"
                               if pt[2] is not None else "depth:--")

                    # 橘色框標示
                    cv2.rectangle(annotated, (x, y), (x+bw, y+bh), (0, 140, 255), 2)
                    cv2.circle(annotated, (cx, cy), 5, (0, 60, 255), -1)
                    cv2.putText(annotated, f"AirPods {xyz_txt}",
                                (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 140, 255), 1)

                    detections.append({
                        "class_id":   -1,
                        "class_name": "airpods",
                        "label_zh":   "AirPods(咖啡色)",
                        "conf":       round(area / (bw * bh), 3),
                        "bbox":       [x, y, x+bw, y+bh],
                        "pixel":      [cx, cy],
                        "camera_xyz": [round(v, 4) if v is not None else None
                                       for v in pt],
                    })

                # ── YOLO 推理 ─────────────────────────────
                results = model(
                    color_img,
                    conf=conf_thresh,
                    classes=filter_classes,
                    verbose=False,
                )[0]

                # ── 逐物件取 3D 座標 ──────────────────────
                detections = []
                annotated = color_img.copy()

                for box in results.boxes:
                    cls_id  = int(box.cls[0])
                    conf    = float(box.conf[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx, cy  = (x1 + x2) // 2, (y1 + y2) // 2

                    # 3×3 中心區域平均深度（更穩定）
                    patch = []
                    for dy in range(-1, 2):
                        for dx in range(-1, 2):
                            px, py = cx + dx, cy + dy
                            if 0 <= px < 848 and 0 <= py < 480:
                                d = df.get_distance(px, py)
                                if d > 0:
                                    patch.append(d)
                    z = float(np.median(patch)) if patch else 0.0

                    # 像素 → 3D 相機座標
                    if z > 0:
                        point3d = rs.rs2_deproject_pixel_to_point(
                            intr, [cx, cy], z
                        )  # [X, Y, Z] 公尺
                    else:
                        point3d = [None, None, None]

                    label = class_label(cls_id, results.names[cls_id])
                    det = {
                        "class_id":   cls_id,
                        "class_name": results.names[cls_id],
                        "label_zh":   label,
                        "conf":       round(conf, 3),
                        "bbox":       [x1, y1, x2, y2],
                        "pixel":      [cx, cy],
                        "camera_xyz": [round(v, 4) if v is not None else None
                                       for v in point3d],  # 公尺，相機座標系
                    }
                    detections.append(det)

                    # ── 繪製標注 ──────────────────────────
                    color = (0, 220, 60)
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                    cv2.circle(annotated, (cx, cy), 4, (0, 60, 255), -1)

                    if z > 0:
                        xyz_txt = (f"X:{point3d[0]:+.3f} "
                                   f"Y:{point3d[1]:+.3f} "
                                   f"Z:{point3d[2]:.3f}m")
                    else:
                        xyz_txt = "depth:--"

                    # 標籤背景
                    tag1 = f"{label} {conf:.0%}"
                    tag2 = xyz_txt
                    (tw1, th1), _ = cv2.getTextSize(tag1, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
                    (tw2, th2), _ = cv2.getTextSize(tag2, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                    tw = max(tw1, tw2)
                    ty = max(y1 - 4, 36)
                    cv2.rectangle(annotated,
                                  (x1, ty - th1 - th2 - 10),
                                  (x1 + tw + 6, ty + 2),
                                  (20, 20, 20), -1)
                    cv2.putText(annotated, tag1, (x1 + 3, ty - th2 - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 60), 1)
                    cv2.putText(annotated, tag2, (x1 + 3, ty - 3),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 200, 255), 1)

                # ── FPS 標注 ──────────────────────────────
                frame_count += 1
                elapsed = time.time() - t0
                fps = frame_count / elapsed if elapsed > 0 else 0
                if frame_count % 30 == 0:
                    t0 = time.time()
                    frame_count = 0

                cv2.putText(annotated,
                            f"FPS:{fps:.1f}  偵測:{len(detections)}",
                            (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (255, 255, 255), 2)

                # ── 更新全域共享 ──────────────────────────
                _, buf = cv2.imencode(".jpg", annotated,
                                     [cv2.IMWRITE_JPEG_QUALITY, 80])
                with _lock:
                    _annotated_frame = buf.tobytes()
                    _detections      = detections
                    _stats["fps"]       = round(fps, 1)
                    _stats["det_count"] = len(detections)

        except Exception as e:
            print(f"[Vision] 串流中斷：{e}")
            _stats["streaming"] = False
            _stats["error"] = str(e)
        finally:
            try:
                pipeline.stop()
            except Exception:
                pass
            with _lock:
                _annotated_frame = None
                _detections = []
        time.sleep(3)


# ─────────────────────────────────────────────────────────
# 外部呼叫 API（給 arm 控制程式用）
# ─────────────────────────────────────────────────────────
def get_detections() -> list[dict]:
    """取得最新偵測結果（thread-safe）"""
    with _lock:
        return list(_detections)


# ─────────────────────────────────────────────────────────
# Flask 路由
# ─────────────────────────────────────────────────────────
HTML = """<!DOCTYPE html>
<html lang="zh-TW">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>手臂視覺偵測</title>
<style>
* { box-sizing:border-box; margin:0; padding:0; }
body { background:#0d1117; color:#e6edf3; font-family:-apple-system,sans-serif; }
header { padding:12px 18px; border-bottom:1px solid #21262d;
         display:flex; align-items:center; gap:10px; }
h1 { font-size:1rem; }
.badge { font-size:.7rem; background:#238636; padding:2px 8px; border-radius:12px; }
.main { display:grid; grid-template-columns:1fr 340px; gap:10px; padding:12px; }
.card { background:#161b22; border:1px solid #21262d; border-radius:8px; overflow:hidden; }
.card h2 { font-size:.8rem; padding:8px 12px; color:#8b949e;
           border-bottom:1px solid #21262d; }
.card img { width:100%; display:block; }
#det-list { padding:8px; display:flex; flex-direction:column; gap:6px;
            max-height:520px; overflow-y:auto; }
.det-item { background:#0d1117; border:1px solid #30363d; border-radius:6px;
            padding:8px 10px; font-size:.78rem; }
.det-label { font-weight:600; color:#58a6ff; margin-bottom:4px; }
.det-conf  { color:#8b949e; font-size:.72rem; }
.det-xyz   { color:#39d353; margin-top:4px; font-family:monospace; font-size:.8rem; }
.det-pixel { color:#6e7681; font-size:.72rem; }
.no-det    { color:#484f58; text-align:center; padding:20px; font-size:.85rem; }
.stats { padding:6px 12px; font-size:.75rem; color:#6e7681;
         border-top:1px solid #21262d; }
@media(max-width:900px){ .main{grid-template-columns:1fr} #det-panel{order:-1} }
</style>
</head>
<body>
<header>
  <h1>YOLOv8 視覺偵測 + 3D 定位</h1>
  <span class="badge">LIVE</span>
</header>
<div class="main">
  <div class="card">
    <h2>D435i RGB 偵測串流</h2>
    <img src="/stream" alt="detection stream">
    <div class="stats" id="stats">FPS: -- | 偵測: --</div>
  </div>
  <div class="card" id="det-panel">
    <h2>偵測結果（3D 座標）</h2>
    <div id="det-list"><div class="no-det">等待偵測...</div></div>
  </div>
</div>
<script>
function update() {
  fetch('/detections').then(r => r.json()).then(data => {
    const list = document.getElementById('det-list');
    const stats = document.getElementById('stats');
    stats.textContent = `FPS: ${data.fps} | 偵測: ${data.count}`;
    if (!data.detections.length) {
      list.innerHTML = '<div class="no-det">目前無偵測到物件</div>';
      return;
    }
    list.innerHTML = data.detections.map(d => {
      const xyz = d.camera_xyz;
      const xyzTxt = (xyz[2] !== null)
        ? `X:${xyz[0]>=0?'+':''}${xyz[0].toFixed(3)}m  Y:${xyz[1]>=0?'+':''}${xyz[1].toFixed(3)}m  Z:${xyz[2].toFixed(3)}m`
        : '深度無效';
      return `<div class="det-item">
        <div class="det-label">${d.label_zh}</div>
        <div class="det-conf">信心值 ${(d.conf*100).toFixed(1)}%</div>
        <div class="det-pixel">像素中心 (${d.pixel[0]}, ${d.pixel[1]})</div>
        <div class="det-xyz">${xyzTxt}</div>
      </div>`;
    }).join('');
  }).catch(()=>{});
}
setInterval(update, 300);
update();
</script>
</body>
</html>"""


@app.route("/")
def index():
    return render_template_string(HTML)


@app.route("/stream")
def stream():
    def gen():
        while True:
            with _lock:
                f = _annotated_frame
            if f is None:
                time.sleep(0.05)
                continue
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + f + b"\r\n"
            time.sleep(1 / 25)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/depth")
def depth_api():
    """
    查詢指定像素的 3D 座標
    GET /depth?x=424&y=240
    回傳: {"pixel":[x,y], "depth_m": 0.643, "camera_xyz":[X,Y,Z]}
    """
    try:
        px = int(request.args.get("x", 424))
        py = int(request.args.get("y", 240))
    except (TypeError, ValueError):
        return jsonify({"error": "invalid x/y"}), 400

    with _lock:
        darr = _depth_arr
        intr = _intr

    if darr is None or intr is None:
        return jsonify({"error": "no depth data yet"}), 503

    px = max(0, min(px, darr.shape[1] - 1))
    py = max(0, min(py, darr.shape[0] - 1))

    z = float(darr[py, px]) / 1000.0  # mm → m
    if z > 0:
        pt = rs.rs2_deproject_pixel_to_point(intr, [px, py], z)
    else:
        pt = [None, None, None]

    return jsonify({
        "pixel":      [px, py],
        "depth_m":    round(z, 4),
        "camera_xyz": [round(v, 4) if v is not None else None for v in pt],
    })


@app.route("/detections")
def detections_api():
    """
    回傳格式：
    {
      "fps": 28.5,
      "count": 2,
      "detections": [
        {
          "class_id": 45,
          "class_name": "bowl",
          "label_zh": "碗(bowl)",
          "conf": 0.87,
          "bbox": [x1, y1, x2, y2],
          "pixel": [cx, cy],
          "camera_xyz": [X, Y, Z]   ← 公尺，相機座標系
        },
        ...
      ]
    }
    """
    with _lock:
        return jsonify({
            "fps":        _stats["fps"],
            "count":      _stats["det_count"],
            "streaming":  _stats["streaming"],
            "detections": list(_detections),
        })


# ─────────────────────────────────────────────────────────
# 入口
# ─────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--classes", type=int, nargs="*",
                        help="只偵測這些 COCO class ID（留空偵測全部）")
    parser.add_argument("--conf", type=float, default=0.4,
                        help="信心值門檻（預設 0.4）")
    parser.add_argument("--port", type=int, default=5003,
                        help="Flask port（預設 5003）")
    args = parser.parse_args()

    print(f"[Vision] 啟動，classes={args.classes}, conf={args.conf}")
    t = threading.Thread(
        target=vision_loop,
        args=(args.classes, args.conf),
        daemon=True,
    )
    t.start()

    print(f"\n  瀏覽器：http://0.0.0.0:{args.port}\n")
    app.run(host="0.0.0.0", port=args.port, threaded=True)
