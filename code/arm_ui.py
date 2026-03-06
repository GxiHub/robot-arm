"""
arm_ui.py — 機械手臂 Web UI（Flask，port 5001）
"""
from __future__ import annotations
import sys
import os
import threading
from flask import Flask, request, jsonify, render_template

# 讓 Python 找到同目錄的 arm_driver
sys.path.insert(0, os.path.dirname(__file__))
from arm_driver import RobotArm

app = Flask(__name__)

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


# ── 啟動 ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("機械手臂 Web UI 啟動於 http://0.0.0.0:5001")
    app.run(host="0.0.0.0", port=5001, debug=False)
