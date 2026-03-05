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
    return render_template("index.html")


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
        JOINTS[axis].stop()

    return jsonify({"ok": True, "axis": axis})


@app.route("/api/estop", methods=["POST"])
def api_estop():
    with _lock:
        arm.estop_all()
    return jsonify({"ok": True, "action": "estop_all"})


# ── 啟動 ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("機械手臂 Web UI 啟動於 http://0.0.0.0:5001")
    app.run(host="0.0.0.0", port=5001, debug=False)
