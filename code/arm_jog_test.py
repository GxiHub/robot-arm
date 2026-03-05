"""
arm_jog_test.py — 五軸逐一 Jog 識別測試
每次只動一顆電機，讓使用者確認哪個地址對哪個關節。

軸配置：
  A - Slave 2（原品牌）
  B - Slave 1（另外品牌，不同寄存器）
  C - Slave 3（原品牌）
  D - Slave 4（原品牌）
  E - Slave 5（原品牌）
"""
from __future__ import annotations
import serial, struct, time

PORT     = "/dev/ttyACM0"
BAUD     = 9600
JOG_SEC   = 0.3   # 每次 Jog 持續秒數（原品牌）
B_ANGLE   = 30    # B 軸每次測試角度（度）

# 原品牌速度（0~10000）
ORIG_SPEED = 150
# B 軸速度（1~800 r/min）
B_SPEED    = 60


# ── Modbus 工具 ───────────────────────────────────────────────────────────────

def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return struct.pack('<H', crc)

def fc06(slave, reg, val) -> bytes:
    f = struct.pack('>BBHH', slave, 0x06, reg, val & 0xFFFF)
    return f + crc16(f)

def fc03(slave, reg, count=1) -> bytes:
    f = struct.pack('>BBHH', slave, 0x03, reg, count)
    return f + crc16(f)

def send(port, frame, wait=0.1):
    port.reset_input_buffer()
    port.write(frame)
    time.sleep(wait)
    return port.read(64)

def read_reg(port, slave, reg) -> int | None:
    raw = send(port, fc03(slave, reg))
    if len(raw) < 7:
        return None
    if crc16(raw[:-2]) != raw[-2:]:
        return None
    return struct.unpack('>H', raw[3:5])[0]


# ── 原品牌 Jog ────────────────────────────────────────────────────────────────

def orig_jog(port, slave, seconds):
    send(port, fc06(slave, 0x009A, ORIG_SPEED))   # 設速度
    send(port, fc06(slave, 0x00C8, 1))             # 正轉
    print(f"  ▶ 正轉中... ({seconds}s)", end='', flush=True)
    time.sleep(seconds)
    send(port, fc06(slave, 0x00C8, 0))             # 停止
    print("  ■ 停止")


# ── B 軸 Jog（另外品牌，按角度控制）────────────────────────────────────────

def b_jog(port, angle=B_ANGLE):
    send(port, fc06(1, 0x04, B_SPEED))    # 設速度
    send(port, fc06(1, 0x01, 1))          # 方向=正轉
    send(port, fc06(1, 0x07, angle))      # 設角度
    send(port, fc06(1, 0x02, 1))          # 運行
    print(f"  ▶ 正轉 {angle}°...", end='', flush=True)
    time.sleep(1.5)                        # 等待動作完成
    send(port, fc06(1, 0x03, 1))          # 停止並清行程
    print("  ■ 停止")


# ── 主程式 ────────────────────────────────────────────────────────────────────

AXES = [
    {"label": "A", "slave": 2, "type": "orig", "desc": "底部第1軸"},
    {"label": "B", "slave": 1, "type": "b",    "desc": "第2軸（另外品牌）"},
    {"label": "C", "slave": 3, "type": "orig", "desc": "第3軸"},
    {"label": "D", "slave": 4, "type": "orig", "desc": "第4軸"},
    {"label": "E", "slave": 5, "type": "orig", "desc": "第5軸（頂部）"},
]

def main():
    print(f"\n串口 {PORT}  |  Baud {BAUD}")
    print("=" * 55)

    try:
        port = serial.Serial(PORT, baudrate=BAUD,
                             bytesize=8, parity='N', stopbits=1, timeout=0.3)
    except Exception as e:
        print(f"❌ 串口開啟失敗：{e}")
        return

    print("\n【Step 1】清原品牌告警")
    print("-" * 55)
    for ax in AXES:
        if ax["type"] != "orig":
            print(f"  軸 {ax['label']} (Slave {ax['slave']}) — B 軸跳過")
            continue
        alarm = read_reg(port, ax["slave"], 0x0082)
        alarm_str = f"0x{alarm:04X}" if alarm is not None else "N/A"
        print(f"  軸 {ax['label']} Slave {ax['slave']} — 告警：{alarm_str}", end="")
        if alarm and alarm not in (0, 0xFFFF):
            for reg in [0x0086, 0x0084, 0x00A0]:
                send(port, fc06(ax["slave"], reg, 0x0001), wait=0.2)
            send(port, fc06(ax["slave"], 0x0082, 0x0000), wait=0.2)
            send(port, fc06(ax["slave"], 0x00C8, 0x0006), wait=0.3)
            time.sleep(0.5)
            alarm2 = read_reg(port, ax["slave"], 0x0082)
            print(f"  → 清除後：{'✅ 已清' if alarm2 == 0 else f'仍有 0x{alarm2:04X}'}")
        else:
            print("  → 無需清除")

    print("\n\n【Step 2】逐軸 Jog 測試")
    print("每顆軸動之前倒數 3 秒，觀察哪個關節在動")
    print("-" * 55)

    for ax in AXES:
        move_desc = f"{B_ANGLE}°" if ax["type"] == "b" else f"{JOG_SEC}s"
        print(f"\n>>> 軸 {ax['label']}（Slave {ax['slave']}）{ax['desc']} — 準備正轉 {move_desc} <<<")
        for i in range(3, 0, -1):
            print(f"    {i}...", end=' ', flush=True)
            time.sleep(1)
        print()

        if ax["type"] == "orig":
            orig_jog(port, ax["slave"], JOG_SEC)
        else:
            b_jog(port)

        print(f"    ── 完畢，等 2 秒後繼續 ──")
        time.sleep(2)

    port.close()

    print("\n\n【總結】軸對應關係（請自行填寫觀察結果）")
    print("=" * 55)
    for ax in AXES:
        print(f"  軸 {ax['label']} Slave {ax['slave']}  →  ？")
    print()


if __name__ == "__main__":
    main()
