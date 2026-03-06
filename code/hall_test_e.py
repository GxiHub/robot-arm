"""
hall_test_e.py — E 軸 Hall 感測器接線測試
Slave 5，原品牌（英鵬飛），RS485 Modbus-RTU

用法：
  sudo python3 hall_test_e.py

操作：
  1. 啟動後選 1（掃描模式）：靠近磁鐵，看哪個寄存器數值有變化
  2. 找到後選 2（即時監控）：確認觸發/釋放是否穩定
"""
from __future__ import annotations
import struct, serial, time, sys

PORT  = "/dev/ttyACM0"
BAUD  = 9600
SLAVE = 5   # E 軸


def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return struct.pack('<H', crc)

def fc03(slave, reg, count=1) -> bytes:
    f = struct.pack('>BBHH', slave, 0x03, reg, count)
    return f + crc16(f)

def fc06(slave, reg, val) -> bytes:
    f = struct.pack('>BBHH', slave, 0x06, reg, val & 0xFFFF)
    return f + crc16(f)

def read_reg(port, slave, reg) -> int | None:
    port.reset_input_buffer()
    port.write(fc03(slave, reg))
    time.sleep(0.12)
    raw = port.read(64)
    if len(raw) < 7:
        return None
    if crc16(raw[:-2]) != raw[-2:]:
        return None
    return struct.unpack('>H', raw[3:5])[0]


def scan_mode(port):
    """掃描模式：一次讀多個寄存器，找出哪個因磁鐵靠近而改變"""
    # 候選寄存器：常見的 IO 狀態 / 限位狀態位置
    candidates = [
        (0x0082, "告警碼"),
        (0x009B, "硬體限位設定"),
        (0x00FC, "IO 狀態(猜)"),
        (0x00FD, "IO 狀態2(猜)"),
        (0x00FE, "IO 狀態3(猜)"),
        (0x00FF, "IO 狀態4(猜)"),
        (0x0100, "狀態1"),
        (0x0101, "狀態2"),
        (0x0102, "狀態3"),
        (0x0060, "IO輸入(猜)"),
        (0x0061, "IO輸入2(猜)"),
        (0x0064, "輸入狀態(猜)"),
    ]

    print("\n[掃描模式] 讀取 E 軸(Slave 5)所有候選寄存器")
    print("先不要靠近磁鐵，記錄基準值...\n")
    time.sleep(1)

    baseline = {}
    for reg, name in candidates:
        val = read_reg(port, SLAVE, reg)
        baseline[reg] = val
        val_str = f"0x{val:04X} ({val})" if val is not None else "無回應"
        print(f"  0x{reg:04X}  {name:20s} = {val_str}")

    print("\n現在靠近磁鐵，3 秒後重新讀取比較...", end='', flush=True)
    for i in range(3, 0, -1):
        print(f" {i}", end='', flush=True)
        time.sleep(1)
    print("\n")

    print("【有變化的寄存器】")
    found = []
    for reg, name in candidates:
        val = read_reg(port, SLAVE, reg)
        old = baseline[reg]
        changed = (val != old)
        marker = " <<<<< 變化!" if changed else ""
        val_str = f"0x{val:04X} ({val})" if val is not None else "無回應"
        old_str = f"0x{old:04X}" if old is not None else "N/A"
        if changed:
            print(f"  0x{reg:04X}  {name:20s}  {old_str} → {val_str}{marker}")
            found.append((reg, name))

    if not found:
        print("  未偵測到變化。請確認：")
        print("  1. 感測器 LED 靠近磁鐵時有亮嗎？")
        print("  2. X0+ 是否接 24V，X0- 是否接感測器 Black 線？")
        print("  3. 寄存器 0x009B 是否有設定 NPN 限位輸入模式？")
    else:
        print(f"\n找到 {len(found)} 個有變化的寄存器，建議用即時監控確認。")
    return found


def monitor_mode(port, reg: int, name: str):
    """即時監控模式：固定輪詢一個寄存器，顯示觸發狀態"""
    print(f"\n[即時監控] 寄存器 0x{reg:04X} ({name})")
    print("靠近/移開磁鐵觀察變化，Ctrl+C 結束\n")
    last = None
    try:
        while True:
            val = read_reg(port, SLAVE, reg)
            if val != last:
                ts = time.strftime("%H:%M:%S")
                if val is not None:
                    bits = f"{val:016b}"
                    status = "【觸發】" if val != 0 else "  釋放  "
                    print(f"  {ts}  0x{val:04X}  {bits}  {status}")
                else:
                    print(f"  {ts}  讀取失敗")
                last = val
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n結束監控。")


def setup_hw_limit(port):
    """設定 0x009B：NPN 模式，X0 為負限位"""
    # 常見設定值：0x0001 = 啟用NPN X0負限位
    # 實際值依英鵬飛型號而異，這裡先讀出現況
    val = read_reg(port, SLAVE, 0x009B)
    print(f"\n目前 0x009B (硬體限位設定) = 0x{val:04X}" if val else "\n讀取 0x009B 失敗")
    print("（如需設定，請提供型號手冊確認正確值）")


def main():
    print(f"串口 {PORT}  Baud {BAUD}  Slave {SLAVE} (E軸)")
    try:
        port = serial.Serial(PORT, baudrate=BAUD,
                             bytesize=8, parity='N', stopbits=1, timeout=0.3)
    except Exception as e:
        print(f"串口開啟失敗：{e}")
        sys.exit(1)

    # 先讀現況
    setup_hw_limit(port)

    print("\n選擇模式：")
    print("  1. 掃描模式（找出哪個寄存器對應 Hall 輸入）")
    print("  2. 即時監控（手動輸入寄存器地址）")
    choice = input("輸入 1 或 2：").strip()

    if choice == "1":
        found = scan_mode(port)
        if found:
            print("\n要繼續監控哪個？")
            for i, (reg, name) in enumerate(found):
                print(f"  {i+1}. 0x{reg:04X} {name}")
            sel = input("輸入編號（或 Enter 跳過）：").strip()
            if sel.isdigit() and 1 <= int(sel) <= len(found):
                r, n = found[int(sel)-1]
                monitor_mode(port, r, n)
    elif choice == "2":
        reg_input = input("輸入寄存器地址（如 0x00FC）：").strip()
        try:
            reg = int(reg_input, 16)
            monitor_mode(port, reg, f"0x{reg:04X}")
        except ValueError:
            print("格式錯誤")

    port.close()

if __name__ == "__main__":
    main()
