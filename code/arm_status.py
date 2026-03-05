"""
arm_status.py — 機械手臂各軸狀態讀取工具
讀取每台步進電機的：速度設定、軟限位、位置回授、告警碼、I/O 狀態

使用方式：
    sudo python3 arm_status.py
    sudo python3 arm_status.py --port /dev/ttyUSB0 --slaves 1 2 3
"""
from __future__ import annotations
import argparse
import struct
import time
import serial


# ── Modbus-RTU CRC ────────────────────────────────────────────────────────────
def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _build_fc03(slave: int, reg: int, count: int) -> bytes:
    """FC03 讀保持寄存器請求幀"""
    frame = struct.pack('>BBHH', slave, 0x03, reg, count)
    crc = _crc16(frame)
    return frame + struct.pack('<H', crc)


def _build_fc04(slave: int, reg: int, count: int) -> bytes:
    """FC04 讀輸入寄存器請求幀"""
    frame = struct.pack('>BBHH', slave, 0x04, reg, count)
    crc = _crc16(frame)
    return frame + struct.pack('<H', crc)


def _parse_response(raw: bytes, expected_count: int) -> list[int] | None:
    """解析 FC03/FC04 回應，回傳 WORD 列表；失敗回傳 None"""
    expected_len = 5 + expected_count * 2  # slave+fc+bytecount + data + crc(2)
    if len(raw) < expected_len:
        return None
    # 驗證 CRC
    payload = raw[:-2]
    crc_recv = struct.unpack('<H', raw[-2:])[0]
    if _crc16(payload) != crc_recv:
        return None
    # 解析資料
    byte_count = raw[2]
    if byte_count != expected_count * 2:
        return None
    words = []
    for i in range(expected_count):
        offset = 3 + i * 2
        words.append(struct.unpack('>H', raw[offset:offset+2])[0])
    return words


def read_regs(port: serial.Serial, slave: int, reg: int, count: int,
              fc: int = 3, timeout: float = 0.15) -> list[int] | None:
    """讀取寄存器，回傳 WORD 列表或 None（無回應/CRC錯誤）"""
    port.reset_input_buffer()
    frame = _build_fc03(slave, reg, count) if fc == 3 else _build_fc04(slave, reg, count)
    port.write(frame)
    time.sleep(timeout)
    raw = port.read(5 + count * 2)
    return _parse_response(raw, count)


def words_to_dword(high: int, low: int) -> int:
    """兩個 WORD 組成帶符號 DWORD"""
    val = (high << 16) | low
    if val >= 0x80000000:
        val -= 0x100000000
    return val


# ── 寄存器定義（依文件整理）────────────────────────────────────────────────────
# (reg_addr, count, fc, 名稱, 說明)
KNOWN_REGS = [
    # 速度與設定
    (0x009A, 1, 3, "運行速度",       "0~10000，預設 300"),
    (0x009B, 1, 3, "硬體限位配置",   "X0~X15 端子方向 / PNP/NPN"),
    # 軟限位
    (0x006E, 2, 3, "軟負限位",       "負方向最大脈衝數 (DWORD)"),
    (0x0070, 2, 3, "軟正限位",       "正方向最大脈衝數 (DWORD)"),
    # 表格控制
    (0x00AA, 1, 3, "表格大小",       ""),
    (0x00AB, 1, 3, "表格指針",       "當前執行位置"),
    (0x00AC, 1, 3, "表格起始地址",   ""),
]

# 常見狀態寄存器區段（依控制器慣例掃描，可能因型號不同而異）
SCAN_REGIONS = [
    # (起始, 長度, fc, 區段說明)
    (0x0000, 8,  4, "輸入暫存器 0x0000~0x0007（狀態/告警）"),
    (0x0000, 8,  3, "保持暫存器 0x0000~0x0007（狀態/告警）"),
    (0x001E, 4,  3, "0x001E~0x0021（位置/速度回授）"),
    (0x0030, 8,  3, "0x0030~0x0037（I/O 狀態區）"),
    (0x0060, 8,  3, "0x0060~0x0067（參數區）"),
    (0x0096, 6,  3, "0x0096~0x009B（速度/限位設定附近）"),
    (0x00C8, 4,  3, "0x00C8~0x00CB（指令狀態）"),
]


# ── 格式化輸出 ────────────────────────────────────────────────────────────────
def fmt_dword(words: list[int], idx: int = 0) -> str:
    val = words_to_dword(words[idx], words[idx+1])
    return f"{val:,}  (0x{val & 0xFFFFFFFF:08X})"


def print_separator(char: str = "─", width: int = 60) -> None:
    print(char * width)


def print_joint_status(port: serial.Serial, slave: int, name: str) -> bool:
    """讀取並印出單一關節狀態，回傳是否有回應"""
    print()
    print_separator("═")
    print(f"  {name}  (Slave ID = {slave})")
    print_separator("═")

    alive = False

    # ── 已知可讀寄存器 ────────────────────────────────────────────────────────
    print("\n[已知寄存器]")
    print(f"  {'地址':<10} {'名稱':<16} {'原始值':<20} {'說明'}")
    print_separator()
    for reg, count, fc, label, desc in KNOWN_REGS:
        words = read_regs(port, slave, reg, count, fc=fc)
        if words is None:
            val_str = "── 無回應 ──"
        else:
            alive = True
            if count == 1:
                val_str = f"{words[0]}  (0x{words[0]:04X})"
                if reg == 0x009A:
                    val_str += f"  → 速度 {words[0]}"
                elif reg == 0x009B:
                    val_str += f"  → 限位設定"
            else:
                val_str = fmt_dword(words)
        print(f"  0x{reg:04X}      {label:<16} {val_str:<30} {desc}")

    if not alive:
        print()
        print("  ⚠️  此 Slave 無回應，請確認接線與站號設定")
        return False

    # ── 掃描狀態區段 ─────────────────────────────────────────────────────────
    print("\n[狀態區段掃描]")
    for start, count, fc, region_name in SCAN_REGIONS:
        words = read_regs(port, slave, start, count, fc=fc)
        if words is None:
            print(f"  FC{fc:02d} {region_name:<44} ── 無回應")
        else:
            hex_vals = " ".join(f"{w:04X}" for w in words)
            dec_vals = " ".join(f"{w}" for w in words)
            print(f"  FC{fc:02d} {region_name}")
            print(f"       HEX: {hex_vals}")
            print(f"       DEC: {dec_vals}")

            # 嘗試解讀位置回授（0x001E~0x001F 常見）
            if start == 0x001E and len(words) >= 2:
                pos = words_to_dword(words[0], words[1])
                print(f"       → 位置回授（推測）: {pos:,} 脈衝")
            if start == 0x001E and len(words) >= 4:
                spd = words_to_dword(words[2], words[3])
                print(f"       → 速度回授（推測）: {spd}")

    return True


# ── 主程式 ────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(description="機械手臂步進電機狀態讀取")
    parser.add_argument("--port",   default="/dev/ttyRS485", help="串口設備")
    parser.add_argument("--baud",   default=115200, type=int, help="波特率")
    parser.add_argument("--slaves", nargs="+", type=int, default=[1, 2],
                        help="要查詢的 Slave ID 列表（預設: 1 2）")
    args = parser.parse_args()

    print(f"\n機械手臂狀態讀取工具")
    print(f"串口: {args.port}  |  波特率: {args.baud}  |  查詢軸: {args.slaves}")

    try:
        port = serial.Serial(
            args.port, baudrate=args.baud,
            bytesize=8, parity='N', stopbits=1,
            timeout=0.2
        )
    except serial.SerialException as e:
        print(f"\n❌ 無法開啟串口 {args.port}: {e}")
        print("請確認設備已連接，或加上 --port 指定正確串口")
        return

    joint_names = {1: "J1-肩", 2: "J2-肘", 3: "J3", 4: "J4", 5: "J5", 6: "J6"}
    results = {}

    for slave in args.slaves:
        name = joint_names.get(slave, f"J{slave}")
        ok = print_joint_status(port, slave, name)
        results[slave] = ok

    port.close()

    print()
    print_separator("═")
    print("  總結")
    print_separator()
    for slave, ok in results.items():
        name = joint_names.get(slave, f"J{slave}")
        status = "✅ 有回應" if ok else "❌ 無回應"
        print(f"  Slave {slave:2d} ({name}): {status}")
    print_separator("═")
    print()


if __name__ == "__main__":
    main()
