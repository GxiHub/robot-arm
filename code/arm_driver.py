"""
arm_driver.py — RS485 步進電機驅動層（機械手臂）
每個關節對應一台步進控制器，透過 Modbus-RTU 控制。

軸配置：
  A - Slave 2 — 底部第1軸（原品牌）
  B - Slave 1 — 第2軸（另外品牌，不同寄存器）
  C - Slave 3 — 第3軸（原品牌）
  D - Slave 4 — 第4軸（原品牌）
  E - Slave 5 — 第5軸，頂部（原品牌）
"""
from __future__ import annotations
import struct
import serial
import time


# ══════════════════════════════════════════════════════════════════════════════
# 共用：Modbus-RTU CRC & 幀建構
# ══════════════════════════════════════════════════════════════════════════════

def _crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack('<H', crc)

def _fc03_frame(slave: int, reg: int, count: int = 1) -> bytes:
    """FC03 讀保持寄存器"""
    frame = struct.pack('>BBHH', slave, 0x03, reg, count)
    return frame + _crc16(frame)

def _fc06_frame(slave: int, reg: int, value: int) -> bytes:
    """FC06 寫單個寄存器"""
    frame = struct.pack('>BBHH', slave, 0x06, reg, value & 0xFFFF)
    return frame + _crc16(frame)

def _fc10_frame(slave: int, reg: int, values: list[int]) -> bytes:
    """FC10 寫多個寄存器"""
    count = len(values)
    data = struct.pack('>BBHHB', slave, 0x10, reg, count, count * 2)
    for v in values:
        data += struct.pack('>H', v & 0xFFFF)
    return data + _crc16(data)

def _dword_regs(value: int) -> list[int]:
    """將 32-bit 整數拆成 [高WORD, 低WORD]"""
    value = value & 0xFFFFFFFF
    return [(value >> 16) & 0xFFFF, value & 0xFFFF]


# ══════════════════════════════════════════════════════════════════════════════
# 原品牌電機 Joint（A/C/D/E 軸）
# ══════════════════════════════════════════════════════════════════════════════

# 原品牌寄存器
_REG_SPEED     = 0x009A   # 運行速度 (WORD, 0~10000)
_REG_RUN_CMD   = 0x00C8   # 運行指令
_REG_RUN_PULSE = 0x00CE   # 相對位置運行 (DWORD)
_REG_SOFT_NEG  = 0x006E   # 軟負限位 (DWORD)
_REG_SOFT_POS  = 0x0070   # 軟正限位 (DWORD)
_REG_HW_LIMIT  = 0x009B   # 硬體限位配置
_REG_ALARM     = 0x0082   # 告警碼
_REG_IO_INPUT  = 0x0006   # IO 輸入狀態（bit0 = Hall 感測器）

_CMD_STOP    = 0      # 減速停止
_CMD_FORWARD = 1      # 正轉
_CMD_REVERSE = 257    # 反轉
_CMD_ESTOP   = 256    # 急停


class Joint:
    """原品牌步進電機控制器（A/C/D/E 軸）"""

    def __init__(self, port: serial.Serial, slave_id: int, name: str = ""):
        self.port = port
        self.slave = slave_id
        self.name = name or f"J{slave_id}"

    def _send(self, frame: bytes, wait: float = 0.05) -> None:
        self.port.write(frame)
        time.sleep(wait)
        self.port.read_all()

    def _read_reg(self, reg: int) -> int | None:
        self.port.reset_input_buffer()
        self.port.write(_fc03_frame(self.slave, reg, 1))
        time.sleep(0.15)
        raw = self.port.read(64)
        if len(raw) < 7:
            return None
        if _crc16(raw[:-2]) != raw[-2:]:
            return None
        return struct.unpack('>H', raw[3:5])[0]

    def set_speed(self, speed: int) -> None:
        """設定速度（0~10000，建議 300~1200）"""
        self._send(_fc06_frame(self.slave, _REG_SPEED, max(0, min(10000, speed))))

    def jog_forward(self) -> None:
        """持續正轉（需手動呼叫 stop）"""
        self._send(_fc06_frame(self.slave, _REG_RUN_CMD, _CMD_FORWARD))

    def jog_reverse(self) -> None:
        """持續反轉（需手動呼叫 stop）"""
        self._send(_fc06_frame(self.slave, _REG_RUN_CMD, _CMD_REVERSE))

    def stop(self) -> None:
        """減速停止"""
        self._send(_fc06_frame(self.slave, _REG_RUN_CMD, _CMD_STOP))

    def estop(self) -> None:
        """急停"""
        self._send(_fc06_frame(self.slave, _REG_RUN_CMD, _CMD_ESTOP))

    def move_relative(self, pulses: int) -> None:
        """相對位置移動（正=正方向，負=負方向）"""
        if pulses >= 0:
            regs = _dword_regs(pulses)
        else:
            regs = _dword_regs(0x100000000 + pulses)
        self._send(_fc10_frame(self.slave, _REG_RUN_PULSE, regs))

    def set_soft_limits(self, neg_pulses: int, pos_pulses: int) -> None:
        """設定軟限位（脈衝數）"""
        self._send(_fc10_frame(self.slave, _REG_SOFT_NEG, _dword_regs(neg_pulses)))
        self._send(_fc10_frame(self.slave, _REG_SOFT_POS, _dword_regs(pos_pulses)))

    def read_alarm(self) -> int | None:
        """讀取告警碼（0=無告警）"""
        return self._read_reg(_REG_ALARM)

    def read_hall(self) -> bool | None:
        """讀取 Hall 感測器狀態（True=觸發, False=釋放, None=讀取失敗）"""
        val = self._read_reg(_REG_IO_INPUT)
        if val is None:
            return None
        return bool(val & 1)


# ══════════════════════════════════════════════════════════════════════════════
# B 軸電機 BAxisJoint（另外品牌，Slave 1）
# ══════════════════════════════════════════════════════════════════════════════

# B 軸寄存器
_B_REG_DIRECTION  = 0x01   # 方向（1=正轉, 0=反轉）
_B_REG_RUN        = 0x02   # 運行/暫停（1=運行, 0=暫停）
_B_REG_STOP       = 0x03   # 停止並清行程（1=停止）
_B_REG_SPEED      = 0x04   # 速度（1~800 r/min）
_B_REG_PULSE      = 0x05   # 脈衝數（0=一直轉）
_B_REG_REVOLUTION = 0x06   # 圈數（0=一直轉）
_B_REG_ANGLE      = 0x07   # 角度（0=一直轉）
_B_REG_ADDRESS    = 0x08   # 裝置地址（01~247）
_B_REG_LIMIT_EN   = 0x0D   # 啟用限位開關（1=開, 0=關）
_B_REG_BAUD       = 0x0F   # 波特率（3=9600 默認）
_B_REG_SET_HOME   = 0x15   # 設置當前位置為原點（1=設置）
_B_REG_GO_HOME    = 0x0A   # 一鍵回原點
_B_REG_CUR_ANGLE  = 0x16   # 讀取已運行角度（有符號）
_B_REG_LIMIT_ST   = 0x17   # 讀取限位開關狀態
_B_REG_HOME_SPEED = 0x1A   # 回原點速度（1~400 r/min）
_B_REG_FACTORY    = 0x20   # 恢復出廠設置（1=設置）


class BAxisJoint:
    """B 軸步進電機控制器（另外品牌，Slave 1）
    通訊格式：Modbus-RTU，9600 bps，8N1
    速度範圍：1~800 r/min
    """

    LIMIT_STATUS = {
        0: "無觸發",
        1: "正轉方向觸發",
        2: "反轉方向觸發",
        3: "正反兩側均觸發",
    }

    def __init__(self, port: serial.Serial, slave_id: int = 1, name: str = "B-第2軸"):
        self.port = port
        self.slave = slave_id
        self.name = name

    def _send(self, frame: bytes, wait: float = 0.08) -> None:
        self.port.write(frame)
        time.sleep(wait)
        self.port.read_all()

    def _read_reg(self, reg: int) -> int | None:
        self.port.reset_input_buffer()
        self.port.write(_fc03_frame(self.slave, reg, 1))
        time.sleep(0.15)
        raw = self.port.read(64)
        if len(raw) < 7:
            return None
        if _crc16(raw[:-2]) != raw[-2:]:
            return None
        val = struct.unpack('>H', raw[3:5])[0]
        # 有符號 16-bit（用於角度讀取）
        return val if val < 0x8000 else val - 0x10000

    # ── 基本運動 ──────────────────────────────────────────────────────────────

    def set_speed(self, rpm: int) -> None:
        """設定速度（1~800 r/min）"""
        rpm = max(1, min(800, rpm))
        self._send(_fc06_frame(self.slave, _B_REG_SPEED, rpm))

    def jog_forward(self) -> None:
        """持續正轉（圈數設0，需手動呼叫 stop）"""
        self._send(_fc06_frame(self.slave, _B_REG_REVOLUTION, 0))   # 連續模式
        self._send(_fc06_frame(self.slave, _B_REG_DIRECTION, 1))     # 正轉
        self._send(_fc06_frame(self.slave, _B_REG_RUN, 1))           # 運行

    def jog_reverse(self) -> None:
        """持續反轉（圈數設0，需手動呼叫 stop）"""
        self._send(_fc06_frame(self.slave, _B_REG_REVOLUTION, 0))   # 連續模式
        self._send(_fc06_frame(self.slave, _B_REG_DIRECTION, 0))     # 反轉
        self._send(_fc06_frame(self.slave, _B_REG_RUN, 1))           # 運行

    def pause(self) -> None:
        """暫停（保留行程，可繼續）"""
        self._send(_fc06_frame(self.slave, _B_REG_RUN, 0))

    def stop(self) -> None:
        """停止並清除行程"""
        self._send(_fc06_frame(self.slave, _B_REG_STOP, 1))

    def move_revolutions(self, revs: int, forward: bool = True) -> None:
        """按圈數移動（revs=圈數，forward=方向）"""
        self._send(_fc06_frame(self.slave, _B_REG_DIRECTION, 1 if forward else 0))
        self._send(_fc06_frame(self.slave, _B_REG_REVOLUTION, max(1, revs)))
        self._send(_fc06_frame(self.slave, _B_REG_RUN, 1))

    def move_angle(self, angle: int, forward: bool = True) -> None:
        """按角度移動（angle=度數 0~65535）"""
        self._send(_fc06_frame(self.slave, _B_REG_DIRECTION, 1 if forward else 0))
        self._send(_fc06_frame(self.slave, _B_REG_ANGLE, angle))
        self._send(_fc06_frame(self.slave, _B_REG_RUN, 1))

    def estop(self) -> None:
        """急停（同 stop）"""
        self.stop()

    # ── 限位開關 ──────────────────────────────────────────────────────────────

    def enable_limit_switch(self, enable: bool = True) -> None:
        """啟用/關閉限位開關"""
        self._send(_fc06_frame(self.slave, _B_REG_LIMIT_EN, 1 if enable else 0))

    def read_limit_status(self) -> str:
        """讀取限位開關狀態"""
        val = self._read_reg(_B_REG_LIMIT_ST)
        if val is None:
            return "讀取失敗"
        return self.LIMIT_STATUS.get(val, f"未知狀態({val})")

    # ── 原點 ──────────────────────────────────────────────────────────────────

    def set_home(self) -> None:
        """設置當前位置為原點"""
        self._send(_fc06_frame(self.slave, _B_REG_SET_HOME, 1))

    def go_home(self) -> None:
        """一鍵回原點（自動判斷方向）"""
        self._send(_fc06_frame(self.slave, _B_REG_GO_HOME, 1))

    def set_home_speed(self, rpm: int) -> None:
        """設定回原點速度（1~400 r/min）"""
        self._send(_fc06_frame(self.slave, _B_REG_HOME_SPEED, max(1, min(400, rpm))))

    # ── 狀態讀取 ──────────────────────────────────────────────────────────────

    def read_angle(self) -> int | None:
        """讀取已運行角度（正=正轉方向，負=反轉方向）"""
        return self._read_reg(_B_REG_CUR_ANGLE)


# ══════════════════════════════════════════════════════════════════════════════
# 機械手臂整合（5 軸）
# ══════════════════════════════════════════════════════════════════════════════

class RobotArm:
    """五軸機械手臂
    A(Slave2) - B(Slave1) - C(Slave3) - D(Slave4) - E(Slave5)
    """

    def __init__(self, port_name: str = "/dev/ttyACM0", baudrate: int = 9600):
        self.port = serial.Serial(
            port_name, baudrate=baudrate,
            bytesize=8, parity='N', stopbits=1,
            timeout=0.3
        )
        # 原品牌軸
        self.a = Joint(self.port, slave_id=2, name="A-底部")
        self.c = Joint(self.port, slave_id=3, name="C-第3軸")
        self.d = Joint(self.port, slave_id=4, name="D-第4軸")
        self.e = Joint(self.port, slave_id=5, name="E-頂部")
        # B 軸（另外品牌）
        self.b = BAxisJoint(self.port, slave_id=1, name="B-第2軸")

        self.all_joints: list[Joint | BAxisJoint] = [self.a, self.b, self.c, self.d, self.e]

    def close(self) -> None:
        self.port.close()

    def estop_all(self) -> None:
        """所有軸急停"""
        for j in self.all_joints:
            j.estop()

    def set_speed_all(self, speed_original: int, b_rpm: int = 60) -> None:
        """統一設定所有軸速度
        speed_original：原品牌軸速度（0~10000）
        b_rpm：B 軸速度（1~800 r/min）
        """
        for j in [self.a, self.c, self.d, self.e]:
            j.set_speed(speed_original)
        self.b.set_speed(b_rpm)


# ══════════════════════════════════════════════════════════════════════════════
# 範例使用
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    arm = RobotArm("/dev/ttyACM0", baudrate=9600)
    try:
        print("設定速度...")
        arm.set_speed_all(speed_original=150, b_rpm=60)

        print("A 軸正轉 1 秒...")
        arm.a.jog_forward()
        time.sleep(1)
        arm.a.stop()

        print("B 軸正轉 1 秒...")
        arm.b.jog_forward()
        time.sleep(1)
        arm.b.stop()

        print(f"B 軸限位狀態：{arm.b.read_limit_status()}")
        print(f"B 軸已運行角度：{arm.b.read_angle()}°")

    finally:
        arm.estop_all()
        arm.close()
