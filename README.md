# 機械手臂專案（Robot Arm Project）

## 概述
5-DOF 桌面型機械手臂，RS485 Modbus-RTU 統一控制，運行於 Raspberry Pi 5（pi53）。
目標：廚師助手 — 抓取籃子、倒料、搬運等廚房輔助任務。

---

## RS485 總線設備清單

| Slave ID | 設備 | 類型 | 品牌 | 狀態 |
|----------|------|------|------|------|
| 1 | B軸（肩關節 J2） | 步進電機 | 另外品牌 | ✅ 正常 |
| 2 | A軸（底座 J1） | 步進電機 | 原品牌 | ✅ 正常 |
| 3 | C軸（肘關節 J3） | 步進電機 | 原品牌 | ✅ 正常 |
| 4 | D軸（腕部 J4） | 步進電機 | 原品牌 | ⚠️ 告警 |
| 5 | E軸（末端 J5） | 步進電機 | 原品牌 | ❌ 離線 |
| 11 | 繼電器模塊 | 16路繼電器 | — | ✅ 正常 |

### 繼電器通道分配（Slave 11）

| 通道 | 功能 | ON (0xFF00) | OFF (0x0000) |
|------|------|-------------|--------------|
| CH0 | B軸電機煞車 | 煞車釋放 | 煞車鎖定 |
| CH1 | 夾爪開關 | 夾爪動作 | 夾爪釋放 |
| CH2~CH15 | 未分配 | — | — |

### 深度鏡頭

| 項目 | 值 |
|------|-----|
| 型號 | Intel RealSense D435I |
| 序號 | 140122071092 |
| 韌體 | 5.17.0.9 |
| USB | 2.1（USB 3.0 可提升至 30fps） |
| RGB | 最高 1920x1080 |
| 深度 | 最高 1280x720，有效範圍 ~33cm~6.5m |
| IMU | 加速度 400Hz + 陀螺儀 400Hz |
| 用途 | 固定天眼：物件偵測 + 3D 定位 |

---

## 通訊設定

| 參數 | 值 |
|------|-----|
| 串口 | `/dev/ttyACM0`（USB-Serial `1a86_USB_Single_Serial`） |
| 波特率 | **9600 bps**（全設備統一） |
| 格式 | 8N1 |
| 協議 | Modbus-RTU |

---

## 硬體規格

| 項目 | 數值 |
|------|------|
| 類型 | 桌面型 5-DOF 串聯式（RR-RR-R） |
| 段數 | 2 段 × 30 cm，總展開 60+ cm |
| 末端負載 | ~2 kg |
| 馬達 | 步進一體機，直接驅動或減速比驅動 |

### 關節配置

| 關節 | 軸代號 | Slave ID | 功能 | 旋轉方向 |
|------|--------|----------|------|----------|
| J1 | A | 2 | 底座旋轉 | Yaw（水平） |
| J2 | B | 1 | 肩關節 | Pitch（垂直） |
| J3 | C | 3 | 肘關節 | Pitch（垂直） |
| J4 | D | 4 | 腕部旋轉 | Yaw |
| J5 | E | 5 | 末端姿態 | Pitch/Roll |

---

## 資料夾結構

```
~/projects/robot_arm/
├── README.md                    本文件
├── docs/
│   ├── register_map.md          RS485 設備總覽 + 寄存器地址表
│   └── structure_analysis.md    結構分析與力矩計算
├── code/
│   ├── arm_driver.py            RS485 驅動核心（Joint + BAxisJoint + RobotArm）
│   ├── arm_status.py            各軸狀態讀取/診斷
│   ├── arm_ui.py                Flask Web UI (port 5001)
│   ├── arm_jog_test.py          逐軸 Jog 測試
│   ├── arm_vision.py            視覺模組
│   ├── hall_test_e.py           E軸 Hall 感測器測試
│   └── templates/index.html     Web 控制面板
└── configs/                     待配置
```

---

## 快速開始

### 安裝依賴
```bash
pip install pyserial pyrealsense2
```

### 測試單軸
```bash
cd ~/projects/robot_arm/code
sudo python3 arm_driver.py
```

### 測試深度鏡頭
```bash
python3 -c "import pyrealsense2 as rs; ctx=rs.context(); print(len(ctx.query_devices()), '台設備')"
```

---

## 開發進度

- [x] Phase 1：RS485 基礎通訊驗證
- [x] Phase 2：單軸 Jog / 速度 / 相對位置控制
- [x] Phase 3：Flask Web UI + 急停
- [x] Phase 4：Hall 感測器（E軸零點偵測）
- [x] Phase 5：繼電器模塊（煞車 + 夾爪）
- [x] Phase 6：深度鏡頭（RealSense D435i）接入
- [ ] Phase 7：自動歸零 + 軟限位校正
- [ ] Phase 8：多軸同步 / 直線插補
- [ ] Phase 9：視覺導引抓取（天眼 + 近眼）
