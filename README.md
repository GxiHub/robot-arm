# 機械手臂專案（Robot Arm Project）

## 概述
2 段 × 30cm 的 RS485 步進電機機械手臂，運行於 pi53。

## 硬體規格
- **段數**：2 段（J1 肩 / J2 肘）
- **每段長度**：30 cm
- **末端負載**：~2 kg
- **通訊**：RS485 Modbus-RTU 115200 bps
- **控制器**：步進電機驅動器（每軸獨立站號）

## 資料夾結構
```
~/projects/robot_arm/
├── README.md               本文件
├── docs/
│   ├── register_map.md     寄存器地址表（完整）
│   └── structure_analysis.md  結構分析與力矩計算
├── code/
│   └── arm_driver.py       Python RS485 驅動層
└── configs/
    └── （軟限位、速度設定等配置檔）
```

## 快速開始

### 安裝依賴
```bash
pip install pyserial
```

### 測試單軸
```bash
cd ~/projects/robot_arm/code
sudo python3 arm_driver.py
```

## 軸站號配置
| 軸 | Slave ID | 說明 |
|----|----------|------|
| J1 | 1 | 肩部（最大力矩軸） |
| J2 | 2 | 肘部 |

## 力矩需求
| 軸 | 計算力矩 | 建議馬達規格 |
|----|----------|-------------|
| J1（肩） | 11.8 N·m（靜態），20~30 N·m（含動態） | ≥20 N·m |
| J2（肘） | 5.9 N·m | ≥8 N·m |

## 開發進度
- [ ] Phase 1：基礎通訊驗證
- [ ] Phase 2：基本運動控制
- [ ] Phase 3：雙軸協調
- [ ] Phase 4：Web UI 整合
