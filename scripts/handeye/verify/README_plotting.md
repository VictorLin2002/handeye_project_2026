# Hand-Eye Calibration Error Visualization

這個工具集用於視覺化和分析hand-eye calibration的重複性測試誤差。

## 功能特點

### 1. 數據收集 (`verify_tag4_simple.py`)
- 自動保存樣本數據到CSV檔案
- 記錄每個樣本的XYZ位置和時間戳一致性

### 2. 誤差視覺化 (`plot_repeatability_errors.py`)
生成多種圖表來分析calibration精度：

#### 完整分析模式（6個圖表）：
1. **3D散點圖** - 顯示所有樣本在3D空間的分佈
2. **時間序列誤差** - XYZ誤差隨時間變化
3. **誤差直方圖** - 誤差分佈統計
4. **XY平面俯視圖** - 包含2σ橢圓
5. **徑向誤差** - 2D和3D徑向誤差分析
6. **統計摘要表** - 關鍵數據總結

#### 簡化模式（2個圖表）：
1. **XYZ誤差vs時間** - 快速查看趨勢
2. **誤差分佈直方圖** - 標準差分析

## 使用方法

### 步驟1：運行重複性測試並保存數據

```bash
# 運行測試，保存50個樣本到CSV
ros2 run <your_package> verify_tag4_simple.py \
  --ros-args \
  -p test_mode:=repeatability \
  -p num_samples:=50 \
  -p save_samples_csv:=/tmp/calibration_samples.csv \
  -p reference_point_mm:="[0.0, -650.0, 83.5]"
```

### 步驟2：生成視覺化圖表

#### 選項A：完整分析（推薦）
```bash
cd /home/Victor/ros2_ws/scripts/handeye/verify

python3 plot_repeatability_errors.py \
  --csv /tmp/calibration_samples.csv \
  --reference 0.0 -650.0 83.5 \
  --output calibration_full_analysis.png
```

#### 選項B：簡化分析（快速檢查）
```bash
python3 plot_repeatability_errors.py \
  --csv /tmp/calibration_samples.csv \
  --reference 0.0 -650.0 83.5 \
  --output calibration_quick_check.png \
  --simple
```

#### 選項C：測試模式（無需實際數據）
```bash
# 使用模擬數據測試繪圖功能
python3 plot_repeatability_errors.py \
  --samples 50 \
  --reference 0.0 -650.0 83.5 \
  --output test_plot.png
```

## 輸出說明

### 統計指標

- **Bias (mm)**: 平均位置與參考點的偏差（系統誤差）
  - 理想值：<5mm (X, Y), <10mm (Z)

- **Std Dev (mm)**: 位置重複性的標準差（隨機誤差）
  - 理想值：<5mm (X, Y), <20mm (Z)

- **Norm Bias**: 三軸偏差的歐幾里得距離
  - 理想值：<10mm

- **RMS Error (3D)**: 均方根誤差
  - 衡量整體精度

- **Max Error (3D)**: 最大誤差
  - 檢測異常值

### 圖表解讀

#### 1. 3D散點圖
- **紅色星星**: 參考點（預期位置）
- **橙色菱形**: 實際平均位置
- **彩色點**: 各個樣本（顏色表示時間順序）
- **觀察重點**: 樣本是否集中？是否有漂移？

#### 2. 時間序列誤差
- **觀察重點**:
  - 是否有趨勢性漂移？
  - 是否有週期性變化？
  - 異常值出現在何時？

#### 3. XY平面俯視圖
- **虛線橢圓**: 2σ範圍（95%信賴區間）
- **觀察重點**: XY平面的重複性

#### 4. 徑向誤差
- **2D徑向**: XY平面的距離誤差
- **3D徑向**: 三維空間的距離誤差
- **觀察重點**: 整體精度穩定性

## 精度評估標準

### ✅ 優秀 (Excellent)
- X/Y bias < 2mm, std < 3mm
- Z bias < 5mm, std < 15mm
- Norm bias < 5mm

### ⚠️ 可接受 (Acceptable)
- X/Y bias < 5mm, std < 5mm
- Z bias < 10mm, std < 20mm
- Norm bias < 10mm

### ❌ 需要重新校正 (Needs Recalibration)
- X/Y bias > 10mm or std > 10mm
- Z bias > 20mm or std > 30mm
- Norm bias > 15mm

## 故障排除

### 問題1：Z軸誤差特別大（>200mm）
**原因**: 參考點設定錯誤
**解決**: 確認 `reference_point_mm` 是Tag表面實際Z高度

### 問題2：誤差有明顯趨勢性漂移
**原因**:
- 相機或機器人溫漂
- Tag移動
- 光照變化

**解決**:
- 等待系統熱穩定
- 固定Tag
- 使用穩定光源

### 問題3：Z軸標準差特別大
**原因**: Z軸測量本質上精度較低（深度估計）
**解決**:
- 改善光照條件
- 調整相機角度（避免太斜）
- 增加Tag尺寸

### 問題4：無法生成圖表
**錯誤**: `ModuleNotFoundError: No module named 'matplotlib'`
**解決**:
```bash
pip3 install matplotlib numpy
```

## 進階使用

### 比較多次校正結果

```bash
# 第一次校正
python3 plot_repeatability_errors.py --csv calibration1.csv --output cal1.png

# 第二次校正
python3 plot_repeatability_errors.py --csv calibration2.csv --output cal2.png

# 視覺比較兩張圖
```

### 自動化批次分析

```bash
#!/bin/bash
# 連續5次測試並生成報告

for i in {1..5}; do
  echo "=== Test $i ==="
  ros2 run pkg verify_tag4_simple.py \
    --ros-args -p save_samples_csv:=/tmp/test_$i.csv

  python3 plot_repeatability_errors.py \
    --csv /tmp/test_$i.csv \
    --output test_${i}_analysis.png \
    --simple
done
```

## 檔案結構

```
scripts/handeye/verify/
├── verify_tag4_simple.py       # 重複性測試主程式
├── plot_repeatability_errors.py # 視覺化工具
└── README_plotting.md          # 本說明文件
```

## 參數說明

### verify_tag4_simple.py 參數
- `test_mode`: "repeatability" 或 "touch"
- `num_samples`: 樣本數量（建議50-100）
- `sample_interval_sec`: 樣本間隔（秒）
- `reference_point_mm`: 參考點XYZ座標（mm）
- `save_samples_csv`: CSV輸出路徑（空字串=不保存）

### plot_repeatability_errors.py 參數
- `--csv`: CSV數據檔案路徑
- `--reference X Y Z`: 參考點座標（mm）
- `--output`: 輸出圖片檔名
- `--simple`: 使用簡化模式（2圖vs6圖）
- `--samples`: 模擬數據樣本數（測試用）

## 建議工作流程

1. **初步測試**: 使用簡化模式快速評估
   ```bash
   python3 plot_repeatability_errors.py --csv data.csv --simple
   ```

2. **詳細分析**: 如果發現問題，使用完整模式深入調查
   ```bash
   python3 plot_repeatability_errors.py --csv data.csv
   ```

3. **記錄結果**: 保存圖表和統計數據供日後參考

4. **調整校正**: 根據分析結果決定是否需要重新校正

## 聯絡與支持

如有問題或建議，請參考主專案文檔。
