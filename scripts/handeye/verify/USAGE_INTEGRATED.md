# ROS 2 驗證節點使用指南（verify_handeye_touch）

## 🎉 新功能

現在 ROS 2 節點 `handeye_verify/verify_tag4_simple.py` 已經整合了自動繪圖功能！一個命令就能完成：
- ✅ 運行repeatability測試
- ✅ 自動保存CSV數據
- ✅ 自動生成分析圖表

## 🚀 快速開始

### 最簡單的用法（推薦）

```bash
# 只需運行這一個命令！
ros2 run handeye_verify verify_handeye_touch \
  --ros-args \
  -p test_mode:=repeatability \
  -p num_samples:=50
```

**自動完成**：
- ✅ 收集50個測試樣本
- ✅ 保存CSV到 `results/handeye_calibration/data/`
- ✅ 生成完整分析圖到 `results/handeye_calibration/plots/`
- ✅ 終端顯示統計結果

## ⚙️ 參數說明

### 新增的繪圖參數

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `auto_plot` | `True` | 是否自動生成圖表 |
| `plot_mode` | `"full"` | 圖表模式：`"full"` / `"simple"` / `"both"` |
| `output_dir` | `results/handeye_calibration` | 輸出目錄 |

### 原有參數（仍可用）

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `num_samples` | `500` | 樣本數量 |
| `sample_interval_sec` | `0.1` | 樣本間隔（秒） |
| `reference_point_mm` | `[0.0, -650.0, 83.5]` | 參考點（mm） |
| `save_samples_csv` | `""` | 自定義CSV路徑（空=自動生成） |

### Touch模式參數

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `repeat_touch` | `false` | 是否在touch流程結束後自動重複 |
| `repeat_interval_sec` | `0.5` | 每次touch流程之間的等待秒數 |

## 📖 使用範例

### 範例1：預設配置（最常用）
```bash
ros2 run handeye_verify verify_handeye_touch \
  --ros-args \
  -p test_mode:=repeatability \
  -p num_samples:=50
```

**結果**：
- CSV: `results/handeye_calibration/data/calibration_samples_YYYYMMDD_HHMMSS.csv`
- 圖表: `results/handeye_calibration/plots/calibration_full_YYYYMMDD_HHMMSS.png`

### 範例2：禁用自動繪圖
```bash
ros2 run handeye_verify verify_handeye_touch \
  --ros-args \
  -p test_mode:=repeatability \
  -p num_samples:=50 \
  -p auto_plot:=false
```

**結果**：只運行測試和保存CSV，不生成圖表

### 範例3：使用簡化圖表（更快）
```bash
ros2 run handeye_verify verify_handeye_touch \
  --ros-args \
  -p test_mode:=repeatability \
  -p num_samples:=50 \
  -p plot_mode:=simple
```

**結果**：生成2個圖表的簡化版本（約170KB vs 440KB）

### 範例4：同時生成兩種圖表
```bash
ros2 run handeye_verify verify_handeye_touch \
  --ros-args \
  -p test_mode:=repeatability \
  -p num_samples:=50 \
  -p plot_mode:=both
```

**結果**：同時生成完整版和簡化版圖表

### 範例5：快速測試（10個樣本）
```bash
ros2 run handeye_verify verify_handeye_touch \
  --ros-args \
  -p test_mode:=repeatability \
  -p num_samples:=10 \
  -p sample_interval_sec:=0.2
```

**結果**：快速完成測試（約2秒），適合調試

### 範例6：自定義輸出目錄
```bash
ros2 run handeye_verify verify_handeye_touch \
  --ros-args \
  -p test_mode:=repeatability \
  -p num_samples:=50 \
  -p output_dir:=/tmp/my_calibration_test
```

**結果**：結果保存到自定義目錄

### 範例7：自定義參考點
```bash
ros2 run handeye_verify verify_handeye_touch \
  --ros-args \
  -p test_mode:=repeatability \
  -p num_samples:=50 \
  -p reference_point_mm:="[0.0, -650.0, 85.0]"
```

### 範例8：Touch模式持續循環
```bash
ros2 run handeye_verify verify_handeye_touch \
  --ros-args \
  -p test_mode:=touch \
  -p repeat_touch:=true \
  -p repeat_interval_sec:=1.0
```

**結果**：每次touch流程結束後等待1秒，然後自動重複直到手動停止。

## 📊 輸出文件

### 默認目錄結構
```
results/handeye_calibration/
├── data/
│   └── calibration_samples_20260116_170530.csv
├── plots/
│   ├── calibration_full_20260116_170530.png    (443KB)
│   └── calibration_simple_20260116_170530.png  (172KB, 如果用both模式)
└── reports/
    └── (手動生成的報告)
```

### CSV格式
```csv
sample_idx,x_mm,y_mm,z_mm,stamp_spread_ms
0,-0.836,-655.910,77.967,0.0
1,-1.234,-656.123,78.456,0.0
...
```

### 圖表內容

#### 完整分析圖（plot_mode=full）
1. **3D散點圖** - 所有樣本在3D空間的分佈
2. **XYZ誤差vs時間** - 時間序列分析
3. **誤差直方圖** - 統計分佈
4. **XY俯視圖** - 包含2σ橢圓
5. **徑向誤差** - 2D和3D誤差分析
6. **統計摘要表** - 關鍵指標

#### 簡化圖表（plot_mode=simple）
1. **XYZ誤差vs時間** - 顯示bias
2. **誤差直方圖** - 顯示標準差

## 🛠️ 故障排除

### 問題1：matplotlib未安裝
**現象**：
```
[WARN] Plotting requested but matplotlib not available: No module named 'matplotlib'.
Install matplotlib to enable plotting: pip3 install matplotlib
```

**解決**：
```bash
pip3 install matplotlib numpy
```

**注意**：即使沒有matplotlib，測試和CSV保存仍正常進行

### 問題2：無法找到AprilTag話題
**現象**：測試一直等待，沒有收集到樣本

**解決**：確認AprilTag檢測器正在運行
```bash
ros2 topic list | grep apriltag
ros2 topic echo /apriltag/tag4_corner0_3d --once
```

### 問題3：繪圖失敗
**現象**：
```
[ERROR] Failed to generate plots: ...
```

**解決**：
1. 檢查輸出目錄是否有寫入權限
2. 確認matplotlib版本兼容：`pip3 list | grep matplotlib`
3. 嘗試禁用繪圖：`-p auto_plot:=false`

### 問題4：CSV文件為空或不完整
**現象**：生成的CSV很小或沒有數據

**檢查**：
1. 樣本數量：`-p num_samples:=50` 確保有足夠樣本
2. Tag是否被檢測到：檢查ROS日誌輸出
3. 時間戳一致性：檢查 `stamp_spread_ms` 是否都是0

## 📈 性能說明

### 運行時間估算
- **測試階段**：約 `num_samples * sample_interval_sec` 秒
  - 例：50樣本 × 0.1秒 = 5秒
- **CSV保存**：< 0.1秒
- **繪圖階段**：
  - 完整圖：3-5秒
  - 簡化圖：1-2秒
  - 兩者都生成：5-7秒

### 文件大小
- **CSV**: 約3-4KB（50樣本）
- **完整圖**: 約440KB
- **簡化圖**: 約170KB

## 🔄 與舊工作流程對比

### 舊方式（分離）
```bash
# 步驟1：運行測試
ros2 run handeye_verify verify_handeye_touch -p save_samples_csv:=/tmp/test.csv

# 步驟2：生成圖表
python3 plot_repeatability_errors.py --csv /tmp/test.csv --output result.png
```

### 新方式（整合）✨
```bash
# 一步完成！
ros2 run handeye_verify verify_handeye_touch -p num_samples:=50
```

**優勢**：
- ✅ 減少50%的命令
- ✅ 自動管理文件路徑
- ✅ 統一的輸出目錄結構
- ✅ 更好的用戶體驗

## 🔙 向後兼容

### 舊工作流程仍可用
```bash
# 直接使用 ROS 2 節點收集資料，再用繪圖腳本分析
ros2 run handeye_verify verify_handeye_touch \
  --ros-args \
  -p test_mode:=repeatability \
  -p save_samples_csv:=/tmp/test.csv

python3 plot_repeatability_errors.py --csv /tmp/test.csv --output result.png
```

### 切換回舊行為
如果您想要舊的行為（只運行測試不繪圖）：
```bash
ros2 run handeye_verify verify_handeye_touch \
  --ros-args \
  -p test_mode:=repeatability \
  -p auto_plot:=false \
  -p save_samples_csv:=/tmp/my_test.csv
```

## 📝 總結

| 特性 | 舊版 | 新版（整合） |
|------|------|------------|
| 命令數量 | 2個 | 1個 ✨ |
| 自動繪圖 | ❌ | ✅ |
| 統一輸出目錄 | ❌ | ✅ |
| 缺少matplotlib | 失敗 | 繼續運行 ✨ |
| 文件管理 | 手動 | 自動 ✨ |

**推薦**：使用新的整合方式，更簡單、更快捷！
