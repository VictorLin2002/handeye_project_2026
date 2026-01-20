# AprilTag 點品質過濾系統

## 🎯 三層過濾機制

### 1️⃣ AprilTag 偵測品質過濾 (decisionMargin)
**參數**: `min_decision_margin` (預設: 30.0)

- **作用**: 過濾整個標籤（5個點全部丟棄）
- **指標**: AprilTag 庫提供的 `decisionMargin`，反映邊緣清晰度和對比度
- **建議值**:
  - `0.0` - 關閉（接受所有偵測）
  - `30.0` - 標準（推薦初始值）
  - `50.0` - 嚴格（高品質環境）
  - `100.0` - 極嚴格（只要最清晰的標籤）

**調整時機**: 
- 標籤被遮擋或模糊時增加此值
- 拒絕率太高時降低此值

### 2️⃣ 深度變異度過濾 (depth variance)
**參數**: `depth_variance_threshold_mm` (預設: 50.0 mm)

- **作用**: 過濾單個點（其他點仍可用）
- **指標**: 深度取樣視窗內的標準差
- **建議值**:
  - `0.0` - 關閉
  - `30.0` - 嚴格（平坦表面）
  - `50.0` - 標準（推薦）
  - `100.0` - 寬鬆（允許曲面或邊緣點）

**調整時機**:
- 十二面體邊緣角點常被拒絕 → 增加閾值
- RMSE 仍太高 → 降低閾值收緊過濾

### 3️⃣ 最少標籤數要求 (minimum tag coverage)
**參數**: `min_tag_coverage` (預設: 3)

- **作用**: 確保姿態估計有足夠約束
- **建議值**:
  - 10標籤系統: `3-5` (30%-50% 覆蓋率)
  - 4標籤系統: `2-3` (50%-75% 覆蓋率)

**調整時機**:
- 部分視角看不到足夠標籤 → 降低此值
- 想確保高品質 → 提高此值

---

## 📊 配置範例

### 場景 1: 手眼校正（初期寬鬆）
```yaml
tag_localizer_node:
  ros__parameters:
    min_decision_margin: 20.0           # 寬鬆接受
    depth_variance_threshold_mm: 100.0  # 允許曲面
    min_tag_coverage: 3                 # 至少3個標籤
    max_rmse_threshold_mm: 100.0        # 寬鬆RMSE
```

### 場景 2: 生產追蹤（中等嚴格）
```yaml
tag_localizer_node:
  ros__parameters:
    min_decision_margin: 40.0           # 中等品質
    depth_variance_threshold_mm: 50.0   # 標準
    min_tag_coverage: 4                 # 至少4個標籤
    max_rmse_threshold_mm: 50.0         # 標準RMSE
```

### 場景 3: 高精度測量（極嚴格）
```yaml
tag_localizer_node:
  ros__parameters:
    min_decision_margin: 60.0           # 只要最清晰的
    depth_variance_threshold_mm: 30.0   # 嚴格深度一致性
    min_tag_coverage: 5                 # 至少5個標籤
    max_rmse_threshold_mm: 30.0         # 嚴格RMSE
```

---

## 🔍 診斷與調試

### 查看過濾統計
```bash
ros2 topic echo /rosout | grep -E "rejected|coverage"
```

會看到類似訊息：
```
Tag 3 rejected: decisionMargin=25.3 < 30.0
Tag 7 center rejected: depth_variance=65.2 mm > 50.0 mm
Insufficient tag coverage: 2 < 3 (rejected: margin=3, variance=8)
```

### 即時調整參數
```bash
# 放寬 decisionMargin
ros2 param set /tag_localizer_node min_decision_margin 20.0

# 放寬深度變異度
ros2 param set /tag_localizer_node depth_variance_threshold_mm 80.0

# 降低最少標籤要求
ros2 param set /tag_localizer_node min_tag_coverage 2
```

### 關閉所有點過濾（只保留RMSE過濾）
```bash
ros2 param set /tag_localizer_node min_decision_margin 0.0
ros2 param set /tag_localizer_node depth_variance_threshold_mm 0.0
ros2 param set /tag_localizer_node min_tag_coverage 1
```

---

## 🎓 調參策略

### 步驟 1: 觀察基準
1. 先關閉所有點過濾（設為 0）
2. 運行系統，記錄:
   - RMSE 範圍
   - accepted/rejected 比例
   - 偵測到的標籤數

### 步驟 2: 啟用標籤品質過濾
1. 設定 `min_decision_margin: 30.0`
2. 觀察有多少標籤被拒絕
3. 如果 >30% 被拒絕 → 降到 20.0
4. 如果 RMSE 仍高 → 提高到 50.0

### 步驟 3: 啟用深度變異度過濾
1. 設定 `depth_variance_threshold_mm: 50.0`
2. 觀察有多少點被拒絕
3. 十二面體邊緣較多可能需要 80-100

### 步驟 4: 設定最少標籤
1. 根據實際能看到的平均標籤數設定
2. 推薦設為平均值的 60%
3. 例如平均看到 6 個 → 設為 `min_tag_coverage: 4`

---

## 🚨 常見問題

### Q: 所有姿態都被拒絕？
**A**: 降低所有閾值：
```bash
ros2 param set /tag_localizer_node min_decision_margin 10.0
ros2 param set /tag_localizer_node depth_variance_threshold_mm 100.0
ros2 param set /tag_localizer_node min_tag_coverage 2
ros2 param set /tag_localizer_node max_rmse_threshold_mm 150.0
```

### Q: RMSE 還是太高？
**A**: 逐步收緊過濾：
1. 先提高 `min_decision_margin` 到 50-60
2. 再降低 `depth_variance_threshold_mm` 到 30-40
3. 最後提高 `min_tag_coverage`

### Q: 某些視角完全無輸出？
**A**: 
- 降低 `min_tag_coverage`（可能那個角度看不到足夠標籤）
- 或增加十二面體上的標籤數量

---

## 📈 效能影響

- **decisionMargin 過濾**: 幾乎無開銷（只是比較）
- **depth_variance 計算**: 每點約 +5% 計算時間
- **整體**: 因為拒絕低品質點，SVD 收斂更快，可能反而變快

建議在 Release 模式編譯以獲得最佳效能。
