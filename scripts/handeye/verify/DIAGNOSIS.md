# Hand-Eye 校正診斷報告

## 問題概述
校正結果糟糕，機器人無法準確觸碰 AprilTag corner0。

## 分析結果

### 1. ✅ 校正矩陣結構正常
- `T_BC` 矩陣的數學結構是有效的
- 行列式 = 1.0（正確的旋轉矩陣）
- 正交性誤差 < 1e-9（非常好）
- 矩陣本身沒有數值問題

### 2. ⚠️ 座標系配置（eye-to-hand）

#### Solver 公式（來自 handeye_solver.cpp 第 767-768 行）：
```
AX = ZB
- A = inv(BE_i) * BE_j   # 末端執行器的相對運動
- B = inv(CO_i) * CO_j   # 物體在相機座標系中的相對運動
- X = T_E_O              # 末端執行器 -> 物體（先求解）
- Z = T_B_C              # 基座 -> 相機（最終結果）
```

這是 **eye-to-hand** 配置（相機固定在環境中）。

#### 當前驗證程式的使用方式：
```python
# verify_tag4_simple.py 第 248 行
pB = (self.T_BC @ pC_h)[:3]  # 用 T_BC 將相機座標轉到基座座標
```

這個使用方式是**正確的**，因為 `T_BC` 表示「基座到相機」的變換。

### 3. 🔍 可能的問題根源

#### A. 座標系方向定義不一致
檢查以下定義是否一致：

1. **機器人 TCP 座標系**
   - CSV 中的 `B_tcp_{x,y,z,rx,ry,rz}` 是哪個座標系？
   - 是 flange 座標系還是已經包含了 TCP offset？

2. **相機光學座標系**
   - ROS 標準：X右, Y下, Z前（朝向場景）
   - 你的 Kinect 發布的座標是否符合這個定義？
   - 檢查 `camera_color_optical_frame` 的定義

3. **AprilTag 物體座標系**
   - Object 原點在哪裡？（Tag 中心？corner0？）
   - Object Z 軸方向？（垂直 tag 平面向外？）

#### B. 探針補償問題
```python
# 第 389 行
flange_touch = corner0_B_corr - tcp_z_B * probe_length
```

**關鍵問題**：
- `probe_length = 0.18895 m` （約 189mm）
- 這個長度是從 **TCP 原點** 到探針尖端嗎？
- TCP 座標系的 Z 軸是指向探針方向嗎？
- 如果探針沿著 -Z 方向，`probe_length` 應該是負數！

#### C. 校正數據收集問題

從 `handeye_samples.csv` 看到：
- 有 20 組數據
- TCP 位置範圍：
  - X: 0.02 ~ 0.17 m
  - Y: -0.64 ~ -0.46 m
  - Z: 0.36 ~ 0.48 m

**可能的問題**：
1. **運動範圍太小**？校正需要大範圍的旋轉和平移
2. **數據集中在一個小區域**？應該覆蓋整個工作空間
3. **旋轉不夠多樣化**？需要各種不同的姿態

#### D. CSV 數據格式問題

確認以下欄位的定義：
```
B_tcp_x, B_tcp_y, B_tcp_z        # TCP 位置在基座座標系中
B_tcp_rx, B_tcp_ry, B_tcp_rz     # TCP 旋轉（axis-angle）
C_p_O_x, C_p_O_y, C_p_O_z        # 物體位置在相機座標系中
C_q_O_x, C_q_O_y, C_q_O_z, C_q_O_w  # 物體姿態（四元數 xyzw）
```

**檢查**：
- `C_p_O` 是 Tag 中心還是 corner0？
- 如果是 Tag 中心，但驗證用 corner0，會有偏差！

## 🔧 建議的診斷步驟

### 步驟 1：驗證座標系定義
創建一個簡單的測試程式：
1. 讓機器人移動到已知位置（例如 X=0.3, Y=-0.5, Z=0.4）
2. 用相機觀察一個靜止的 AprilTag
3. 用 T_BC 變換相機看到的位置到基座座標系
4. 比較計算結果和已知的 Tag 位置（用尺子測量）
5. 誤差應該 < 10mm

### 步驟 2：檢查探針長度
1. 實際測量從 TCP 座標系原點到探針尖端的距離
2. 確認 TCP Z 軸方向
3. 如果 Z 軸指向手臂內側，探針沿 +Z，則 `probe_length` 為正
4. 如果探針沿 -Z，則 `probe_length` 應為負

### 步驟 3：重新收集校正數據
使用改進的數據收集策略：
- 至少 30-50 組數據
- 覆蓋工作空間的不同區域
- 包含多樣化的旋轉（至少 30° 以上的相對旋轉）
- 相機和 Tag 的距離變化要大（例如 0.4m ~ 1.0m）

### 步驟 4：檢查 Tag 姿態定義
確認：
- Tag localizer 輸出的姿態是 **Tag 中心**
- 但 `tag4_corner0_3d_node` 輸出的是 **corner0 位置**
- 這兩者不能混用！

## 🎯 快速測試方案

運行以下命令創建診斷腳本：
```bash
cd /home/Victor/ros2_ws/scripts/handeye/verify
python3 test_calibration_simple.py
```

這個腳本會：
1. 讀取當前的 T_BC
2. 模擬一個簡單的轉換測試
3. 輸出預期結果 vs 實際結果的比較
