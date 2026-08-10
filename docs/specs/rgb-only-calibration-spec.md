# Gemini 2 XL RGB-only → Base0 Calibration Specification

## 文件資訊

- 狀態：Software implemented; hardware acceptance pending
- 核准日期：2026-08-10
- 適用範圍：`tools/rgb_base0_calibration`
- SDK：Orbbec SDK v1.10.18（必須精確匹配）
- 安全等級：獨立實驗、唯讀手臂姿態、不得控制機械手臂移動或 DO
- 現況：RGB-only 軟體修改、離線測試與建置已完成；尚未執行 live camera/robot pose capture，也尚未通過至少六點 ground truth

本文件是 RGB-only 修改的實作與驗收規格。軟體完成不等於實機驗收完成；仍須通過 live profile/K/D 擷取與本文件的 ground truth gate，才可討論整合主程式。

## 1. 目標

使用 Gemini 2 XL 的原始 RGB 像素、該 RGB profile 的原廠內參／畸變、已靜止的 HIWIN Tool2/Base0 pose 與球心平面，計算球心在 Base0 的三維座標。整個流程不取得 Depth profile、不啟用 Depth stream，也不以量測深度決定射線尺度。

目標流程：

```text
YOLO bbox center (u,v)
→ RGB Brown 反畸變
→ RGB optical unit ray
→ Tool2 / Base0 rotation
→ 與 Base0 球心平面相交
→ ball center (X,Y,Z) in Base0
```

## 2. 已確認輸入

| 項目 | 值／規則 |
|---|---|
| 相機 | 只連接一台 Orbbec Gemini 2 XL |
| Color profile | `1280x720 MJPG`，選相機實際提供的最高 FPS；目前實機為 10 FPS |
| RGB optical axes | `+X` 影像右、`+Y` 影像下、`+Z` 朝前 |
| Orbbec SDK | `1.10.18`，預設 `C:\Program Files\OrbbecSDK 1.10.18\SDK` |
| Robot | HIWIN RA605-GC，控制器預設 `192.168.0.1` |
| 校正 Tool/Base | Tool2 / Base0 |
| Tool2→RGB translation | `[0,0,0] mm`，依使用者目前實體設定 |
| RGB→Tool2 rotation | identity，依使用者目前實體軸對齊設定 |
| Tool2 ABC 暫定轉換 | degree；active column vector；`R=Rz(C)Ry(B)Rx(A)` |
| 桌布平面 | `Z_table=-233.51 mm` |
| 球直徑／半徑 | `44.5 mm`／`22.25 mm` |
| 球心平面 | `Z_target=-211.26 mm` |
| YOLO confidence | 預設 `0.3` |

HIWIN ABC 公式是使用者核准的暫定 Z-Y-X 規則，不是目前公開 HIWIN 文件已證實的唯一官方公式。所有輸出必須保留 `rotation_convention_source=user_approved_temporary`。

## 3. RGB-only 邊界

### 3.1 必須保留

- 精確檢查 Orbbec SDK runtime version 為 `1.10.18`。
- 列舉裝置並要求恰好一台 Gemini 2 XL。
- 從 Color stream profile list 精確選擇 `1280x720 MJPG` 的最高 FPS。
- 從同一個已選定的 `VideoStreamProfile` 讀取 `getIntrinsic()` 與 `getDistortion()`。
- 啟用並擷取 Color stream，保存未 resize、crop、flip、rotate 或 letterbox 的原始 MJPG frame。
- 保存 serial number、firmware、SDK version、profile、K、D 與座標軸定義。
- 讀取完全靜止的 Tool2/Base0 pose，前後取樣並確認姿態不變。
- YOLO 僅提供原始影像上的 bbox center `(u,v)`；幾何計算維持在 C++。

### 3.2 必須移除

- Depth profile 列舉與選擇。
- `config->enableStream(depthProfile)`。
- D2C hardware/software alignment mode。
- `getD2CDepthProfileList(...)` 與任何 matched Color/Depth profile 要求。
- `Pipeline::getCalibrationParam(config)`。
- 為取得 RGB 射線而保存或合成 `OBCalibrationParam`。
- `CoordinateTransformHelper::calibration2dTo3dUndistortion`、`calibration3dTo2d` 與 `transformationInitXYTables`。
- depth=1／1000 的 SDK helper 比例檢查，以及任何 Depth extrinsic、Depth frame 或深度單位。

移除上述項目不表示刪除相機內參、畸變、Tool2 pose、RGB→Base0 外參或平面交點；這些仍是 RGB-only 幾何的必要資料。

## 4. 校正資料契約

校正檔應升版，並至少保存下列欄位：

### 4.1 裝置與 profile

- schema version、建立時間、`experimental=true`、`authorized_for_robot_motion=false`
- SDK version、camera model、device name、serial number、firmware version
- Color width、height、FPS、format
- camera frame name 與 optical axes

### 4.2 RGB 內參與畸變

- `fx, fy, cx, cy`
- Orbbec 欄位原順序：`k1,k2,k3,k4,k5,k6,p1,p2`
- `distortion_family=orbbec_brown`
- profile API 未暴露的 distortion variant 必須明記為未知，不得假裝已由 enum 證實
- inverse projection 的演算法名稱、版本、收斂門檻與最大迭代次數

不得把 Orbbec 欄位順序直接當成 OpenCV 8-vector 順序，也不得使用 `cv::fisheye`。若實作採用 Brown-Conrady/rational 方程，必須在程式、校正檔與測試 fixture 明確記錄係數映射；該映射在獨立官方證據不足時只能視為待實機驗證的工程假設。

### 4.3 Robot 與外參

- robot model/IP、Tool2、Base0
- Tool2 的原始 X、Y、Z、A、B、C 與角度單位
- 姿態取樣數、取樣窗口、XYZ/ABC spread 門檻
- 暫定旋轉公式及其來源標記
- `R_Base0_from_Tool2`、`R_Tool2_from_RGB`、`R_Base0_from_RGB`
- `t_Tool2_to_RGB` 與 `C_Base0`
- 每個 rotation matrix 的正交誤差與 determinant 檢查結果

### 4.4 桌面與球

- `Z_table`、球直徑、球半徑、`Z_target=Z_table+ball_radius`
- translation unit 為 mm，table plane model 為 constant Base0 Z

## 5. 像素射線演算法

### 5.1 輸入限制

- `(u,v)` 必須有限且落在原始 Color frame 有效範圍。
- 像素必須對應儲存 profile 的原始影像；經 resize/crop/letterbox 的座標必須先由 YOLO 還原到原圖，否則拒絕。
- live camera 的 serial/profile/K/D 必須與校正檔一致，否則拒絕。

### 5.2 反畸變

由 distorted pixel 先轉成畸變正規化座標，再以明確版本化的 Brown inverse solver 求 undistorted `(x,y)`。solver 必須：

- 使用 double precision。
- 有最大迭代次數與收斂門檻，不得無限迭代。
- 每步拒絕 NaN、Infinity、近零分母或不合理發散。
- 未收斂即拒絕該點，不回傳 pinhole fallback。
- 以同一已記錄的 forward model 回投影，要求 residual ≤ `0.25 px`。

forward round-trip 只能證明 solver 與其自身模型一致，不能單獨證明該模型就是相機實際畸變；最終系統正確性仍由 ground truth gate 決定。

### 5.3 單位射線與 Base0 交點

```text
q_RGB = [x, y, 1]
d_RGB = q_RGB / ||q_RGB||
d_Base0 = R_Base0_from_RGB * d_RGB
lambda = (Z_target - C_Base0.z) / d_Base0.z
P_Base0 = C_Base0 + lambda * d_Base0
```

必須驗證：

- `d_RGB.z > 0` 且單位向量 norm 誤差在核准門檻內。
- `R_Base0_from_RGB` 有限、正交且 determinant 接近 `+1`。
- `d_Base0.z` 不接近零。
- `lambda` 有限且 `>0`。
- 交點 Z 與 `Z_target` 在數值容差內。

本流程不需要球到鏡頭的量測距離；`lambda` 是由已知球心平面求得的幾何距離。

## 6. YOLO 觀測規則

- Python 只執行現有 Ultralytics YOLO，不控制 Orbbec SDK、不做射線或 Base0 轉換。
- 每張影像每個球類別最多接受一顆；同類別多個偵測只取 confidence 最高者，其他完整記錄為 duplicate。
- 袋口每張影像最多六個；超過六個必須警告並保留診斷。
- 自動模式擷取 10 張，球類別至少出現 8 張；中心先取 median，移除距離超過 5 px 的 observation，保留至少 8 個後再取 final median，median radial distance 必須 ≤2 px。
- 第一版只計算球的 Base0 點。球點確認正確後，必須提醒使用者另行核准與實作袋口 Base0 計算。

## 7. 驗證與驗收

### 7.1 軟體測試

- profile selection：無 profile、錯誤格式、多相機、非 Gemini 2 XL 均 fail closed。
- calibration I/O：JSON/YAML round-trip、缺欄、重複欄、錯型別、非有限值、舊 schema。
- Brown solver：中心、四角、邊界、YOLO subpixel、強畸變、未收斂、近零分母。
- geometry：rotation matrix、前向射線、平行平面、負 lambda、球心平面。
- live-match：serial/profile/K/D 任一不符即拒絕。
- 禁用搜尋：production source 不得再出現 Depth/D2C/full-calibration helper 路徑。

### 7.2 實機 gate

- 中心像素射線應接近 RGB `+Z`；四角方向需符合 `+X` 右、`+Y` 下。
- 至少六個分布於桌面的 ground-truth 球心點。
- XY RMS error ≤3 mm。
- 每點 XY error ≤5 mm。
- `error_x/error_y` 對 `u/v` 的四個 Pearson correlation 絕對值皆 <0.7。
- Tool2/RGB 原點與軸對齊、暫定 Z-Y-X 旋轉的正負方向必須由實體點位共同驗證。

`0.25 px`、3 mm、5 mm 與 0.7 是本專案核准的工程門檻，不是 Orbbec 或 HIWIN 官方規格。未通過時仍輸出完整診斷，但不得標示成功、不得整合主程式。

## 8. 安全與錯誤處理

- 校正與驗證程式只允許 `set_tool_number(2)`、`set_base_number(0)`、讀取 motion state/pose，並在結束時復原原 Tool/Base。
- 不得包含 motion、motor、alarm-clear 或 DO 命令。
- 手臂必須先由操作者移到拍照姿態並完全靜止。
- 連線、相機、檔案或驗證失敗時必須嘗試安全清理；清理失敗要合併回報，不得把失敗描述成成功。
- 不覆寫既有非空輸出目錄或既有校正檔。
- 所有產物維持 `experimental` 與 `authorized_for_robot_motion=false`。

## 9. 完成定義

只有下列項目全部成立，RGB-only 修改才算完成：

1. Depth/D2C/full `OBCalibrationParam` 相依已從獨立工具移除。
2. 精確 RGB profile 的 K/D 可在實機穩定讀取並寫入校正檔。
3. Brown inverse solver 與所有 fail-closed 測試通過。
4. 校正與自動／手動驗證流程可建置執行。
5. 至少六點實機 ground truth 通過全部門檻。
6. 輸出仍禁止機械手臂動作。

目前第 1、3、4、6 項已完成；第 2 項仍待 live Gemini 2 XL 確認，第 5 項仍待使用者提供並量測實機 ground truth，因此整體完成定義尚未達成。

主程式整合與袋口 Base0 計算都不屬於本修改。球點通過後，下一步必須先提醒使用者核准袋口規格；主程式整合必須再次取得明確同意。
