# 鏡頭畸變校正

## 準備照片

將棋盤照片放進：

```text
calibration/lens/images
```

照片要求：

- 每張都是 `1280×720`。
- 棋盤為 `9×6` 個內角點。
- 建議拍攝 `25～30` 張，至少需要 `15` 張有效照片。
- 棋盤需出現在中央、四邊與四角，並包含不同距離和傾斜角度。
- 不要縮放、裁切、旋轉或使用數位變焦。
- 相機焦距、對焦與正式撞球辨識時保持一致。

支援 `.jpg`、`.jpeg`、`.png`、`.bmp`，不讀取子資料夾。

## 執行校正

在專案根目錄執行：

```powershell
.\.venv\Scripts\python.exe .\python\calibrate_intrinsics.py
```

程式會：

1. 偵測 `9×6` 棋盤內角點。
2. 計算相機矩陣及標準5參數畸變。
3. 最多進行3輪統計離群照片排除。
4. 保存每張照片的採用／排除報告和預覽圖。
5. 合格時安全更新 `camera_intrinsics_latest.yml`。
6. 自動開啟原始／去畸變即時比較畫面。

品質標準：

- RMS ≤ `0.5 px`：良好。
- `0.5～1.0 px`：警告，但仍可進行測試。
- RMS > `1.0 px`：不合格，不更新上一份合格參數。

如需暫時調整合格上限：

```powershell
.\.venv\Scripts\python.exe .\python\calibrate_intrinsics.py --max-rms 1.2
```

## 單獨開啟即時測試

```powershell
.\.venv\Scripts\python.exe .\python\test_lens_undistortion.py
```

操作鍵：

- `S`：保存原始、去畸變及並排比較圖。
- `Q` 或 `Esc`：離開。

所有參數、報告、預覽和即時截圖都會保存在：

```text
calibration/lens/results
```

這套程式只處理鏡頭內參與畸變，不會修改 Homography、球心射線平面、
Base0 座標或控制機械手臂。
