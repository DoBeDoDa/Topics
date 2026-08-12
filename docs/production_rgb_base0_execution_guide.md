# Production RGB→Base0 校正、YOLO 與 C++ 啟動指南

更新日期：2026-08-12

## 1. 固定校正檔

Production Python 只會載入：

```text
C:\Users\10003\OneDrive\Desktop\camera_test\config\vision\camera_calibration.json
```

程式不會搜尋 timestamp 歷史目錄，也不會回退到 Homography。一般啟動不必每次重新校正；只有相機／Tool3 安裝、相機 profile、桌面 Base0 Z 或 Tool3→RGB 外參改變，或 validation 失敗時才重新校正。

## 2. 安全限制

- `rgb_base0_calibrate.exe` 與 `rgb_base0_validate.exe` 只讀取 robot pose，不包含 robot motion、motor、alarm reset 或 DO 命令。
- `bin\main.exe` 是 production 主程式，可能進入機械手臂執行流程，不是單純顯示 TCP 資料的 viewer。
- 校正 JSON 保持 `experimental=true` 與 `authorized_for_robot_motion=false`。可供 production 視覺座標計算使用，不代表已授權機械手臂運動。
- Tool1／Base0、Strike Z、A/B/C、球桿 forward axis、Base0 `+Z` 與 DO 尚未完成受控實機驗收前，不得進行真實擊球。

## 3. 建置

請使用 Developer PowerShell for VS 2022：

```powershell
cd C:\Users\10003\OneDrive\Desktop\camera_test

cmake --build .\build\rgb_base0_calibration `
  --target rgb_base0_calibrate rgb_base0_validate rgb_base0_geometry_bridge `
  --config Release
```

幾何橋接應位於：

```text
build\rgb_base0_calibration\rgb_base0_geometry_bridge.dll
```

正式 C++ 主程式可用 VS Code task `Build with HRSDK (MSVC)` 建置。建置後確認 `bin\main.exe` 的時間確實更新。

## 4. 已有校正時先執行 validation

關閉其他會占用 Gemini 相機的程式，確認機械手臂完全停止，再執行：

```powershell
.\build\rgb_base0_calibration\rgb_base0_validate.exe `
  --calibration .\config\vision\camera_calibration.json
```

Validation 會檢查 Tool3／Base0、robot pose 穩定性、Gemini RGB profile、K/D、Brown inverse、RGB ray、Base0 平面交點及 YOLO sidecar。失敗時不要啟動 production，先查看最新輸出目錄的：

```text
terminal_log.txt
yolo_terminal_log.txt
raw_frames\frame_00.jpg ... frame_09.jpg
```

## 5. 必須重新校正時

只有在 Tool3→RGB 參數與桌面 Base0 Z 已確認後才執行：

```powershell
.\build\rgb_base0_calibration\rgb_base0_calibrate.exe `
  --z-table -233.510000 `
  --robot-ip 192.168.0.1
```

若 Tool3→RGB 不是零位移／單位旋轉，必須提供實際量測值，不可猜測：

```powershell
.\build\rgb_base0_calibration\rgb_base0_calibrate.exe `
  --z-table -233.510000 `
  --robot-ip 192.168.0.1 `
  --tool3-to-rgb-mm "x,y,z" `
  --r-tool3-from-rgb "r11,r12,r13,r21,r22,r23,r31,r32,r33"
```

校正成功後仍須重新執行 validation。

## 6. 啟動 production YOLO

第一個 PowerShell 視窗：

```powershell
cd C:\Users\10003\OneDrive\Desktop\camera_test
python .\python\robot.py
```

正常流程會載入固定 calibration、由 C++ bridge 嚴格驗證、載入 `bin\best.pt`、開啟 `1280×720 @ 10 FPS MJPG`，最後顯示：

```text
[TCP] Waiting for the existing C++ client on port 12345...
```

OpenCV 無法確認 Gemini serial number；若同時連接多台相同 profile 的相機，仍須人工確認 camera index 0。

## 7. 另一個視窗

在機械手臂安全條件與 production 執行授權均完成後，第二個視窗才可執行：

```powershell
cd C:\Users\10003\OneDrive\Desktop\camera_test
.\bin\main.exe
```

正確順序是先開 `robot.py`，再開 `main.exe`。目前沒有已確認完全不碰硬體、只顯示 32 值的獨立 receiver，因此不要把 `main.exe` 當成 YOLO viewer。

## 8. 單張影像與 32 值格式

每個 capture event 只讀一張 RGB image、執行一次 YOLO。同一球類別只保留最高 confidence；袋口少於六個時拒絕整張，多於六個時先取 confidence 最高六個。袋口在 raw pixel space 排序後，球與袋口都呼叫相同的 C++ RGB→Base0 geometry，最後嚴格驗證並傳送一行 newline-delimited CSV。

```text
0..17   Ball_1 ... Ball_9 的 X,Y
18,19   Ball_cue X,Y
20..31  P1 ... P6 的 X,Y
```

未辨識到的球使用完整 pair：

```text
-9999.0,-9999.0
```

不得出現單邊 sentinel、NaN、Inf、欄位數不是 32，或重送上一張影像的舊座標。Python 送入 C++ 的值已是 Robot Base0 XY mm；C++ 不得再做 pixel、Homography 或第二次平面轉換。

## 9. `ConfigurationMissing`

目前核准的 production 設定為：

```text
VISION_MAX_FRAME_BYTES = 1024
VISION_RECEIVE_TIMEOUT_MS = 2000
VISION_OBSERVATION_BOUNDS = X[-750,750], Y[150,1000]
```

若仍看到 P1-03 `ConfigurationMissing`，最常見原因是執行了舊版 `bin\main.exe`。重新建置並比較執行檔與 `src\BilliardConfig.cpp` 的時間；不可透過關閉 parser 或把安全設定改成零繞過錯誤。

## 10. 驗證狀態與實機待辦

已離線確認固定 calibration 嚴格載入、C++ Brown/ray/plane projection、15 項 production Python 測試、32 欄／finite／sentinel／袋口／newline gate，以及 production C++ 主程式完整連結。

仍需實機確認 camera index 0、YOLO 真實準確度、六袋穩定性、ground-truth Base0 XY 誤差，以及所有機械手臂與 DO 安全條件。
