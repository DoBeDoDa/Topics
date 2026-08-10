# 撞球機器人專案目前狀態

> 更新日期：2026-08-10
> Repository：`camera_test` / `Topics`
> 目前分支：`main`
> 本文件用途：提供目前架構、已完成能力、安全邊界與待辦事項的快速快照。完整規格仍以 `AGENTS.md`、`docs/specs/` 與 active tickets 為準。

## 1. 專案目標

本專案整合相機影像、YOLO 球體辨識、Robot Base0 平面座標、撞球策略、HIWIN RA605-GC 運動規劃與受控執行流程。

系統採 fail-closed 原則：資料、校正、姿態、路徑或安全狀態不完整時，必須拒絕規劃或拒絕執行，不得使用猜測值、假座標或繞過安全 gate。

目前包含兩條明確分離的流程：

1. 正式主流程：Python 視覺輸出 Base0 平面座標，C++ 負責穩定狀態、球路規劃與執行整合。
2. 獨立實驗工具：Gemini 2 XL RGB-only 像素射線與 Base0 球心平面交點驗證；尚未整合進主程式。

## 2. 系統資料流

```text
USB camera
  → Python / YOLO
  → pixel-to-Base0 planar calibration
  → newline-delimited 32-value CSV
  → C++ strict parser
  → three-event StableTableState
  → TargetSelector
  → Algorithm + BilliardPhysics
  → ShotPlan / NoPlan
  → MotionPlanner
  → ExecutionPlan
  → fake/offline execution
  → RealHardware（僅限全部硬體 gate 通過後）
```

Python 傳入 C++ 的 32 個值已是 Robot Base0 平面 `X、Y` 毫米。C++ 不得再次執行 pixel conversion、Homography、相機補償、TableFrame→Base0 或第二次平面映射。

## 3. 主要目錄

| 路徑 | 目前責任 |
|---|---|
| `python/` | 相機、YOLO、偵測篩選、目前正式 pixel→Base0 平面轉換與 TCP server |
| `src/` | C++ parser、shot-cycle、穩定狀態、球路、運動規劃、HRSDK adapter 與應用整合 |
| `tests/` | Phase 1/2 C++ 離線測試與 Python 校正測試 |
| `tools/rgb_base0_calibration/` | Gemini 2 XL RGB-only 獨立校正與驗證工具 |
| `calibration/` | 鏡頭校正影像、結果與說明 |
| `docs/specs/` | 系統、外部契約、Shot Brain、Shot Executor 與 RGB-only 規格 |
| `docs/tickets/` | 分階段能力、依賴與驗證紀錄 |
| `docs/research/` | 官方資料研究與推論限制 |
| `History/`, `docs/archive/` | 歷史資料，不是目前規格來源 |

## 4. C++ 模組責任

| 模組 | 責任 |
|---|---|
| `BilliardConfig` | 集中保存可調參數、Tool/Base 與姿態設定 |
| `SocketClient` | production TCP 連線與 newline framing |
| `VisionDataParser` | 嚴格解析 32 值、finite 與 sentinel |
| `TableState` | shot-cycle 內三個 ReceiveEvent 的穩定生命週期 |
| `TargetSelector` | 唯一選擇最低號存在的 1～9 號合法目標球 |
| `Algorithm` | DirectPot/KickPot 候選、策略與共同評分 |
| `BilliardPhysics` | 撞球幾何與碰撞判定 |
| `MathUtils` | 純數學、旋轉矩陣與角度工具 |
| `MotionPlanner` | 由 ShotPlan 與核准設定建立 ExecutionPlan |
| `RobotController` | HRSDK 連線、Tool/Base、診斷、motion 與 DO adapter |
| `BilliardApp` | 唯一 application owner，整合視覺、規劃與執行狀態機 |

不得建立第二套同功能的 Socket、Parser、Algorithm、MotionPlanner、RobotController、BilliardApp、MathUtils 或 BilliardPhysics。

## 5. Python–C++ 外部契約

- 每筆資料必須恰有 32 個非空有限數值並以 newline 結束。
- Index `0–17`：1～9 號球 Base0 `X、Y`。
- Index `18–19`：母球 Base0 `X、Y`。
- Index `20–31`：六個袋口 Base0 `X、Y`。
- 精確的 `-9999.0,-9999.0` 表示物件不存在；單邊 sentinel 使整筆資料無效。
- V1 不增加 handshake、sender frame ID 或 timestamp。
- freshness 由既有 CameraPose settle、舊 buffer flush、shot-cycle gate 與累積 reset 管理。
- 一個 `StartRequested` 只允許一個完整 shot cycle；完成後回到 `WaitingForStart`。

## 6. 目前已實作能力

### 6.1 視覺與規劃

- Python 相機與 YOLO 推論流程。
- 32 值外部契約與嚴格 C++ parser。
- shot-cycle gate 與三事件穩定桌面狀態。
- 最低號合法目標球選擇。
- DirectPot 與一次母球碰庫 KickPot 候選。
- 碰撞、袋口、共同評分與 deterministic tie-break。
- 成功時產生 `ShotPlan`；沒有候選時回傳具名 `NoPlan`。

### 6.2 運動與安全架構

- `MotionPlanner`、ExecutionPlan 與 fake/offline execution 架構。
- HRSDK Tool/Base、診斷、motion 與 dual-DO adapter 程式碼。
- 真實執行仍受設定、校正、路徑與 controlled acceptance gate 限制。

檔案或測試存在不代表真實硬體已授權。工作區目前包含多項尚未提交的使用者修改，正式完成狀態仍須以 active ticket 與驗證紀錄判定。

## 7. Gemini 2 XL RGB-only 獨立工具

### 7.1 已實作

- 精確要求 Orbbec SDK v1.10.18。
- 只允許一台 Gemini 2 XL。
- 選擇 `1280x720 MJPG` 的最高可用 FPS，目前預期為 10 FPS。
- config 只啟用選定的 Color stream。
- Color-only pipeline 啟動後，先驗證 live Color frame。
- 透過 `Pipeline::getCameraParam()` 只取 `rgbIntrinsic` 與 `rgbDistortion`。
- 驗證 `fx/fy` finite 且為正、主點範圍、intrinsic 尺寸與八個 distortion coefficients。
- 保存 live profile、RGB K/D、序號、韌體與 SDK 版本。
- 使用 `rgb_brown_rational_v1` 反解；最多 50 次、tolerance `1e-12`、forward residual ≤`0.25 px`。
- 使用球直徑 `44.5 mm`、半徑 `22.25 mm`、`Z_table=-233.51 mm` 與 `Z_target=-211.26 mm`。
- 不啟用 Depth、D2C、matched profile、Depth extrinsic 或 synthetic depth。
- 不包含 robot motion、motor、alarm-clear 或 DO 命令。
- 所有輸出維持 `experimental=true` 與 `authorized_for_robot_motion=false`。

### 7.2 尚待驗收

- 尚未確認 live Gemini 2 XL 的新 K/D 擷取結果。
- 尚未完成至少六個 Base0 ground-truth 球心點驗證。
- 尚未驗收 XY RMS ≤3 mm、每點 ≤5 mm 與誤差趨勢門檻。
- 袋口 Base0 計算尚未實作。
- 尚未授權整合主程式。

即使 RGB K/D 成功取得，也不代表 Base0 ground-truth 或硬體驗收完成。

## 8. 建置與最小測試

### 8.1 主程式

主程式需要 Windows x64、Visual Studio C++17、相符版本的 HRSDK header/library/runtime，以及 Python `.venv`、OpenCV、NumPy、Ultralytics/PyTorch 與 `bin/best.pt`。

一般啟動順序：

```powershell
.\.venv\Scripts\Activate.ps1
python .\python\robot.py
```

另一個終端再啟動已建置的 C++ 主程式：

```powershell
.\bin\main.exe
```

目前真實硬體必要設定仍不完整；遇到 fail-closed 時不得填入猜測值或繞過 gate。

### 8.2 RGB-only 工具

在 Visual Studio x64 環境：

```powershell
cmake -S tools/rgb_base0_calibration -B build/rgb_base0_calibration -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/rgb_base0_calibration --parallel
ctest --test-dir build/rgb_base0_calibration --output-on-failure
```

最近一次針對 RGB K/D 修改的最小驗證結果：

- `rgb_base0_calibrate` 與 `rgb_base0_validate` 建置成功。
- `rgb_base0_geometry_tests`：1/1 通過。
- production source 未發現 Depth/D2C/full calibration helper 路徑。
- 未執行 live 相機或真實機械手臂流程。

## 9. 已確認安全原則

- 手臂型號：HIWIN RA605-GC。
- 正式擊球：Tool1 / Base0；Tool1 TCP 是球桿尖端。
- RGB-only 實驗：Tool2 / Base0。
- `ptp_pos()` 第二參數是 motion mode，不是 Tool 編號。
- `motion_reachable()` 只表示目標姿態可達，不代表完整 PTP path 安全。
- LIN path 必須另用 `motion_check_lin()`。
- XYZ 不變而改變 A/B/C，仍可能造成法蘭與關節大幅移動。
- 未確認座標方向、旋轉與實機路徑前，只允許計算與診斷。
- 擊球後第一段 motion 必須使用重新讀取的 actual pose，保持 X/Y/A/B/C 不變，沿已確認安全的 Base0 `+Z` 做垂直 LIN safe lift。
- 氣動狀態未知時不得移動。

## 10. 禁止自行假設的項目

- `a、b、c` 的正式旋轉順序。
- A/B 人工校正基準、核准範圍與 step。
- HIWIN A/B/C→RX/RY/RZ 的正式順序、範圍與奇異點處理。
- Tool1 實體球桿 forward axis。
- Base0 `+Z` 是否為實體安全上方。
- 鏡頭畸變校正的未核准擴充範圍。
- `motion_reachable()` 失敗時的未核准替代策略。

## 11. Skill 使用政策

專案中的 TDD 與 Diagnosing Bugs Skill 均保留，但禁止隱式啟用：

```yaml
policy:
  allow_implicit_invocation: false
```

只有使用者明確呼叫 `$tdd` 或 `$diagnosing-bugs` 時才可使用。一般修改採最短合理流程：檢查直接相關程式、聚焦修改、建置受影響 target、執行最少必要既有測試。

## 12. 下一步

1. 在完全靜止的 Tool2/Base0 拍照姿態執行 RGB-only 校正。
2. 核對 live `1280x720 MJPG` profile、RGB K/D 與輸出診斷。
3. 使用至少六個 Base0 ground-truth 球心點完成驗收。
4. 球點通過後，再由使用者核准袋口 Base0 規格。
5. 袋口與整體座標通過後，另行取得主程式整合授權。
6. 真實擊球前完成 Phase 2 姿態、path、safe lift、DO 與 Base0 `+Z` controlled acceptance。

## 13. 權威文件

- `AGENTS.md`：專案協作與安全規則。
- `CONTEXT.md`：統一領域術語。
- `docs/project-overview.md`：完整專案導覽。
- `docs/specs/billiards-system-refactor-master-spec.md`：系統主規格。
- `docs/specs/python-cpp-external-contract.md`：Python–C++ 外部契約。
- `docs/specs/phase-1-shot-brain-spec.md`：Phase 1 規格。
- `docs/specs/phase-2-shot-executor-spec.md`：Phase 2 規格。
- `docs/specs/rgb-only-calibration-spec.md`：RGB-only 校正規格。
- `tools/rgb_base0_calibration/README.md`：獨立工具操作說明。
