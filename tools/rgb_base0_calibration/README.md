# Gemini 2 XL RGB → HIWIN Base0 獨立校正與驗證工具

這個目錄是獨立實驗工具，不會接入主程式，也不會把結果送給機械手臂。它只做兩件事：

1. `rgb_base0_calibrate`：讀取已靜止的 HIWIN Tool3／Base0 姿態與 Gemini 2 XL RGB 內參，建立等價的 YAML、JSON 校正檔。
2. `rgb_base0_validate`：把原始 RGB 像素用版本化 Brown inverse solver 反投影成光線，再與 Base0 水平球心平面相交，輸出實驗點位與完整診斷。

## 必要前提與安全限制

- Windows、Orbbec SDK **恰為 v1.10.18**，預設安裝位置：`C:\Program Files\OrbbecSDK 1.10.18\SDK`。
- 只連接一台 Orbbec Gemini 2 XL。
- Gemini 2 XL 原生資料連線是 USB 2.0；即使插入藍色 USB 3.x 連接埠，SDK 顯示 `USB2.0` 仍屬正常。
- 相機必須提供 `1280x720 MJPG`；程式會選該格式實際可用的最高 FPS，並把 FPS 寫入校正檔。使用者實機目前回報最高為 10 FPS。沒有符合 profile 時直接停止並列出 SDK 回報的 profiles。
- 程式只啟用選定的 Color profile，先啟動 Color-only pipeline 並確認第一張 live frame 是選定的 `1280x720 MJPG`，再從 Orbbec SDK v1.10.18 `Pipeline::getCameraParam()` 回傳值中只取 `rgbIntrinsic` 與 `rgbDistortion`。它不查詢或啟用 Depth、不做 D2C，也不呼叫 `getCalibrationParam()`。
- RGB K/D 必須與實際 live Color frame 尺寸一致；焦距、主點與全部畸變係數通過有限值和範圍檢查後才會寫入校正檔，否則直接停止。校正檔同時保存經 live frame 驗證的 Color profile、相機序號、韌體與 SDK 版本。
- HIWIN RA605-GC 控制器預設 IP `192.168.0.1`；操作者要先把手臂放在固定拍攝姿態並完全停止。
- 控制器中的 Tool3 必須已由操作者設定好，而且 Tool3 原點就是 RGB optical center、Tool3 軸與 RGB optical 軸對齊。
- 程式只暫時呼叫 `set_tool_number(3)`、`set_base_number(0)` 與讀取狀態／姿態函式。程式碼中沒有移動、馬達、清除警報或 DO 命令。
- 每次都保存原 Tool/Base，完成後回復並關閉連線。姿態在相機前後都取 3 次、約 500 ms，XYZ spread 上限 0.1 mm、ABC 包角 spread 上限 0.05°。
- 所有結果均標記 `experimental`、`authorized_for_robot_motion=false`，禁止餵入主程式或真實移動流程。

## 已確認與暫定的座標定義

- Gemini RGB optical frame：`+X` 向影像右、`+Y` 向影像下、`+Z` 朝鏡頭前方場景，為右手座標。
- `t_Tool3_to_RGB=[0,0,0]`、`R_Tool3_from_RGB=I`。
- 依使用者目前核准的暫定規則：A 是 X roll、B 是 Y pitch、C 是 Z yaw，單位 degree，active column-vector：

  `R_Base0_from_RGB = Rz(C) * Ry(B) * Rx(A)`

這個矩陣順序不是 HIWIN 公開手冊已證實的官方 Pose 轉換公式，因此校正檔與終端都會寫出：

`rotation_convention_source: "user_approved_temporary"`

未來若實測需要修正，可在校正時提供 `--tool3-to-rgb-mm x,y,z` 與 row-major 的
`--r-tool3-from-rgb r00,r01,...,r22`；預設仍是已確認的零位移與 identity。最終值依下式建立並逐一驗證：

`C_Base0 = Tool3_position_Base0 + R_Base0_from_Tool3 * t_Tool3_to_RGB`

`R_Base0_from_RGB = R_Base0_from_Tool3 * R_Tool3_from_RGB`

## 像素到 Base0 的計算

對原始 `1280x720` RGB 像素 `(u,v)`：

1. 以 `fx,fy,cx,cy` 把 distorted pixel 轉成正規化座標。
2. 使用 `rgb_brown_rational_v1` 迭代反解 undistorted 座標。最多 50 次、normalized convergence tolerance `1e-12`；未收斂、分母接近零、Jacobian 奇異或發散時停止。
3. 用同一 forward model 回投影，誤差必須不超過 `0.25 px`。這只證明 solver 與其工程模型自洽，不證明模型就是相機的真實畸變；最後仍須通過 Base0 ground truth。
4. 形成 `[x,y,1]` 並正規化為 RGB optical 單位向量，再乘上 `R_Base0_from_RGB`。
5. 撞球直徑為 `44.5 mm`、半徑為 `22.25 mm`；球心平面為 `Z_target = Z_table + 22.25 mm`，交點比例：

   `lambda = (Z_target - C_Base0.z) / ray_Base0.z`

6. `lambda<=0` 或光線近乎平行時拒絕；流程完全不使用 Depth stream。

畸變係數依 SDK 欄位順序保存為 `k1,k2,k3,k4,k5,k6,p1,p2`。程式不把它們直接當成 OpenCV 8-vector，也不使用 `cv::fisheye`。目前版本化工程映射為：radial numerator=`k1,k2,k3`、radial denominator=`k4,k5,k6`、tangential=`p1,p2`。Orbbec profile API 未暴露足以獨立證實此完整方程的 variant，因此校正檔會明確保存 `engineering_assumption_pending_ground_truth_validation`，實機 ground truth 未通過前不得整合主程式。

## 建置

專案預設尋找：

- `C:\Program Files\OrbbecSDK 1.10.18\SDK\include`
- `C:\Program Files\OrbbecSDK 1.10.18\SDK\lib\OrbbecSDK.lib`
- repository 的 `include\HRSDK.h`、`lib\HRSDK.lib`、`bin\HRSDK.dll`

注意：目前 repository 的 `.gitignore` 排除 `*.lib` 與 `/bin/`，所以乾淨 clone 不會自動取得 HRSDK `.lib/.dll`。必須從已安裝／隨機器提供且版本相符的 x64 HRSDK 放置或指定：

- `-DHRSDK_INCLUDE_DIR=<含 HRSDK.h 的目錄>`
- `-DHRSDK_LIBRARY=<HRSDK.lib 完整路徑>`
- `-DHRSDK_RUNTIME_DLL=<HRSDK.dll 完整路徑>`

缺少其中任何一項時 CMake 會列出確切缺失路徑並停止，不會猜測版本或位置。

在已載入 Visual Studio x64 編譯環境的終端執行：

```powershell
cmake -S tools/rgb_base0_calibration -B build/rgb_base0_calibration -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/rgb_base0_calibration --parallel
ctest --test-dir build/rgb_base0_calibration --output-on-failure
```

建置後會把 HRSDK 與 Orbbec v1.10.18 的必要 DLL 複製到兩個執行檔旁。

## 1. 建立校正檔

操作者需要提供唯一的幾何輸入：桌布表面在 Robot Base0 的 `Z_table`，單位 mm。

```powershell
build/rgb_base0_calibration/rgb_base0_calibrate.exe --z-table <Z_table_mm>
```

預設輸出：

```text
output/rgb_base0_calibration/<timestamp>/
  camera_calibration.json
  camera_calibration.yaml
  raw_frames/frame_00.jpg
  terminal_log.txt
```

YAML 與 JSON 寫完後會立刻用嚴格 parser 讀回並檢查語意完全相同。schema v1.3 明確保存 `authorized_for_robot_motion=false`、K、D、工程模型／係數映射、solver 版本與門檻、frame 定義、Tool3 原始 XYZABC、三個 rotation matrices、Tool3→RGB 修正、`C_Base0`、constant-Z plane 與單位。缺欄、重複欄、錯誤型別、非有限數值、序號或 profile 不合都會報錯；每個 rotation matrix 都檢查 `R^T R`、正交誤差與 determinant。

## 2. 自動 YOLO 驗證

Python 只做 YOLO；相機、SDK 光線、旋轉矩陣與 Base0 交點都在 C++。預設使用既有 `.venv`、`bin/best.pt` 與 confidence 0.3：

```powershell
build/rgb_base0_calibration/rgb_base0_validate.exe --calibration <camera_calibration.json>
```

程式擷取完全未 resize／crop／flip／rotate／letterbox 的 10 張原始影像。每一 frame、每一球類別只保留 confidence 最高者，其餘同類別會保留在紀錄中並標記 `dropped_duplicate`。球類別的跨 frame 條件：

- 至少在 8/10 frame 出現。
- 先計算中心點初始 median，移除距離超過 5 px 的 observation。
- 移除後仍至少 8 個。
- 重新計算 final median，inlier 的 median radial distance 不超過 2 px。
- 未通過的類別不產生 Base0 XYZ，但拒絕原因與全部觀測仍完整輸出。

袋口只列出偵測；任何 frame 超過 6 個會警告。第一版不計算袋口 Base0 點位。確認球點正確後，必須提醒並另行實作、驗證袋口計算。

每次輸出：

```text
output/rgb_base0_validation/<timestamp>/
  raw_frames/
  annotated_frames/
  detections.json
  detections.csv
  stable_ball_pixels.csv
  results.json
  results.csv
  terminal_log.txt
  yolo_terminal_log.txt
```

`yolo_terminal_log.txt` 會在啟動 YOLO sidecar 前建立，並在成功或失敗時保留；內容以
`[stdout]`、`[stderr]` 區段保存 Python 訊息與 traceback，供失敗診斷使用。

## 3. 手動像素驗證

不執行 YOLO，直接輸入原始影像像素；參數可重複：

```powershell
build/rgb_base0_calibration/rgb_base0_validate.exe `
  --calibration <camera_calibration.yaml> `
  --manual point_1,640.0,360.0 `
  --manual point_2,300.5,200.5
```

## 4. Ground truth 驗收

CSV 欄位固定為：

```csv
class_name,x_mm,y_mm,z_mm
Ball_1,100.0,200.0,-211.36
Ball_2,150.0,250.0,-211.36
```

執行：

```powershell
build/rgb_base0_calibration/rgb_base0_validate.exe `
  --calibration <camera_calibration.json> `
  --ground-truth <ground_truth.csv>
```

自動通過條件：至少 6 個名稱匹配的穩定點、XY RMS ≤3 mm、每點 XY error ≤5 mm，而且 `error_x/error_y` 對 `u/v` 的四個 Pearson 相關係數絕對值都小於 0.7。0.7 是明確、可調整的操作門檻，不是 HIWIN 或 Orbbec 官方標準；可用 `--trend-limit` 改變並會記錄在結果中。提供 ground truth 而未通過時，程式仍完整寫出診斷，但 exit code 為 2。

目前已確認 `Z_table=-233.51 mm`，因此本配置的球心平面是 `-233.51+22.25=-211.36 mm`。若桌高改變，ground truth 的 Z 也必須跟著更新。

## 軟體狀態與尚未完成

- RGB-only Color profile、pipeline 啟動後的 `getCameraParam()` RGB K/D 擷取、Brown inverse solver 與 schema v1.3 已實作。
- 尚未在使用者實機執行新的 RGB-only 校正，因此尚未確認 live Gemini 2 XL K/D 擷取與 Color-only stream capture；即使 K/D 取得成功，也仍須完成 Base0 ground-truth 驗證。
- 未做真實機械手臂移動或主程式整合；這是刻意的安全界線。
- 尚未以至少六個 Base0 ground truth 球心完成物理驗收。
- 袋口 Base0 計算刻意延後；球點確認後必須提醒使用者補做。
- HIWIN ABC 的正式矩陣公式仍待對應控制器／HRSS 版本的官方確認；目前只能使用已明確標記的暫定 Z‑Y‑X 規則。
