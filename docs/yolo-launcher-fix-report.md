# RGB→Base0 Validation YOLO Launcher 修正報告

更新日期：2026-08-11

適用程式：`rgb_base0_validate`

GitHub `main` 修正提交：`48fcf57`（`Use native Windows YOLO launcher`）

## 1. 結論

`rgb_base0_validate` 的 YOLO sidecar 啟動問題已修正。

原本的 `std::system()` 加上 Windows shell redirection 會在 Python 啟動前失敗，造成：

```text
[launcher diagnostic] stdout capture file was not created
[launcher diagnostic] stderr capture file was not created
[ERROR] YOLO sidecar failed with process exit code 1
```

目前已改用 Windows 原生 `CreateProcessW()`，不再透過 `cmd.exe`。Python stdout、stderr、真正的 child exit code及啟動錯誤都能正確保存。

## 2. 根因

原始 launcher 使用：

```cpp
std::system(command.c_str());
```

同一個 command string 內包含：

```text
"python.exe" "yolo_detect.py" ... > "yolo_stdout.tmp" 2> "yolo_stderr.tmp"
```

`quoteWindowsArgument()` 的實作符合 CreateProcess/C runtime `argv` 引用規則，但不等同於 `cmd.exe` shell command的引用規則。

由於整條命令以雙引號開頭，且最後也是被引用的stderr路徑，`cmd.exe`會套用特殊的首尾引號解析。結果是命令在執行Python之前就失敗，shell redirection也沒有建立stdout/stderr檔案。

相同命令形式已獨立重現：

```text
RAW_SYSTEM_RETURN=1
STDOUT_EXISTS=False
STDERR_EXISTS=False
The system cannot find the path specified.
```

Python、Ultralytics、`best.pt`、`yolo_detect.py`與10張原始影像本身並不是根因。

## 3. 修改範圍

唯一修改的程式檔：

```text
tools/rgb_base0_calibration/src/validate_main.cpp
```

未修改：

- `tools/rgb_base0_calibration/scripts/yolo_detect.py`
- Tool3/Base0姿態讀取與驗證
- Orbbec Gemini 2 XL相機邏輯
- Brown distortion反投影
- RGB optical frame假設
- pixel→Base0幾何
- calibration JSON/YAML schema
- YOLO類別選擇與穩定判定規則
- Depth/D2C行為
- production TCP與32值CSV契約
- 機械手臂運動或DO控制

## 4. 修正後的Launcher

目前的啟動流程為：

1. 啟動前先建立`yolo_terminal_log.txt`。
2. 驗證Python、sidecar script與weights路徑。
3. 使用`CreateFileW()`建立可繼承的stdout/stderr擷取檔handle。
4. 將child stdin連到`NUL`。
5. 使用`CreateProcessW()`明確指定`.venv\Scripts\python.exe`。
6. 透過`STARTF_USESTDHANDLES`將Python stdout/stderr導向擷取檔。
7. 使用`WaitForSingleObject()`等待Python結束。
8. 使用`GetExitCodeProcess()`取得真正的Python exit code。
9. 關閉process、thread與file handles。
10. 將擷取結果以`[stdout]`、`[stderr]`區段寫入`yolo_terminal_log.txt`。
11. 寫入並關閉日誌後，才處理非零exit code。

`std::system()`、PowerShell與`cmd.exe`都不再參與YOLO sidecar啟動。

## 5. 新增診斷資訊

`terminal_log.txt`現在會記錄：

```text
[YOLO DEBUG] launcher=CreateProcessW
[YOLO DEBUG] working_directory=...
[YOLO DEBUG] python=...
[YOLO DEBUG] script=...
[YOLO DEBUG] frames=...
[YOLO DEBUG] output=...
[YOLO DEBUG] weights=...
[YOLO DEBUG] yolo_log=...
[YOLO DEBUG] stdout_capture=...
[YOLO DEBUG] stderr_capture=...
[YOLO DEBUG] command=...
[YOLO DEBUG] create_process_started=true|false
[YOLO DEBUG] wait_result=...
[YOLO DEBUG] launcher_error=...
[YOLO DEBUG] child_exit_code=...
```

若Python、script或weights在啟動前缺失，`yolo_terminal_log.txt`仍會存在，並保存明確的launcher diagnostic。

## 6. 驗證結果

### 6.1 建置

精確目標建置成功：

```text
rgb_base0_validate
```

執行檔：

```text
C:\Users\10003\OneDrive\Desktop\camera_test\build\rgb_base0_calibration\rgb_base0_validate.exe
```

### 6.2 最小相關測試

```text
rgb_base0_geometry_tests: 1/1 passed
```

### 6.3 YOLO成功路徑（無硬體）

使用既有validation的10張raw frames執行實際sidecar：

```text
output/rgb_base0_validation/20260811_134618/raw_frames
```

結果：

```text
child exit code: 0
raw detections: 109
total hole detections: 60
stdout capture: 已建立，25,444 bytes
stderr capture: 已建立，0 bytes
```

`yolo_terminal_log.txt`含有實際sidecar輸出及：

```text
[YOLO] completed; raw detections=109 total hole detections=60
```

不再包含：

```text
capture file was not created
```

### 6.4 Python故意失敗路徑（無硬體）

故意讓Python輸出stdout、stderr後拋出`RuntimeError`。

結果：

```text
create_process_started=true
wait_result=0
launcher_error=0
child_exit_code=1
```

最終`yolo_terminal_log.txt`成功保留：

```text
[stdout]
deliberate stdout

[stderr]
deliberate stderr
Traceback (most recent call last):
...
RuntimeError: deliberate launcher verification failure
```

## 7. 完整Validation重跑結果

修正後曾使用最新校正檔重新執行完整validation：

```text
output/rgb_base0_calibration/20260811_134444/camera_calibration.json
```

當次在進入相機及YOLO前停止：

```text
[ERROR] Failed to connect to HIWIN controller at 192.168.0.1
```

因此：

- 原生YOLO launcher已透過既有10張影像獨立驗證。
- 完整robot pose→camera→YOLO→Base0流程仍需在HIWIN控制器可連線時重跑。
- 當次沒有相機擷取、YOLO執行或機械手臂移動。

## 8. 重新建置

請在Developer PowerShell for VS 2022或已載入Visual Studio開發環境的PowerShell執行：

```powershell
cd C:\Users\10003\OneDrive\Desktop\camera_test

cmake --build .\build\rgb_base0_calibration `
  --target rgb_base0_validate `
  --config Release
```

若顯示：

```text
ninja: no work to do.
```

代表現有執行檔已是最新版本。

## 9. 重新執行Validation

確認HIWIN控制器`192.168.0.1`可以連線，且手臂位於校正檔記錄的Tool3/Base0姿態後執行：

```powershell
cd C:\Users\10003\OneDrive\Desktop\camera_test

.\build\rgb_base0_calibration\rgb_base0_validate.exe `
  --calibration ".\output\rgb_base0_calibration\20260811_134444\camera_calibration.json"
```

最新結果會建立在：

```text
output/rgb_base0_validation/<timestamp>/
```

至少應檢查：

```text
terminal_log.txt
yolo_terminal_log.txt
raw_frames/frame_00.jpg ... frame_09.jpg
detections.json
detections.csv
stable_ball_pixels.csv
```

## 10. 目前仍未完成的功能

standalone validation目前只會對穩定球像素執行RGB→Base0計算。洞口偵測仍被明確設定為：

```text
hole_calculation = deferred_list_only
```

因此目前尚未完成：

- 六個洞口的Base0座標計算與ground-truth驗證。
- 將新Tool3 RGB→Base0 validation結果接入正式`robot.py`。
- 將新流程產生的球／洞口座標送入production 32值TCP契約。

這些項目不屬於本次launcher修正範圍。

## 11. 安全與工具使用確認

- 本次修正沒有新增任何robot motion、motor、alarm-clear或DO命令。
- 無硬體YOLO測試只讀取既有10張影像。
- 未使用`$tdd`。
- 未使用`$diagnosing-bugs`。
