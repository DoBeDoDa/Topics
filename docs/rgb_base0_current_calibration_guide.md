# RGB-only → Base0 校正與 Current Calibration 使用指南

更新日期：2026-08-12

## 1. 功能與輸出

Standalone `rgb_base0_calibrate` 只讀取 HIWIN Tool3／Base0 pose、擷取 Gemini 2 XL RGB、建立 RGB→Base0 校正資料，並保存：

```text
output/rgb_base0_calibration/<timestamp>/
    camera_calibration.json
    camera_calibration.yaml
    raw_frames/
    terminal_log.txt
```

完整校正與嚴格回讀全部成功後，歷史 JSON 才會安全發布為：

```text
config/vision/camera_calibration.json
```

這個 runtime calibration、`.tmp` 與 `.rollback.tmp` 均不納入 Git。Production `robot.py` 會載入固定 current calibration 進行視覺座標計算；JSON 中的 `authorized_for_robot_motion=false` 仍會保留，視覺使用不等於運動授權。

## 2. 安全發布

發布前會完成 Tool3／Base0 與 pose 穩定性、相機 profile、K/D、旋轉矩陣、歷史 JSON/YAML 回讀及語意等價驗證。驗證後的 JSON bytes 先寫入 `.tmp` 並再次嚴格解析，Windows 上以 named mutex 協調發布：既有 current 使用 `ReplaceFileW`，首次發布使用 `MoveFileExW(..., MOVEFILE_WRITE_THROUGH)`。

若替換後的最終驗證失敗，會使用 `.rollback.tmp` 還原原 current calibration。任何步驟失敗都不會把未驗證資料提升為 current，也不會輸出成功 `[DONE]`。

## 3. 建置

```powershell
cd C:\Users\10003\OneDrive\Desktop\camera_test

cmake --build .\build\rgb_base0_calibration `
  --target rgb_base0_calibrate rgb_base0_publication_tests `
  --config Release
```

`ninja: no work to do.` 代表原始碼沒有比現有執行檔更新，不是建置失敗。

查看參數且不連接硬體：

```powershell
.\build\rgb_base0_calibration\rgb_base0_calibrate.exe --help
```

## 4. 執行校正

執行前確認機械手臂完全停止、控制器可連線、相機未被占用、`--z-table` 是 Base0 桌布平面的真實 Z，且 Tool3→RGB 外參符合實際安裝。

```powershell
.\build\rgb_base0_calibration\rgb_base0_calibrate.exe `
  --z-table -233.510000 `
  --robot-ip 192.168.0.1
```

未指定外參時會假設 Tool3→RGB 零位移與單位旋轉。只有 RGB 光學中心與 Tool3 原點、軸向確實重合時才能接受。已量測外參時使用：

```powershell
.\build\rgb_base0_calibration\rgb_base0_calibrate.exe `
  --z-table -233.510000 `
  --robot-ip 192.168.0.1 `
  --tool3-to-rgb-mm "x,y,z" `
  --r-tool3-from-rgb "r11,r12,r13,r21,r22,r23,r31,r32,r33"
```

旋轉矩陣採 row-major，且必須是有效旋轉矩陣。不可猜測外參。

## 5. 判斷成功

完整成功會看到歷史 JSON/YAML 通過嚴格回讀、current calibration updated，以及：

```text
[DONE] Experimental calibration captured and current calibration published successfully.
```

可確認固定檔案存在：

```powershell
Test-Path .\config\vision\camera_calibration.json
```

若出現 `[ERROR]`、`ERROR:` 或沒有成功 `[DONE]`，就不能把該次執行視為成功。

## 6. 離線驗證

```powershell
ctest --test-dir .\build\rgb_base0_calibration `
  -C Release `
  --output-on-failure `
  -R "^rgb_base0_(publication|geometry)_tests$"
```

Publication tests 涵蓋首次發布、既有 current 安全替換、歷史檔案不變、缺檔／解析／原子替換失敗時保留 current、暫存檔不得直接提升，以及最終語意與 byte 等價。

## 7. 剩餘實機風險

- 發布成功不會證明 Tool3／RGB 實體原點與軸向真的重合。
- Named mutex 只能協調使用相同發布機制的程序。
- 真實相機、桌面與 robot pose 的校正誤差仍須以 ground truth 實測。
- 本工具不送出 robot motion、motor、alarm reset 或 DO；current calibration 亦不授權這些動作。
