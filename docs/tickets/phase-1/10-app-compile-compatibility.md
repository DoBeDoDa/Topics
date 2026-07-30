# P1-10 — BilliardApp、test_cueball與完整編譯相容

**Status:** ready-for-agent

**Blocked by:** P1-08 — TargetSelector與Algorithm遷移；P1-09 — MotionProfile與MotionPlanner相容遷移

## 1. Ticket ID與標題

P1-10：收斂Phase 1新API呼叫端，恢復主程式、calibration及test_cueball編譯相容。

## 2. 目的

完成Phase 1的compile-only整合切片。BilliardApp及test_cueball必須理解新的
Parser、單幀處理、穩定TableState與幾何Result；尚未實作的Socket三幀收集不得
用單幀或假資料繞過。

## 3. 前置依賴

- P1-08完成。
- P1-09完成。
- 兩個離線測試target在各自分支均為綠色。

## 4. 新增檔案

- 無。

## 5. 修改檔案

- `src/BilliardApp.cpp`
- `src/test_cueball.cpp`
- `.vscode/tasks.json`，更新主程式／cueball source清單及Phase 1新source。
- `src/BilliardApp.h`只有在新API宣告無法以現有成員完成編譯時才可做最小修改；
  若不需要則不得變更。

## 6. 明確不得修改的檔案與行為

- 不修改RobotController、SocketClient、main或calibrate內容。
- 不修改Python。
- 不實作三幀Socket收集、H按鈕或CompetitionAuto。
- 不執行main、calibrate或test_cueball。
- 不送reachable、LIN、PTP、DO或任何HRSDK命令。
- 不改test_cueball既有輸出文字、換行或無關診斷。
- 不製造TableState來繞過StableFrameValidator。

## 7. API或資料型別變更

- BilliardApp處理VisionParseResult、SingleFrameResult、
  StableFrameResult、Target selection failure及ShotDecisionResult。
- 尚缺production bounds、stable tolerance或三幀收集時必須fail closed。
- test_cueball移除舊Math／Physics／MotionProfile符號，改用新介面。
- build task加入CameraCompensator、VisionFrameProcessor、
  StableFrameValidator及其他必要Phase 1 sources。

## 8. 逐步實作內容

1. 將BilliardApp Parser呼叫改成VisionParseResult。
2. 將成功ParsedVisionFrame交給VisionFrameProcessor；失敗狀態直接停止。
3. 不在本ticket建立Socket三幀buffer；沒有三個Valid ProcessedVisionFrame時，
   不得呼叫TargetSelector。
4. 若production bounds／tolerance未設定，明確回報ConfigurationMissing並停止。
5. 將TargetSelector及Algorithm新Result錯誤傳遞至現有流程邊界。
6. 將test_cueball舊`-9000`判斷、Math相機補償、Physics bool／假Point及
   MotionProfile欄位改為新API的compile-only相容。
7. 保留test_cueball既有輸出格式及使用者換行修改。
8. 更新三個既有build task的C++17及必要source，但不新增run task。
9. 確認兩個離線測試target仍不含BilliardApp／控制依賴。

## 9. 測試案例

- Parser失敗、單幀失敗與stable資料不可用均fail closed。
- 沒有三個Valid frame時不進TargetSelector。
- InvalidGeometry不進MotionPlanner。
- test_cueball所有舊符號已遷移且編譯成功。
- 兩個離線測試仍全部通過。
- main、calibrate、test_cueball只做build驗證。
- 比較test_cueball修改前後輸出字串及換行，除必要錯誤狀態外不變。

## 10. 實際測試或建置命令

先執行允許的離線測試：

```powershell
& .\bin\phase1_core_tests.exe
& .\bin\phase1_algorithm_regression_tests.exe
```

再使用VS Code build task，只編譯、不執行：

```text
Build with HRSDK (MSVC)
Build Calibration Tool (MSVC)
Build Cueball Test (MSVC)
```

建置完成後只檢查檔案存在，不啟動：

```powershell
Get-Item .\bin\main.exe,.\bin\calibrate.exe,.\bin\test_cueball.exe
```

## 11. 驗收條件

- [ ] 兩個離線測試仍通過。
- [ ] main、calibrate、test_cueball均可編譯。
- [ ] 三個控制／診斷執行檔完全未執行。
- [ ] BilliardApp不以單幀或假TableState進入策略。
- [ ] test_cueball只做必要介面遷移並保留輸出格式。
- [ ] 測試targets仍沒有HRSDK、RobotController、SocketClient、BilliardApp或DO。

## 12. 回滾方式

- Revert本ticket commit。
- 保留P1-01至P1-09的純模組與測試。
- 不還原或覆寫test_cueball原有輸出換行修改。

## 13. Safety Critical限制

- 編譯控制程式不等於授權執行。
- 不得為恢復舊流程而跳過三幀Stable邊界。
- 不得在ConfigurationMissing時使用0或猜測production值。
- 不得觸碰RobotController運動安全缺陷；那不屬於Phase 1。

## 14. 完成後應產生的Git diff範圍

```text
M  src/BilliardApp.cpp
M  src/test_cueball.cpp
M  .vscode/tasks.json
M  src/BilliardApp.h  # 僅在編譯介面確實需要時
```

不得包含RobotController、SocketClient、main、calibrate、Python或HRSDK檔案。

## 15. 建議commit message

```text
refactor(app): migrate phase 1 result APIs
```

