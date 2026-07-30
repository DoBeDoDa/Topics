# P1-11 — Phase 1全面驗收與Code Review

**Status:** ready-for-agent

**Blocked by:** P1-10 — BilliardApp、test_cueball與完整編譯相容

## 1. Ticket ID與標題

P1-11：執行Phase 1離線驗收、依賴稽核、禁用符號搜尋與最終Code Review。

## 2. 目的

用單一驗收關卡證明Phase 1符合v1.1規格：兩個允許的離線測試通過、控制程式
只編譯不執行、危險legacy符號清除、策略回歸相容且沒有硬體副作用。

## 3. 前置依賴

- P1-10完成。
- P1-01至P1-10各自commit、測試與diff已完成審查。

## 4. 新增檔案

- 預期無。
- 若團隊要求保存驗收證據，可另開文件ticket；本ticket不自行擴張規格目錄。

## 5. 修改檔案

- 預期無production修改。
- 若發現缺陷，應重開或回到責任ticket修正；只有純驗收測試缺口可修改
  `tests/phase1_core_tests.cpp`、`tests/phase1_algorithm_regression_tests.cpp`
  或`.vscode/tasks.json`。

## 6. 明確不得修改的檔案與行為

- 不新增Phase 2至Phase 6功能。
- 不修改Algorithm策略排序或強制開火策略。
- 不填入未校正production值。
- 不執行main、calibrate、test_cueball。
- 不執行、連線或測試RobotController、SocketClient、HRSDK或DO。
- 不以「驗收順手修正」擴大production diff；發現問題應回責任ticket。

## 7. API或資料型別變更

- 無新API。
- 驗證既有Result不變量、三階段視覺型別、Physics status及五欄MotionProfile。

## 8. 逐步實作內容

1. 確認remote、branch、工作樹及各ticket commit範圍。
2. 重新build兩個Phase 1離線測試target。
3. 執行且只執行兩個離線測試。
4. compile-only建置main、calibrate、test_cueball。
5. 使用source搜尋確認禁用符號與`-9000`missing門檻消失。
6. 稽核兩個測試binary的link dependencies。
7. 審查TargetSelector沒有CameraCompensator依賴。
8. 審查TableState只有StableFrameValidator可產生。
9. 審查Algorithm有效fixture回歸及Invalid fail-closed。
10. 審查MotionProfile值與aggregate欄位順序。
11. 執行`git diff --check`及範圍審查。

## 9. 測試案例

- 重跑Phase 1 spec第15章全部core cases。
- 重跑直球、受阻轉反彈及安全反彈回歸fixture。
- 檢查所有status+optional矛盾組合均被測試拒絕。
- 檢查精確成對sentinel與小於`-9000`有限值分流。
- 檢查三幀presence、中位數、tolerance邊界。
- 檢查checkRoute三種空列表案例。
- 檢查PRODUCTION／TEST MotionProfile五欄值。

## 10. 實際測試或建置命令

允許執行：

```powershell
& .\bin\phase1_core_tests.exe
if ($LASTEXITCODE -ne 0) { throw "phase1_core_tests failed" }

& .\bin\phase1_algorithm_regression_tests.exe
if ($LASTEXITCODE -ne 0) { throw "phase1_algorithm_regression_tests failed" }
```

只允許build、不得run：

```text
Build with HRSDK (MSVC)
Build Calibration Tool (MSVC)
Build Cueball Test (MSVC)
```

靜態稽核：

```powershell
rg -n "applyCameraCompensation|getTiltOffset|tiltRyDeg|moveBackMm|MISSING_COORDINATE_LIMIT" .\src
rg -n "[<>]=?\\s*-9000\\.0" .\src
rg -n "CameraCompensator|ParsedVisionFrame|ProcessedVisionFrame" `
  .\src\TargetSelector.h .\src\TargetSelector.cpp
rg -n "HRSDK|RobotController|SocketClient|digital|PNEUMATIC" .\tests

dumpbin /dependents .\bin\phase1_core_tests.exe
dumpbin /dependents .\bin\phase1_algorithm_regression_tests.exe

git diff --check
git status --short
```

`YAW_OFFSET_DEG`另行確認沒有C++呼叫端；若只剩未使用定義，依P1-09決議處理。

## 11. 驗收條件

- [ ] `phase1_core_tests`通過。
- [ ] `phase1_algorithm_regression_tests`通過。
- [ ] main、calibrate、test_cueball編譯成功且未執行。
- [ ] 兩個測試binary不依賴HRSDK或控制／Socket模組。
- [ ] 禁用legacy符號與門檻式sentinel判斷清除。
- [ ] TargetSelector只接受TableState且不依賴CameraCompensator。
- [ ] Algorithm策略回歸相容，Invalid不產生decision。
- [ ] MotionProfile值未改且標記尚未實機驗證。
- [ ] 無Python、HRSDK、RobotController、SocketClient或DO diff。
- [ ] Code Review沒有未解Safety Critical finding。

## 12. 回滾方式

- 若驗收失敗，不以本ticket掩蓋；回到最早引入缺陷的ticket修正或revert。
- 若本ticket只有測試補強，revert其獨立commit不影響production API。
- 不使用`git reset --hard`。

## 13. Safety Critical限制

- 禁止為了讓驗收通過而降低assert、放寬Invalid或使用預設production值。
- 禁止執行任何HRSDK-linked binary。
- 禁止把編譯成功、reachable或策略回歸宣告為實機運動安全。
- 發現策略Invalid仍可到達動作邊界時，Phase 1不得標記完成。

## 14. 完成後應產生的Git diff範圍

預期：

```text
No production diff
```

若只補驗收測試，最多：

```text
M  tests/phase1_core_tests.cpp
M  tests/phase1_algorithm_regression_tests.cpp
M  .vscode/tasks.json
```

任何production修正必須回原責任ticket，不得混入驗收commit。

## 15. 建議commit message

若有純測試／驗收補強：

```text
test: complete phase 1 safety acceptance
```

若無diff，無需建立空commit；以review／PR驗收紀錄結案。

