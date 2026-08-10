# P1-03 — CameraCompensator

**Status:** Superseded — historical only, do not implement

**Blocked by:** P1-02 — 基礎型別、GeometryResults與MathUtils

## Superseded Classification and Requirement Migration

- Classification：Superseded；Approved Specs明確禁止C++ CameraCompensator與double compensation。
- 核心「C++再次補償」及新增`CameraCompensator.cpp/.h`要求：obsolete，不遷移。
- finite／overflow validation、fail-closed、Diagnostic與不得raw Point fallback：遷移至active P1-03。
- 純MathUtils不得含camera compensation：已由Completed P1-02保留。
- 原測試中只驗證補償公式者失效；通用非法輸入／Result不變量測試遷移至P1-03。
- 本文件只供Git歷史追溯，不是Approved需求、active ticket或實作依據。

## 1. Ticket ID與標題

P1-03：將既有相機線性殘差補償抽成可獨立測試的CameraCompensator。

## 2. 目的

提供單一、純粹且fail-closed的相機補償元件，使補償不再由MathUtils或
TargetSelector負責，同時完全保留現有數學公式。

## 3. 前置依賴

- P1-02完成。
- Point、isFinite與optional失敗語意已固定。

## 4. 新增檔案

- `src/CameraCompensator.h`
- `src/CameraCompensator.cpp`

## 5. 修改檔案

- `tests/phase1_core_tests.cpp`
- `.vscode/tasks.json`，將CameraCompensator加入core test target。

## 6. 明確不得修改的檔案與行為

- 不修改相機補償公式或production常數。
- 不修改Parser、TargetSelector、Algorithm或BilliardApp。
- 不處理`-9999.0`或任何sentinel。
- 不檢查桌面bounds或必要球資料。
- 不加入Socket、HRSDK或RobotController依賴。

## 7. API或資料型別變更

- 新增`CameraCompensationParameters`：
  offsetXmm、offsetYmm、referenceXmm、referenceYmm、compensationKx、
  compensationKy。
- 新增`CameraCompensator`：
  - constructor
  - `isConfigured()`
  - `std::optional<Point> compensate(Point) const noexcept`
- 非有限參數、輸入、中間值或輸出不得產生Point。

## 8. 逐步實作內容

1. 建立參數型別與設定有效性檢查。
2. 逐字對照規格實作既有X/Y補償公式。
3. 使用有限值檢查保護輸入、中間結果與輸出。
4. 未配置時`isConfigured()`為false，`compensate()`回nullopt。
5. 加入第一行責任註解：只做既有線性殘差補償，不負責sentinel。
6. 將source加入core test target。

## 9. 測試案例

- raw等於reference時只加固定offset。
- 只設定X offset、只設定Y offset。
- Kx／Ky非0時比例補償符合固定公式。
- 正常有限座標。
- raw含NaN或Infinity。
- 任一參數非有限。
- 大值造成補償中間結果或輸出非有限。
- 驗證`-9999.0`不具有特殊missing語意；若作為有限Point傳入，只按公式計算，
  不在此元件轉nullopt。

## 10. 實際測試或建置命令

```powershell
cl.exe /std:c++17 /EHsc /nologo /utf-8 /I .\src /I .\tests `
  .\tests\phase1_core_tests.cpp `
  .\src\MathUtils.cpp `
  .\src\CameraCompensator.cpp `
  /Fe:.\bin\phase1_core_tests.exe

& .\bin\phase1_core_tests.exe
```

## 11. 驗收條件

- [ ] 固定補償公式的所有測試通過。
- [ ] 非有限或溢位情況回nullopt。
- [ ] CameraCompensator不認識sentinel與桌面bounds。
- [ ] MathUtils中沒有相機補償回流。
- [ ] 沒有TargetSelector、Algorithm或硬體依賴。

## 12. 回滾方式

- Revert本ticket commit。
- 移除CameraCompensator source及其test task來源。
- 若P1-05已依賴本元件，必須先回滾P1-05及其下游。

## 13. Safety Critical限制

- 禁止補償失敗時回傳raw Point作fallback。
- 禁止自行調整公式、校正參數或填入新production值。
- 禁止把sentinel規則搬進CameraCompensator。

## 14. 完成後應產生的Git diff範圍

```text
A  src/CameraCompensator.h
A  src/CameraCompensator.cpp
M  tests/phase1_core_tests.cpp
M  .vscode/tasks.json
```

## 15. 建議commit message

```text
refactor(vision): extract camera compensator
```
