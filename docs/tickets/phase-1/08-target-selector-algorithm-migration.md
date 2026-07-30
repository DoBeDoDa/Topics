# P1-08 — TargetSelector與Algorithm遷移

**Status:** ready-for-agent

**Blocked by:** P1-06 — StableFrameValidator與TableState；P1-07 — BilliardPhysics安全幾何API

## 1. Ticket ID與標題

P1-08：讓目標選擇與既有策略只消費穩定TableState及明確幾何結果。

## 2. 目的

完成從穩定視覺核心資料到ShotDecisionResult的策略切片。有效幾何維持既有
直球／反彈策略排序；任何Invalid幾何必須在建立ShotDecision前終止。

## 3. 前置依賴

- P1-06完成，TableState只由三幀Stable產生。
- P1-07完成，Physics不再回bool混合狀態或假Point。

## 4. 新增檔案

- 無。

## 5. 修改檔案

- `src/TargetSelector.h`
- `src/TargetSelector.cpp`
- `src/Algorithm.h`
- `src/Algorithm.cpp`
- `tests/phase1_algorithm_regression_tests.cpp`
- `.vscode/tasks.json`，完成algorithm regression target的純模組清單。

## 6. 明確不得修改的檔案與行為

- 不修改CameraCompensator、Parser、StableFrameValidator或MotionPlanner。
- TargetSelector不得include、持有、建構、注入或呼叫CameraCompensator。
- 不接受ParsedVisionFrame或ProcessedVisionFrame。
- 不修改直球／反彈策略排序、角度門檻、袋口選擇順序或策略文字。
- 不修正「安全路徑受阻，強制開火」策略；只阻止Invalid幾何進入該分支。
- 不加入防守球、Base0庫邊重構或其他後續策略。

## 7. API或資料型別變更

- `TargetSelector::select`唯一視覺輸入為`const TableState&`。
- optional球／袋口缺失以選擇失敗或跳過候選表示，不補預設Point。
- Algorithm所有Physics optional／status必須顯式處理。
- 新增`ShotDecisionStatus{Success, InvalidGeometry}`。
- 新增`ShotDecisionResult`：
  Success必須有ShotDecision；InvalidGeometry必須為nullopt。

## 8. 逐步實作內容

1. 移除TargetSelector內的compensated helper與MathUtils相機補償呼叫。
2. 直接使用TableState內已補償且穩定的Point。
3. 依現有行為選最低號存在球、合法袋口及p2／p3反彈邊界。
4. 方向或夾角API回nullopt時跳過該候選，不以0度替代。
5. 全部候選無效時回明確selection failure。
6. 將Algorithm遷移至PathStatus、IntersectionResult與optional Point。
7. 任一必要幾何為Invalid／nullopt時回InvalidGeometry。
8. Valid且Blocked的既有策略仍走原本策略順序，包括現有強制開火文字分支。
9. 建立固定有限、非退化fixture，鎖定既有策略輸出。

## 9. 測試案例

- TargetSelector選擇最低號存在球。
- 已落袋球nullopt會被跳過。
- 無cue ball、無目標球、無有效袋口、缺p2／p3時失敗。
- 退化夾角候選被跳過；全部退化時選擇失敗。
- TargetSelector source沒有CameraCompensator依賴。
- 直球clear fixture保持直球策略。
- 直球受阻fixture保持切換反彈策略。
- 現有安全反彈fixture保持策略名稱、aim target及狀態。
- getGhostBall／intersection／route為Invalid時回InvalidGeometry且無decision。
- Invalid不得進入強制開火分支。
- ShotDecisionResult所有status／optional不變量。

## 10. 實際測試或建置命令

```powershell
cl.exe /std:c++17 /EHsc /nologo /utf-8 /I .\src /I .\tests `
  .\tests\phase1_algorithm_regression_tests.cpp `
  .\src\MathUtils.cpp `
  .\src\CameraCompensator.cpp `
  .\src\VisionDataParser.cpp `
  .\src\VisionFrameProcessor.cpp `
  .\src\StableFrameValidator.cpp `
  .\src\BilliardPhysics.cpp `
  .\src\TargetSelector.cpp `
  .\src\Algorithm.cpp `
  .\src\BilliardConfig.cpp `
  /Fe:.\bin\phase1_algorithm_regression_tests.exe

& .\bin\phase1_algorithm_regression_tests.exe

rg -n "CameraCompensator|applyCameraCompensation|ParsedVisionFrame|ProcessedVisionFrame" `
  .\src\TargetSelector.h .\src\TargetSelector.cpp
```

最後一個搜尋不得找到禁止依賴。

## 11. 驗收條件

- [ ] TargetSelector只接受TableState。
- [ ] TargetSelector完全不依賴CameraCompensator。
- [ ] 所有optional幾何失敗均被顯式處理。
- [ ] InvalidGeometry沒有ShotDecision payload。
- [ ] Invalid不進入強制開火分支。
- [ ] 有效直球／反彈回歸fixture與既有策略相容。
- [ ] algorithm regression target不含硬體或Socket依賴。

## 12. 回滾方式

- Revert本ticket commit。
- P1-06與P1-07可獨立保留。
- 若P1-10已使用新ShotDecisionResult，先回滾P1-10。

## 13. Safety Critical限制

- 禁止用0度、原始Point或任意袋口補償Invalid。
- 禁止Invalid候選觸發任何決策payload。
- 禁止趁遷移修改策略排序或實作防守球。
- regression測試不得link或執行RobotController、SocketClient或HRSDK。

## 14. 完成後應產生的Git diff範圍

```text
M  src/TargetSelector.h
M  src/TargetSelector.cpp
M  src/Algorithm.h
M  src/Algorithm.cpp
M  tests/phase1_algorithm_regression_tests.cpp
M  .vscode/tasks.json
```

## 15. 建議commit message

```text
refactor(algorithm): consume validated table state
```

