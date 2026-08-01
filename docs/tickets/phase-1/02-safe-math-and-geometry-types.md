# P1-02 — 基礎型別、GeometryResults與MathUtils

**Status:** Completed

**Blocked by:** P1-01 — C++17建置與離線測試框架

## 1. Ticket ID與標題

P1-02：建立安全基礎型別、具名幾何結果與純MathUtils API。

## 2. 目的

以一條可離線驗證的純數學切片，消除零向量假角度、非有限輸入、相減溢位及
跨模組責任。完成後，後續視覺、物理及運動相容ticket具有一致的optional失敗語意。

## 3. 前置依賴

- P1-01完成。
- `phase1_core_tests`可獨立編譯與執行。

## 4. 新增檔案

- `src/GeometryResults.h`

## 5. 修改檔案

- `src/Point.h`
- `src/MathUtils.h`
- `src/MathUtils.cpp`
- `tests/phase1_core_tests.cpp`
- `.vscode/tasks.json`，只限core test source清單。

## 6. 明確不得修改的檔案與行為

- 不修改BilliardPhysics、Algorithm、TargetSelector、MotionPlanner或BilliardApp。
- 不修改BilliardConfig中的production數值。
- 不加入CameraCompensator實作。
- 不加入HRSDK姿態矩陣／Euler功能。
- 不執行任何非Phase 1離線測試程式。

## 7. API或資料型別變更

- 保留／明確化`Point`與`Vector2D`。
- 新增`AxisAlignedBounds2D`。
- 新增`PathStatus`、`IntersectionStatus`與`IntersectionResult`。
- 新增或替換：
  - `isFinite(Point/Vector2D)`
  - `getVector`
  - `getLength`
  - `getDistance`
  - `normalize`
  - `getVectorAngleDeg`
  - `getAngleBetweenVectorsDeg`
- 可能失敗的API回`std::optional`。
- `IntersectionResult`必須遵守Intersects有Point、其他狀態nullopt。
- 從MathUtils移除CameraCompensation與`getTiltOffset`；移除不再需要的
  `Offset3D`。

## 8. 逐步實作內容

1. 整理Point／Vector2D定義並加入AxisAlignedBounds2D。
2. 建立GeometryResults具名enum與Result不變量。
3. 讓MathUtils不再include BilliardConfig。
4. 使用`std::hypot`計算長度／距離。
5. 使用`std::atan2`建立`[-180,180]`方向角。
6. 夾角cosine在`std::acos`前clamp至`[-1,1]`。
7. 零向量、NaN、Infinity及非有限中間結果回nullopt。
8. 移除MathUtils中的相機補償及姿態傾角偏移實作。
9. 將新MathUtils source加入core test target。

## 9. 測試案例

- +X、+Y、-X、-Y與四個斜向象限。
- `(0,0)`長度為0，但normalize與方向角無結果。
- 同向0°、垂直90°、反向180°。
- 任一零向量的夾角無結果。
- NaN與Infinity輸入無結果。
- `1e150`級有限向量由hypot得到有限長度。
- Point相減溢位回nullopt。
- acos邊界先clamp。
- IntersectionResult所有status/payload合法與矛盾組合測試。

## 10. 實際測試或建置命令

```powershell
cl.exe /std:c++17 /EHsc /nologo /utf-8 /I .\src /I .\tests `
  .\tests\phase1_core_tests.cpp `
  .\src\MathUtils.cpp `
  /Fe:.\bin\phase1_core_tests.exe

& .\bin\phase1_core_tests.exe
```

另執行：

```powershell
rg -n "BilliardConfig|applyCameraCompensation|getTiltOffset|Offset3D" `
  .\src\MathUtils.h .\src\MathUtils.cpp
```

結果不得出現被禁止的MathUtils責任。

## 11. 驗收條件

- [x] 所有MathUtils測試通過。
- [x] MathUtils不include BilliardConfig。
- [x] 零向量不產生假角度。
- [x] 非有限或溢位運算不產生成功值。
- [x] GeometryResults不變量有測試。
- [x] CameraCompensation、getTiltOffset及Offset3D已離開MathUtils。

後續呼叫端與完整流程相容由P1-03至P1-09及P2-01至P2-03收斂；本ticket不得加入危險legacy wrapper
來掩蓋尚未遷移的呼叫端。

## 12. 回滾方式

- Revert本ticket commit。
- 若下游ticket已使用新Math API，先反向回滾下游。
- 不部分回滾MathUtils header而留下ABI／編譯不一致。

## 13. Safety Critical限制

- 禁止用0度、原始Point或預設Point代替失敗。
- 禁止用單一`0.001`處理不同量綱。
- 禁止在本ticket推導或實作HRSDK Euler規則。

## 14. 完成後應產生的Git diff範圍

```text
M  src/Point.h
M  src/MathUtils.h
M  src/MathUtils.cpp
A  src/GeometryResults.h
M  tests/phase1_core_tests.cpp
M  .vscode/tasks.json
```

## 15. 建議commit message

```text
refactor(math): add safe geometry primitives
```

## 16. Approved Spec References

- Master Spec §9 P1-02、§7安全不變量。
- Phase 1 Shot Brain Spec §4 P1-02、§8 Geometry and Collision。

## 17. Existing Responsibility Owners

- `Point.h`、`GeometryResults.h`、`MathUtils.h/.cpp`及`phase1_core_tests`。
- Existing Files Explicitly Not to Duplicate：不得建立第二套Point、GeometryResults、MathUtils或production math tree。

## 18. Hardware Level與Regression Requirements

- Hardware Level：完全離線純數學。
- Regression：後續幾何、策略與MotionPlanner只能消費已完成安全API；不得恢復CameraCompensation、假Point或0度fallback。

## 19. Definition of Done與完成證據

- Implementation commit：`216bcb7`。
- Completed scope與existing tests保留，不重建、不重新實作。
- Approved Specs revalidation：PASS（2026-08-01）；後續新增幾何型別若屬其他能力，由其ticket以delta方式擴充。

## 20. Requirement Traceability

- 舊P1-02全部有效要求由本Completed ticket保留。
- 十二能力coverage：P1-02，Coverage Complete = YES。

## 21. New File Justification

- None expected；本票已完成，禁止建立第二套數學primitive ticket或production模組。
