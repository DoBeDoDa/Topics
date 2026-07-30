# P1-07 — BilliardPhysics安全幾何API

**Status:** ready-for-agent

**Blocked by:** P1-02 — 基礎型別、GeometryResults與MathUtils

## 1. Ticket ID與標題

P1-07：將BilliardPhysics改為具名狀態與optional結果，拒絕所有退化假成功。

## 2. 目的

提供可單獨測試的撞球幾何切片，讓路徑、鬼球、鏡射與交點計算明確區分成功、
阻擋、無解及無效輸入，不再回傳假Point或混合bool語意。

## 3. 前置依賴

- P1-02完成。
- GeometryResults及安全Math API可用。

## 4. 新增檔案

- 無。

## 5. 修改檔案

- `src/BilliardPhysics.h`
- `src/BilliardPhysics.cpp`
- `tests/phase1_core_tests.cpp`
- `.vscode/tasks.json`，將BilliardPhysics加入core test target。

## 6. 明確不得修改的檔案與行為

- 不修改Algorithm策略順序或TargetSelector。
- 不填入production球路安全餘量及未校正容差。
- 不處理Base0桌面幾何、正式庫邊模型或防守球。
- 不把Invalid轉成Clear、Blocked或假Point。
- 不加入HRSDK或運動程式。

## 7. API或資料型別變更

- 新增`CollisionParameters`，區分mm與無因次容差。
- `checkPath`與`checkRoute`回`PathStatus`。
- `getGhostBall`、`getPerpendicularTarget`、
  `getSlantedBankTarget`回`std::optional<Point>`。
- `getIntersection`回`IntersectionResult`。
- PathStatus：Clear、Blocked、Invalid。
- IntersectionStatus：Intersects、NoIntersection、Invalid。

## 8. 逐步實作內容

1. 集中驗證CollisionParameters有限值及有效範圍。
2. checkPath先拒絕非有限與退化路徑，再計算有限線段最短距離。
3. 阻擋門檻固定為`ballDiameterMm + clearanceMarginMm`。
4. checkRoute先驗證路徑與參數，再處理障礙列表。
5. 有效空障礙列表回Clear；退化路徑或無效參數即使空列表也回Invalid。
6. route合併優先順序為Invalid、Blocked、Clear。
7. 鬼球及鏡射方向退化時回nullopt。
8. 交點區分唯一交點、合法無交點及重合／退化Invalid。
9. 不保留回傳輸入Point的legacy fallback。

## 9. 測試案例

- Clear、Blocked及剛好等於門檻。
- clearanceMargin fixture為0與大於0。
- start=end、非有限Point、非法參數。
- 有效空列表Clear、退化空列表Invalid、非法參數空列表Invalid。
- route同時含Blocked與Invalid時Invalid優先。
- 有效鬼球與destination==target退化。
- railA==railB退化。
- 唯一交點、平行無交點、交點在線段外、重合線及非有限輸入。
- 所有IntersectionResult status／optional不變量。

## 10. 實際測試或建置命令

```powershell
cl.exe /std:c++17 /EHsc /nologo /utf-8 /I .\src /I .\tests `
  .\tests\phase1_core_tests.cpp `
  .\src\MathUtils.cpp `
  .\src\BilliardPhysics.cpp `
  /Fe:.\bin\phase1_core_tests.exe

& .\bin\phase1_core_tests.exe
```

## 11. 驗收條件

- [ ] PathStatus與IntersectionStatus全部測試通過。
- [ ] 零長度路徑永不回Clear。
- [ ] 空列表前先驗證路徑及參數。
- [ ] 退化鬼球、鏡射及交點不回假Point。
- [ ] 不存在混合Clear／Invalid語意的bool路徑API。
- [ ] 未填任何production容差。

## 12. 回滾方式

- Revert本ticket commit。
- 若P1-08已遷移Algorithm，先回滾P1-08及下游。
- 不只回滾header而保留不相容實作。

## 13. Safety Critical限制

- Invalid必須比Blocked優先傳播。
- 不得以空障礙列表掩蓋無效路徑。
- 不得將無唯一解的重合線回報為有效交點。
- 不得因保持舊策略相容而恢復假成功。

## 14. 完成後應產生的Git diff範圍

```text
M  src/BilliardPhysics.h
M  src/BilliardPhysics.cpp
M  tests/phase1_core_tests.cpp
M  .vscode/tasks.json
```

## 15. 建議commit message

```text
refactor(physics): return explicit geometry results
```

