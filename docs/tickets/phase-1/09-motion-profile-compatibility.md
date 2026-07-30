# P1-09 — MotionProfile與MotionPlanner相容遷移

**Status:** ready-for-agent

**Blocked by:** P1-02 — 基礎型別、GeometryResults與MathUtils

## 1. Ticket ID與標題

P1-09：移除姿態／幾何混合欄位，固定五欄MotionProfile並遷移MotionPlanner。

## 2. 目的

完成最小機械姿態相容切片：MotionPlanner改用安全Math API，RZ只由二維
shot direction決定，移除getTiltOffset與moveBackMm，同時逐欄保留現有RX／RY值。

## 3. 前置依賴

- P1-02完成。
- 新MathUtils optional API已固定。

## 4. 新增檔案

- 無。

## 5. 修改檔案

- `src/BilliardConfig.h`
- `src/BilliardConfig.cpp`
- `src/MotionPlanner.h`
- `src/MotionPlanner.cpp`
- `tests/phase1_core_tests.cpp`，只加入純設定／規劃相容測試。

## 6. 明確不得修改的檔案與行為

- 不修改RobotController、BilliardApp、main、calibrate或test_cueball。
- 不改PRODUCTION_MOTION或TEST_MOTION現有RX／RY數值。
- 不實作RX／RY候選搜尋、CartesianPose、50 mm新strike TCP模型或HRSDK Euler。
- 不執行MotionPlanner產生的任何運動計畫。
- 不將LIN、PTP或reachable邏輯加入MotionPlanner。

## 7. API或資料型別變更

MotionProfile欄位及順序固定為：

```text
strikeZ
safeZ
rxDeg
ryDeg
standoffExtraMm
```

- 移除`tiltRyDeg`。
- 移除`moveBackMm`。
- 移除`YAW_OFFSET_DEG`使用；若無其他合法呼叫端，連同未使用常數移除。
- MotionPlanner處理Math optional失敗並回既有明確規劃失敗。

## 8. 逐步實作內容

1. 將MotionProfile改成規格指定的五欄及順序。
2. PRODUCTION逐欄遷移為`{-216.0,-160.0,0.0,-180.0,0.0}`。
3. TEST逐欄遷移為`{-140.0,-150.0,-10.0,180.0,0.0}`。
4. 以欄位註解核對aggregate initializer，禁止placeholder。
5. 將`tiltRyDeg`所有本ticket呼叫改成純Pose語意`ryDeg`。
6. 完全移除moveBackMm與getTiltOffset呼叫；strike X/Y不再加該offset。
7. RZ使用`getVectorAngleDeg(aimTarget-cueBall)`，不加YAW_OFFSET_DEG。
8. Math optional無值時規劃失敗，不產生MotionPlan。
9. 保留現有ready／strike結構、Z值、RX／RY值及standoff計算。

## 9. 測試案例

- MotionProfile欄位數與欄位順序編譯驗證。
- PRODUCTION五欄值逐一比對。
- TEST五欄值逐一比對。
- +X、+Y、-X、-Y及四象限aim direction產生對應RZ。
- cueBall==aimTarget或非有限方向使規劃失敗。
- RZ沒有固定yaw加法。
- moveBackMm為0的舊有效fixture，strike X/Y在移除offset後保持相容。
- 搜尋source不存在tiltRyDeg、moveBackMm或getTiltOffset。

## 10. 實際測試或建置命令

```powershell
cl.exe /std:c++17 /EHsc /nologo /utf-8 /I .\src /I .\tests `
  .\tests\phase1_core_tests.cpp `
  .\src\MathUtils.cpp `
  .\src\BilliardConfig.cpp `
  .\src\MotionPlanner.cpp `
  /Fe:.\bin\phase1_core_tests.exe

& .\bin\phase1_core_tests.exe

rg -n "tiltRyDeg|moveBackMm|getTiltOffset|YAW_OFFSET_DEG" `
  .\src\BilliardConfig.h .\src\BilliardConfig.cpp `
  .\src\MotionPlanner.h .\src\MotionPlanner.cpp
```

除非規格允許保留未使用的YAW constant定義，其他禁止符號不得出現；任何
YAW_OFFSET_DEG呼叫一律不允許。

## 11. 驗收條件

- [ ] MotionProfile精確為五欄指定順序。
- [ ] 兩個aggregate initializer逐欄正確，RX／RY值未改。
- [ ] tiltRyDeg、moveBackMm及getTiltOffset呼叫消失。
- [ ] RZ不加入YAW_OFFSET_DEG。
- [ ] 退化方向fail closed。
- [ ] 未加入任何Phase 2姿態搜尋或HRSDK功能。
- [ ] 文件／註解明示PRODUCTION_MOTION尚未實機驗證。

## 12. 回滾方式

- Revert本ticket commit。
- 若P1-10已遷移test_cueball或BilliardApp，先回滾P1-10。
- 必須整體回滾header、initializers與MotionPlanner，避免欄位錯位。

## 13. Safety Critical限制

- 禁止把現有`RX=0, RY=-180`改成訪談預期值；Phase 1只改語意，不改值。
- 禁止使用RY做XY三角位移。
- 禁止將編譯成功宣告為姿態安全。
- 禁止執行任何產生的MotionPlan。

## 14. 完成後應產生的Git diff範圍

```text
M  src/BilliardConfig.h
M  src/BilliardConfig.cpp
M  src/MotionPlanner.h
M  src/MotionPlanner.cpp
M  tests/phase1_core_tests.cpp
```

## 15. 建議commit message

```text
refactor(motion): clarify motion profile fields
```

