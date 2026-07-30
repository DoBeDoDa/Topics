# P1-04 — ParsedVisionFrame與嚴格VisionDataParser

**Status:** ready-for-agent

**Blocked by:** P1-02 — 基礎型別、GeometryResults與MathUtils

## 1. Ticket ID與標題

P1-04：建立ParsedVisionFrame及精確、transaction式的32欄CSV Parser。

## 2. 目的

在wire protocol邊界消除`DetectedPoint{bool, Point}`與範圍式sentinel判斷。
完成後，Parser只產生已完成語法、有限值及sentinel處理的
`ParsedVisionFrame`，不產生補償座標或核心TableState。

## 3. 前置依賴

- P1-02完成。
- Point與status+optional Result不變量已固定。

## 4. 新增檔案

- 無；`ParsedVisionFrame`放入既有純資料型別檔案。

## 5. 修改檔案

- `src/TableState.h`
- `src/VisionDataParser.h`
- `src/VisionDataParser.cpp`
- `tests/phase1_core_tests.cpp`
- `.vscode/tasks.json`，將Parser source加入core test target。

## 6. 明確不得修改的檔案與行為

- 不修改Python 32欄順序、sentinel值或換行框架。
- 不修改CameraCompensator、TargetSelector、Algorithm、MotionPlanner。
- 不執行相機補償、桌面bounds驗證或必要球驗證。
- 不使用`MISSING_COORDINATE_LIMIT`或`<-9000`判斷missing。
- 不開Socket、不執行SocketClient。

## 7. API或資料型別變更

- 新增`ParsedVisionFrame`，包含9顆object ball、cue ball及6個pocket的
  `std::optional<Point>`。
- 新增`VisionParseStatus`：
  Success、InvalidFieldCount、InvalidToken、NonFiniteValue、
  InvalidSentinelPair。
- 新增`VisionParseResult`：
  Success必須有ParsedVisionFrame，其他status必須為nullopt。
- Parser成功輸出只能是ParsedVisionFrame。
- wire層只保留：
  `inline constexpr double MISSING_COORDINATE_SENTINEL = -9999.0;`

## 8. 逐步實作內容

1. 依現有順序定義ParsedVisionFrame，不改32欄mapping。
2. 精確分割32個token，拒絕不足、過多及空token。
3. 每個token必須完整轉換，不接受尾隨字元。
4. 拒絕NaN、Infinity及數值轉換overflow。
5. 只有`x == -9999.0 && y == -9999.0`轉為nullopt。
6. 單軸精確等於`-9999.0`回InvalidSentinelPair。
7. 其他任何有限值，包括`-9998.0`及小於`-9000.0`的值，保留為present Point。
8. 先建構局部結果；任一欄失敗時不得回傳partial frame。
9. 將Parser加入core test target。

## 9. 測試案例

- 32個合法有限值及欄位mapping。
- 31欄、33欄。
- 空token。
- `1.0abc`等尾隨字元。
- NaN、+Infinity、-Infinity及overflow。
- 精確成對`(-9999.0,-9999.0)`轉nullopt。
- X-only及Y-only sentinel均為InvalidSentinelPair。
- `(-9998.0,-9998.0)`仍是present。
- 其他小於`-9000.0`有限座標仍是present。
- 所有success／failure status與optional payload不變量。

## 10. 實際測試或建置命令

```powershell
cl.exe /std:c++17 /EHsc /nologo /utf-8 /I .\src /I .\tests `
  .\tests\phase1_core_tests.cpp `
  .\src\MathUtils.cpp `
  .\src\VisionDataParser.cpp `
  /Fe:.\bin\phase1_core_tests.exe

& .\bin\phase1_core_tests.exe

rg -n "MISSING_COORDINATE_LIMIT|[<>]=?\\s*-9000\\.0" `
  .\src\VisionDataParser.h .\src\VisionDataParser.cpp
```

最後一個搜尋不得找到門檻式missing判斷。

## 11. 驗收條件

- [ ] 32欄合法frame正確產生ParsedVisionFrame。
- [ ] 所有格式及非有限輸入fail closed。
- [ ] 只有精確成對`-9999.0`成為nullopt。
- [ ] 單軸sentinel使整幀失敗。
- [ ] `-9998.0`及其他小於`-9000.0`的有限值不被當成missing。
- [ ] Parser沒有補償、bounds或策略責任。

## 12. 回滾方式

- Revert本ticket commit。
- 若P1-05及下游已使用ParsedVisionFrame，先反向回滾下游。
- 不修改Python做回滾配合。

## 13. Safety Critical限制

- 禁止容忍partial frame。
- 禁止把非法token轉成0或sentinel。
- 禁止以近似浮點比較辨認sentinel。
- sentinel不得流入MathUtils、Physics、Algorithm或MotionPlanner。

## 14. 完成後應產生的Git diff範圍

```text
M  src/TableState.h
M  src/VisionDataParser.h
M  src/VisionDataParser.cpp
M  tests/phase1_core_tests.cpp
M  .vscode/tasks.json
```

## 15. 建議commit message

```text
refactor(vision): parse strict optional frames
```

