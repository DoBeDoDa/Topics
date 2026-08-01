# P1-05 — VisionFrameProcessor與ProcessedVisionFrame

**Status:** Superseded — historical only, do not implement

**Blocked by:** P1-03 — CameraCompensator；P1-04 — ParsedVisionFrame與嚴格Parser

## Superseded Classification and Requirement Migration

- Classification：Superseded；核心「C++單幀相機補償」與新增`VisionFrameProcessor.cpp/.h`責任被Approved Specs禁止。
- strict single-frame validation、bounds config、success value／Diagnostic、partial payload禁止：遷移至active P1-03。
- 三event presence／median／reset：遷移至active P1-04。
- PlayableBallCenterRegion及table／pocket safety geometry：遷移至active P1-05。
- CameraCompensator dependency、raw→compensated Point及ProcessedVisionFrame補償語意：obsolete，不遷移。
- 本文件只供Git歷史追溯，不是Approved需求、active ticket或實作依據。

## 1. Ticket ID與標題

P1-05：建立單幀補償與驗證切片，成功時只產生ProcessedVisionFrame。

## 2. 目的

將ParsedVisionFrame轉成已補償、已驗證的單幀資料，並在設定缺失、補償失敗、
桌面外座標或必要資料缺失時fail closed。此ticket不收集或比較三幀。

## 3. 前置依賴

- P1-03完成，CameraCompensator API穩定。
- P1-04完成，Parser只輸出ParsedVisionFrame。

## 4. 新增檔案

- `src/VisionFrameProcessor.h`
- `src/VisionFrameProcessor.cpp`

## 5. 修改檔案

- `src/TableState.h`，加入ProcessedVisionFrame。
- `tests/phase1_core_tests.cpp`
- `.vscode/tasks.json`，將VisionFrameProcessor加入core test target。

## 6. 明確不得修改的檔案與行為

- 不加入StableFrameValidator或三幀容器。
- 不修改TargetSelector、Algorithm、BilliardApp或SocketClient。
- 不填入Base0桌面bounds production值。
- 不將補償失敗的raw Point當fallback。
- 不讓VisionFrameProcessor產生TableState。

## 7. API或資料型別變更

- 新增`ProcessedVisionFrame`，欄位形狀與ParsedVisionFrame相同，但語意不同。
- 新增`VisionProcessingConfig`，其中tableBoundsMm為optional。
- 新增`SingleFrameStatus`：
  Valid、CompensationFailed、MissingRequiredData、OutsideTableBounds、
  Invalid、ConfigurationMissing。
- 新增`SingleFrameResult`：
  Valid必須有ProcessedVisionFrame；其他status必須為nullopt。
- `VisionFrameProcessor::process(const ParsedVisionFrame&)`。

## 8. 逐步實作內容

1. 驗證bounds設定存在、有限且min小於max。
2. 對每個present Point呼叫CameraCompensator；nullopt保持nullopt。
3. 任一補償失敗立即回CompensationFailed，不產生partial payload。
4. 對所有補償後present Point做有限值與Base0 bounds驗證。
5. 依現有策略必要資料驗證cue ball、至少一顆object ball、至少一個可選袋口，
   以及目前反彈邊界所需p2、p3。
6. 只有全部成功才建構ProcessedVisionFrame。
7. 不把任何未校正bounds寫入BilliardConfig。

## 9. 測試案例

- 合法ParsedVisionFrame加fixture bounds產生Valid ProcessedVisionFrame。
- 每個present Point都套用一次既有補償公式。
- nullopt物件保持nullopt且不呼叫compensate。
- bounds缺失為ConfigurationMissing。
- bounds含非有限值、min>=max為Invalid。
- 補償失敗為CompensationFailed。
- 補償後任一點桌面外為OutsideTableBounds。
- cue ball、全部object ball、必要袋口缺失為MissingRequiredData。
- 所有非Valid status payload均為nullopt。
- 小於`-9000.0`的有限座標由Parser保留，再由本層範圍拒絕。

## 10. 實際測試或建置命令

```powershell
cl.exe /std:c++17 /EHsc /nologo /utf-8 /I .\src /I .\tests `
  .\tests\phase1_core_tests.cpp `
  .\src\MathUtils.cpp `
  .\src\CameraCompensator.cpp `
  .\src\VisionDataParser.cpp `
  .\src\VisionFrameProcessor.cpp `
  /Fe:.\bin\phase1_core_tests.exe

& .\bin\phase1_core_tests.exe
```

## 11. 驗收條件

- [ ] 所有單幀status測試通過。
- [ ] Valid是唯一可攜帶ProcessedVisionFrame的狀態。
- [ ] 設定缺失與非法設定明確分流。
- [ ] 任一點失敗使整幀fail closed。
- [ ] VisionFrameProcessor沒有三幀、Socket、TargetSelector或TableState生產責任。

## 12. 回滾方式

- Revert本ticket commit。
- 若P1-06及下游已使用ProcessedVisionFrame，先回滾下游。
- CameraCompensator與Parser可獨立保留。

## 13. Safety Critical限制

- 禁止缺少bounds時繼續production路徑。
- 禁止忽略單一球或袋口的補償／範圍失敗。
- 禁止產生partial ProcessedVisionFrame。
- 禁止填入猜測的桌面範圍。

## 14. 完成後應產生的Git diff範圍

```text
A  src/VisionFrameProcessor.h
A  src/VisionFrameProcessor.cpp
M  src/TableState.h
M  tests/phase1_core_tests.cpp
M  .vscode/tasks.json
```

## 15. 建議commit message

```text
refactor(vision): validate processed frames
```
