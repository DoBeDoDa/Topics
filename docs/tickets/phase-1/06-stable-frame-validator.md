# P1-06 — StableFrameValidator與TableState

**Status:** ready-for-agent

**Blocked by:** P1-05 — VisionFrameProcessor與ProcessedVisionFrame

## 1. Ticket ID與標題

P1-06：以固定三個單幀Valid輸入建立唯一可供策略使用的穩定TableState。

## 2. 目的

完成純離線三幀穩定切片：presence pattern一致、各軸中位數與歐氏距離均通過
後才產生TableState。這張票不負責Socket收集、等待或CompetitionAuto。

## 3. 前置依賴

- P1-05完成。
- ProcessedVisionFrame只代表單幀Valid。

## 4. 新增檔案

- `src/StableFrameValidator.h`
- `src/StableFrameValidator.cpp`

## 5. 修改檔案

- `src/TableState.h`，固定TableState為三幀穩定後型別並移除DetectedPoint。
- `tests/phase1_core_tests.cpp`
- `.vscode/tasks.json`，將StableFrameValidator加入core test target。

## 6. 明確不得修改的檔案與行為

- 不修改SocketClient、BilliardApp、TargetSelector或Algorithm。
- 不建立三幀Socket queue或重試狀態機。
- 不填入正式stableFrameToleranceMm。
- 不讓Parser或VisionFrameProcessor直接產生TableState。
- 不使用平均數取代中位數。

## 7. API或資料型別變更

- 新增`StableFrameConfig{optional<double> stableFrameToleranceMm}`。
- 新增`StableFrameStatus`：
  Stable、Unstable、InvalidInput、InvalidConfiguration、
  ConfigurationMissing。
- 新增`StableFrameResult`：
  Stable必須有TableState；其他status必須為nullopt。
- `validate(const std::array<ProcessedVisionFrame,3>&)`。
- TableState與Parsed／Processed為不同具名型別。

## 8. 逐步實作內容

1. 缺少tolerance回ConfigurationMissing。
2. tolerance非有限或小於0回InvalidConfiguration。
3. 防禦性拒絕任何非有限輸入點。
4. 比較9顆object ball、cue ball及6個pocket的全部presence bits。
5. 任一presence差異回Unstable。
6. 對每個三幀皆存在的物件，X與Y分別取三值中位數。
7. 使用`std::hypot`計算各幀Point到中位數Point的距離。
8. 任一距離大於tolerance回Unstable；等於tolerance通過。
9. 三幀皆缺失的物件在TableState保留nullopt。
10. 全部通過後才一次建立TableState。

## 9. 測試案例

- 三幀相同。
- 三幀不同但全部在tolerance內。
- X與Y中位數分別來自不同幀。
- object ball、cue ball或pocket presence變動。
- 任一距離超過tolerance。
- 距離剛好等於tolerance。
- tolerance缺失、負值、NaN與Infinity。
- 輸入Point含NaN或Infinity。
- 非必要物件三幀皆missing。
- 任一上游SingleFrameResult非Valid時，pipeline不得呼叫validator。
- 所有status／optional payload不變量。

## 10. 實際測試或建置命令

```powershell
cl.exe /std:c++17 /EHsc /nologo /utf-8 /I .\src /I .\tests `
  .\tests\phase1_core_tests.cpp `
  .\src\MathUtils.cpp `
  .\src\CameraCompensator.cpp `
  .\src\VisionDataParser.cpp `
  .\src\VisionFrameProcessor.cpp `
  .\src\StableFrameValidator.cpp `
  /Fe:.\bin\phase1_core_tests.exe

& .\bin\phase1_core_tests.exe
```

## 11. 驗收條件

- [ ] 三幀presence演算法完整測試。
- [ ] X/Y分別取中位數，未使用平均數或整幀Point替代。
- [ ] tolerance缺失及無效設定明確分流。
- [ ] 只有Stable攜帶TableState。
- [ ] TableState只由StableFrameValidator成功產生。
- [ ] 沒有Socket、H按鈕或CompetitionAuto程式。

## 12. 回滾方式

- Revert本ticket commit。
- 若P1-08及下游已接受TableState，先回滾下游。
- P1-05單幀處理可獨立保留。

## 13. Safety Critical限制

- 禁止單幀或兩幀資料直接建立TableState。
- 禁止presence不一致時合併座標。
- 禁止tolerance缺失時使用0或猜測值。
- 禁止failure status攜帶TableState。

## 14. 完成後應產生的Git diff範圍

```text
A  src/StableFrameValidator.h
A  src/StableFrameValidator.cpp
M  src/TableState.h
M  tests/phase1_core_tests.cpp
M  .vscode/tasks.json
```

## 15. 建議commit message

```text
refactor(vision): add stable frame validation
```

