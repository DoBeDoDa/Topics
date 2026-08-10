# P1-01 — C++17建置與離線測試框架

**Status:** Completed

**Blocked by:** None — can start immediately

## 1. Ticket ID與標題

P1-01：C++17建置與兩個完全隔離的Phase 1離線測試骨架。

## 2. 目的

先建立可重複執行、無硬體依賴的最小驗證通道。完成後，後續每張Phase 1 ticket
都能在不載入HRSDK、不開Socket及不操作手臂的情況下編譯與測試。

## 3. 前置依賴

- 無。
- 開始前確認工作樹，既有修改不得被覆寫或納入本ticket。

## 4. 新增檔案

- `tests/TestHarness.h`
- `tests/phase1_core_tests.cpp`
- `tests/phase1_algorithm_regression_tests.cpp`

## 5. 修改檔案

- `.vscode/tasks.json`

## 6. 明確不得修改的檔案與行為

- 不修改任何`src/*.h`或`src/*.cpp`。
- 不修改Python、HRSDK標頭或library。
- 不執行`main`、`calibrate`、`test_cueball`。
- 不讓兩個測試target編譯或link RobotController、SocketClient、BilliardApp或
  數位輸出程式。
- 不在本ticket重構任何核心API。

## 7. API或資料型別變更

- 不變更production API。
- `TestHarness`只提供最小斷言、失敗計數、近似浮點比較及process exit code。
- 測試target名稱固定為：
  - `phase1_core_tests`
  - `phase1_algorithm_regression_tests`

## 8. 逐步實作內容

1. 在所有現有MSVC task加入`/std:c++17`；GCC task加入`-std=c++17`。
2. 建立不依賴第三方套件的`TestHarness`。
3. 建立兩個各自擁有唯一`main()`的測試骨架。
4. 新增兩個MSVC build task與兩個run task。
5. 初始build task只編譯各自測試骨架及TestHarness，不先納入尚未重構的模組。
6. 確認輸出分別為`bin/phase1_core_tests.exe`及
   `bin/phase1_algorithm_regression_tests.exe`。
7. 確認run task只啟動上述兩個離線測試。

## 9. 測試案例

- 成功斷言不增加failure count。
- 失敗斷言會產生可讀訊息並增加failure count；可用局部harness self-test驗證，
  最終正式測試仍須exit 0。
- `expectNear`能接受容差內差異並拒絕容差外差異。
- 兩個測試執行檔均可獨立啟動並以0結束。
- 依賴清單中沒有HRSDK、RobotController、SocketClient或DO程式。

## 10. 實際測試或建置命令

```powershell
New-Item -ItemType Directory -Force -Path .\build,.\bin | Out-Null

cl.exe /std:c++17 /EHsc /nologo /utf-8 /I .\tests `
  .\tests\phase1_core_tests.cpp `
  /Fe:.\bin\phase1_core_tests.exe

cl.exe /std:c++17 /EHsc /nologo /utf-8 /I .\tests `
  .\tests\phase1_algorithm_regression_tests.cpp `
  /Fe:.\bin\phase1_algorithm_regression_tests.exe

& .\bin\phase1_core_tests.exe
& .\bin\phase1_algorithm_regression_tests.exe
```

等價的VS Code task標籤必須能完成相同build與run。

## 11. 驗收條件

- [x] 所有相關task明確使用C++17。
- [x] 兩個測試骨架都可獨立編譯與執行。
- [x] 兩個測試執行檔exit code均為0。
- [x] 測試target沒有任何HRSDK／RobotController／SocketClient／DO依賴。
- [x] production原始碼零變更。

## 12. 回滾方式

- Revert本ticket單一commit。
- 移除新增測試骨架及新增task，恢復原tasks內容。
- 不使用`git reset --hard`。

## 13. Safety Critical限制

- 測試task不得誤用現有`Build with HRSDK`、Calibration或Cueball task作為依賴。
- run task不得指向`main.exe`、`calibrate.exe`或`test_cueball.exe`。
- 不得因只是空骨架就省略依賴隔離檢查。

## 14. 完成後應產生的Git diff範圍

```text
M  .vscode/tasks.json
A  tests/TestHarness.h
A  tests/phase1_core_tests.cpp
A  tests/phase1_algorithm_regression_tests.cpp
```

不得包含`src/`、`python/`、HRSDK或其他文件修改。

## 15. 建議commit message

```text
test: add isolated C++17 phase 1 harness
```

## 16. Approved Spec References

- Master Spec §9 P1-01、§10 Testing Decisions。
- Phase 1 Shot Brain Spec §4 P1-01、§16。

## 17. Existing Responsibility Owners

- `.vscode/tasks.json`與既有`tests/TestHarness.h`、兩個Phase 1離線測試target。
- Existing Files Explicitly Not to Duplicate：不得建立第二套test harness或平行build system。

## 18. Hardware Level與Regression Requirements

- Hardware Level：完全離線；禁止Socket、HRSDK、RobotController與DO。
- Regression：後續P1-03至P2-03不得破壞兩個離線target的隔離與C++17建置。

## 19. Definition of Done與完成證據

- Implementation commit：`bca0eb4`。
- Completed scope與existing tests保留，不重建、不改回Planned。
- Approved Specs revalidation：PASS（2026-08-01），無新增delta requirement。

## 20. Requirement Traceability

- 舊P1-01全部有效要求由本Completed ticket保留。
- 十二能力coverage：P1-01，Coverage Complete = YES。

## 21. New File Justification

- None expected；本票已完成，後續不得以同一能力建立新ticket或新框架。
