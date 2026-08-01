# P1-03 — Existing 32-Value Contract, Parser and Cycle Boundary

**Status:** Ready for Implementation

**Blocked by:** P1-02 — Completed

## 1. ID

P1-03

## 2. Title

既有32值External Contract、本地Cycle／Session Boundary、嚴格Parser與單幀驗證。

## 3. Status

Ready for Implementation；這是下一個未完成能力。

## 4. Purpose

在不改Python wire的前提下，讓既有SocketClient、VisionDataParser與BilliardApp只把CameraPose settle後、本cycle的新32值Base0 XY frame轉成可稽核ReceiveEvent與SingleFrameResult。

## 5. Approved Spec References

- Master Spec §5.1～§6、§9 P1-03、§10.1。
- Python–C++ External Contract §4～§10。
- Phase 1 Shot Brain Spec §4 P1-03、§5 Data Lifecycle。

## 6. Existing Responsibility Owners

- `SocketClient.h/.cpp`：唯一production TCP transport、newline framing、buffer flush與disconnect結果。
- `VisionDataParser.h/.cpp`：嚴格32值Parser。
- `TableState.h`：Parsed／Validated frame、ReceiveEvent與Result domain types的原地演進。
- `BilliardConfig.h/.cpp`：最大frame長度、timeout、觀測bounds及必要config。
- `BilliardApp.h/.cpp`：CameraPose後capture-window gate及必要整合。

## 7. Existing Files Expected to Change

- 上述既有owner、`tests/phase1_core_tests.cpp`及必要的`.vscode/tasks.json`離線target清單。

## 8. Existing Files Explicitly Not to Duplicate

- 不得新增第二個SocketClient、VisionDataParser、BilliardApp、`ProductionVisionTransportAdapter`、AttestedVisionSession服務或平行v2 input tree。

## 9. Scope

- 保留newline-delimited 32-value CSV與既有欄位順序。
- exact token count、non-empty token、完整數值消費、finite、overflow、paired `-9999`及single-side sentinel拒絕。
- maximum frame byte length、clean close與transport error區分。
- 本地connection identity、shot-cycle identity、嚴格遞增ReceiveEvent ID與monotonic receive time。
- CameraPose成功／stopped／settle後flush舊buffer、reset累積並開capture window。
- 六袋wire XY與球資料進入ValidatedVisionFrame；具名Result與Diagnostic分離。

## 10. Out of Scope

- Python protocol重設、mandatory RuntimeCalibrationAttestation、sender frame ID／timestamp、相機補償、三幀演算法、ShotBrain、HRSDK、DO與真實硬體。

## 11. Preconditions

- P1-01、P1-02 Completed；四份Approved Specs是唯一需求權威。

## 12. Dependencies

- Blocked by P1-02；完成後解鎖P1-04。

## 13. Detailed Requirements

1. 只接受恰好32個完整finite數值token，index 0–17為1～9號球、18–19為母球、20–31為六袋。
2. 成對`(-9999,-9999)`轉nullopt；單邊sentinel整frame拒絕；Parser後不保留sentinel語意。
3. 1～9號球各自都可合法使用paired sentinel表示不存在；九顆全部不存在仍可形成ValidatedVisionFrame，不是Parser failure。
4. Parser只負責wire與single-frame validation，不選target、不判斷lowest-number target，也不產生`NoEligibleTarget`。
5. Parser不得修改Base0 planar XY原值，不得做bounds以外的幾何可行性、相機補償或座標轉換。
6. 一個Start建立一個shot-cycle identity；CameraPose gate開啟前資料全部丟棄。
7. reconnect、timeout、Parser failure、cycle change或flush使舊event失效並通知P1-04 reset。
8. V1 freshness只證明本地receive事件；無法證明不同camera exposure，記為Known Non-Blocking V1 Limitation。

## 14. Fail-Closed Requirements

- malformed、partial、超長、NaN、Infinity、overflow、必要母球／六袋缺失，或任一座標格式／sentinel規則非法，均不得產生ValidatedVisionFrame。
- 合法paired sentinel所表示的編號球absence不是失敗；不得因不存在任何編號球而拒絕frame或產生target層結果。
- Diagnostic不得被當作Point、frame、StableTableState或Plan；不得以0、raw Point或partial frame fallback。

## 15. Acceptance Criteria

- [ ] 既有32值wire與Python完全相容，無新control handshake。
- [ ] 只有capture window開啟後、同connection／cycle的合法frame可形成ReceiveEvent。
- [ ] Parser完整覆蓋格式、finite與exact sentinel規則。
- [ ] 1～9號球全部為合法paired sentinel時Parser成功；Target qualification留給P1-06。
- [ ] 六袋wire XY保留固定ID並可進P1-04。
- [ ] SocketClient clean close、error、timeout及stale buffer可區分且會reset。
- [ ] Phase 1離線測試不開真實Socket。

## 16. Test Requirements

- fake byte／connection／cycle source；覆蓋31／32／33欄、空token、尾隨字元、sentinel、非finite、超長frame。
- Case A（Parser段）：1～9號球全部為paired sentinel且母球／六袋合法，結果為成功ValidatedVisionFrame並可進P1-04。
- Case B：任一Point只有單邊sentinel，結果為`InvalidFormat`且不得進P1-04。
- 覆蓋CameraPose前frame、flush、event ID、跨cycle、disconnect／reconnect與相同payload的新本地event。
- 證明Parser沒有CameraCompensator、TableFrame→Base0、ShotBrain或硬體依賴。

## 17. Hardware Level

完全離線；production Socket integration只做compile／contract驗證，不執行真實連線。

## 18. Regression Requirements

- 保留現有32值欄位順序、newline framing及Base0 XY原值。
- P1-01／P1-02測試保持綠色；不得破壞SocketClient既有唯一owner地位。

## 19. Definition of Done

- AC與錯誤注入全部通過；source／dependency audit無重複transport或相機補償；單一可回滾commit。

## 20. Requirement Traceability

- Refactor／Rename自舊P1-04 strict parser。
- Merge舊P1-03仍有效的finite、fail-closed、diagnostics與tests；C++ CameraCompensator核心責任Superseded。
- Merge舊P1-05仍有效的單幀validation、bounds、Result與Diagnostic；補償與新VisionFrameProcessor責任Superseded。

## 21. New File Justification

None expected。優先原地修改既有owners；不得先建立新production helper再找理由。
