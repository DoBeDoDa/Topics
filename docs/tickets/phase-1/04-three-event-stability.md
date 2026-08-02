# P1-04 — Three-Event Stability Lifecycle

Status: Ready for Implementation
Blocked by: P1-03 — Completed

## 1. ID

P1-04

## 2. Title

同一shot cycle三事件穩定生命週期與StableTableState。

## 3. Status

Planned

## 4. Purpose

只在三個合法、連續、同cycle ReceiveEvents的球與六袋都一致時，產生唯一可交給ShotBrain的StableTableState。

## 5. Approved Spec References

- Master Spec §5.2、§6、§9 P1-04。
- External Contract §6～§10。
- Phase 1 Spec §4 P1-04、§5～§6。

## 6. Existing Responsibility Owners

- `TableState.h`、`BilliardConfig.h/.cpp`及現有`BilliardApp`資料生命週期。
- 只有證明無合理existing owner時，才可新增唯一穩定狀態helper。

## 7. Existing Files Expected to Change

- `TableState.h`、`BilliardConfig.h/.cpp`、必要的`BilliardApp` integration、`tests/phase1_core_tests.cpp`與離線task。

## 8. Existing Files Explicitly Not to Duplicate

- 不得新增Phase1Pipeline.cpp、第二個Parser、第二個BilliardApp或多個stability owners。

## 9. Scope

- ReceiveEvent stream、NeedMoreEvents／Stable／Unstable／TimedOut與Phase1PipelineResult。
- 球及固定ID六袋presence、三幀X/Y median、各自tolerance與完全reset。
- source event IDs、connection／cycle identity與穩定pocket centers進StableTableState。

## 10. Out of Scope

- CSV parsing、Socket transport、candidate generation、Robot與硬體。

## 11. Preconditions

- P1-03合法SingleFrameResult與cycle reset signal可用。

## 12. Dependencies

- Blocked by P1-03；完成後解鎖P1-05。

## 13. Detailed Requirements

1. 恰好三個嚴格遞增event；不得跨connection、cycle或timeout。
2. 母球與六袋必須符合Approved Contract；1～9號球的presence pattern必須在三個ReceiveEvents間一致，但可以合法為九顆全部absent。
3. 每個存在物件X/Y分別取median；球與袋口使用各自具名mm tolerance；absent編號球不得生成假Point或default Point。
4. 任一invalid event、必要母球／六袋缺失、存在物件jump、編號球presence pattern改變、disconnect或timeout清空全部，下一合法event重新算第1幀。
5. 只有Stable status可攜帶StableTableState。
6. `CueBall=present`、`Ball1..Ball9=all absent`且六袋合法／穩定時必須可形成StableTableState；這不是NoPlan或Pipeline failure。

## 14. Fail-Closed Requirements

- 不得以單幀／兩幀、平均值、partial state、舊cycle event或預設Point建立StableTableState。

## 15. Acceptance Criteria

- [ ] 三event、presence、median、tolerance與reset規則完整。
- [ ] 三個event皆為zero-object-ball presence pattern時可形成StableTableState，且所有編號球維持absent。
- [ ] 六袋與球使用同一cycle，無unstable pocket／stable ball混用。
- [ ] Pipeline failure不成為NoPlan。
- [ ] StableTableState只有單一production owner。

## 16. Test Requirements

- identical／within tolerance／boundary／jump／missing／presence change／cross-cycle／timeout／reconnect fixtures。
- 驗證X/Y median可來自不同event；所有非Stable status沒有success value。
- Case A（Stability段）：三個合法ReceiveEvents皆為1～9號球全部absent，可形成StableTableState。
- Case C：三幀object-ball presence pattern不同，結果為`Unstable`並完全reset；新合法event重新作為第1幀。
- Case B邊界確認：P1-03拒絕的單邊sentinel不得進入stability accumulation。

## 17. Hardware Level

完全離線，使用fake events。

## 18. Regression Requirements

- P1-03 Parser及P1-01／P1-02保持綠色；不改32值wire。

## 19. Definition of Done

- 離線測試、dependency audit與reset error injection通過；單一commit可回滾。

## 20. Requirement Traceability

- Rename／Refactor舊P1-06 StableFrameValidator。
- 接收舊P1-05的presence／single-frame failure propagation，但不接收相機補償。

## 21. New File Justification

Default：None expected。若跨幀狀態無合理owner且塞入既有類別會破壞單一責任，才可新增一個唯一helper，ticket內須記錄證據與被取代owner；名稱不預設。
