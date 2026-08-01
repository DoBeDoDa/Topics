# P2-03 — Real HRSDK/Dual-DO Adapters and Controlled Acceptance

**Status:** Planned

**Blocked by:** P2-02

## 1. ID

P2-03

## 2. Title

既有RobotController／HRSDK／雙DO強化與受控真實硬體驗收。

## 3. Status

Planned — 唯一New coverage gap。

## 4. Purpose

在全部offline／fake gates通過後，原地強化既有RobotController與BilliardApp，逐級驗證真實motion、雙DO、垂直SafeLift與CameraPose return。

## 5. Approved Spec References

- Master Spec §5.3、§7、§9 P2-03、§10。
- Phase 2 Spec §4 P2-03、§10～§15。

## 6. Existing Responsibility Owners

- `RobotController.h/.cpp`：HRSDK、Tool／Base、reachable、PTP／LIN、DO及具名adapter results。
- `BilliardApp.h/.cpp`：既有cycle orchestration。
- `BilliardConfig.h/.cpp`：Tool1／Base0、CameraPose、calibration與timing。
- `main.cpp`：只有必要application entry整合。

## 7. Existing Files Expected to Change

- 上述owners及受控驗收文件／test fixture；不修改Python vision protocol。

## 8. Existing Files Explicitly Not to Duplicate

- 不得新增第二個RobotController、BilliardApp、MotionPlanner、HRSDK wrapper tree、SocketClient或parallel v2 application。

## 9. Scope

- SDK return／timeout／motion-state error mapping、Tool1／Base0顯式設定確認、reachable與motion_check_lin。
- dual DO互斥、同pulse、OFF evidence、PolicyAcceptedPneumaticCompletion與UnknownUnsafe。
- no-fire calibration verification、低速安全高度、氣壓安全狀態、單球可立即停止擊發、actual-pose SafeLift及CameraPose return。

## 10. Out of Scope

- 連續自動擊球、Python protocol修改、策略／評分修改、未驗證中繼路徑與LegalContact real execution預設啟用。

## 11. Preconditions

- P2-02全部fake/error-injection gates通過；人工急停、斷氣／斷電與現場安全程序可用。

## 12. Dependencies

- Blocked by P2-02；最終能力。

## 13. Detailed Requirements

1. 依序驗收adapter mapping、mock/no-hardware、connected no-move、低速safe-height motion、氣壓斷電DO、單球擊發、vertical SafeLift與CameraPose return。
2. no-fire確認Tool1 TCP、cueForwardAxisTool、CToolOffset、A/B/C mapping與Base0 +Z實體安全上方。
3. LIN前必須motion_check_lin；reachable不等於路徑安全；失敗不得改PTP。
4. DO failure best-effort雙OFF；無法確認進UnknownUnsafe並要求人工安全處置。
5. 一個Start一個cycle，不得連續自動驗收。

## 14. Fail-Closed Requirements

- 任一驗收級失敗停止升級；UnknownUnsafe禁止任何motion／return／再次擊發。
- 不得把OffCommandAccepted宣稱為PhysicalOffConfirmed。

## 15. Acceptance Criteria

- [ ] 全部八級驗收依序留下可稽核證據。
- [ ] Tool/Base、姿態、cue axis、+Z及FixedForceEnvelope已人工核准。
- [ ] DO互鎖、pulse、delay、completion與失效安全通過。
- [ ] post-strike第一motion為actual-pose vertical LIN，之後才CameraPose。
- [ ] 無連續自動擊球或重複production owner。

## 16. Test Requirements

- 先mock contract與error injection；真實階段採氣壓斷電、低速、安全高度、單球、人工可立即停止。
- 覆蓋SDK codes、timeout、disconnect、motion stopped、DO OFF accepted／unknown及急停流程。

## 17. Hardware Level

Controlled real hardware；只有本票最後階段允許HIWIN RA605-GC與真實DO。

## 18. Regression Requirements

- P1全部與P2-01／P2-02離線tests保持綠色；不得把hardware workaround下沉到Algorithm或MotionPlanner策略。

## 19. Definition of Done

- 全部階段證據與安全審查通過，無Critical／High finding，單一可回滾commit；不得以只compile／reachable宣告完成。

## 20. Requirement Traceability

- Classification：New，因完整Existing Ticket Inventory沒有任何舊票承接real HRSDK／dual-DO／controlled acceptance；舊P1-09明確排除HRSDK，舊P1-10／P1-11亦禁止硬體執行。
- 接收舊P1-11仍有效的最終dependency audit與安全驗收要求。

## 21. New File Justification

New ticket justified by verified coverage gap；不代表新增production cpp。實作仍原地修改既有RobotController、BilliardApp、BilliardConfig及必要main.cpp。
