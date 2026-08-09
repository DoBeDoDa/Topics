# P2-02 — Fake Complete Execution State Machine

**Status:** Planned

**Blocked by:** P2-01

## 1. ID

P2-02

## 2. Title

fake／offline完整單cycle執行狀態機與雙DO安全。

## 3. Status

Planned

## 4. Purpose

用fake Motion／Pneumatic adapters驗證從Start、CameraPose、capture、planning、StrikeReady、雙DO、actual-pose SafeLift、return到WaitingForStart的完整且fail-closed單cycle。

## 5. Approved Spec References

- Master Spec §6～§7、§9 P2-02。
- Phase 2 Spec §4 P2-02、§7～§14。

## 6. Existing Responsibility Owners

- `BilliardApp.h/.cpp`：唯一application cycle orchestration。
- `MotionPlanner.h/.cpp`：ExecutionPlan consumption contract。
- `RobotController.h/.cpp`介面與`BilliardConfig.h/.cpp`。
- fake adapters只位於`tests/`或test fixtures。

## 7. Existing Files Expected to Change

- 上述production interfaces、tests中的fake adapters／executor fixtures及必要build task。

## 8. Existing Files Explicitly Not to Duplicate

- 不得新增第二個BilliardApp、RobotController、MotionPlanner、SocketClient或production fake controller。

## 9. Scope

- WaitingForStart→StartRequested→CameraPose gate→capture／planning result→ExecutionPlan validation→StrikeReady→dual DO→actual pose SafeLift→CameraPose→WaitingForStart。
- ExecutionPolicy、FixedForceEnvelope、MotionAdapter／PneumaticAdapter Results、SafeFailure與UnknownUnsafe。
- fake Motion／Pneumatic adapters僅是automated/offline test harness，
  不是production runtime mode；本票不新增Fake production mode。

## 10. Out of Scope

- real HRSDK／DO、真實Socket執行、真實機械手臂與連續自動擊球。

## 11. Preconditions

- P2-01 ExecutionPlan contract完成；Phase 1可用fixtures提供ShotPlan／NoPlan。

## 12. Dependencies

- Blocked by P2-01；完成後解鎖P2-03。

## 13. Detailed Requirements

1. 一個Start只允許一個cycle；CameraPose stopped／settle／flush／reset後才收event。
2. NoPlan完成cycle但不擊發；ShotPlan需通過policy、calibration與FixedForceEnvelope。
3. Robot到StrikeReady並停止後，整個DO1 ON/OFF、delay、DO2 ON/OFF、completion wait期間Pose固定。
4. DO1／DO2永遠互斥且共用單一pneumaticPulseMs。
5. 氣動完成後讀actual pose，第一個motion只能是checked vertical LIN；完成前禁止Camera PTP。
6. SafeLift後才可PTP CameraPose，確認停止後CycleCompleted→WaitingForStart；不自動下一桿。

## 14. Fail-Closed Requirements

- motion／LIN／DO／communication failure禁止後續正常命令；LIN失敗不得PTP fallback。
- OFF失敗或狀態未知進UnknownUnsafe，禁止safe lift、return與新cycle。

## 15. Acceptance Criteria

- [ ] fake完整正常cycle recorded commands精確符合spec。
- [ ] 每個狀態precondition／success／failure transition可注入測試。
- [ ] 氣動期間無Robot motion，post-strike無TCPretreat。
- [ ] actual-pose vertical LIN與CameraPose gate順序正確。
- [ ] UnknownUnsafe為terminal。

## 16. Test Requirements

- Start重入、Camera move／settle、NoPlan、policy／envelope拒絕、reachable／LIN／motion failure、DO每一步failure、timeout／disconnect、actual pose failure、safe-lift與return failure。
- 驗證recorded command exact ordering、互斥、同pulse與無禁止命令。

## 17. Hardware Level

完全離線fake／mock；不得載入HRSDK或操作DO。

## 18. Regression Requirements

- Phase 1與P2-01 tests保持綠色；fake不得成為production RobotController副本。

## 19. Definition of Done

- 正常與全部error-injection paths通過；duplicate owner audit為NO；單一commit。

## 20. Requirement Traceability

- Split自舊P1-10 BilliardApp整合責任。
- Merge舊P1-11的fail-closed／dependency／acceptance要求。
- 不建立新的Start ticket；完整cycle由本能力承接。

## 21. New File Justification

None expected for production；新Markdown是舊App ticket split。fake檔案只可在tests中，須說明其interface seam。
