# P1-09 — Phase 1 Integration and Auditable ShotPlan/NoPlan

**Status:** Completed
**Implementation commit:** 987c8696754cf073dc38b57c48be69631852748f

AC: PASS
Scope: PASS
Tests: PASS
Compile: PASS
Critical: None
High: None
Medium: None

Deferred / non-blocking:
BilliardApp::processReceiveEvent完整硬體耦合狀態機測試延至P2-02；
P1-09純Pipeline／Brain seams已覆蓋。

**Blocked by:** P1-08

## 1. ID

P1-09

## 2. Title

Phase 1 pipeline／ShotBrain概念整合與可稽核ShotPlan／NoPlan。

## 3. Status

Completed

## 4. Purpose

由既有BilliardApp、TableState與Algorithm原地整合P1-03～P1-08，使合法StableTableState產生完整PlanningResult，且所有pipeline failure與NoPlan分層清楚。

## 5. Approved Spec References

- Master Spec §5.2、§6、§9 P1-09。
- Phase 1 Spec §5、§14～§17。

## 6. Existing Responsibility Owners

- `BilliardApp.h/.cpp`：Phase1Pipeline協調。
- `Algorithm.h/.cpp`：ShotBrain概念整合。
- `TableState.h`與既有domain headers：PlanningResult、ShotPlan、NoPlan與audit型別。

## 7. Existing Files Expected to Change

- 上述owners、Phase 1離線tests與必要task source清單。

## 8. Existing Files Explicitly Not to Duplicate

- 不得新增ShotBrain.cpp、Phase1Pipeline.cpp、第二個BilliardApp、第二個Algorithm或平行v2 source tree。

## 9. Scope

- `ReceiveEvent stream → Phase1PipelineResult`與獨立`StableTableState + configs → PlanningResult` seams。
- ShotPlan Common／Pot／Kick audit fields、source event／cycle identity、FixedForceMode與CandidateDiagnostics摘要。
- `PlanningResult = ShotPlan | NoPlan`；pipeline errors不成為NoPlan。

## 10. Out of Scope

- Robot Pose、Z／A／B／C、Socket執行、HRSDK、DO、main／calibrate／test_cueball執行。

## 11. Preconditions

- P1-03～P1-08完成且離線tests綠色。

## 12. Dependencies

- Blocked by P1-08；完成後解鎖P2-01。

## 13. Detailed Requirements

1. ShotBrain API只接受StableTableState、TableGeometryConfig、BrainConfig。
2. CSV、freshness、timeout、Parser與三幀累積留在pipeline owner。
3. ShotPlan只含Base0 planar XY撞球語意，不含Robot Pose／硬體欄位。
4. NoPlan只由合法StableTableState後的規劃層產生且無fallback plan。
5. BilliardApp整合不執行production Socket、RobotController或硬體於Phase 1驗收。

## 14. Fail-Closed Requirements

- Parser／stability／planning任一failure停止；Diagnostic不得轉成Point、Candidate、ShotPlan或ExecutionPlan。

## 15. Acceptance Criteria

- [x] Pipeline與Brain seams可分別離線重現。
- [x] PotOnly ShotPlan／NoPotCandidate及所有variant audit完整。
- [x] pipeline failures不列為NoPlanReason。
- [x] ShotPlan無Robot Z／Pose／HRSDK／DO。
- [x] 無ShotBrain.cpp或Phase1Pipeline.cpp要求。

## 16. Test Requirements

- end-to-end fake events到PlanningResult；Parser error、NeedMore、Unstable、timeout、NoPlan、Direct與Kick success。
- variant payload、success value／Diagnostic與CandidateDiagnostic isolation。

## 17. Hardware Level

完全離線；不開production Socket、不link或執行RobotController。

## 18. Regression Requirements

- P1-01～P1-08測試保持綠色；既有main／diagnostic call sites的後續compile migration不得用fallback繞過新契約。

## 19. Definition of Done

- Phase 1全離線coverage與dependency audit通過；單一可回滾commit；P2-01可只消費ShotPlan。

## 20. Requirement Traceability

- Refactor／Split自舊P1-10 BilliardApp integration。
- Merge舊P1-11的Phase 1 acceptance與dependency audit；硬體驗收移至P2-03。

## 21. New File Justification

None expected；概念責任由既有BilliardApp、Algorithm與TableState承接。
