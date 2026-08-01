# Billiards Existing Ticket Refactor Index

## Approved Requirements Authority

To Tickets已按Existing Ticket Refactoring完成。需求與驗收allowlist只有：

1. `docs/specs/billiards-system-refactor-master-spec.md`
2. `docs/specs/python-cpp-external-contract.md`
3. `docs/specs/phase-1-shot-brain-spec.md`
4. `docs/specs/phase-2-shot-executor-spec.md`

Archive與superseded tickets只作Git歷史及traceability，不是實作依據。

## Existing Ticket Inventory and Classification

| Old path/title/status | Current responsibility/dependencies | Existing files / tests / hardware | Valid requirements retained | Obsolete/conflicting requirements | Classification → Approved capability / owners |
|---|---|---|---|---|---|
| `phase-1/01-cpp17-offline-test-framework.md`；P1-01；原ready | C++17離線harness；無blocker | tasks、TestHarness、2 tests；offline | build隔離、無HRSDK／Socket、AC與test commands | 無 | **Completed/Keep** → P1-01；existing tasks/tests；commit `bca0eb4` |
| `phase-1/02-safe-math-and-geometry-types.md`；P1-02；原ready | safe primitives；blocked by P1-01 | Point／GeometryResults／MathUtils／core tests；offline | finite、optional、純Math、無fallback | 無 | **Completed/Keep** → P1-02；commit `216bcb7` |
| old `03-camera-compensator.md`；原ready | C++相機補償；blocked by P1-02 | proposed new CameraCompensator、core tests；offline | finite、fail-closed、diagnostics、tests | C++補償與新CameraCompensator被Specs禁止 | **Superseded/Merge** → valid parts P1-03；pure Math boundary P1-02；archived |
| old `04-strict-vision-parser.md`；原ready | 32值Parser；blocked by P1-02 | VisionDataParser／TableState／core tests；offline | exact 32、token、finite、paired sentinel、Result | 舊ID與缺少cycle semantics | **Refactor/Rename** → active P1-03；SocketClient／Parser／TableState／Config／App |
| old `05-single-frame-processor.md`；原ready | 補償後single frame；blocked by old 03/04 | proposed VisionFrameProcessor／TableState／tests；offline | bounds、config、Result、partial拒絕 | C++補償、ProcessedVisionFrame與新processor owner | **Superseded/Merge** → P1-03 validation、P1-04 stability、P1-05 bounds geometry；archived |
| old `06-stable-frame-validator.md`；原ready | 三幀median／presence；blocked by old 05 | TableState／proposed single helper／tests；offline | three-event、median、tolerance、reset | 舊dependency及未區分cycle／pocket tolerance | **Rename/Refactor** → P1-04；TableState／Config／App stability owner |
| old `07-safe-billiard-physics.md`；原ready | safe geometry；blocked by P1-02 | BilliardPhysics／GeometryResults／tests；offline | path status、intersection、degenerate拒絕 | 缺完整table／pocket／effective rail模型 | **Rename/Refactor** → P1-05；Physics／Point／Results／Config |
| old `08-target-selector-algorithm-migration.md`；原ready | target與舊策略；blocked by old 06/07 | TargetSelector／Algorithm／Physics／regression tests；offline | lowest target、invalid geometry、existing owners | 舊策略排序、單袋與forced fire衝突Approved Specs | **Split/Refactor** → P1-06 Direct、P1-07 Kick、P1-08 scoring/PotOnly |
| old `09-motion-profile-compatibility.md`；原ready | MotionPlanner姿態相容；blocked by P1-02 | MotionPlanner／Config／Math／tests；offline | planner owner、direction-derived C、finite failure | 舊五欄profile、保留未校正RX/RY、無A/B/cue-axis/safe-lift | **Move/Rename/Refactor** → P2-01；MotionPlanner／Config／Math |
| old `10-app-compile-compatibility.md`；原ready | App call-site整合；blocked by old 08/09 | BilliardApp／test_cueball／tasks；compile-only | pipeline fail-closed、App唯一協調owner、tests isolation | 舊camera processor依賴、只compile且缺完整cycle | **Split/Refactor** → P1-09 Phase1 integration、P2-02 fake full cycle |
| old `11-phase-1-acceptance-review.md`；原ready | 獨立總驗收；blocked by old 10 | tests/tasks；offline／compile-only | dependency audit、error cases、diff scope、safety review | 額外驗收identity與舊規格／禁硬體範圍 | **Superseded/Merge** → AC/tests/DoD分散P1-03～P2-03；archived |

每張舊票的Current AC、test requirements與hardware level已在表中摘要，完整原文由Git rename history或`docs/tickets/archive`保留；有效要求在各active ticket的Requirement Traceability明確指向。

## Final Capability Coverage Audit

| Capability | Existing Ticket Source | Final Ticket | Classification | Existing C++ Owners | Dependencies | Status | Coverage Complete? | Duplicate Responsibility? | Requirement Loss? |
|---|---|---|---|---|---|---|---|---|---|
| P1-01 | old P1-01 | phase-1/01 | Completed/Keep | tasks/tests | None | Completed | YES | NO | NO |
| P1-02 | old P1-02 | phase-1/02 | Completed/Keep | Point/Results/Math | P1-01 | Completed | YES | NO | NO |
| P1-03 | old 03+04+05 | phase-1/03 | Refactor/Rename/Merge | SocketClient/Parser/TableState/Config/App | P1-02 | Ready | YES | NO | NO |
| P1-04 | old 05+06 | phase-1/04 | Rename/Refactor | TableState/Config/App stability | P1-03 | Planned | YES | NO | NO |
| P1-05 | old 05+07 | phase-1/05 | Rename/Refactor/Merge | Physics/Point/Results/Config | P1-04 | Planned | YES | NO | NO |
| P1-06 | old 08 | phase-1/06 | Split/Refactor | TargetSelector/Algorithm/Physics/TableState | P1-05 | Planned | YES | NO | NO |
| P1-07 | old 07+08 | phase-1/07 | Split/Refactor | Algorithm/Physics/Point | P1-05 | Planned | YES | NO | NO |
| P1-08 | old 08+11 | phase-1/08 | Split/Refactor/Merge | Algorithm/Physics/TargetSelector/Config | P1-06,P1-07 | Planned | YES | NO | NO |
| P1-09 | old 10+11 | phase-1/09 | Split/Refactor/Merge | Algorithm/TableState/BilliardApp | P1-08 | Planned | YES | NO | NO |
| P2-01 | old 09 | phase-2/01 | Move/Rename/Refactor | MotionPlanner/Config/Math | P1-09 | Planned | YES | NO | NO |
| P2-02 | old 10+11 | phase-2/02 | Split/Refactor/Merge | BilliardApp/MotionPlanner/RobotController interface/Config/test fakes | P2-01 | Planned | YES | NO | NO |
| P2-03 | no reasonable old hardware ticket | phase-2/03 | **New — verified gap** | RobotController/BilliardApp/Config/necessary main | P2-02 | Planned | YES | NO | NO |

## New and Production-File Audit

- New ticket：只有P2-03。證據：所有11張舊票都限制在Phase 1 offline／compile-only或明確禁止HRSDK、RobotController與真實DO，無票可合理承接controlled hardware acceptance。
- New production cpp要求：None。
- P1-04只有在完成existing-owner分析後，才可能建立一個唯一stability helper；這是條件式justification，不是預先要求。
- 下一個可實作ticket：P1-03。
