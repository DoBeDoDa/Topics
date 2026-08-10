# Phase 1 Active Tickets

## Authority

需求與驗收只來自四份Approved active specs；`docs/archive`、`docs/tickets/archive`與舊ticket文字只能作歷史traceability。

Ticket status及implementation evidence以各ticket正文為唯一權威。

## Active Capability Coverage

| ID | Ticket | Fixed dependency | Existing C++ Owners |
|---|---|---|---|
| P1-01 | [C++17與離線測試框架](01-cpp17-offline-test-framework.md) | None | existing tasks／tests |
| P1-02 | [安全型別、GeometryResults與純MathUtils](02-safe-math-and-geometry-types.md) | P1-01 | Point／GeometryResults／MathUtils |
| P1-03 | [32值Contract、Parser與Cycle Boundary](03-external-contract-parser-cycle-boundary.md) | P1-02 | SocketClient／VisionDataParser／TableState／BilliardConfig／BilliardApp |
| P1-04 | [三event穩定生命週期](04-three-event-stability.md) | P1-03 | TableState／BilliardConfig／BilliardApp stability owner |
| P1-05 | [桌面、袋口、庫邊與碰撞幾何](05-table-pocket-rail-collision-geometry.md) | P1-04 | BilliardPhysics／Point／GeometryResults／BilliardConfig |
| P1-06 | [最低號目標與DirectPot](06-lowest-target-direct-pot.md) | P1-05 | TargetSelector／Algorithm／BilliardPhysics／TableState |
| P1-07 | [一次母球碰庫KickPot](07-one-rail-cue-ball-kick-pot.md) | P1-05 | Algorithm／BilliardPhysics／Point |
| P1-08 | [Direct/Kick評分與PotOnly](08-direct-kick-scoring-pot-only.md) | P1-06、P1-07 | Algorithm／BilliardPhysics／TargetSelector／BilliardConfig |
| P1-09 | [Phase 1整合與ShotPlan/NoPlan](09-phase-1-integration-shot-plan.md) | P1-08 | Algorithm／TableState／BilliardApp |

## Dependency

```text
P1-01
→ P1-02
→ P1-03
→ P1-04
→ P1-05
→ P1-06 + P1-07
→ P1-08
→ P1-09
→ P2-01
```

P1-06與P1-07可在P1-05後平行。

## Shared Boundaries

- Phase 1完全離線、Base0 planar XY-only；不開production Socket、不執行RobotController／HRSDK／DO。
- 概念名稱不要求同名cpp；禁止ShotBrain.cpp、Phase1Pipeline.cpp、CameraCompensator、第二套Parser／Algorithm／Physics／Math。
- 每票優先原地修改Existing Responsibility Owners，保持單一commit、可review及可回滾。
