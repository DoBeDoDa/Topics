# Billiards Active Ticket Index

Ticket status及implementation evidence以各ticket正文為唯一權威。

| ID | Ticket | Capability | Fixed dependency | Existing responsibility owners |
|---|---|---|---|---|
| P1-01 | [C++17與離線測試框架](phase-1/01-cpp17-offline-test-framework.md) | C++17 offline test harness | None | existing tasks／tests |
| P1-02 | [安全型別、GeometryResults與純MathUtils](phase-1/02-safe-math-and-geometry-types.md) | safe types／geometry／math primitives | P1-01 | Point／GeometryResults／MathUtils |
| P1-03 | [32值Contract、Parser與Cycle Boundary](phase-1/03-external-contract-parser-cycle-boundary.md) | existing wire、strict parser、local cycle boundary | P1-02 | SocketClient／VisionDataParser／TableState／BilliardConfig／BilliardApp |
| P1-04 | [三event穩定生命週期](phase-1/04-three-event-stability.md) | three-event stability lifecycle | P1-03 | TableState／BilliardConfig／BilliardApp stability owner |
| P1-05 | [桌面、袋口、庫邊與碰撞幾何](phase-1/05-table-pocket-rail-collision-geometry.md) | table／pocket／rail／collision geometry | P1-04 | BilliardPhysics／Point／GeometryResults／BilliardConfig |
| P1-06 | [最低號目標與DirectPot](phase-1/06-lowest-target-direct-pot.md) | lowest-number target／DirectPot | P1-05 | TargetSelector／Algorithm／BilliardPhysics／TableState |
| P1-07 | [一次母球碰庫KickPot](phase-1/07-one-rail-cue-ball-kick-pot.md) | one-rail cue-ball KickPot | P1-05 | Algorithm／BilliardPhysics／Point |
| P1-08 | [Direct/Kick評分與PotOnly](phase-1/08-direct-kick-scoring-pot-only.md) | common scoring／deterministic selection／PotOnly | P1-06、P1-07 | Algorithm／BilliardPhysics／TargetSelector／BilliardConfig |
| P1-09 | [Phase 1整合與ShotPlan/NoPlan](phase-1/09-phase-1-integration-shot-plan.md) | Phase 1 integration／ShotPlan／NoPlan | P1-08 | Algorithm／TableState／BilliardApp |
| P2-01 | [MotionPlanner到ExecutionPlan](phase-2/01-motion-planner-execution-plan.md) | MotionPlanner／ExecutionPlan | P1-09 | MotionPlanner／BilliardConfig／MathUtils |
| P2-02 | [fake完整執行狀態機](phase-2/02-fake-execution-state-machine.md) | fake complete execution state machine | P2-01 | BilliardApp／MotionPlanner／RobotController interface／BilliardConfig／test fakes |
| P2-03 | [real adapters與受控驗收](phase-2/03-real-adapters-controlled-acceptance.md) | real adapters／controlled hardware acceptance | P2-02 | RobotController／BilliardApp／BilliardConfig／necessary main |

## Fixed Dependency

```text
P1-01 → P1-02 → P1-03 → P1-04 → P1-05
                                      ├→ P1-06 ─┐
                                      └→ P1-07 ─┴→ P1-08 → P1-09
                                                               → P2-01 → P2-02 → P2-03
```
