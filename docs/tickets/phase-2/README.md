# Phase 2 Active Tickets

## Authority

以四份Approved active specs及Phase 1成功ShotPlan為權威；Phase 2不得重選球、重新評分、相機補償或平面座標轉換。

## Active Capability Coverage

| ID | Ticket | Status | Blocked by | Existing C++ Owners |
|---|---|---|---|---|
| P2-01 | [MotionPlanner到ExecutionPlan](01-motion-planner-execution-plan.md) | Planned | P1-09 | MotionPlanner／BilliardConfig／MathUtils |
| P2-02 | [fake完整執行狀態機](02-fake-execution-state-machine.md) | Planned | P2-01 | BilliardApp／MotionPlanner／RobotController interface／BilliardConfig／test fakes |
| P2-03 | [real adapters與受控驗收](03-real-adapters-controlled-acceptance.md) | Planned | P2-02 | RobotController／BilliardApp／BilliardConfig／必要main |

## Dependency and Hardware Gates

```text
P1-09 → P2-01 → P2-02 → P2-03
```

- P2-01、P2-02完全離線／fake。
- 只有P2-03最後階段允許受控真實硬體。
- 不得建立第二個MotionPlanner、RobotController、BilliardApp或production fake controller。
