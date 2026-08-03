# P1-05 — Table, Pocket, Rail and Collision Geometry

**Status:** Completed
**Implementation commit:** 8c896923ce44d2e7d67f716fab9fc56686041bf5

AC: PASS
Scope: PASS
Tests: PASS
Compile: PASS
Critical: None
High: None
Medium: None

Deferred:
Strategy caller migration由P1-06／P1-07承接。
Production geometry參數未核准時維持ConfigurationMissing，不阻擋P1-05完成。

**Blocked by:** P1-04

## 1. ID

P1-05

## 2. Title

Base0平面桌面、袋口、effective rail、GhostBall與碰撞幾何。

## 3. Status

Completed

## 4. Purpose

在既有BilliardPhysics與安全Math primitives中建立Direct／Kick共用、可離線驗證且fail-closed的唯一幾何基礎。

## 5. Approved Spec References

- Master Spec §6～§9 P1-05。
- Phase 1 Spec §7～§8、§16幾何測試。

## 6. Existing Responsibility Owners

- `BilliardPhysics.h/.cpp`、`Point.h`、`GeometryResults.h`、`BilliardConfig.h/.cpp`。
- `MathUtils`只提供純數學，不擁有撞球規則或production參數。

## 7. Existing Files Expected to Change

- 上述owners、`tests/phase1_core_tests.cpp`與必要離線task source清單。

## 8. Existing Files Explicitly Not to Duplicate

- 不得新增第二套BilliardPhysics／MathUtils、TableFrameToBase0Converter、CameraCompensator或平行geometry tree。

## 9. Scope

- PlayableBallCenterRegion、PocketExitSegment、PocketCaptureCorridor、RailReflectionRegion。
- PhysicalRailSegment與向內平移一次球半徑的EffectiveCueBallRailSegment。
- stable wire pocket center與PocketModelConfig解析成ResolvedPocketModel。
- `G = T - 2r*normalize(P-T)`、BallSurfaceContactPoint diagnostic、線段碰撞、鏡射、ray／segment交點。

## 10. Out of Scope

- 選球、評分、Robot Pose、力度、旋轉、摩擦、恢復係數、jaw完整物理與兩庫以上。

## 11. Preconditions

- P1-02安全primitives Completed；P1-04提供StableTableState與六袋stable centers。

## 12. Dependencies

- Blocked by P1-04；完成後P1-06與P1-07可平行。

## 13. Detailed Requirements

1. 母球／Ghost point留在PlayableBallCenterRegion；目標球只可經選定PocketExitSegment進對應corridor。
2. Pocket corridor、出口正向穿越、outward normal、entry angle與virtual target符合Approved Spec唯一公式。
3. Effective rail由physical rail沿finite單位inward normal平移`r`一次；collision margin不得重複加入offset。
4. 所有線段最近距離處理端點；不同路徑有明確障礙端點排除。
5. 零長度、重疊、平行／無交點／重合、非finite與非法configuration具名區分。

## 14. Fail-Closed Requirements

- 不得回原Point、`Point{0,0}`、0度或假交點；Invalid優先於Blocked／Clear。
- CandidateDiagnostic不得成為success geometry value。

## 15. Acceptance Criteria

- [x] `||T-G|| = 2r`且G/T/P共線、方向正確。
- [x] physical/effective rails與排除區映射正確，ball radius只加入一次。
- [x] pocket corridor／exit／entry-angle與錯誤出口完整測試。
- [x] 路徑碰撞、退化及Result payload不變量通過。
- [x] Phase 1只使用Base0 XY。

## 16. Test Requirements

- Ghost 2r、K diagnostic、regions、0／threshold／180° entry angle。
- effective rail、normal reversal、empty segment、collision margin、segment endpoints。
- ConfigurationMissing／InvalidConfiguration／NoIntersection／Blocked／Clear fixtures。

## 17. Hardware Level

完全離線純幾何。

## 18. Regression Requirements

- P1-02 API與舊有效collision fixtures保持綠色；不得恢復bool混合語意或舊假Point fallback。

## 19. Definition of Done

- 所有幾何與error-injection測試通過；BilliardPhysics是唯一撞球幾何owner；單一commit。

## 20. Requirement Traceability

- Rename／Refactor舊P1-07 BilliardPhysics安全幾何API。
- 接收舊P1-05仍有效的bounds／configuration validation；相機補償不遷移。

## 21. New File Justification

None expected；原地演進既有BilliardPhysics與domain headers。
