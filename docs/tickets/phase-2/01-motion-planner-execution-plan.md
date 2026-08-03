# P2-01 — Existing MotionPlanner to ExecutionPlan

**Status:** Ready for Implementation

**Blocked by:** P1-09

## 1. ID

P2-01

## 2. Title

既有MotionPlanner原地演進：StrikeReadyPose、校正姿態、cue-axis驗證與ExecutionPlan。

## 3. Status

Ready for Implementation

## 4. Purpose

把Base0 planar XY ShotPlan轉成可離線驗證的SafeApproachPose、StrikeReadyPose及safe-lift derivation contract，不重新做撞球策略或座標轉換。

## 5. Approved Spec References

- Master Spec §5.3、§6、§9 P2-01。
- Phase 2 Spec §4 P2-01、§5～§9、§14 P2-01。

## 6. Existing Responsibility Owners

- `MotionPlanner.h/.cpp`與既有`MotionPlan`原地演進為ExecutionPlan語意。
- `BilliardConfig.h/.cpp`：manual Strike Z、A/B bounds、CToolOffset、cueForwardAxisTool、ready gap與safe lift設定。
- `MathUtils.h/.cpp`：純方向／旋轉數學。

## 7. Existing Files Expected to Change

- 上述owners、`tests/phase1_core_tests.cpp`或新增的tests內Phase 2離線fixture、必要build task。
- `test_cueball.cpp`既有A/B prototype只可在本票實作時搬移至MotionPlanner，不得複製；本輪ticket refactor不修改該程式。

## 8. Existing Files Explicitly Not to Duplicate

- 不得新增ExecutionPlanner、第二個MotionPlanner、TableFrameToBase0Converter、第二套MathUtils或parallel pose tree。

## 9. Scope

- `TCPreadyXY = Cball - (r + readyGapMm)d`、人工`ZstrikeManual`。
- finite bounded deterministic A/B search；C=`normalizeAngle(atan2(dy,dx)+CToolOffset)`且A/B不得改C。
- 每個候選驗證投影cueForwardAxis與d的角差。
- SafeApproach、StrikeReady、transit／Camera references、safeLiftHeight與actual-pose derivation rule的ExecutionPlan。

## 10. Out of Scope

- coordinate conversion、選球、Ghost／Kick／scoring、camera compensation、real HRSDK、real DO與執行狀態機。

## 11. Preconditions

- P1-09提供成功ShotPlan；必要校正設定具有版本與人工核准狀態。

## 12. Dependencies

- Blocked by P1-09；完成後解鎖P2-02。

## 13. Detailed Requirements

1. ShotPlan XY原值直用，不做TableFrame→Base0或第二次映射。
2. GhostBallPoint不得當TCP；d必須finite且為單位向量。
3. A/B只在核准有限範圍、固定step／order／tie-break搜尋。
4. cueForwardAxisTool是校正屬性，不預設+X；投影退化或超`maxCueDirectionErrorDeg`拒絕。
5. PostStrikeSafeLiftPose不預存planned start；只記錄由氣動後current actual pose保持X/Y/A/B/C、Z+height的規則。

## 14. Fail-Closed Requirements

- 缺校正、nonfinite、unreachable或所有A/B候選失敗回NoExecutablePlan，無任意Pose／0角／範圍擴張fallback。

## 15. Acceptance Criteria

- [ ] StrikeReady XY、manual Z、A/B、C與cue-axis公式完整。
- [ ] MotionPlanner沒有平面座標轉換或策略責任。
- [ ] ExecutionPlan只含已驗證Pose／rules及具名失敗。
- [ ] safe lift明確從runtime actual pose推導。
- [ ] 沒有第二個planner。

## 16. Test Requirements

- Base0 XY原值、四象限C、A/B bounds/order/tie、cue-axis alignment、ready gap、Ghost/TCP分離、missing calibration與all-candidate failure。

## 17. Hardware Level

完全離線；fake reachability／pose validation，不link HRSDK。

## 18. Regression Requirements

- 保留既有MotionPlanner唯一owner；移除舊tilt／moveBack混合責任時不得以未驗證姿態維持相容。

## 19. Definition of Done

- P2-01離線tests及source audit通過；ExecutionPlan可供fake executor消費；單一commit。

## 20. Requirement Traceability

- Move／Rename／Refactor自舊P1-09 MotionProfile與MotionPlanner相容遷移。
- 舊A/B prototype責任從`test_cueball`移入既有MotionPlanner；不複製。

## 21. New File Justification

None expected；只原地演進既有MotionPlanner與config／math owners。
