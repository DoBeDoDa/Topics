# P1-06 — Lowest-Number Target and DirectPot

**Status:** Ready for Implementation

**Blocked by:** P1-05

## 1. ID

P1-06

## 2. Title

最低號目標資格與六袋DirectPot候選。

## 3. Status

Planned

## 4. Purpose

由StableTableState固定選最低號存在球，對六個stable wire pocket centers建立完整且可稽核的DirectPot可行候選。

## 5. Approved Spec References

- Master Spec §6、§9 P1-06。
- Phase 1 Spec §9 Target Qualification、§10 DirectPot。

## 6. Existing Responsibility Owners

- `TargetSelector.h/.cpp`：最低號資格。
- `Algorithm.h/.cpp`：Direct候選協調。
- `BilliardPhysics.h/.cpp`：Ghost、路徑與碰撞。
- `TableState.h`：Stable input及candidate/result型別原地演進。

## 7. Existing Files Expected to Change

- 上述owners、`tests/phase1_algorithm_regression_tests.cpp`及必要離線task。

## 8. Existing Files Explicitly Not to Duplicate

- 不得新增ShotBrain.cpp、第二個TargetSelector／Algorithm／BilliardPhysics或平行candidate engine。

## 9. Scope

- TargetSelector作為唯一target qualification owner，從StableTableState嚴格選擇最低號存在球；對六袋完整產生DirectPot。
- 每個候選驗證Gpot、cue path、target path、pocket exit／corridor、entry angle、切球角、障礙與finite。
- 可行Candidate與CandidateDiagnostic分離。

## 10. Out of Scope

- Kick、共同評分、LegalContact production fallback、Robot與力度。

## 11. Preconditions

- P1-05共用幾何可用；StableTableState含球與六袋stable centers。

## 12. Dependencies

- Blocked by P1-05；與P1-07可平行；兩者完成後解鎖P1-08。

## 13. Detailed Requirements

1. 只選最低號存在球，不遍歷其他球作首次接觸目標。
2. 對Pocket ID 1～6各建立至多一個DirectPot；不得預選單袋。
3. 母球路徑`C → Gpot`、目標球路徑`T → selected ResolvedPocketModel.VirtualPocketTarget`分開碰撞與region檢查。
4. 不可行候選只留下具名Diagnostic，不進正式集合。
5. StableTableState中1～9號球全部absent時回`NoEligibleTarget`；不得forced target、default ball，亦不得改寫成Parser或stability failure。
6. 無Direct只代表Direct集合空，P1-08仍需與Kick集合整合。

## 14. Fail-Closed Requirements

- 必要資料、方向、路徑、entry angle或configuration無效即拒絕候選；不得forced Direct或預設袋口。

## 15. Acceptance Criteria

- [ ] lowest-number規則與六袋組合完整。
- [ ] StableTableState中0顆編號球時，唯一結果為`NoEligibleTarget`；P1-03與P1-04不得提前攔截此合法state。
- [ ] Direct兩條路徑與障礙端點排除獨立正確。
- [ ] Gpot為2r且是cue path終點。
- [ ] 所有不可行候選不進評分集合。
- [ ] TargetSelector只接受StableTableState且沒有相機補償。

## 16. Test Requirements

- 最低號、missing balls、六袋、clear／blocked cue path、clear／blocked target path、錯誤出口、entry threshold、degenerate／nonfinite。
- Case A（Target段）：P1-03接受九顆paired sentinel、P1-04產生zero-object-ball StableTableState後，P1-06回`NoEligibleTarget`且沒有fallback target。

## 17. Hardware Level

完全離線。

## 18. Regression Requirements

- 保留TargetSelector最低號有效行為；移除舊單袋／強制開火與CameraCompensator依賴。

## 19. Definition of Done

- Direct候選與Diagnostic fixture全綠、無硬體／Socket依賴、單一commit。

## 20. Requirement Traceability

- Split／Refactor自舊P1-08 TargetSelector與Algorithm遷移中的最低號及Direct有效要求。
- 共同評分移至P1-08；整合Result移至P1-09。

## 21. New File Justification

None expected；使用既有TargetSelector、Algorithm與BilliardPhysics。
