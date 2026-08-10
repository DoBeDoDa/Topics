# P1-08 — Direct/Kick Scoring, Deterministic Selection and PotOnly

**Status:** Completed
**Implementation commit:** 4d01e2b61f6a835f52cfd9e04e370a4b91704fb5

AC: PASS
Scope: PASS
Tests: PASS
Compile: PASS
Critical: None
High: None
Medium: None

Previous tie-break coverage finding closed: YES

**Blocked by:** P1-06, P1-07

## 1. ID

P1-08

## 2. Title

Direct／Kick共同正規化評分、確定性選擇與PotOnly NoPlan。

## 3. Status

Completed

## 4. Purpose

把全部可行DirectPot與KickPot放進同一評分集合，以Direct soft preference及確定性tie-break選出最佳方案；沒有Pot時誠實回NoPlan。

## 5. Approved Spec References

- Master Spec §6、§9 P1-08。
- Phase 1 Spec §12～§15。

## 6. Existing Responsibility Owners

- `Algorithm.h/.cpp`：共同評分與選擇協調。
- `BilliardPhysics.h/.cpp`、`TargetSelector.h/.cpp`：candidate metrics與資格。
- `BilliardConfig.h/.cpp`：scoring ranges、raw weights、epsilon與PlanningMode。

## 7. Existing Files Expected to Change

- 上述owners、domain/result headers、algorithm regression tests及離線task。

## 8. Existing Files Explicitly Not to Duplicate

- 不得新增ShotBrain、第二個scoring engine、strategy tree或平行configuration owner。

## 9. Scope

- 六項Initial Experimental Weights、raw/effective weights、normalization、clamp、total cost與audit。
- Direct kick penalty soft preference、高品質Kick可勝出、固定deterministic tie-break。
- 預設`PlanningMode=PotOnly`，空集合回`NoPlan(NoPotCandidate)`。
- LegalContact只保留顯式manual／research mode且real hardware預設OFF。

## 10. Out of Scope

- production最佳權重、自動調參、母球後續／洗袋、力度與Robot execution。

## 11. Preconditions

- P1-06與P1-07提供互斥於Diagnostic的可行candidate sets。

## 12. Dependencies

- Blocked by P1-06、P1-07；完成後解鎖P1-09。

## 13. Detailed Requirements

1. 不可行候選先排除，再共同評分。
2. raw weights finite、非負、sum>0；effective=`raw/sum`。
3. 每個raw量、單位、range、clamp及不適用欄位遵守Spec；nonfinite／缺設定fail-closed。
4. Direct不硬優先；kick penalty只形成soft preference。
5. PotOnly無可行Pot回NoPotCandidate，不自動LegalContact。

## 14. Fail-Closed Requirements

- 無效weight、range、cost、tie epsilon或candidate不得建立ShotPlan或復活Diagnostic。

## 15. Acceptance Criteria

- [x] Direct與Kick共同集合，較佳Kick可勝過差Direct。
- [x] raw/effective weight與全部normalization可稽核。
- [x] tie-break與容器／生成順序無關。
- [x] PotOnly空集合回NoPlan(NoPotCandidate)，無fallback。
- [x] LegalContact不進正常production cycle。

## 16. Test Requirements

- 每個公式、missing／invalid ranges、weight sum、overflow、tie epsilon、Direct優勢與Kick勝出fixture。
- PotOnly與manual research mode隔離測試。

## 17. Hardware Level

完全離線。

## 18. Regression Requirements

- 保留最低號規則、Direct／Kick geometry；不得恢復舊硬策略排序或forced fire。

## 19. Definition of Done

- scoring／selection／NoPlan fixtures全綠，完整audit fields，單一commit。

## 20. Requirement Traceability

- Split／Refactor自舊P1-08的策略選擇責任；取代「維持舊策略排序／強制開火」失效要求。
- 舊P1-11的評分、禁用fallback與回歸驗收分散至本票AC／tests。

## Approved ranked-output extension

- 既有scoring只執行一次；用既有`tieEpsilon`／`tieBreakBetter()`反覆選出完整ranked Pot，rank #1必須等於既有`selectBestPot()`。
- `PotOnly`策略結果仍保持無Pot時`NoPlan(NoPotCandidate)`；production執行fallback不屬於本票決策。

## 21. New File Justification

None expected；新Markdown是舊Algorithm ticket split，production仍由既有Algorithm承接。
