# P1-07 — One-Rail Cue-Ball KickPot

**Status:** Planned

**Blocked by:** P1-05

## 1. ID

P1-07

## 2. Title

一次母球碰庫KickPot候選。

## 3. Status

Planned

## 4. Purpose

對每段有效effective rail，以理想鏡射建立母球先碰一次庫邊再命中最低號球的KickPot，並以與Direct相同的進袋幾何驗證目標球。

## 5. Approved Spec References

- Master Spec §6與§9 P1-07。
- Phase 1 Spec §11 KickPot、§15 Fixed Force Semantics。

## 6. Existing Responsibility Owners

- `Algorithm.h/.cpp`協調候選；`BilliardPhysics.h/.cpp`擁有鏡射、交點、路徑與碰撞；`Point.h`／`GeometryResults.h`承接結果型別。

## 7. Existing Files Expected to Change

- 上述owners、`tests/phase1_algorithm_regression_tests.cpp`與必要離線task。

## 8. Existing Files Explicitly Not to Duplicate

- 不得新增ShotBrain、第二個Algorithm／BilliardPhysics或獨立bank engine。

## 9. Scope

- 六段EffectiveCueBallRailSegment、唯一CueBallReboundPoint、cue-ball路徑`C → R → Gpot`與target-ball路徑`T → VirtualPocketTarget`。
- `P`若用於公式，唯一表示`selected ResolvedPocketModel.VirtualPocketTarget`，不得表示raw wire pocket center。
- 每段碰撞／region、rail排除區、entry angle及`maxKickRailAngleDeg`硬門檻。
- 理想入射角等於反射角與rail-angle risk diagnostic。

## 10. Out of Scope

- target-bank、兩庫以上、kick動力學、旋轉、摩擦、恢復係數、動態力度與Robot。

## 11. Preconditions

- P1-05 effective rail、Ghost與碰撞幾何完成。

## 12. Dependencies

- Blocked by P1-05；與P1-06平行；兩者完成後解鎖P1-08。

## 13. Detailed Requirements

1. 對每個有效effective rail最多一個幾何KickPot候選。
2. R必須位於有效segment、RailReflectionRegion且避開袋口／端點排除區。
3. Kick第二段終點是Gpot，不是T或surface contact K。
4. 三段路徑各自檢查碰撞與適用region。
5. 超過Phase 1 kick角門檻在評分前排除；通過者仍可能被Phase 2力度envelope拒絕。
6. Kick只改變母球碰撞前路徑：Direct為`C → Gpot`，Kick為`C → R → Gpot`；兩者的target-ball路徑均為`T → selected ResolvedPocketModel.VirtualPocketTarget`。
7. Direct與Kick必須共用同一袋的ResolvedPocketModel、VirtualPocketTarget、PocketExitSegment、PocketCaptureCorridor與pocket-entry geometry；Kick不得改用raw wire pocket center或替換target-ball pocket model。

## 14. Fail-Closed Requirements

- 平行、無交點、重合、退化、nonfinite、非法normal或鏡射不變量失敗只產生Diagnostic。

## 15. Acceptance Criteria

- [ ] 六rail生成、R與兩段cue path唯一正確。
- [ ] `||T-Gpot||=2r`且Kick第二段終點為Gpot。
- [ ] 同一袋的Direct與Kick使用相同ResolvedPocketModel及VirtualPocketTarget；只有cue-ball path不同。
- [ ] 入射／反射、排除區、碰撞與角度硬門檻測試通過。
- [ ] Phase 1不讀取pneumaticPulseMs或FixedForceEnvelope。

## 16. Test Requirements

- 六rail、clear／blocked segments、intersection edge cases、normal reversal、angle threshold、第二庫禁止及動力學參數不影響候選。
- Case D：同一袋的Direct與Kick，其target-ball path都終止於同一VirtualPocketTarget，且使用同一PocketExitSegment／PocketCaptureCorridor／entry geometry。
- Case E：Kick只把cue-ball path由`C → Gpot`改為`C → R → Gpot`；不得改變target-ball pocket model或以raw wire pocket center作endpoint。

## 17. Hardware Level

完全離線幾何近似。

## 18. Regression Requirements

- 不恢復舊反彈假Point或強制開火；P1-05 geometry tests保持綠色。

## 19. Definition of Done

- 所有Kick fixtures與error injection通過，無力度／硬體依賴，單一commit。

## 20. Requirement Traceability

- Split自舊P1-08反彈策略要求，並依Approved Specs由P1-05安全幾何補足。
- 評分與Direct比較移至P1-08。

## 21. New File Justification

None expected；新Markdown來自舊ticket split，不代表新production module。
