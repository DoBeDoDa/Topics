# Phase 1 Tracer-Bullet Tickets

## 基準

- Repository：`C:\Users\User\Desktop\Topics-main-git`
- 分支：`main`
- Master Spec：v1.1，Approved for phased implementation
- Phase 1 Spec：v1.1，Ready for Implement
- 本目錄只拆解 Phase 1；不得延伸至 Phase 2 至 Phase 6。

## Ticket順序與狀態

| ID | Ticket | Status | Blocked by | 驗收摘要 |
|---|---|---|---|---|
| P1-01 | [C++17建置與離線測試框架](01-cpp17-offline-test-framework.md) | ready-for-agent | None | 兩個空白測試骨架可獨立編譯執行，依賴圖不含HRSDK |
| P1-02 | [基礎型別、GeometryResults與MathUtils](02-safe-math-and-geometry-types.md) | ready-for-agent | P1-01 | 安全Math API與optional失敗語意通過核心測試 |
| P1-03 | [CameraCompensator](03-camera-compensator.md) | ready-for-agent | P1-02 | 固定補償公式與非法輸入測試通過 |
| P1-04 | [ParsedVisionFrame與嚴格Parser](04-strict-vision-parser.md) | ready-for-agent | P1-02 | 精確32欄與精確成對sentinel契約通過 |
| P1-05 | [VisionFrameProcessor與ProcessedVisionFrame](05-single-frame-processor.md) | ready-for-agent | P1-03、P1-04 | 單幀補償、設定及範圍驗證fail closed |
| P1-06 | [StableFrameValidator與TableState](06-stable-frame-validator.md) | ready-for-agent | P1-05 | 三幀presence、中位數及距離驗證通過 |
| P1-07 | [BilliardPhysics安全幾何API](07-safe-billiard-physics.md) | ready-for-agent | P1-02 | 路徑、鬼球、鏡射及交點不再產生假成功 |
| P1-08 | [TargetSelector與Algorithm遷移](08-target-selector-algorithm-migration.md) | ready-for-agent | P1-06、P1-07 | TargetSelector只吃TableState，策略回歸相容 |
| P1-09 | [MotionProfile與MotionPlanner相容遷移](09-motion-profile-compatibility.md) | ready-for-agent | P1-02 | 五欄MotionProfile逐欄遷移且姿態值不變 |
| P1-10 | [BilliardApp、test_cueball與完整編譯相容](10-app-compile-compatibility.md) | ready-for-agent | P1-08、P1-09 | 所有Phase 1 API收斂，控制程式只編譯不執行 |
| P1-11 | [Phase 1全面驗收與Code Review](11-phase-1-acceptance-review.md) | ready-for-agent | P1-10 | 兩個離線測試通過，禁用符號與硬體副作用清零 |

`ready-for-agent`表示ticket內容已足夠交付；有blocker的ticket仍必須等所有
blocking edges完成後才能開始。

## 相依關係

```text
P1-01
└─ P1-02
   ├─ P1-03 ─┐
   ├─ P1-04 ─┴─ P1-05 ─ P1-06 ─┐
   ├─ P1-07 ─────────────────────┴─ P1-08 ─┐
   └─ P1-09 ────────────────────────────────┴─ P1-10 ─ P1-11
```

P1-03、P1-04、P1-07及P1-09在P1-02完成後可平行進行。P1-05必須等相機與
Parser兩條支線完成；P1-08必須等穩定TableState及安全Physics完成。

## 建議實作順序

1. P1-01
2. P1-02
3. 平行：P1-03、P1-04、P1-07、P1-09
4. P1-05
5. P1-06
6. P1-08
7. P1-10
8. P1-11

若不平行，依ticket編號順序執行即可。每張ticket應使用獨立commit；進入下一張
前，必須完成該票指定的離線測試及diff審查。

## Integration branch規則

P1-02會移除舊Math API，而其所有production呼叫端要到P1-08、P1-09及P1-10
才完成收斂。為避免把「完整主程式暫時無法編譯」的中間commit直接合併到main：

- P1-01可獨立落地。
- P1-02至P1-10應在同一條Phase 1 integration branch上各自保留獨立commit。
- 每個中間commit必須讓該票指定的離線target保持綠色。
- 完整main／calibrate／test_cueball編譯綠色的承諾點是P1-10。
- P1-11完成全面驗收後，才可將整條integration branch合併到main。
- 禁止用危險legacy wrapper、假Point或假角度換取中間主程式綠色。

## 所有Ticket共同安全邊界

- 不執行`main`、`calibrate`或`test_cueball`。
- 測試不得include、編譯或link HRSDK、RobotController、SocketClient、
  BilliardApp或任何數位輸出程式。
- 不連線HRSDK、不開Socket、不送運動、不操作DO。
- 不修改Python 32欄wire protocol。
- 不修改Algorithm直球／反彈策略排序。
- 不修正強制開火策略；但Invalid幾何不得到達該分支。
- 不填入Base0桌面bounds、三幀tolerance、球路安全餘量或其他未校正
  production數值。
- 不實作姿態搜尋、CartesianPose、正式擊球模型、自動比賽、氣壓或其他
  Phase 2至Phase 6內容。
- 編譯成功不代表目前PRODUCTION_MOTION可安全上機。
- 不覆寫、還原或提交不屬於當張ticket的使用者修改。

## 共用回滾規則

- 每張ticket使用獨立commit。
- 優先使用`git revert <ticket-commit>`回滾，不使用`git reset --hard`。
- 若下游ticket已依賴上游API，必須先由下游往上游反向回滾。
- 回滾不得觸及`History/`、`debug_frame.png`、`黑白棋盤.docx`或使用者既有的
  `src/test_cueball.cpp`輸出換行修改。
