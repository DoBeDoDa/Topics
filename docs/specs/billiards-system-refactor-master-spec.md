# 撞球機械手臂重構 Master Architecture Spec

## 文件資訊

- 狀態：Approved
- Final Workflow Verification：PASS（2026-08-01）
- 規格版本：2.0
- 基準 commit：`216bcb7`
- 適用系統：Python 視覺、C++ 擊球大腦、C++ 擊球手
- 下一步：To Tickets／Existing Ticket Refactor；本文件不構成實作或真實硬體操作授權

## 1. 權威與文件關係

發生衝突時依序採用：

1. 使用者明確 Confirmed Decisions。
2. 本次架構健檢修正。
3. 本 Master Spec。
4. 各子系統 active spec。
5. P1-01、P1-02 已完成實作與其測試。
6. 實際程式碼與 Git 歷史。
7. superseded spec。
8. 舊 tickets。

Active specs：

- `python-cpp-external-contract.md`：wire、Robot Base0 planar XY 與 Parser 邊界。
- `phase-1-shot-brain-spec.md`：C++ 擊球大腦。
- `phase-2-shot-executor-spec.md`：C++ 擊球手。

`../archive/specs/phase-1-math-geometry-refactor-spec.md` 已被取代，只保留歷史與 P1-01／P1-02 追溯用途。

相同規則只由一份 active spec 定義；其他文件只能引用，不得複製後改寫。
後續To Tickets必須只使用上述三份子系統active specs與本Master的明確allowlist；
`docs/archive`及任何superseded文件一律排除，即使工具遞迴掃描`docs`亦不得納入。

## 2. Problem Statement

目前系統將相機補償、wire sentinel、撞球幾何、候選選擇、機械姿態與 HRSDK／DO 控制混在同一流程。無效資料可能被轉成預設 Point 或角度，不可行球路可能被強制執行，LIN 檢查失敗後仍可能繼續移動，且軟體無法在通訊失敗時保證氣動輸出已關閉。

重構必須建立三個不可跨越的責任區：Python 視覺、C++ 擊球大腦、C++ 擊球手，並使演算法完全離線可驗證、硬體執行預設拒絕未校正或未知狀態。

## 3. Solution

系統採兩個 Phase、十二個主要能力切片：

- Phase 1只消費Python已完成校正與轉換的Robot Base0平面XY毫米資料，直接在同一平面產生可稽核`ShotPlan | NoPlan`。
- Phase 2直接使用ShotPlan的Base0平面XY，以人工校正Z、核准範圍內的A／B搜尋及由擊球方向計算的C建立Tool1 TCP執行計畫；不得再做TableFrame→Base0或第二次平面映射。計畫先以fake驗證，再由最後一張ticket接入HRSDK與真實雙DO。
- 所有可能失敗的邊界使用具名結果；禁止預設成功、強制 fallback 或失敗後繼續正常流程。

## 4. User Stories

1. 作為視覺開發者，我要讓 Python 成為全部相機校正與補償的唯一責任端，避免 double compensation。
2. 作為演算法開發者，我要只接收Robot Base0平面XY毫米座標，讓Phase 1不依賴相機處理或機械手臂API。
3. 作為測試者，我要從receive-event stream離線重現Phase1PipelineResult，並在Stable seam以StableTableState獨立重現PlanningResult。
4. 作為操作者，我要在沒有合法方案時看到具名原因，而不是讓系統強制擊球。
5. 作為安全審查者，我要能由 ShotPlan 查明路徑、角度、淨空、分數與設定版本。
6. 作為機械手臂開發者，我要由ExecutionPlan取得以Base0平面XY、人工Strike Z、核准A／B及方向C建立的Pose，而不重新計算撞球策略或轉換平面座標。
7. 作為硬體操作者，我要讓未校正力度、未知 DO 狀態或失敗路徑阻止一切後續動作。
8. 作為 ticket 作者，我要能依十二個能力邊界建立可獨立驗證、commit 與回滾的工作。

## 5. 系統責任邊界

### 5.1 Python 視覺

唯一負責：影像擷取、球體辨識、鏡頭校正、perspective transform、homography、pixel轉Robot Base0平面XY毫米、固定offset與相機殘差補償，並以既有newline-delimited 32值CSV送出當次球與六袋Base0平面XY。

Python 不負責撞球候選、碰撞安全、Base0 XY之後的Robot Pose／Tool姿態、HRSDK 或 DO。

### 5.2 C++ Phase 1

Phase 1由兩個不可混用的責任層組成：

- `Phase1Pipeline`：接收ReceiveEvent stream，執行SingleFrameResult、
  StabilityResult及Phase1PipelineResult生命週期。P1-03負責External Contract、
  既有32值External Contract、本地shot-cycle/session邊界、Parser與單幀驗證；P1-04只負責三幀穩定生命週期。
- `ShotBrain`：只接受StableTableState、TableGeometryConfig與BrainConfig，負責
  撞球幾何、DirectPot、一次母球KickPot、共同評分及PlanningResult。V1預設`PlanningMode = PotOnly`；LegalContact只可作顯式manual／research能力。
  P1-09整合此層，但不接收CSV或ReceiveEvent。

ShotBrain不得負責Parser、receive freshness、timeout或三幀累積；只有
StabilityResult中的Stable success value可以呼叫ShotBrain。

Phase 1 P1-01至P1-09的核心演算法與驗收完全離線，不得執行相機補償、任何平面座標轉換、真實production Socket、HRSDK、RobotController、DO或真實機械手臂。P1-03以fake byte／connection／cycle source驗證既有32值contract、本地session／shot-cycle gate及parser；同一ticket可原地補強既有SocketClient與BilliardApp的production framing、stale-buffer protection及connection lifecycle，但不得在離線驗收中開啟Socket。Phase1Pipeline與ShotBrain核心不得依賴SocketClient。

### 5.3 C++ Phase 2：Shot Executor

唯一負責：由Base0平面XY ShotPlan建立Base0中的Tool1 TCP Pose、ExecutionPolicy、ExecutionPlan、PTP／LIN、fake與real adapters、雙DO與安全終止。StrikeReadyPose的X／Y由母球中心及擊球方向直接計算，Z來自人工校正Strike Z，A／B只可在版本化且人工核准的小區間內確定性搜尋，C由平面擊球方向計算；每個A／B候選還必須驗證校正球桿forward axis投影與擊球方向一致。

Phase 2不得執行TableFrame→Base0、相機補償、camera offset correction或第二次平面座標映射。

### 5.4 概念責任與既有檔案映射

Spec中的概念名稱不強制對應同名cpp；本專案採repository-constrained原地重構：

- `ShotBrain`概念由既有`Algorithm + BilliardPhysics + TargetSelector`承接。
- `Phase1Pipeline`概念由既有`BilliardApp`協調、`VisionDataParser`及唯一三幀穩定狀態物件共同承接。
- `ExecutionPlan`概念由既有`MotionPlan／MotionPlanner`原地演進。
- Execution State Machine由既有`BilliardApp + RobotController`承接。
- Production Transport由既有`SocketClient + BilliardApp`整合承接。

不得因概念名稱新增`ShotBrain.cpp`、`Phase1Pipeline.cpp`、`ExecutionPlanner.cpp`、第二個MotionPlanner、第二個RobotController、第二個SocketClient、第二個VisionDataParser、CameraCompensator、TableFrameToBase0Converter、第二個BilliardApp或第二套MathUtils／BilliardPhysics。

## 6. 全域資料流

```text
Python calibrated Robot Base0 planar-XY-mm CSV
→ ReceiveEvent
→ Phase1Pipeline
   → SingleFrameResult
   → StabilityResult
      └─ Stable success value only
         → ShotBrain(
              StableTableState,
              TableGeometryConfig,
              BrainConfig)
         → PlanningResult = ShotPlan | NoPlan
   → Phase1PipelineResult
      └─ PlanningCompleted(PlanningResult)
         ├─ NoPlan: stop before Phase 2
         └─ ShotPlan success value
            → ExecutionPolicy
            → Base0 XY + calibrated Z/A/B/C → Tool1/TCP ExecutionPlan
→ fake state machine
→ HRSDK / dual-DO adapters
→ Completed | SafeFailure | UnknownUnsafe
```

使用者操作與真實執行的唯一端到端順序為；一個`StartRequested`只允許一個shot cycle，完成後不得自動開始下一球：

```text
WaitingForStart
→ StartRequested
→ BilliardApp協調手臂移至CameraPose
→ 確認CameraPose成功且Robot motion stopped
→ 等待camera settle
→ SocketClient丟棄／flush CameraPose前的舊buffer
→ 重置三幀累積並開啟本shot-cycle capture window
→ Python拍照、辨識及pixel→Robot Base0 planar XY
→ SocketClient接收已轉換完成的newline-delimited 32值XY
→ VisionDataParser嚴格解析
→ 同一connection與shot cycle內、capture window開啟後的三個本地ReceiveEvent
→ 球與六袋資料一致性及三幀穩定
→ 最低號目標球
→ 六袋GhostBallPoint
→ DirectPot及一次母球碰庫KickPot
→ 碰撞檢查及共同評分（Direct顯著soft preference；高品質Kick可勝出）
├─ PotOnly無可行Direct／Kick：NoPlan(NoPotCandidate)
│  → CycleCompleted → WaitingForStart；不得自動LegalContact
└─ Base0 planar XY ShotPlan
   → MotionPlanner以Base0 XY、人工Strike Z、核准A/B及方向C建立Pose
   → Robot移至StrikeReadyPose並確認停止
   → DO1 ON → DO1 OFF → directionChangeDelay → DO2 ON → DO2 OFF
   → mechanismCompletionWait
   → PolicyAcceptedPneumaticCompletion
   → 讀取actual pose，保持X/Y/A/B/C不變，檢查並垂直LIN至PostStrikeSafeLiftPose
   → 確認安全高度
   → PTP返回CameraPose並確認停止
   → CycleCompleted
   → WaitingForStart
```

NoPlan只表示合法StableTableState已交給ShotBrain，但沒有合法撞球規劃結果。
Parser錯誤、NeedMoreEvents、Unstable、timeout、receive錯誤或connection reset屬於
Phase1PipelineResult，不得成為NoPlanReason。

Phase 1幾何統一使用PlayableBallCenterRegion、PocketExitSegment、PocketCaptureCorridor與
RailReflectionRegion；Kick統一使用PhysicalRailSegment、
EffectiveCueBallRailSegment及CueBallReboundPoint。這些幾何的唯一完整定義位於
Phase 1 Shot Brain Spec。

GhostBallPoint唯一表示母球與目標球首次接觸瞬間的母球球心位置，不是Robot TCP
strike position。ShotPlan不得包含Robot TCP位置；Phase 2不得直接把GhostBallPoint
轉成Robot Pose，而必須使用ShotPlan中的母球目前中心與擊球方向，依Phase 2 Shot
Executor Spec建立TCP strike-ready位置。

第一版KickPot只使用EffectiveCueBallRailSegment上的理想鏡射幾何，假設母球球心
方向滿足入射角等於反射角。此模型不估測或補償庫邊恢復係數、碰庫速度衰減、球體
旋轉、桌布摩擦或庫邊材質差異。Phase 1只依幾何門檻建立候選；Phase 2只以實驗核准
的單一固定力度envelope作fail-closed執行gate，不得對個別候選動態調整力度。

V1維持既有newline-delimited 32值CSV，不新增mandatory Python control handshake、
sender frame ID或sender timestamp。freshness由既有`BilliardApp + SocketClient`在本地
建立：connection identity、shot-cycle identity、嚴格遞增ReceiveEvent ID、CameraPose
settle後才開啟的capture window、舊buffer flush及三幀累積reset。disconnect、reconnect、
timeout或Parser failure均使本cycle累積失效；只有同一cycle中capture window開啟後的
三個有效event可形成StableTableState。RuntimeCalibrationAttestation、
AttestedVisionSession及versioned control message只列於External Contract的
`Future Hardening / Optional Future Protocol`，不是V1 production gate、ticket或驗收依賴。

32值wire中的六袋XY是本cycle規劃使用的唯一pocket center來源，必須與動態球一起通過
presence、finite、固定Pocket ID及三幀一致性／median規則。`TableGeometryConfig`只擁有
Pocket ID／type、rail topology、inward／outward normals、effective rails、capture corridor、
exit、collision與revision，並可對wire位置做容差驗證；它不得以另一組static pocket center
取代當次wire pocket center，也不得把不穩定單幀袋口與Stable球狀態混用。

Tool1的物理球桿forward axis是版本化人工校正屬性，不預設為局部`+X`。P2-03必須在
no-fire驗收中確認Tool1 TCP、forward axis、`CToolOffset`、A／B／C映射及Base0 `+Z`
確為物理安全上方；任一未確認即`CalibrationRequired`且禁止擊發。

ShotPlan稽核採`CommonAuditFields + plan-type-specific variant payload`。Pot、Kick及
LegalContact欄位的唯一適用性契約位於Phase 1 Shot Brain Spec；不適用欄位不得以0、
預設角度、空Point或其他假成功值補齊。

## 7. 全域安全不變量

- NaN、Infinity、溢位、退化幾何、缺少必要設定、矛盾資料、無交點、不可行路徑、不可達姿態及通訊／硬體失敗一律 fail-closed。
- 禁止回傳 `Point{0,0}`、預設 Point、原始輸入 Point、0 度或任意 Pose 作為失敗 fallback。
- 不可行候選必須在評分前排除，不得只給低分。
- 被拒候選只能存在CandidateDiagnostic；不得轉型為正式Candidate、ShotPlan或
  ExecutionPlan。
- 規劃失敗不得建立 ExecutionPlan；執行失敗不得觸發後續正常 DO 或移動。
- `motion_reachable()` 只代表目標可達，不代表 PTP 路徑安全。
- LIN 必須先通過 `motion_check_lin()`；失敗不得改用未驗證 PTP。
- DO1 與 DO2 永遠互斥。
- OFF命令失敗、結果未知或通訊中斷時進入`UnknownUnsafe`，禁止任何後續
  機械手臂動作。
- 無DO回讀時，成功寫入OFF只能記為`OffCommandAccepted`；是否允許繼續取決於
  P2-03硬體安全案例與ExecutionPolicy，不得宣稱`PhysicalOffConfirmed`。
- 擊球後第一個Robot motion只能是保持X／Y／A／B／C不變且已通過`motion_check_lin()`的垂直safe lift；確認安全高度前禁止任何retract PTP或camera PTP。氣動狀態未知時禁止抬升與返回。

## 8. 結果與設定語意

所有Result統一分成success value與Diagnostic metadata：

- 成功status必須有success value。
- 非成功status不得有success value。
- 成功與失敗都可以帶有Diagnostic metadata。
- Diagnostic不得被轉型或視為合法Point、Pose、Candidate、StableTableState、
  ShotPlan或ExecutionPlan，也不得包含fallback成功值。

設定錯誤統一分類：

- `ConfigurationMissing`：必要設定、校正或 normalization range 不存在。
- `InvalidConfiguration`：設定非有限、負值、範圍反轉或不符合不變量。
- `CalibrationRequired`：結構有效但尚未完成真實執行校正。
- `IntegrationRequired`：必要production application／infrastructure adapter尚未整合或完成相容性驗收；不得以此狀態表示校正缺失。
- `UnsupportedConfigurationVersion`：Deployment Calibration Manifest或其他版本化設定使用不受支援的schema version。

上述狀態可供離線測試，但不得被轉成可執行硬體計畫。

## 9. Phase 與十二個能力切片

| ID | 能力 | Phase |
|---|---|---|
| P1-01 | C++17 與離線測試框架 | Brain |
| P1-02 | 安全型別、GeometryResults 與純 MathUtils | Brain |
| P1-03 | 既有32值External Contract、本地Cycle／Session Boundary、嚴格 Parser 與單幀驗證 | Brain |
| P1-04 | 三幀穩定生命週期與 StableTableState | Brain |
| P1-05 | 桌面、袋口、庫邊與碰撞幾何 | Brain |
| P1-06 | 最低號球資格與 DirectPot | Brain |
| P1-07 | 一次母球 KickPot | Brain |
| P1-08 | Direct／Kick共同評分、確定性選擇與 PotOnly；LegalContact為選配研究能力 | Brain |
| P1-09 | ShotBrain 與 ShotPlan／NoPlan | Brain |
| P2-01 | ShotPlan→ExecutionPlan、StrikeReadyPose 與球桿方向驗證 | Executor |
| P2-02 | fake adapters、狀態機與雙 DO 安全 | Executor |
| P2-03 | HRSDK／真實 DO adapters 與受控驗收 | Executor |

不得為單一函式、enum、測試、build 修正或單一檔案另建主要 ticket。

### 9.1 Existing Responsibility Owners

| ID | 必須優先原地修改的既有owner |
|---|---|
| P1-01 | 既有離線建置與測試框架；已完成 |
| P1-02 | `Point`、`GeometryResults`、`MathUtils`；已完成 |
| P1-03 | `SocketClient`、`VisionDataParser`、`TableState`、`BilliardConfig`及必要的`BilliardApp` session integration |
| P1-04 | `TableState`、`BilliardConfig`及唯一三幀穩定狀態物件 |
| P1-05 | `BilliardPhysics`、`Point`、`GeometryResults`、`BilliardConfig` |
| P1-06 | `TargetSelector`、`Algorithm`、`BilliardPhysics`、`TableState` |
| P1-07 | `Algorithm`、`BilliardPhysics`、`Point` |
| P1-08 | `Algorithm`、`BilliardPhysics`、`TargetSelector`、`BilliardConfig` |
| P1-09 | `Algorithm`、`TableState`、`BilliardApp` |
| P2-01 | `MotionPlanner`、`BilliardConfig`、`MathUtils` |
| P2-02 | `BilliardApp`、`MotionPlanner`、`RobotController`介面、`BilliardConfig`及tests中的fake adapters |
| P2-03 | `RobotController`、`BilliardApp`、`BilliardConfig`，必要時既有`main.cpp` |

P1-04是Phase 1唯一可能合理新增production cpp的切片，但仍須先證明責任目前不存在、沒有合理既有owner、新檔不形成重複且會成為唯一owner。其他ticket不得以概念名稱強迫拆檔或建立平行實作。

### 9.2 Repository-Constrained Refactor治理

1. 每張ticket先列Existing Responsibility Owners、相關headers／tests及允許修改的責任。
2. 優先修改既有函式、類別與資料型別；不建立平行v2系統。
3. 不得保留舊實作再新增同功能實作；拆檔必須搬移責任，不得複製責任。
4. 新檔案必須證明責任目前不存在、沒有合理既有owner、不會形成重複，且新檔會成為唯一owner。
5. 不要求每個cpp都修改；只在現有缺口確實需要時改動。
6. 所有變更維持可編譯、可獨立review與回滾的小步commit。
7. 不得為符合概念架構而改變使用者指定的實際擊球流程。

### 9.3 Existing Ticket Refactoring治理

- To Tickets先建立完整Existing Ticket Inventory，再逐張與四份active specs及既有C++ owners比較。
- 每張舊ticket必須標記`Keep | Refactor | Rename | Move | Merge | Split | Superseded | Completed`；只有確有未覆蓋能力缺口時才可標記`New`。
- 有效需求、診斷、失敗處理與測試必須保留並追蹤；過時責任必須移除，驗收條件及owner必須重新綁定。rename／move／merge／split只在能力邊界確實需要時執行。
- 不得將舊tickets整批archive後另造十二張平行文件。十二個ID是固定能力identity，不代表必須建立十二份全新Markdown。
- P1-01與P1-02標記`Completed`，保留implementation commit、已完成scope、tests及依active specs重新驗證的結果；後續只開delta work，不重寫其歷史。
- CameraCompensator或VisionFrameProcessor等舊ticket中的C++相機補償責任失效，但其中仍有效的validation、diagnostics、failure handling及tests必須先遷移並建立trace，整張ticket才可Superseded／archive。
- To Tickets的權威allowlist仍只有本Master及三份active specs；`docs/archive`及superseded spec不得成為需求來源。

## 10. Testing Decisions

主要測試 seam：

```text
Parser seam:
raw wire + receive metadata → SingleFrameResult

Pipeline seam:
ReceiveEvent stream → Phase1PipelineResult

Brain seam:
StableTableState + TableGeometryConfig + BrainConfig → PlanningResult

Executor seam:
ShotPlan + ExecutionPolicy + calibrations → ExecutionResult + recorded commands
```

- Brain seam驗證KickPot在effective rail的理想入射角／反射角相等不變量，並證明Phase 1不讀取力度或庫邊／桌布動力學參數。
- Executor seam驗證全部可執行方案只使用同一個已校正固定氣動脈衝；超出FixedForceEnvelope時fail-closed，且不依候選距離或角度動態改變力度。

- P1-01至P1-09全部完全離線，不載入HRSDK、production Socket、RobotController或DO，沒有真實transport例外。
- P1-03以fake byte／connection／cycle source離線驗證newline framing、最大frame長度、嚴格32值Parser、本地ReceiveEvent、disconnect語意、cycle/session reset與stale-buffer protection，不開真實Socket。
- 既有`SocketClient + BilliardApp` production transport integration只接收既有32值data frame，執行本地capture-window gate後把bytes交給P1-03邊界；它不得包含規劃／硬體邏輯、不得成為ShotBrain依賴，也不得移入P2-03 HRSDK／DO adapter。
- P2-01、P2-02 只使用 fake／mock。
- P2-03 先通過 adapter contract 與錯誤注入，最後才允許氣壓斷電、低速、單球、人工急停環境的受控硬體驗收。
- 測試驗證外部行為與安全不變量，不綁定私有函式或容器順序。

### 10.1 Production Readiness Prerequisites

```text
Operating-system Socket
→ existing SocketClient + BilliardApp integration
→ CameraPose settled + stale-buffer flush + shot-cycle gate
→ P1-03 data/session interface
→ ReceiveEvent
→ Phase1Pipeline
```

- 現有`SocketClient`是唯一production transport owner；P1-03可原地擴充其最大frame長度、newline framing、connection lifecycle、本地identity及clean close／error區分。
- 該integration不得被宣稱由P1-03離線fake驗收自動完成；未整合或未驗收時，Phase 1與ShotBrain仍可完全離線完成，但production端到端operation狀態為`IntegrationRequired`並禁止啟用。
- 此production readiness前置依賴不新增第13個主要能力identity。
- To Tickets必須以Existing Ticket Refactoring方式，透過`Keep | Refactor | Rename | Move | Merge | Split | Superseded | Completed`，使最終active ticket set完整覆蓋既定12個能力identity。
- 十二個ID代表能力coverage，不代表必須重新產生12份新的Markdown ticket。
- 只有在完整Existing Ticket Inventory與coverage分析後，確認沒有任何existing ticket可以合理承接某能力時，才允許建立`New` ticket。
- `CalibrationRequired`只用於校正／FixedForceEnvelope未完成；transport integration缺失使用`IntegrationRequired`。
- Production vision Socket責任不屬於P2-03；P2-03只負責HRSDK、真實DO及受控機械手臂驗收。不得另建`ProductionVisionTransportAdapter.cpp`包裝或重複既有SocketClient；只有在明確取代並移除舊owner責任時才可另案審查。

## 11. Out of Scope

- target-bank、兩次以上庫邊、組合球、借球、跳球。
- 母球碰撞後位置、洗袋、旋轉、二次碰撞及完整動力學。
- 自動校正評分權重或固定力度。
- 未經 ExecutionPolicy 授權的 LegalContact 真實執行。
- 在本規格階段執行 main、calibrate、test_cueball、HRSDK、DO 或機械手臂。

## 12. Further Notes

- 評分權重為 `Initial Experimental Weights`，不是 production 最佳值。
- 暫定 `0 mm` 碰撞 margin 只可作離線實驗；真實執行前必須校正。
- Base0、Tool1、人工Strike Z、A／B基準及核准搜尋範圍、C adapter映射與safe-lift高度均須驗證，不等同已證明所有姿態與路徑安全。
- To Tickets 必須引用 active specs，不得以 superseded spec 或舊 tickets 覆蓋本文件。
