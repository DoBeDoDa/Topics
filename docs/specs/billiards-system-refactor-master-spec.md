# 撞球機械手臂重構 Master Architecture Spec

## 文件資訊

- 狀態：Approved
- 規格版本：2.0
- 基準 commit：`216bcb7`
- 適用系統：Python 視覺、C++ 擊球大腦、C++ 擊球手
- 下一步：To Tickets；本文件不構成實作或真實硬體操作授權

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

- `python-cpp-external-contract.md`：wire、TableFrame 與 Parser 邊界。
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

- Phase 1 只消費 TableFrame 毫米資料，產生可稽核 `ShotPlan | NoPlan`。
- Phase 2 將 ShotPlan 經標定轉換為 Base0 中的 Tool1 TCP 執行計畫，先以 fake 驗證，再由最後一張 ticket 接入 HRSDK 與真實雙 DO。
- 所有可能失敗的邊界使用具名結果；禁止預設成功、強制 fallback 或失敗後繼續正常流程。

## 4. User Stories

1. 作為視覺開發者，我要讓 Python 成為全部相機校正與補償的唯一責任端，避免 double compensation。
2. 作為演算法開發者，我要只接收 TableFrame 毫米座標，讓 Phase 1 不依賴相機或機械手臂。
3. 作為測試者，我要從receive-event stream離線重現Phase1PipelineResult，並在Stable seam以StableTableState獨立重現PlanningResult。
4. 作為操作者，我要在沒有合法方案時看到具名原因，而不是讓系統強制擊球。
5. 作為安全審查者，我要能由 ShotPlan 查明路徑、角度、淨空、分數與設定版本。
6. 作為機械手臂開發者，我要由 ExecutionPlan 得到已標定 frame 與 Pose，而不重新計算撞球策略。
7. 作為硬體操作者，我要讓未校正力度、未知 DO 狀態或失敗路徑阻止一切後續動作。
8. 作為 ticket 作者，我要能依十二個能力邊界建立可獨立驗證、commit 與回滾的工作。

## 5. 系統責任邊界

### 5.1 Python 視覺

唯一負責：影像擷取、球體辨識、鏡頭校正、perspective transform、homography、pixel 轉 TableFrame 毫米、固定 offset 與相機殘差補償。

Python 不負責撞球候選、碰撞安全、robot frame、Pose、HRSDK 或 DO。

### 5.2 C++ Phase 1

Phase 1由兩個不可混用的責任層組成：

- `Phase1Pipeline`：接收ReceiveEvent stream，執行SingleFrameResult、
  StabilityResult及Phase1PipelineResult生命週期。P1-03負責External Contract、
  Attested Session Boundary、Parser與單幀驗證；P1-04只負責三幀穩定生命週期。
- `ShotBrain`：只接受StableTableState、TableGeometryConfig與BrainConfig，負責
  撞球幾何、DirectPot、一次母球KickPot、評分、LegalContact及PlanningResult。
  P1-09整合此層，但不接收CSV或ReceiveEvent。

ShotBrain不得負責Parser、receive freshness、timeout或三幀累積；只有
StabilityResult中的Stable success value可以呼叫ShotBrain。

Phase 1 P1-01至P1-09完全離線，不得執行相機補償、TableFrame到robot frame轉換、真實production Socket、HRSDK、RobotController、DO或真實機械手臂。P1-03只定義transport-neutral contract、session state、attestation gate及adapter interface，並以fake byte／connection／session source驗證；Phase1Pipeline與ShotBrain核心不得依賴production transport。

### 5.3 C++ Phase 2：Shot Executor

唯一負責：已標定的 TableFrame 到 Base0 轉換、Base0 中的 Tool1 TCP Pose、ExecutionPolicy、ExecutionPlan、PTP／LIN、fake 與 real adapters、雙 DO 與安全終止。

Robot frame 轉換不是相機補償。Phase 2 必須執行必要的標定轉換，但不得重新套用鏡頭補償。

## 6. 全域資料流

```text
Python calibrated TableFrame-mm CSV
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
            → calibrated TableFrame→Base0→Tool1/TCP ExecutionPlan
→ fake state machine
→ HRSDK / dual-DO adapters
→ Completed | SafeFailure | UnknownUnsafe
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

32值wire不攜帶calibration revision。靜態Deployment Calibration Manifest只能驗證
部署設定，不能證明正在送frame的Python process實際載入哪個revision。每個receive
session在第一個32值frame前，Python必須透過版本化RuntimeCalibrationAttestation
控制握手聲明實際載入revision及session identity；C++只在runtime attested revision、
C++ active revision、manifest expected revision及manifest ID全部一致後接受該session。
不可重用session ID是AttestedVisionSession的主要identity；Python process ID只作
Diagnostic metadata，不能延續attestation或三幀累積。
完整manifest、attestation與connection lifecycle契約唯一由Python–C++ External
Contract定義。若缺少承載runtime attestation handshake的production transport integration，production維持IntegrationRequired；CalibrationRequired只表示校正或力度envelope尚未完成。

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
| P1-03 | External Contract、Attested Session Boundary、嚴格 Parser 與單幀驗證 | Brain |
| P1-04 | 三幀穩定生命週期與 StableTableState | Brain |
| P1-05 | 桌面、袋口、庫邊與碰撞幾何 | Brain |
| P1-06 | 最低號球資格與 DirectPot | Brain |
| P1-07 | 一次母球 KickPot | Brain |
| P1-08 | 正規化評分、確定性選擇與 LegalContact | Brain |
| P1-09 | ShotBrain 與 ShotPlan／NoPlan | Brain |
| P2-01 | ShotPlan→ExecutionPlan、frame 與 Pose | Executor |
| P2-02 | fake adapters、狀態機與雙 DO 安全 | Executor |
| P2-03 | HRSDK／真實 DO adapters 與受控驗收 | Executor |

不得為單一函式、enum、測試、build 修正或單一檔案另建主要 ticket。

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
- P1-03核心contract、control／data parsers、session state、attestation gate與transport adapter interface全部使用fake byte／session source離線驗證，不開真實Socket。
- 獨立application／infrastructure邊界`ProductionVisionTransportAdapter`只接收versioned control message與32值data frame，再把bytes交給P1-03定義的純interface；它不得包含規劃／硬體邏輯、不得成為Phase1Pipeline／ShotBrain依賴，也不得移入P2-03 HRSDK／DO adapter。
- P2-01、P2-02 只使用 fake／mock。
- P2-03 先通過 adapter contract 與錯誤注入，最後才允許氣壓斷電、低速、單球、人工急停環境的受控硬體驗收。
- 測試驗證外部行為與安全不變量，不綁定私有函式或容器順序。

### 10.1 Production Readiness Prerequisites

```text
Operating-system Socket
→ ProductionVisionTransportAdapter
→ P1-03 control/data/session interface
→ AttestedVisionSession
→ ReceiveEvent
→ Phase1Pipeline
```

- 若repository已有ProductionVisionTransportAdapter，只需在獨立application／infrastructure整合中依P1-03 interface完成相容性驗證。
- 若沒有合格adapter，它是端到端production operation的外部整合前置依賴，不得宣稱已由Phase 1離線驗收完成。
- Adapter未整合或未驗收時，Phase 1與ShotBrain仍可完全離線完成，但production端到端operation狀態為`IntegrationRequired`並禁止啟用。
- 此前置依賴不新增第13張主要ticket；To Tickets仍只產生既定12張，並在production readiness清單追蹤integration狀態。
- `CalibrationRequired`只用於校正／FixedForceEnvelope未完成；不得用於transport integration缺失。
- ProductionVisionTransportAdapter不屬於P2-03；P2-03只負責HRSDK、真實DO及受控機械手臂驗收。

## 11. Out of Scope

- target-bank、兩次以上庫邊、組合球、借球、跳球。
- 母球碰撞後位置、洗袋、旋轉、二次碰撞及完整動力學。
- 自動校正評分權重或固定力度。
- 未經 ExecutionPolicy 授權的 LegalContact 真實執行。
- 在本規格階段執行 main、calibrate、test_cueball、HRSDK、DO 或機械手臂。

## 12. Further Notes

- 評分權重為 `Initial Experimental Weights`，不是 production 最佳值。
- 暫定 `0 mm` 碰撞 margin 只可作離線實驗；真實執行前必須校正。
- Base0、Tool1、`RX=-180°`、`RY=0°` 是待驗證基準，不等同已證明所有姿態與路徑安全。
- To Tickets 必須引用 active specs，不得以 superseded spec 或舊 tickets 覆蓋本文件。
