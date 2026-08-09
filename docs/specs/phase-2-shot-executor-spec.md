# Phase 2：C++ Shot Executor Spec

## 文件資訊

- 狀態：Approved
- Final Workflow Verification：PASS（2026-08-01）
- 規格版本：1.1
- 能力範圍：P2-01 至 P2-03
- 唯一權威：Base0 planar XY到ExecutionPlan／Pose、ExecutionPolicy、test-only fake狀態機、PTP／LIN、固定力度、雙DO、HRSDK與硬體驗收

## 1. Problem Statement

現有流程把二維角度直接混入Euler欄位，且未明確限制A／B姿態搜尋；LIN檢查失敗後仍可能繼續PTP。氣動控制只寫單一DO並忽略回傳碼，擊球後也沒有先垂直安全抬升，無法表示OFF失敗或通訊中斷後的未知危險狀態。

## 2. Solution

Phase 2只消費有效的Base0平面XY ShotPlan，由既有MotionPlanner直接以XY、人工校正Strike Z、人工核准小區間A／B搜尋及方向C產生ExecutionPlan，不做TableFrame→Base0或第二次平面映射。再由fake adapters驗證完整狀態機、錯誤注入、雙DO互鎖及垂直safe lift；最後P2-03才接入HRSDK與真實DO。

正式BilliardApp runtime只提供兩種運行模式：

1. PlanningTest：
   接收既有32值Base0座標，完成穩定判斷、Phase 1 ShotPlan
   與P2-01 ExecutionPlan建立及診斷輸出；
   不初始化、不連線、不操作真實Robot、HRSDK或DO。

2. RealHardware：
   在全部ExecutionPolicy、calibration及revision驗證通過後，
   使用既有BilliardApp single-cycle流程及P2-03 real adapters
   執行完整真實擊球。

P2-02 fake adapters僅供automated/offline tests驗證RealHardware
state machine安全性，不是production runtime mode。

人工Tool/Base/DO/+Z等診斷屬Controlled Hardware Acceptance，
不是第三種BilliardApp runtime mode。

## 3. User Stories

1. 作為機械手臂開發者，我要直接由Base0平面XY方案建立Tool1 TCP Pose，而不重新計算撞球策略或轉換平面座標。
2. 作為安全審查者，我要在任何真實命令前用fake驗證所有狀態與失敗轉移。
3. 作為操作者，我要讓未授權LegalContact、未校正力度或無效frame阻止執行。
4. 作為硬體操作者，我要讓LIN檢查失敗、DO互鎖失敗或通訊中斷停止所有後續正常動作。
5. 作為故障處理者，我要能區分已知安全失敗與無法確認DO狀態的UnknownUnsafe。

6. 作為演算法開發者，我要在不初始化或操作真實機械手臂與DO的情況下，接收正常32值座標並完成Phase 1／P2-01規劃及終端診斷，以驗證選球、路徑、碰撞、評分與最終ExecutionPlan是否正確。

## 4. Capability Boundaries

### P2-01：ExecutionPlan and Pose

- 驗證ShotPlan不變量與calibration revisions。
- 驗證ShotPlan的local connection／shot-cycle identity、source event IDs與calibration revisions完整；缺少或不一致不得建立ExecutionPlan。
- 驗證CommonAuditFields與plan-type-specific variant payload相符；拒絕LegalContact中的Pot-only假成功值。
- 由Base0平面XY、人工Strike Z、核准範圍A／B及方向C建立Tool1/TCP Pose；不得執行TableFrame→Base0。
- 產生SafeApproachPose、StrikeReadyPose、版本化safe-lift derivation rule／height及CameraPose return語意計畫；最終PostStrikeSafeLiftPose只能在氣動完成後由current actual pose建立。
- 不連結HRSDK，不執行命令。

### P2-02：Fake Execution and Dual-DO Safety

- 定義MotionAdapter與PneumaticAdapter契約。
- 使用fake adapters執行完整狀態機及錯誤注入。
- 驗證ExecutionPolicy、固定力度envelope、DO互斥與UnknownUnsafe。
- 以既有BilliardApp owner驗證StartRequested、CameraPose、capture cycle、planning、execution、safe lift、return及WaitingForStart的完整單cycle流程。
- 不連結真實HRSDK或DO。
- P2-02 fake adapters及fake execution只屬automated/offline test harness，
不得作為production BilliardApp::run() runtime mode。

### P2-03：Real Adapters and Controlled Acceptance

- 實作HRSDK與真實雙DO adapters。
- 驗證Tool1／Base0、SDK回傳碼、reachable與LIN。
- 只有最後驗收步驟可在受控環境使用真實硬體。
- 以相同BilliardApp流程接入既有RobotController／HRSDK／DO，不承接vision Socket或另建application state machine。
- Production BilliardApp只在RealHardware mode使用P2-03 real adapters。

PlanningTest mode不得初始化RobotController硬體連線，
不得送出Tool/Base、motor、motion或DO命令。
### Existing Responsibility Owners

| ID | 既有owner |
|---|---|
| P2-01 | `MotionPlanner`、`BilliardConfig`、`MathUtils`；既有`MotionPlan`原地演進為ExecutionPlan語意 |
| P2-02 | `BilliardApp`、`MotionPlanner`、`RobotController`介面、`BilliardConfig`及tests中的fake adapters |
| P2-03 | `RobotController`、`BilliardApp`、`BilliardConfig`，必要時既有`main.cpp` |

ExecutionPlan不要求新增ExecutionPlanner.cpp；Execution State Machine由既有
BilliardApp與RobotController承接。Fake adapters只可在測試中替代硬體依賴，不得
複製整套控制器；Real adapters必須強化既有RobotController，不得建立第二個
RobotController或第二個application。

## 5. Coordinate Frames

### 5.1 Required Calibrations

- Base0 planar calibration及revision：必須與ShotPlan及受控部署設定一致；V1不依賴runtime attestation。
- Tool1 TCP：以氣動推桿完全縮回時的實體尖端標定。
- `cueForwardAxisTool`：版本化人工校正的Tool-local有限單位向量；不得在沒有實機證據時固定宣稱為`+X`。
- Tool1相對法蘭的controller設定與revision。
- 人工校正Strike Z、A／B基準與核准搜尋範圍、C Tool offset及safe lift高度的版本化設定。

ShotPlan中的Base0平面XY Point及direction不得再旋轉、平移或補償。方向向量必須重新驗證finite與單位長度。

### 5.2 Target Pose Semantics

- 擊球方向來自ShotPlan，不由Phase 2重新選擇。
- 經目前A／B／C姿態旋轉後，`cueForwardAxisTool`投影至Base0 XY必須與擊球方向對齊；`+X`只有在版本化校正明確指定時才可作其值。
- 核心Robot Pose語意為`Pose = (X,Y,Z,A,B,C)`。
- `A0`、`B0`是人工校正且版本化的基準；A／B只可在人工核准的小區間內搜尋。
- `C`唯一由Base0平面擊球單位方向`d=(dx,dy)`計算：`C = normalizeAngle(atan2(dy,dx) + CToolOffset)`。
- HRSDK的RX／RY／RZ表示與A／B／C的確切映射只存在HRSDK adapter邊界，且必須經P2-03驗證。
- HRSDK目標表示Base0中的Tool1 TCP Pose。adapter設定Tool1後，不得再手動重複套用同一Tool平移。
- 任一revision、方向、姿態設定或adapter角度映射無效時拒絕ExecutionPlan。

### 5.3 Bounded A/B Search

```text
A ∈ [A0 - deltaA, A0 + deltaA]
B ∈ [B0 - deltaB, B0 + deltaB]
```

- `A0`、`B0`、`deltaA`、`deltaB`、搜尋step、順序與tie-break必須finite、版本化且經人工核准。
- 搜尋必須有限且確定性；不得擴張範圍、產生未核准中繼Pose或以0／任意姿態fallback。
- 所有候選無效或不可達時回`NoExecutablePlan`，不得繼續Robot命令。
- 每個A／B候選必須投影已校正`cueForwardAxisTool`至Base0 XY，並滿足`angle(projectedCueForwardAxisXY, d) <= maxCueDirectionErrorDeg`；投影退化、非finite或超限即拒絕該候選。
- A／B搜尋不得修改由擊球方向唯一算出的C。
- 禁止無界、未校正或未核准的姿態搜尋；允許上述核准小區間內的確定性可達姿態搜尋。
- `test_cueball`中的既有A／B prototype若仍有效，後續只可移入／重構至既有MotionPlanner並補測試，不得複製成第二套搜尋實作。

## 6. StrikeReadyPose and Ready Gap

- GhostBallPoint是首次接觸瞬間的母球球心位置，不是Robot TCP位置；Phase 2不得直接把GhostBallPoint轉成Robot Pose。Robot在氣動擊發前到達的是`StrikeReadyPose`，不是TCP接觸點或`StrikePose`。
- ShotPlan中的母球目前中心`Cball`與擊球方向單位向量`d`位於Robot Base0平面XY。令`r`為ShotPlan所攜且與TableGeometry revision一致的`ballRadiusMm`、ready gap為`g = readyGapMm`，縮回尖端的strike-ready水平位置唯一為：

```text
TCPready = Cball - (r + g) * d
```

- `d`必須finite且在具名容差內為單位向量；無效時拒絕ExecutionPlan。
- `TCPready`必須使用母球目前中心`Cball`，不得以`Gpot`、`Gcontact`或`GkickContact`代替。
- `TCPready`的XY直接是Base0平面位置，不得執行任何座標轉換；Z由獨立、人工核准且版本化的Strike Z校正決定。
- 校正的`cueForwardAxisTool`沿`d`；氣動伸出後的物理行程由FixedForceEnvelope與氣動校正負責。
- 縮回尖端到母球表面的距離為`readyGapMm`。
- 基準關係：`centerToTcpMm = ballRadiusMm + readyGapMm`。
- 既有50 mm中心距只能作人工可調初始基準，不是通用物理常數。
- Strike Z由實機手動校正；缺少時回`ConfigurationMissing`。
- 氣動有效行程必須足以跨越readyGap並位於已驗證範圍；否則`CalibrationRequired／NoExecutablePlan`。

## 7. ExecutionPlan

P2-01只接受`PlanningResult`中的成功`ShotPlan` value。`NoPlan`、`CandidateDiagnostic`或任何pipeline Diagnostic均不是可轉換輸入；Phase 2不得從診斷資料重建、補全或復活被Phase 1拒絕的候選。

ShotPlan驗證必須遵守Phase 1的variant契約：DirectPot只需Common+Pot、KickPot需Common+Pot+Kick、DirectLegalContact需Common+LegalContact、KickLegalContact需Common+LegalContact+Kick。缺少必要payload、出現不適用成功欄位，或以0／預設角度／空Point補齊時拒絕ExecutionPlan；NotApplicable Diagnostic不得被當作成功值。

ExecutionPlan至少包含：

- ShotPlan identity、type及calibration revisions。
- Base0中的Tool1 TCP SafeApproachPose與StrikeReadyPose。
- `safeLiftHeightMm`與「氣動完成後由current actual pose保持X／Y／A／B／C、只增加Z」的唯一PostStrikeSafeLiftPose derivation rule；不得預存一個planned lift-start pose取代actual pose。
- transit joint與camera joint references。
- 需要執行的PTP／LIN段及其檢查方式。
- FixedForceEnvelope evaluation結果。
- ExecutionPolicy決策。
- DO1／DO2 timing profile reference。
- 每個狀態的preconditions、success conditions與failure transition。
- Production ExecutionPolicy mode只區分PlanningTest與RealHardware。
Fake adapter selection屬test dependency injection，不是ExecutionPlan runtime mode。

ExecutionPlan不得修改目標球、袋口、反彈點、評分或策略類型。

ExecutionPlan建構結果遵守共同Result規則：只有成功status可攜帶ExecutionPlan value；非成功status的value為空，但可攜帶不含fallback Pose／Plan的Diagnostic metadata。

## 8. ExecutionPolicy

ExecutionPolicy的production runtime mode只區分：

1. PlanningTest
2. RealHardware

PlanningTest：

- 可接收正式32-value Base0座標。
- 可執行vision receive/stability、Phase 1 PlanningResult及P2-01 ExecutionPlan。
- 可輸出完整規劃診斷。
- 不初始化真實HRSDK hardware session。
- 不設定真實Tool/Base。
- 不啟用motor。
- 不送出PTP/LIN。
- 不操作DO1/DO2。
- 不需要RealHardware專用的實機calibration gate才能進行純規劃。

RealHardware：

- 執行完整真實擊球cycle。
- 預設disabled。
- 必須explicit authorization。
- 任何真實hardware command之前，
  必須驗證全部必要policy、calibration與revision。
- LegalContact預設禁止。

Fake adapters不是ExecutionPolicy production runtime mode。
它們只用於P2-02/P2-03 automated/offline tests。

- Pot方案只有位於對應FixedForceEnvelope且全部校正有效時可執行。
- LegalContact在real hardware預設禁止。
- 未授權LegalContact轉成`NoExecutablePlan`並要求人工介入，不得自動擊發。
- LegalContact完成專門路徑、力度及單球驗收後，才可由顯式設定啟用。
- policy缺失、矛盾或未版本化時回`ConfigurationMissing／InvalidConfiguration`。
- Manual diagnostic不是BilliardApp production runtime mode。
Tool1、Base0、ABC mapping、DO、Base0 +Z等人工診斷，
屬P2-03 Controlled Hardware Acceptance程序。

## 9. FixedForceEnvelope

Phase 1只提供`FixedForceMode`；Phase 2以人工校正且實驗核准的單一固定氣動脈衝執行。所有獲准plan type共用同一個版本化`pneumaticPulseMs`，不得依個別候選的距離、切球角、袋口入射角、kick角度、分數或方案類型動態增減力度。

每個可執行方案類型的envelope至少定義：

- allowed plan types。
- min／max total path length。
- max cutting angle。
- max pocket entry angle。
- Kick的`maxExecutableKickRailAngleDeg`；它來自Phase 2 `FixedForceEnvelopeConfig`，是經固定力度實機校正的執行門檻，可等於或嚴於Phase 1 `BrainConfig`中的幾何`maxKickRailAngleDeg`，不得更寬。
- 共用`pneumaticPulseMs`與calibration revision。

預設政策：

- DirectPot：完成Direct專門校正後可啟用。
- KickPot：預設停用，需獨立距離、角度與固定力度驗收。
- LegalContact：真實硬體預設停用，需專門驗收。

Phase 1已把超過`maxKickRailAngleDeg`的Kick視為幾何不可行，因此它不可能以ShotPlan到達Phase 2。通過Phase 1幾何但超出實驗核准的`FixedForceEnvelope`（包括較嚴格的`maxExecutableKickRailAngleDeg`）時，原ShotPlan仍是合法規劃結果，但Phase 2回`NoExecutablePlan`且不建立可執行ExecutionPlan。不得自動增加力度、修改ShotPlan、改選CandidateDiagnostic或為候選產生不同pulse。缺少校正時回`ConfigurationMissing／CalibrationRequired`。固定力度不代表所有幾何可行候選具有足夠能量。

## 10. Motion Sequence

10.1 PlanningTest

WaitingForStart
→ StartRequested
→ flush/discard stale vision data as applicable
→ reset current-cycle stability
→ open planning receive cycle
→ receive current-cycle 32-value events
→ three-event stability
→ StableTableState
→ Phase 1 PlanningResult
→ if ShotPlan: build P2-01 ExecutionPlan
→ print planning diagnostics/result
→ CycleCompleted
→ WaitingForStart

PlanningTest禁止：

Robot connection
Tool command
Base command
motor command
PTP
LIN
DO1
DO2
CameraPose robot motion
safe lift robot motion

10.2 RealHardware

WaitingForStart
→ StartRequested
→ verify mode == RealHardware
→ with zero hardware calls, validate RealHardware authorization and static deployment configuration
→ initialize/connect real hardware
→ establish DO1 safe OFF state
→ establish DO2 safe OFF state
→ configure/confirm Tool1 and Base0
→ required motor/controller preparation
→ only now move CameraPose
→ confirm CameraPose reached and stopped
→ camera settle
→ flush/discard stale vision input and reset current-cycle accumulation
→ open the current-cycle capture window
→ receive current-cycle 32-value frames and complete three-event stability
→ Phase 1 ShotPlan／NoPlan
→ build the current P2-01 ExecutionPlan
→ validate this ExecutionPlan's ExecutionPolicy, revisions, LegalContact authorization and FixedForce/pneumatic profile
→ perform RealHardware-only Cartesian reachability and required LIN path checks
→ StrikeReady
→ DO
→ actual pose
→ safe lift
→ CameraPose
→ WaitingForStart

RealHardware不得在CameraPose到達、停止、settle及current-cycle capture完成前，
建立或使用本次ShotPlan／ExecutionPlan。靜態authorization／deployment config
可在connect前fail closed驗證，但當次plan policy與revision只能在current-cycle
ExecutionPlan建立後驗證。

- 正式送出笛卡兒目標前必須設定並確認Tool1與Base0。
- ExecutionPlan的`X,Y,Z,A,B,C`是Base0中的Cartesian Tool1 TCP Pose，
  不是joint values；Cartesian PTP必須使用既有HRSDK `ptp_pos()`，
  Cartesian LIN必須使用`lin_pos()`。
- `ptp_axis()`只接受joint `A1～A6`，不得傳入Cartesian `X,Y,Z,A,B,C`。
- Cartesian target reachability使用`motion_reachable()`；它只證明target
  pose可達，不證明整條PTP path安全。
- LIN segment必須使用`motion_check_lin(start,end)`檢查，失敗或不可達
  均不得呼叫`lin_pos()`，也不得fallback成PTP。
- 每次motion command、position read及SDK診斷都必須檢查具名結果。
- 取得actual pose失敗不得以planned pose代替。
- LIN check失敗不得改用PTP下降。
- `motion_reachable()`成功不代表PTP路徑安全；P2-03需使用可取得的安全檢查及受控驗收。
- 任一步失敗不得繼續正常strike或DO。

DO序列中Robot保持靜止；實際擊球由氣動推桿完成。擊球後唯一正常第一個Robot motion
是從DO2與氣動完成後讀取的current actual pose開始垂直LIN安全抬升。令該actual pose為
`(Xs,Ys,Zs,As,Bs,Cs)`：

```text
Xlift = Xs
Ylift = Ys
Zlift = Zs + safeLiftHeightMm
Alift = As
Blift = Bs
Clift = Cs
```

- `safeLiftHeightMm`必須finite且嚴格大於0；缺少為`ConfigurationMissing`，非法為`InvalidConfiguration`。
- P2-03必須以no-fire受控驗收確認Base0 `+Z`是實體安全上方；未確認、方向反轉或校正矛盾時為`CalibrationRequired／InvalidConfiguration`，禁止擊發。
- 到達安全高度前不得改變X／Y／A／B／C、沿擊球反方向後退或PTP返回CameraPose。
- safe-lift LIN必須先通過`motion_check_lin()`；失敗不得fallback成PTP。
- 氣動結果未知或未達`PolicyAcceptedPneumaticCompletion`時進入相應失敗狀態；`UnknownUnsafe`時禁止抬升與返回。
- 正常流程不得包含strike→ready PTP或strike→camera PTP。

## 11. Dual-DO Contract

硬體語意：DO1擊出，DO2收回；電磁閥保持切換後狀態，兩者不得同時ON。

正常序列：

```text
RealHardware initialization：

DO1 OFF
→ 驗證DO1 OFF command result
→ 若policy要求則確認目前可接受的OFF evidence
→ 成功後才處理DO2

DO2 OFF
→ 驗證DO2 OFF command result
→ 若policy要求則確認目前可接受的OFF evidence

只有雙DO都達到ExecutionPolicy接受的safe startup evidence，
才允許第一次Robot motion。
validate RealHardware authorization
↓
validate configurations
↓
connect/init hardware
↓
DO1 OFF + check
↓
DO2 OFF + check
↓
Tool/Base/controller preparation
↓
CameraPose
```

- DO1與DO2共用人工調整的`pneumaticPulseMs`。
- 換向等待與機構完成等待是不同參數，不得以脈衝時間替代。
- 所有時間必須有正值、有限上限及calibration revision。
- 任一命令失敗立即停止正常序列；不得繼續下一個正常ON命令。
- 若硬體沒有DO或氣缸回授，而所有寫入命令均成功，軟體狀態只能記為
  `OffCommandAccepted`，不得記為或宣稱`PhysicalOffConfirmed`。
- 沒有回授的真實執行只有在P2-03已驗證正常通訊下的控制器／閥體行為，且
  ExecutionPolicy明確接受該硬體安全案例時，才可由`OffCommandAccepted`繼續。

`PolicyAcceptedPneumaticCompletion`只在ExecutionPolicy依目前硬體能力接受可觀測證據時成立：

- 有經驗證實體回授時，證據可包含DO readback、valve feedback、cylinder retracted sensor或其他核准回授；只有回授確認安全才可形成`PhysicalOffConfirmed`。
- 無實體回授時，必須同時具備所有DO OFF寫入成功、adapter回報`OffCommandAccepted`、通訊保持正常、P2-03已驗證該控制器／閥體案例，且ExecutionPolicy明確允許；此路徑只能形成`OffCommandAccepted → PolicyAcceptedPneumaticCompletion`，不得升級成`PhysicalOffConfirmed`。
- 沒有ExecutionPolicy授權時，`OffCommandAccepted`不足以允許任何後續Robot移動。

## 12. Failure and UnknownUnsafe

本章所有結果均區分成功value與Diagnostic metadata。只有成功status可攜帶ExecutionPlan或執行成功value；失敗、`NoExecutablePlan`及`UnknownUnsafe`的成功value必須為空，但可攜帶具名原因、狀態、已接受命令及安全處置等Diagnostic。Diagnostic不得包含或被當作fallback Pose、ExecutionPlan或允許後續動作的依據。

對外測試seam統一為`ShotPlan + ExecutionPolicy + calibrations → ExecutionResult`。ExecutionResult以具名status表達計畫拒絕、SafeFailure、Completed或UnknownUnsafe，並遵守上述success value／Diagnostic metadata不變量。Completed audit必須記錄採用的pneumatic completion evidence及其結果是`PhysicalOffConfirmed`或僅為policy接受的`OffCommandAccepted`。

UnknownUnsafe只適用於已進入RealHardware hardware interaction的cycle；
PlanningTest不存在真實DO／Robot state，因此不得虛構UnknownUnsafe hardware evidence。

### 12.1 SafeFailure

在任何DO ON前發生的驗證、Pose、reachable、LIN或motion失敗，若可確認雙DO仍OFF，回SafeFailure並禁止擊發。

### 12.2 DO Failure

任一DO／通訊錯誤時：

1. 停止所有新正常命令。
2. best-effort要求DO1 OFF及DO2 OFF。
3. 若無法確認OFF或物理狀態，進入UnknownUnsafe。

此處「無法確認OFF」至少包含OFF命令回傳失敗、timeout、連線中斷或adapter
無法判定命令是否送達。單純缺少物理回授但OFF命令明確成功時使用
`OffCommandAccepted`，並受前述ExecutionPolicy與硬體驗收限制。

### 12.3 UnknownUnsafe

- Safety Critical terminal state。
- 禁止Robot retract、return camera、任何PTP、LIN或再次擊發。
- 禁止宣稱氣缸已收回或DO已實體OFF。
- 要求人工斷氣／斷電、急停及現場確認。
- 只有獨立、明確且經驗證的人工reset／安全撤離程序可以離開。

軟體寫入成功只證明命令結果，不證明實體閥體或氣缸狀態。硬體責任包括斷電安全狀態、電氣／控制互鎖、急停、氣源切斷及實際回授（若有）。

## 13. Adapter Contracts

MotionAdapter與PneumaticAdapter所有操作回傳具名結果，禁止`void`吞掉錯誤碼。

Fake adapters必須能注入：連線失敗、設定Tool/Base失敗、position read失敗、unreachable、LIN check失敗、motion失敗、DO1／DO2各步失敗、timeout及未知狀態。

Fake adapters只存在automated/offline test seam。
Production BilliardApp::run()不得將Fake視為runtime mode。


Real adapters只做SDK語意轉換與錯誤映射；不得包含策略、幾何、預設Pose或失敗fallback。

## 14. Testing Decisions

### P2-01

- 完全離線測試Base0 planar XY原值傳遞、calibration revision、版本化`cueForwardAxisTool`、ready gap、Pose有限性及禁止任何第二次平面座標轉換。
- 驗證ShotPlan local connection／shot-cycle identity或source event IDs缺失／不一致時拒絕ExecutionPlan。
- 驗證`TCPready = Cball - (r + g)d`、獨立Strike Z、非finite／非單位`d`拒絕，並防止GhostBallPoint或`Gpot`被誤作TCP位置。
- 驗證A／B只在版本化人工核准小區間內依固定step與tie-break確定性搜尋，C只由`atan2(dy,dx)+CToolOffset`產生且A／B不得修改C；每個候選投影的cue forward axis與`d`夾角不得超過`maxCueDirectionErrorDeg`，所有候選失敗回NoExecutablePlan且沒有fallback Pose。
- 驗證四種plan type只接受正確的Common／Pot／Kick／LegalContact payload組合；LegalContact含Pot-only值、NotApplicable被視為成功值或預設欄位時拒絕。
- 驗證只有ShotPlan成功value可產生ExecutionPlan；NoPlan、pipeline Diagnostic與CandidateDiagnostic皆被拒絕，且不會生成fallback Pose／Plan。

### P2-02
P2-02使用test-only fake adapters驗證未來RealHardware execution state machine。
P2-02不要求production BilliardApp::run()提供Fake runtime mode。

- 驗證一個StartRequested恰好一個cycle、CameraPose stopped／settle前不收frame、舊buffer flush、三幀只屬當前cycle、完成後返回WaitingForStart且不自動下一球。
- 驗證未授權LegalContact、未校正envelope、非法時間、DO互斥、OFF失敗及UnknownUnsafe。
- 驗證Phase 1幾何門檻與Phase 2力度門檻來自不同設定來源且保持分離：超過Phase 1 `BrainConfig.maxKickRailAngleDeg`的項目不可能成為輸入ShotPlan；通過Phase 1但超過`FixedForceEnvelopeConfig.maxExecutableKickRailAngleDeg`或其他FixedForceEnvelope範圍者回`NoExecutablePlan`。
- 驗證Phase 2不得從CandidateDiagnostic復活候選、不得自動增力，也不得在envelope拒絕後建立ExecutionPlan。
- 驗證全部可執行DirectPot、KickPot及經政策授權的LegalContact使用相同版本化`pneumaticPulseMs`；改變候選距離、角度、分數或plan type不得改變pulse，超出核准envelope只能回`NoExecutablePlan`。
- 驗證任何失敗後 recorded commands 不包含被禁止的後續動作。
- 驗證氣動完成後第一個Robot motion只能是已通過`motion_check_lin`的垂直safe lift；X／Y／A／B／C保持不變，確認安全高度前不得出現retract PTP或camera PTP，UnknownUnsafe後不得出現任何motion。
- fake PneumaticAdapter覆蓋有回授`PhysicalOffConfirmed`、無回授但policy接受`OffCommandAccepted`、無policy授權、OFF失敗、timeout、斷線及結果未知；Completed必須記錄實際採用的完成證據。

### P2-03

依序驗收：

1. adapter contract與錯誤碼映射。
2. 無HRSDK硬體的mock整合。
3. 連線但不移動的Tool1／Base0與Pose診斷。
4. 低速、安全高度、氣壓斷電的單段motion。
5. 氣壓電源仍斷電的DO邏輯。
6. 單球、人工可立即停止、非連續比賽的受控擊球。

任一步失敗即停止後續驗收，不得跳級。

PlanningTest
必須證明：
收到座標
→ Phase1
→ P2-01
→ 輸出結果
並且：
0 HRSDK connect
0 Tool/Base
0 motor
0 motion
0 DO

RealHardware
必須證明：
policy/config validation
發生在任何hardware command之前
以及：
DO1安全建立
→ DO2安全建立
→ 才能CameraPose


## 15. Acceptance Gates

真實硬體執行前必須全部滿足：
本章Acceptance Gates僅限制RealHardware mode。
PlanningTest不要求完成RealHardware實機驗收即可進行純規劃，
但仍必須遵守Phase1/P2-01本身所需的資料與規劃設定。

- Base0 planar calibration與Tool1 calibration有效且revision一致；不存在C++ TableFrame→Base0步驟。
- A／B核准搜尋範圍、C／HRSDK角度映射、`cueForwardAxisTool`、`CToolOffset`及方向投影已以不擊發診斷確認；不得只因程式假設`+X`而通過。
- Base0 `+Z`已以不擊發受控驗收確認為實體安全上方。
- Strike Z、ready gap、`safeLiftHeightMm`、氣動行程及安全高度已人工確認。
- Direct或Kick對應FixedForceEnvelope已校正。
- PTP／LIN速度、reachable及路徑檢查策略已驗收。
- DO1／DO2編號、互鎖、脈衝、換向延遲與完成等待已驗收。
- ExecutionPolicy已明確規定有／無實體回授時可接受的PolicyAcceptedPneumaticCompletion證據，且Completed audit可區分`PhysicalOffConfirmed`與`OffCommandAccepted`。
- 斷線、OFF失敗、急停與人工斷氣流程已演練。

## 16. Out of Scope

- 重新解析Python影像或相機補償。
- 重新選球、修改ShotPlan或重新評分。
- 自動尋找力度、無界／未校正／未核准姿態搜尋或未驗證中繼點；§5.3人工核准小區間內的確定性A／B搜尋不在此排除範圍。
- 未經政策授權的LegalContact真實執行。
- 正式連續自動比賽。

## 17. Further Notes

Base0、Tool1、A／B基準、核准搜尋範圍與C adapter映射都必須實機驗證。即使TCP XYZ不變，姿態改變仍可能造成法蘭與關節大幅移動；編譯成功、target reachable或fake成功均不等同實體路徑安全。
