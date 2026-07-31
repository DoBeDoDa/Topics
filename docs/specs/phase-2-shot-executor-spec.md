# Phase 2：C++ Shot Executor Spec

## 文件資訊

- 狀態：Approved
- 規格版本：1.0
- 能力範圍：P2-01 至 P2-03
- 唯一權威：frame轉換、ExecutionPlan、Pose、ExecutionPolicy、fake狀態機、PTP／LIN、固定力度、雙DO、HRSDK與硬體驗收

## 1. Problem Statement

現有流程把二維角度直接混入Euler欄位，沒有明確TableFrame→Base0契約；LIN檢查失敗後仍可能繼續PTP。氣動控制只寫單一DO並忽略回傳碼，無法表示OFF失敗或通訊中斷後的未知危險狀態。

## 2. Solution

Phase 2只消費有效ShotPlan，先以已標定frame chain產生ExecutionPlan，再由fake adapters驗證完整狀態機、錯誤注入及雙DO互鎖。最後P2-03才接入HRSDK與真實DO，且只有完成校正envelope的方案可在受控條件執行。

## 3. User Stories

1. 作為機械手臂開發者，我要把TableFrame方案轉成Base0中的Tool1 TCP Pose，而不重新計算撞球策略。
2. 作為安全審查者，我要在任何真實命令前用fake驗證所有狀態與失敗轉移。
3. 作為操作者，我要讓未授權LegalContact、未校正力度或無效frame阻止執行。
4. 作為硬體操作者，我要讓LIN檢查失敗、DO互鎖失敗或通訊中斷停止所有後續正常動作。
5. 作為故障處理者，我要能區分已知安全失敗與無法確認DO狀態的UnknownUnsafe。

## 4. Capability Boundaries

### P2-01：ExecutionPlan and Pose

- 驗證ShotPlan不變量與calibration revisions。
- 驗證ShotPlan的AttestedVisionSession／manifest reference與目前部署及calibration revisions一致；缺少或不一致不得建立ExecutionPlan。
- 驗證CommonAuditFields與plan-type-specific variant payload相符；拒絕LegalContact中的Pot-only假成功值。
- 執行TableFrame→Base0→Tool1/TCP frame chain。
- 產生approach、strike定位及return語意計畫。
- 不連結HRSDK，不執行命令。

### P2-02：Fake Execution and Dual-DO Safety

- 定義MotionAdapter與PneumaticAdapter契約。
- 使用fake adapters執行完整狀態機及錯誤注入。
- 驗證ExecutionPolicy、固定力度envelope、DO互斥與UnknownUnsafe。
- 不連結真實HRSDK或DO。

### P2-03：Real Adapters and Controlled Acceptance

- 實作HRSDK與真實雙DO adapters。
- 驗證Tool1／Base0、SDK回傳碼、reachable與LIN。
- 只有最後驗收步驟可在受控環境使用真實硬體。

## 5. Coordinate Frames

### 5.1 Required Calibrations

- `T_Base0_Table`：把TableFrame point轉成Base0 point的已標定剛體轉換。
- TableFrame及其revision：必須與ShotPlan一致。
- Tool1 TCP：以氣動推桿完全縮回時的實體尖端標定。
- Tool1相對法蘭的controller設定與revision。

點使用完整旋轉與平移；方向向量只使用旋轉部分並重新驗證finite與單位長度。

### 5.2 Target Pose Semantics

- 擊球方向來自ShotPlan，不由Phase 2重新選擇。
- Tool1局部`+X`對齊Base0中的擊球方向；`-X`是退桿方向。
- `RX=-180°`、`RY=0°`是待驗證水平基準。
- `RZ`由Base0平面中的擊球方向產生。
- Euler轉換只存在HRSDK adapter邊界；核心Pose使用明確旋轉表示。
- HRSDK目標表示Base0中的Tool1 TCP Pose。adapter設定Tool1後，不得再手動重複套用同一Tool平移。
- 任一frame、revision、矩陣正交性、方向或Euler分解無效時拒絕ExecutionPlan。

## 6. Strike Position and Ready Gap

- GhostBallPoint是首次接觸瞬間的母球球心位置，不是Robot TCP strike position；Phase 2不得直接把GhostBallPoint轉成Robot Pose。
- ShotPlan中的母球目前中心`C`與擊球方向單位向量`d`位於TableFrame。令`r`為ShotPlan所攜且與TableGeometry revision一致的`ballRadiusMm`、ready gap為`g = readyGapMm`，縮回尖端的strike-ready水平位置唯一為：

```text
TCPready = C - (r + g) * d
```

- `d`必須finite且在具名容差內為單位向量；無效時拒絕ExecutionPlan。
- `TCPready`必須使用母球目前中心`C`，不得以`Gpot`、`Gcontact`或`GkickContact`代替。
- `TCPready`先依已標定TableFrame→Base0轉換建立水平位置；Z由獨立Strike Z校正決定。
- Tool1局部`+X`沿`d`；氣動伸出後的物理行程由FixedForceEnvelope與氣動校正負責。
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
- Base0中的Tool1 TCP safe-approach與strike定位Pose。
- transit joint與camera joint references。
- 需要執行的PTP／LIN段及其檢查方式。
- FixedForceEnvelope evaluation結果。
- ExecutionPolicy決策。
- DO1／DO2 timing profile reference。
- 每個狀態的preconditions、success conditions與failure transition。

ExecutionPlan不得修改目標球、袋口、反彈點、評分或策略類型。

ExecutionPlan建構結果遵守共同Result規則：只有成功status可攜帶ExecutionPlan value；非成功status的value為空，但可攜帶不含fallback Pose／Plan的Diagnostic metadata。

## 8. ExecutionPolicy

ExecutionPolicy至少區分fake、manual diagnostic與real hardware模式。

- Pot方案只有位於對應FixedForceEnvelope且全部校正有效時可執行。
- LegalContact在real hardware預設禁止。
- 未授權LegalContact轉成`NoExecutablePlan`並要求人工介入，不得自動擊發。
- LegalContact完成專門路徑、力度及單球驗收後，才可由顯式設定啟用。
- policy缺失、矛盾或未版本化時回`ConfigurationMissing／InvalidConfiguration`。

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

正常順序：

```text
SafeIdle
→ validating ShotPlan / policy / calibrations
→ Joint PTP to transit
→ Cartesian PTP to safe approach
→ read actual pose
→ motion_check_lin(actual, strike pose)
→ LIN to strike positioning pose
→ verify motion stopped and strike preconditions
→ pneumatic strike/retract sequence
→ verify PolicyAcceptedPneumaticCompletion
→ Joint PTP to camera joint
→ Completed
```

- 正式送出笛卡兒目標前必須設定並確認Tool1與Base0。
- 每次motion command、position read及SDK診斷都必須檢查具名結果。
- 取得actual pose失敗不得以planned pose代替。
- LIN check失敗不得改用PTP下降。
- `motion_reachable()`成功不代表PTP路徑安全；P2-03需使用可取得的安全檢查及受控驗收。
- 任一步失敗不得繼續正常strike或DO。

## 11. Dual-DO Contract

硬體語意：DO1擊出，DO2收回；電磁閥保持切換後狀態，兩者不得同時ON。

正常序列：

```text
best-effort DO1 OFF and DO2 OFF at initialization
→ DO1 ON for pneumaticPulseMs
→ DO1 OFF
→ wait directionChangeDelayMs
→ DO2 ON for pneumaticPulseMs
→ DO2 OFF
→ wait mechanismCompletionWaitMs
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

Real adapters只做SDK語意轉換與錯誤映射；不得包含策略、幾何、預設Pose或失敗fallback。

## 14. Testing Decisions

### P2-01

- 完全離線測試frame point／direction轉換、calibration revision、Tool +X對齊、ready gap、Pose有限性及禁止重複Tool transform。
- 驗證ShotPlan AttestedVisionSession／manifest reference缺失或與目前部署不一致時拒絕ExecutionPlan。
- 驗證`TCPready = C - (r + g)d`、獨立Strike Z、非finite／非單位`d`拒絕，並防止GhostBallPoint或`Gpot`被誤作TCP位置。
- 驗證四種plan type只接受正確的Common／Pot／Kick／LegalContact payload組合；LegalContact含Pot-only值、NotApplicable被視為成功值或預設欄位時拒絕。
- 驗證只有ShotPlan成功value可產生ExecutionPlan；NoPlan、pipeline Diagnostic與CandidateDiagnostic皆被拒絕，且不會生成fallback Pose／Plan。

### P2-02

- 使用fake adapters驗證每一合法狀態轉移與每一錯誤注入點。
- 驗證未授權LegalContact、未校正envelope、非法時間、DO互斥、OFF失敗及UnknownUnsafe。
- 驗證Phase 1幾何門檻與Phase 2力度門檻來自不同設定來源且保持分離：超過Phase 1 `BrainConfig.maxKickRailAngleDeg`的項目不可能成為輸入ShotPlan；通過Phase 1但超過`FixedForceEnvelopeConfig.maxExecutableKickRailAngleDeg`或其他FixedForceEnvelope範圍者回`NoExecutablePlan`。
- 驗證Phase 2不得從CandidateDiagnostic復活候選、不得自動增力，也不得在envelope拒絕後建立ExecutionPlan。
- 驗證全部可執行DirectPot、KickPot及經政策授權的LegalContact使用相同版本化`pneumaticPulseMs`；改變候選距離、角度、分數或plan type不得改變pulse，超出核准envelope只能回`NoExecutablePlan`。
- 驗證任何失敗後 recorded commands 不包含被禁止的後續動作。
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

## 15. Acceptance Gates

真實硬體執行前必須全部滿足：

- TableFrame→Base0與Tool1 calibration有效且revision一致。
- RX／RY／RZ及Tool +X方向已以不擊發診斷確認。
- Strike Z、ready gap、氣動行程及安全高度已人工確認。
- Direct或Kick對應FixedForceEnvelope已校正。
- PTP／LIN速度、reachable及路徑檢查策略已驗收。
- DO1／DO2編號、互鎖、脈衝、換向延遲與完成等待已驗收。
- ExecutionPolicy已明確規定有／無實體回授時可接受的PolicyAcceptedPneumaticCompletion證據，且Completed audit可區分`PhysicalOffConfirmed`與`OffCommandAccepted`。
- 斷線、OFF失敗、急停與人工斷氣流程已演練。

## 16. Out of Scope

- 重新解析Python影像或相機補償。
- 重新選球、修改ShotPlan或重新評分。
- 自動尋找力度、姿態或未驗證中繼點。
- 未經政策授權的LegalContact真實執行。
- 正式連續自動比賽。

## 17. Further Notes

Base0、Tool1、`RX=-180°`、`RY=0°`是待驗證基準。即使TCP XYZ不變，姿態改變仍可能造成法蘭與關節大幅移動；編譯成功、target reachable或fake成功均不等同實體路徑安全。
