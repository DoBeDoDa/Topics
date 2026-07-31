# Phase 1：C++ Shot Brain Spec

## 文件資訊

- 狀態：Approved
- 規格版本：1.0
- 基準 commit：`216bcb7`
- 能力範圍：P1-01 至 P1-09
- 唯一權威：Parser後資料生命週期、TableGeometry、候選、評分、LegalContact、ShotPlan／NoPlan

## 1. Problem Statement

現有核心流程直接從單幀資料選球，以單一袋口角度及不完整庫邊模型產生決策；退化幾何可能回傳原始Point，不安全路徑可能被強制執行，且輸出缺乏可稽核路徑、分數與失敗語意。

## 2. Solution

Phase 1建立兩個完全離線的責任層：Phase1Pipeline負責嚴格輸入與三個receive
events的動態球穩定；ShotBrain只接受StableTableState、TableGeometryConfig與
BrainConfig，負責DirectPot、一次母球KickPot、實驗性正規化評分、最低限度
LegalContact及完整`ShotPlan | NoPlan`。

## 3. User Stories

1. 作為演算法開發者，我要讓sentinel只存在於Parser，使幾何只處理有效optional資料。
2. 作為測試者，我要由三個離線receive events重現StableTableState。
3. 作為九號球操作者，我要永遠先以最低號存在球為合法目標。
4. 作為策略開發者，我要比較全部可行DirectPot與一次KickPot，而不是預選單一袋口。
5. 作為安全審查者，我要讓每段路徑及退化幾何具有具名結果。
6. 作為研究者，我要看到每個候選的原始量、正規化成本、權重與總成本。
7. 作為操作者，我要在沒有進球方案時得到誠實的LegalContact或NoPlan，而不是假安全球。
8. 作為Phase 2開發者，我要收到不含Robot Pose的完整ShotPlan。

## 4. Capability Boundaries

### P1-01

保留既有C++17與離線測試框架，不重新拆分。

### P1-02

保留既有安全Point／Vector、GeometryResults與純MathUtils。MathUtils不得依賴相機、BilliardConfig業務參數、Parser、策略、MotionPlanner或HRSDK。

### P1-03

能力名稱：`External Contract、Attested Session Boundary、嚴格Parser與單幀驗證`。

P1-03包含RuntimeCalibrationAttestation資料契約、versioned control-message schema與
parser、static manifest與runtime attestation validation、AttestedVisionSession狀態模型、
以不可重用session ID為主要identity的connection／session綁定與session gate、
reconnect／session ID改變時
失效及重置、32值data frame只在attested session下形成ReceiveEvent、全部錯誤的具名
Result／Diagnostic、transport adapter interface及fake byte／session source完整離線測試。
Python process ID只作可能重用的Diagnostic metadata，不得延續attestation或三幀累積。

P1-03不包含ShotBrain、三幀穩定演算法、相機補償、RobotController、HRSDK、DO、
真實機械手臂、production策略決策、真實TCP Socket、production Socket連線、
production SocketClient具體實作或application orchestration。P1-03只定義
transport-neutral interface；ProductionVisionTransportAdapter是獨立application／
infrastructure前置依賴，不屬於P1-03離線核心驗收，不得成為Phase1Pipeline／ShotBrain
依賴或移至P2-03。其integration未完成時production為IntegrationRequired。

### P1-04

實作Phase1Pipeline的receive-event生命週期、StabilityResult、三幀動態球
presence、median與完全重置；只有Stable success value才產生StableTableState。
P1-04不解析control message或32值CSV，也不建立／驗證AttestedVisionSession。

### P1-05

實作TableGeometryConfig、PocketModel、PlayableBallCenterRegion、
PocketExitSegment、PocketCaptureCorridor、RailReflectionRegion、PhysicalRailSegment、
EffectiveCueBallRailSegment、GhostBallPoint、球體碰撞、鏡射及交點。

### P1-06

決定最低號存在球，對六袋產生全部DirectPot候選。

### P1-07

對六段有效庫邊產生全部一次母球KickPot候選。

### P1-08

驗證與正規化評分、確定性決勝；無Pot候選時產生LegalContact。

### P1-09

整合ShotBrain並由StableTableState、TableGeometryConfig與BrainConfig輸出可稽核
PlanningResult。P1-09不接收CSV、ReceiveEvent，不負責freshness、timeout、Parser
或三幀累積。

## 5. Data Lifecycle

```text
Phase1Pipeline:
ReceiveEvent stream + InputContractConfig + StabilityConfig
→ SingleFrameResult
→ StabilityResult
→ on Stable success value only, call ShotBrain through its separate API
→ Phase1PipelineResult

ShotBrain:
StableTableState + TableGeometryConfig + BrainConfig
→ qualified target
→ feasible Candidate set + separate CandidateDiagnostics
→ scored feasible Candidates
→ PlanningResult
```

概念結果分層：

- `SingleFrameResult`：`Valid(success value)`或`Rejected(Diagnostic metadata)`。
- `StabilityResult`：`NeedMoreEvents`、`Stable(success value)`、`Unstable`、
  `TimedOut`或`InvalidConfiguration`。
- `PlanningResult`：`ShotPlan`或`NoPlan`。
- `Phase1PipelineResult`：`Waiting`、`InputFailure`、`StabilityFailure`或
  `PlanningCompleted(PlanningResult)`。此最外層組合結果不改變ShotBrain API；
  ShotBrain仍只接受StableTableState、TableGeometryConfig與BrainConfig。

只有StabilityResult的Stable success value可以呼叫ShotBrain。CSV格式錯誤、
paired sentinel錯誤、NeedMoreEvents、timeout、Unstable、receive event錯誤及
connection reset不得成為NoPlanReason。

Phase1Pipeline及ShotBrain啟用前必須已通過External Contract的static manifest與
runtime session雙重gate：Python runtime attested、C++ active與manifest expected
revision及manifest ID全部一致。manifest中的Python expected欄位不是runtime evidence；
缺少attestation、revision不一致、session改變或schema不支援不得進入規劃，並依
External Contract清除三幀累積。單一32值payload不得被視為revision一致的證據。
在P1-03 RuntimeCalibrationAttestation契約尚未完成前，P1-03能力本身未完成；若純契約已完成但ProductionVisionTransportAdapter尚未整合／驗收，production維持IntegrationRequired。CalibrationRequired只用於校正缺失。

- Parsed／Validated／Stable型別不得隱式互轉。
- 所有Result的成功status必須有success value；非成功status不得有success value。
  成功與失敗都可以有Diagnostic metadata，但Diagnostic不得被轉型或視為合法
  Point、StableTableState、Candidate、ShotPlan或ExecutionPlan，也不得包含
  fallback成功值。
- StableTableState只含動態球的穩定中位數與來源event metadata；袋口來自TableGeometryConfig。
- 任一格式錯誤、必要球缺失、presence改變、位置超限、timeout或event ID錯誤會清空整個累積器。
- 異常後第一個有效event重新算第1幀。
- 暫時以三幀presence判定在場球；無法區分已進袋與連續漏偵測，不能宣稱完整比賽狀態驗證。

## 6. StableTableState

- 必須恰由三個合法、連續receive events產生。
- 比較母球及1至9號球presence；袋口不參與。
- 每個三幀皆存在球分別取X中位數及Y中位數。
- 每幀點到medianPoint的歐氏距離不得大於`stableFrameToleranceMm`；等於門檻通過。
- 三幀皆缺失的編號球保留nullopt。
- 母球與至少一顆編號球必須存在。
- tolerance缺失回`ConfigurationMissing`；負值或非有限回`InvalidConfiguration`。

## 7. TableGeometry and PocketModel

TableGeometryConfig 是六袋、六段PhysicalRailSegment及三種合法區域的唯一規劃
權威，至少包含：

- TableFrame revision。
- PlayableBallCenterRegion。
- 六個具名PocketModel及其PocketExitSegment與PocketCaptureCorridor。
- 六段具名PhysicalRailSegment、桌內單位法線及端點／袋口排除區。
- 由physical rails推導的RailReflectionRegion。
- 球半徑、球直徑與球對球collision margin。

PocketModel包含：

- pocket ID與corner／side類型。
- TableFrame中的入口中心Point。
- 單位外向法線。
- finite且嚴格大於0的`virtualTargetOffsetMm`及VirtualPocketTarget。
- 與該pocket ID唯一綁定的PocketExitSegment。
- 與該pocket ID唯一綁定的PocketCaptureCorridor。
- 必要、finite且嚴格大於0的`corridorHalfWidthMm`。
- 必要、finite且嚴格大於0的`pocketBoundaryProbeEpsilonMm`。
- 必要、finite且滿足`0 <= exitCrossingEpsilon < 1`的無因次方向門檻。
- 角袋或中袋最大入射角。

第一版`PocketCaptureCorridor`採有向保守矩形。令入口中心為`E`、外向單位法線
為`nOut`、與桌面共面且垂直於`nOut`的單位橫向量為`nSide`、
`L = ||VirtualPocketTarget - E||`、半寬為`w = corridorHalfWidthMm`，則矩形唯一表示為：

```text
q = E + s * nOut + t * nSide
0 <= s <= L
|t| <= w
VirtualPocketTarget = E + L * nOut
```

- centerline起點固定為`E`，終點固定為VirtualPocketTarget，方向固定為`nOut`。
- 目標球中心進入此矩形後，才可經對應袋口離開PlayableBallCenterRegion。
- VirtualPocketTarget必須位於矩形內，且每個corridor只能綁定一個pocket ID。
- `virtualTargetOffsetMm`或`corridorHalfWidthMm`缺失為ConfigurationMissing；任一為0、負值、NaN或Infinity皆為InvalidConfiguration。
- `L`非有限／非正、矩形零面積、橫向量無法建立、法線錯誤或矩形與入口／VirtualPocketTarget矛盾皆為InvalidConfiguration。
- 此矩形只是第一版出口／捕獲區近似，不宣稱完整袋口寬度、jaw碰撞或真實進袋物理。

`PocketExitSegment`是`closure(PlayableBallCenterRegion)`與該
PocketCaptureCorridor之間非空、連續且唯一的合法交界，並遵守：

- segment位於PlayableBallCenterRegion邊界上，與corridor相交且只綁定同一pocket ID。
- 令入口中心為`E`、外向單位法線為`nOut`、segment為`S`；`E`必須位於`S`的相對內部，不得只位於端點。
- `nOut`必須由PlayableBallCenterRegion指向該PocketCaptureCorridor的捕獲方向。令具名正數`epsilon = pocketBoundaryProbeEpsilonMm`，必須同時滿足：

  ```text
  E - epsilon * nOut ∈ interior(PlayableBallCenterRegion)
  E + epsilon * nOut ∈ PocketCaptureCorridor
  E + epsilon * nOut ∉ interior(PlayableBallCenterRegion)
  ```

- 不得落在一般有效反彈庫邊中。
- 不得與其他PocketExitSegment重疊；若只在幾何端點接觸，僅在TableGeometryConfig明確列為允許且不造成pocket ID歧義時成立。
- corridor與playable region之間有空隙、過度重疊造成多重穿越、存在多個出口、出口不連續、pocket ID不一致、法線反轉、probe兩側都在playable、兩側都在corridor或無法建立上述局部跨界關係皆為InvalidConfiguration。
- PocketExitSegment、`pocketBoundaryProbeEpsilonMm`或`exitCrossingEpsilon`缺失為ConfigurationMissing；epsilon非正／非finite或exitCrossingEpsilon超出`[0,1)`為InvalidConfiguration。

零法線、無法正規化、區域非有限或缺少六袋亦為InvalidConfiguration；其他必要區域設定缺失為ConfigurationMissing。第一版PocketExitSegment與矩形corridor仍只是保守出口模型，不宣稱完整jaw、袋角或真實袋口物理。

## 8. Geometry and Collision

### 8.1 Regions

`PlayableBallCenterRegion`是一般球心合法移動區域，由實體playing surface向桌內
縮球半徑形成。母球所有路徑、GhostBallPoint及尚未進入袋口的目標球路徑必須
位於此區域。

每個`PocketCaptureCorridor`是由所選袋口入口延伸至VirtualPocketTarget的保守
合法捕獲區域：

- 只有所選目標球可透過所選PocketCaptureCorridor離開
  PlayableBallCenterRegion。
- 目標球穿越所選PocketExitSegment前必須位於合法桌面球心區域；穿越後必須立即進入同一pocket ID的PocketCaptureCorridor並最後到達VirtualPocketTarget。
- 目標球穿越出口時的finite單位移動方向`d`必須滿足`dot(d, nOut) > exitCrossingEpsilon`。等於或接近0代表沿出口切行，不是合法穿越；小於0代表由corridor反向進桌，必須拒絕。
- VirtualPocketTarget必須位於對應corridor。
- 目標球不得穿越其他PocketExitSegment、一般桌面邊界，亦不得先離開PlayableBallCenterRegion後再進入corridor或利用不連續空隙進袋。
- 母球及其他球不得使用PocketExitSegment或PocketCaptureCorridor越界。

`RailReflectionRegion`由全部有效EffectiveCueBallRailSegment構成，只允許一次
Kick的母球中心CueBallReboundPoint使用；不得將一般越界路徑當成rail reflection。

### 8.2 Physical and Effective Rails

`PhysicalRailSegment`表示實體庫邊，rail ID永遠對應此segment。每段physical rail
必須具有指向PlayableBallCenterRegion的有限桌內單位法線`nInward`。

母球中心鏡射與交點只使用：

```text
EffectiveCueBallRailSegment
= PhysicalRailSegment + ballRadiusMm * nInward
```

- `CueBallReboundPoint`表示母球球心位於effective rail上的反彈位置。
- 球半徑只在physical→effective平移加入一次。
- 球對球collision margin不得加入rail offset，也不得重複加入球半徑。
- physical rail的袋口與端點排除區必須以相同縱向參數映射／裁切到effective rail。
- 法線方向錯誤、非單位、非有限、零長度physical rail或平移／裁切後無有效區段
  均為InvalidConfiguration。

### 8.3 GhostBallPoint

GhostBallPoint唯一表示「母球與目標球首次接觸瞬間，母球球心應位於的位置」。
它不是目標球中心、兩球表面接觸點、球桿尖端或Robot TCP位置。

令`C`為母球目前球心、`T`為目標球球心、`r`為球半徑、`P`為
VirtualPocketTarget。Pot方案使用：

```text
u = normalize(P - T)
Gpot = T - 2r * u
```

`Gpot`必須滿足`||T-Gpot|| = 2r`，且Gpot、T、P共線，Gpot位於T相對P的
反方向。若需要表面接觸診斷：

```text
BallSurfaceContactPoint K = T - r * u
```

K只能存在Diagnostic metadata，不得成為母球中心規劃路徑終點。

零長度方向、非有限值、無法正規化、母球與目標球重疊或初始球心距離小於
`2r`皆fail-closed，不得回傳T、K或原始Point。

### 8.4 Pocket Entry Angle

DirectPot與KickPot使用同一個袋口入射角定義。令目標球球心為`T`、
VirtualPocketTarget為`P`：

```text
uPocket = normalize(P - T)
entryAngle = acos(clamp(dot(uPocket, pocket.outwardUnitNormal), -1, 1))
```

- `acos`結果轉成degree後形成`entryAngle`；0度代表目標球正沿袋口外向／捕獲方向移動，角度越大越斜，180度代表完全反向並必須拒絕該候選。
- 等於PocketModel門檻通過；超過門檻即幾何不可行，只可形成CandidateDiagnostic。
- `clamp`只處理已驗證finite的浮點捨入，不得使用`abs(dot)`掩蓋反向法線。
- 外向法線非有限、非單位、指向反向或與PocketCaptureCorridor centerline矛盾時為InvalidConfiguration；候選方向與合法capture方向矛盾時候選Invalid。
- Scoring的pocket entry raw value唯一使用此`entryAngle`。

### 8.5 Collision

- 路徑結果為`Clear | Blocked | Invalid`。
- 零長度、非有限、溢位、負margin或非法容差為Invalid。
- 球對球阻擋門檻為`ballDiameterMm + clearanceMarginMm`；距離等於門檻視為Blocked。
- 初始實驗margin可為`0 mm`，但不得視為production校正值。
- 最近距離必須限制在線段，正確處理兩端點。
- 每條候選路徑明確列出應排除的起點／終點相關球；不得以座標特例忽略障礙。
- 鏡射、ray-segment交點及方向正規化失敗必須顯式回傳失敗。
- 被拒候選只可形成CandidateDiagnostic，不得進入正式Candidate集合或評分。

## 9. Target Qualification

- 合法目標是StableTableState中最低號的存在球。
- 不遍歷其他編號球作為第一接觸目標。
- 對全部六個PocketModel建立候選。
- 沒有編號球時回`NoPlan(NoEligibleTarget)`，不得建立預設目標。
- `NoEligibleTarget`保留為defensive status及future ExpectedBallSet compatibility；在目前「StableTableState必有至少一顆編號球」的正式不變量下正常不可達。正常Phase1Pipeline整合fixture不得產生它；直接呼叫ShotBrain且輸入違反上層假設時可作防禦測試。
- 本期不使用ExpectedBallSet；此限制必須出現在診斷與文件。

## 10. DirectPot

每個DirectPot至少驗證：

1. `u = normalize(P - T)`有效，且依§8.3建立`Gpot = T - 2r*u`。
2. `Gpot`位於`PlayableBallCenterRegion`，並滿足全部GhostBallPoint不變量。
3. 母球中心路徑明確為`C → Gpot`且Clear；不得以`T`或表面接觸點`K`取代終點。
4. 目標球中心路徑為`T → P`且Clear：穿越所選PocketExitSegment前位於合法桌面球心區域，穿越後立即進入同一PocketModel的PocketCaptureCorridor並到達其中的`P`。
5. 兩條路徑使用各自正確的障礙端點排除。
6. 切球角合法。
7. 袋口入射角不超過該PocketModel門檻。
8. 目標球不得經其他PocketExitSegment、錯誤capture corridor、一般實體庫邊或不連續空隙離開桌面。
9. 所有中間量有限。

任何一項失敗即排除候選，不產生低分候選；拒絕資訊只可記入`CandidateDiagnostic`。

## 11. KickPot

- 只支援母球先碰一次庫邊，再碰最低號球，目標球直接進袋。
- 對六個`EffectiveCueBallRailSegment`使用理想鏡射幾何；不得用`PhysicalRailSegment`計算母球球心鏡射或交點。
- `CueBallReboundPoint`是母球球心的有效反彈點，必須位於相應effective segment及`RailReflectionRegion`，且不在映射後的袋口／端點排除區。
- Pot方向及GhostBallPoint仍依§8.3計算：`u = normalize(P - T)`、`Gpot = T - 2r*u`。
- 母球中心路徑明確為`C → CueBallReboundPoint → Gpot`；第二段不得終止於`T`或`K`。
- 目標球中心路徑為`T → P`，必須依序穿越所選PocketExitSegment、進入同一PocketCaptureCorridor並到達VirtualPocketTarget。
- 三段分別檢查障礙、適用region及端點排除；每段的球半徑與collision margin只能依§8.2及§8.5各計一次。
- kick入射角不得超過Phase 1幾何門檻`maxKickRailAngleDeg`。超過即幾何不可行，不得進入正式候選或評分集合，只可留下`CandidateDiagnostic`。
- 第一版假設母球在effective rail滿足入射角等於反射角。令`R`為CueBallReboundPoint、`nInward`為該effective rail的桌內單位法線：

  ```text
  dIncoming = normalize(R - C)
  dOutgoing = normalize(Gpot - R)
  idealReflected = dIncoming - 2 * dot(dIncoming, nInward) * nInward

  incidenceAngle = acos(clamp(dot(-dIncoming, nInward), -1, 1))
  reflectionAngle = acos(clamp(dot(dOutgoing, nInward), -1, 1))
  ```

  `dOutgoing`必須在具名方向容差內等於`idealReflected`，且`incidenceAngle`與
  `reflectionAngle`必須在具名角度容差內相等。任一方向無法正規化、角度非finite、
  法線無效或等角不變量失敗時，候選Invalid並只可形成CandidateDiagnostic。
- 理想鏡射只是幾何近似，不估測或補償庫邊恢復係數、碰庫速度衰減、球體旋轉、桌布摩擦、庫邊材質差異或能量損失。
- 不支援target-bank或第二次庫邊。

通過`maxKickRailAngleDeg`的KickPot仍可能超出Phase 2較嚴格的`FixedForceEnvelope`；此時ShotPlan在Phase 1仍可成立，由Phase 2回`NoExecutablePlan`。Phase 1不得讀取或依賴`pneumaticPulseMs`、硬體力度校正或Phase 2 envelope。

## 12. Scoring Model

只有完整可行候選可以評分。全部項目轉成`[0,1]`成本，越低越好；任何raw、normalized或total非有限時候選Invalid。

### 12.1 Initial Experimental Weights

| 成本 | 權重 |
|---|---:|
| kick penalty | 0.30 |
| cutting angle | 0.30 |
| total distance | 0.20 |
| clearance risk | 0.10 |
| pocket entry angle | 0.05 |
| kick rail angle risk | 0.05 |

表中數值是`raw weights`的初始實驗預設，不是production最佳值。載入時必須：

1. 驗證每個raw weight皆finite且非負。
2. 以足以偵測overflow的方式計算raw總和；總和必須finite且大於0。
3. 計算`effectiveWeight_i = rawWeight_i / rawWeightSum`。
4. 驗證全部effective weights finite、非負，且其總和只需在具名浮點容差內等於1。

使用者輸入的raw總和不必等於1。任一raw非法、總和為0、非有限或正規化失敗皆回`InvalidConfiguration`，不得建立評分集合。

### 12.2 Normalization

| 成本 | Raw／單位 | 公式與range來源 |
|---|---|---|
| kick penalty | kick count／無因次 | Direct=0，Kick=1；大於1不可行 |
| cutting angle | degree | `clamp(angle/maxCutAngleDeg,0,1)` |
| distance | 全分段總長mm | `clamp((d-minDistanceMm)/(maxDistanceMm-minDistanceMm),0,1)` |
| clearance risk | 最小球心淨空mm | `1-clamp((clearance-blockedThreshold)/(preferredClearanceMm-blockedThreshold),0,1)` |
| pocket entry | 相對外向法線degree | `clamp(angle/maxEntryAngleForClass,0,1)` |
| kick rail angle | 相對庫邊法線degree | Direct=0；Kick=`clamp(angle/maxKickRailAngleDeg,0,1)` |

- `clamp(x,0,1)`只在輸入與range先驗證成功後使用，不能掩蓋NaN或無效分母。
- min／max來自具名ScoringConfig或TableGeometry推導值。
- `max <= min`、缺少range、非有限或單位錯置回`ConfigurationMissing／InvalidConfiguration`。
- Direct的不適用kick成本為0，不動態重算其他權重。
- normalization ranges完成前只能使用離線fixture，不宣稱物理可比或可供硬體執行。
- ShotPlan稽核資料必須同時保存raw weights、raw總和及effective normalized weights；總成本只使用effective weights計算。

### 12.3 Selection and Tie Break

總成本差在非負有限epsilon內視為近似平手，依序選：

1. kick次數較少。
2. 切球角較小。
3. 最小淨空較大。
4. 總路徑較短。
5. pocket ID較小。
6. rail ID較小。

候選生成或容器順序不得影響結果。

## 13. LegalContact

只有沒有任何可行Pot候選時才生成：

- `DirectLegalContact`
- `KickLegalContact`

它只保證母球先接觸最低號球及接觸前路徑幾何可行；不保證洗袋、碰撞後位置、二次碰撞、對手難度或局勢安全，名稱不得改成SafetyShot。

- `DirectLegalContact`使用`v = normalize(T - C)`與`Gcontact = T - 2r*v`；母球中心路徑明確為`C → Gcontact`。
- 對每個有效`EffectiveCueBallRailSegment`，`KickLegalContact`唯一使用下列幾何。令`Cmirror`為`C`相對該effective rail的鏡射點：

```text
vKickContact = normalize(T - Cmirror)
GkickContact = T - 2r * vKickContact
R = intersection(ray/segment Cmirror → GkickContact,
                 EffectiveCueBallRailSegment)
actual cue-ball center path = C → R → GkickContact
```

- `R`是CueBallReboundPoint，必須位於該有效effective rail及RailReflectionRegion，且不得位於映射後的袋口／端點排除區。
- `C → R`與`R → GkickContact`分別檢查碰撞及適用region。
- `||T-GkickContact|| = 2r`；第二段不得終止於`T`或BallSurfaceContactPoint。
- 平行、無交點、零長度、非有限、重疊或無法正規化皆fail-closed。
- 每個effective rail最多產生一個此種直接首次接觸候選。
- 仍不評估碰撞後結果、洗袋、二次碰撞或安全球價值。
- `Gcontact`及`GkickContact`皆須滿足§8.3的首次接觸、距離、重疊、finite與region不變量。

多個候選依序選：Direct優先、最小淨空較大、總路徑較短、rail ID較小。LegalContact不使用Pot評分權重。

ShotPlan必須標示它需要Phase 2 ExecutionPolicy顯式授權。

## 14. ShotPlan and NoPlan

`PlanningResult`只能是具名ShotPlan或NoPlan。

ShotPlan語意固定為`CommonAuditFields + plan-type-specific variant payload`。實際C++型別名稱可由To Tickets與實作決定，但欄位適用性不得改變。

`CommonAuditFields`供所有`DirectPot | KickPot | DirectLegalContact | KickLegalContact`使用，至少包含：

- plan identity與plan type。
- source event IDs、AttestedVisionSession identity及attestation／manifest reference。
- TableFrame及TableGeometry revisions。
- cue ball snapshot、target ball ID與snapshot，以及與TableGeometry revision一致的`ballRadiusMm`。
- finite二維擊球單位方向、GhostBallPoint、cue path segments及minimum clearance。
- relevant geometry configuration revisions、limitations及Diagnostic metadata。
- `FixedForceMode`。

`PotPlanAuditFields`只存在於DirectPot與KickPot，至少包含pocket ID、VirtualPocketTarget、target ball path、pocket entry angle、cutting angle、raw／normalized scoring costs、raw／effective weights、raw weight總和、total cost、tie-break fields及Pot CandidateDiagnostics摘要。

`KickPlanAuditFields`只存在於KickPot與KickLegalContact，至少包含rail ID、CueBallReboundPoint、kick angle及effective rail reference。

`LegalContactAuditFields`只存在於DirectLegalContact與KickLegalContact，至少包含legal first-contact guarantee、selected contact GhostBallPoint、direct／kick priority、clearance、total path length、rail ID（如適用）、`requiresExplicitExecutionAuthorization = true`及Pot search failure Diagnostic。

LegalContact不得攜帶pocket ID、pocket entry angle、作為Pot評分值的cutting angle、Pot raw／normalized costs、Pot weights、Pot total cost或假VirtualPocketTarget。任何不適用欄位不得填0、預設角度、空Point或假值；可用型別上不存在、`nullopt`、variant payload或具名NotApplicable Diagnostic，但NotApplicable不得被視為success value。

ShotPlan不得包含Robot TCP位置、Robot Pose、RX／RY／RZ、HRSDK、DO編號、脈衝毫秒或硬體物件。

`NoPlan`只可由已取得合法`StableTableState`後的ShotBrain規劃層產生。`NoPlanReason`只保留：

- `NoEligibleTarget`
- `NoLegalContact`
- `InvalidBrainConfiguration`
- `NumericalPlanningFailure`

沒有可行Pot不是最終NoPlanReason；`NoPotCandidate`只可作為中間Planning
Diagnostic標籤。此時Planning Diagnostic必須記錄
`feasiblePotCount = 0`、Pot CandidateDiagnostics及
`proceededToLegalContact = true`，然後繼續評估LegalContact；不得因沒有Pot而跳過。

Parser錯誤、receive錯誤、`NeedMoreEvents`、不穩定資料或timeout屬於`SingleFrameResult`、`StabilityResult`或`Phase1PipelineResult`，不得包裝成NoPlan。NoPlan不得攜帶fallback ShotPlan；拒絕摘要只能是Diagnostic metadata，不能被型別化為Candidate或Plan。

## 15. Fixed Force Semantics

Phase 1不最佳化力度，只表示`FixedForceMode`。這不代表所有候選有足夠能量，也不授權真實擊發。Phase 1只套用自身幾何可行門檻，包括`maxKickRailAngleDeg`；路徑類型、距離與角度是否位於已校正的硬體`FixedForceEnvelope`由Phase 2判斷。Phase 1的任何config、候選或測試不得依賴`pneumaticPulseMs`或硬體力度校正資料。

## 16. Testing Decisions

- P1-01至P1-09全部完全離線；Phase 1任何驗收或測試不得開production Socket。
- P1-03以fake byte／session source驗證control parser、data parser、manifest／attestation validation、session state、identity binding、gate、reconnect失效及具名Result／Diagnostic。
- Pipeline seam：`ReceiveEvent stream → Phase1PipelineResult`，測試fixture顯式提供InputContractConfig與StabilityConfig；覆蓋Parser拒絕、`NeedMoreEvents`、timeout、`PlanningCompleted`、缺少runtime attestation、revision／manifest ID不一致、connection reset、session ID改變及process ID重用／改變後清空累積並重新handshake，且證明相同process ID不能延續舊session、非planning的pipeline結果不成為NoPlan。
- Brain seam：`StableTableState + TableGeometryConfig + BrainConfig → PlanningResult`；ShotBrain測試不得提供CSV、ReceiveEvent、freshness或三幀累積責任。
- P1-05額外使用安全幾何單元測試覆蓋退化、平行、端點、等於門檻及非法設定；並覆蓋Physical rail向桌內平移`r`、桌內法線方向錯誤、非單位／非有限法線、排除區映射、零長度與平移後空區段、CueBallReboundPoint確實位於effective rail、球半徑／collision margin未重複加入，以及六段physical rail ID在推導後保持穩定。
- GhostBallPoint測試必須驗證`||T-G|| = 2r`（具名容差內）、`G/T/P`共線、`G`位於目標球的袋口反方向、零長度或非有限方向、兩球重疊、初始球心距離小於`2r`、Direct終點為`G`、Kick第二段終點為`G`，以及不得以`T`或`K`替代`G`。
- Pocket entry測試覆蓋0度、等於門檻、超限及180度反向，並證明DirectPot、KickPot與Scoring raw value使用同一公式且不使用`abs(dot)`。
- Direct測試獨立驗證`C → Gpot`與`T → P`，包含唯一合法PocketExitSegment、正向合法穿越、法線反向、沿出口切行、由corridor反向進桌、`E`不在segment相對內部、出口附近corridor／playable方向錯置、兩者間空隙、過度重疊、多重出口、錯誤PocketExitSegment／corridor、一般庫邊穿出、母球利用出口／corridor越界，以及virtual target不會被一般playable bounds誤殺；PocketExitSegment或相關epsilon缺失為`ConfigurationMissing`，錯誤ID、非法epsilon、零長度／零面積／非有限幾何為`InvalidConfiguration`。
- Kick測試覆蓋六個effective rail segments、映射後排除區、每段碰撞、Phase 1 `maxKickRailAngleDeg`硬拒絕及禁止第二次庫邊；並驗證`dOutgoing == idealReflected`與入射角等於反射角（各自在具名容差內）、非法法線／非finite角度fail-closed，以及候選結果不受恢復係數、速度衰減、旋轉、桌布摩擦、庫邊材質或力度參數影響。
- Scoring測試覆蓋每個公式、clamp、缺range、負值／NaN／Infinity raw weight、raw總和不等於1時正規化、effective總和在epsilon內為1、總和為0、總和overflow、raw/effective稽核資料及確定性平手。
- LegalContact測試證明只在無Pot時出現，且不宣稱SafetyShot。
- KickLegalContact測試對每個effective rail驗證`Cmirror`、`vKickContact`、`GkickContact`與唯一交點`R`，兩段碰撞／region、`||T-GkickContact|| = 2r`、每rail最多一候選，以及平行、無交點、零長度、非有限、重疊、排除區與以`T`／BallSurfaceContactPoint作終點時全部拒絕。
- 無Pot測試驗證`feasiblePotCount = 0`、Pot CandidateDiagnostics及`proceededToLegalContact = true`，且`NoPotCandidate`不會成為NoPlanReason。
- PocketModel參數測試分別覆蓋`virtualTargetOffsetMm`與`corridorHalfWidthMm`的正常正值、0、負值、NaN及Infinity。
- ShotPlan variant測試證明DirectPot具有Pot欄位、KickPot具有Pot與Kick欄位、DirectLegalContact沒有Pot-only欄位、KickLegalContact只有LegalContact與Kick欄位；不適用欄位不得填0／預設值，LegalContact仍可攜帶Pot搜尋失敗Diagnostic。
- 被拒候選只能出現在`CandidateDiagnostic`，不得進入可行候選或評分集合。
- Pipeline／NoPlan測試必須分別證明Parser錯誤、`NeedMoreEvents`、`Unstable`及timeout不產生NoPlan；只有合法StableTableState進入ShotBrain後無規劃候選才可產生NoPlan，且NoPlan不帶fallback ShotPlan。
- 任何失敗結果或NoPlan fixture不得產生Point、角度、Candidate、ShotPlan或Pose fallback；Diagnostic metadata不得被下游當作成功value。

## 17. Out of Scope

- 所有真實Socket／production application orchestration、ProductionVisionTransportAdapter具體實作或整合、RobotController、HRSDK、DO、main、calibrate、test_cueball。P1-03只提供transport-neutral interface及fake離線驗收。
- CameraCompensator或任何相機補償。
- TableFrame→Base0。
- target-bank、組合球、借球、跳球、多庫。
- 母球後續、洗袋、完整物理或自動力度。

## 18. Further Notes

Phase 1仍需校正：stable tolerance、TableGeometry、`virtualTargetOffsetMm`、`corridorHalfWidthMm`、PocketExitSegments、`pocketBoundaryProbeEpsilonMm`、`exitCrossingEpsilon`、角／中袋入射角、collision margin、scoring ranges及幾何`maxKickRailAngleDeg`。Phase 2另行校正fixed-force envelope及氣動參數。任一層缺少該層必要值時必須回ConfigurationMissing，不得填猜測值。
