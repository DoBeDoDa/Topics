# Python–C++ External Contract

## 文件資訊

- 狀態：Approved
- 規格版本：1.0
- 上層權威：`billiards-system-refactor-master-spec.md`
- 唯一權威範圍：32值wire、versioned control message、Attested Session Boundary、TableFrame、靜態袋口觀測、receive freshness、Parser與單幀結果

## 1. Problem Statement

現有 newline CSV 可以傳送球與袋口座標，但沒有 frame ID、timestamp 或 frame metadata；袋口又同時被當成靜態桌面幾何和每幀動態物件。舊 Parser 以範圍門檻判斷 missing，會讓單邊 sentinel、非有限值或重送資料進入規劃。

## 2. Solution

保留 32 值 wire 相容性，將精確 sentinel 限制在 Parser 邊界；由 C++ 接收端為每個完整 newline frame 建立本地 ReceiveEvent。動態球進入三幀穩定流程，wire 袋口只形成ObservedPocketSet供診斷、啟動一致性檢查或獨立校正流程使用，不參與動態球presence，也不得直接初始化正式規劃用TableGeometryConfig。

## 3. User Stories

1. 作為 Python 開發者，我要維持既有 32 值封包，避免不必要的同步部署。
2. 作為 C++ 開發者，我要在單一邊界消除 sentinel，使下游只看到 optional 或具名錯誤。
3. 作為演算法測試者，我要能區分動態球資料與靜態袋口觀測。
4. 作為安全審查者，我要知道三幀只保證三次本地 receive event，而不是三次不同相機曝光。

## 4. Wire Contract

### 4.1 Framing

- 編碼：UTF-8 相容的 ASCII 數字、逗號與換行。
- 一個 frame 以單一 `\n` 結束。
- 一個 frame 必須恰有 32 個非空數值 token。
- 不得跳過空 token、接受多餘 token 或接受數值後尾隨字元。
- 接收端必須設定具名的最大 frame byte length；缺少或超過上限即拒絕。

### 4.2 欄位順序

| Index | 語意 |
|---:|---|
| 0–17 | 1 至 9 號球的 X、Y，依球號遞增 |
| 18–19 | 母球 X、Y |
| 20–31 | 1 至 6 號袋口觀測 X、Y，依 pocket ID 遞增 |

所有座標單位為毫米，並位於同一個 active TableFrame。

### 4.3 Sentinel

- 只有精確 `x == -9999.0 && y == -9999.0` 表示該欄位缺失。
- 只有 X 或只有 Y 等於 `-9999.0` 時，整個 frame 為 `InvalidFormat`。
- 不得使用 `< -9000`、`<= -9000` 或其他範圍門檻。
- Parser 成功後不得保留 sentinel；缺失值轉成 `nullopt`。

### 4.4 數值驗證

- 每個 present token 必須完整解析為有限 double。
- NaN、Infinity、溢位、空白外的尾隨內容或 token 數錯誤皆拒絕。
- Parser 不執行相機補償、桌面 bounds、三幀穩定、選球或 robot frame 轉換。

## 5. TableFrame Contract

- Python 負責把 pixel 經鏡頭校正、homography、固定 offset 與殘差補償轉成 TableFrame 毫米。
- active TableFrame calibration必須以不可含糊的實體描述唯一定義：
  - 實體原點。
  - `+X`實體方向、`+Y`實體方向及離開桌面平面的`+Z`實體方向。
  - 三軸皆為有限單位向量且互相正交。
  - `+X cross +Y = +Z`，形成右手座標系。
  - wire Pocket ID 1至6各自對應的實體袋口。
  - Physical Rail ID 1至6各自對應的實體庫邊與順序。
  - calibration revision。
- 32 值 wire 不攜帶 frame metadata或revision。Deployment Calibration Manifest只提供static deployment configuration；正在送frame的Python process另須透過§5.2 RuntimeCalibrationAttestation提供runtime evidence。
- 缺少任一必要軸、原點、ID mapping或revision時回`ConfigurationMissing`。
- 軸非有限、非單位、非正交、左右手錯誤或ID mapping矛盾時回
  `InvalidConfiguration`；Python與C++ revision不一致時回`ConfigurationMismatch`。
- Phase 1 不得把 TableFrame 誤稱 Base0，也不得執行 TableFrame→Base0。

### 5.1 Deployment Calibration Manifest

版本化manifest至少包含：

- manifest schema version。
- TableFrame calibration revision。
- TableFrame原點及三軸定義revision。
- Pocket ID mapping revision。
- Physical Rail ID mapping revision。
- TableGeometry revision（若本部署使用）。
- 產生時間或具名部署識別。
- Python expected calibration revision；此欄位只是static deployment expectation，不是runtime evidence。
- C++ active／expected calibration revision。

啟動與部署規則：

1. Python與C++必須各自載入，或由部署工具綁定至同一份manifest。
2. static deployment comparison驗證manifest expected revision、C++ configured／active revision及Python expected revision一致。
3. static三者一致只證明部署設定一致，不能證明Python runtime實際載入revision，亦不能單獨啟用production規劃。
4. 缺少任一static revision來源或manifest回`ConfigurationMissing`，任一static不一致回`ConfigurationMismatch`。
5. manifest無法解析、欄位矛盾回`InvalidConfiguration`；schema version不支援回`UnsupportedConfigurationVersion`。
6. production不得只依人工記憶或未版本化設定接受校正資料。
7. 未來若版本化32值data wire，必須另行版本化，不得靜默改變欄位順序。

### 5.2 RuntimeCalibrationAttestation

本版本選定「session建立時的獨立版本化控制握手」：正在產生32值frames的Python
process必須在實際載入calibration後、送出該session第一個data frame前，透過同一
Socket connection的control handshake送出`RuntimeCalibrationAttestation`。控制訊息
不屬於32值data frame，必須有可與data framing無歧義區分的版本化message type及長度。

Attestation至少包含：

- 不可重用的session ID；它是AttestedVisionSession的主要identity。
- Python process ID；它只作Diagnostic metadata，可能被作業系統重用，不得單獨作安全證據。
- Python process實際載入的TableFrame calibration revision。
- deployment／manifest ID。
- manifest schema version。
- startup timestamp或嚴格單調session ID。

責任與驗證語意：

1. Python process在校正載入成功後產生runtime attestation；manifest中的Python expected欄位不得代替此證據。
2. C++ receive/session邊界解析、驗證attestation並建立只對該connection與不可重用session ID有效的`AttestedVisionSession`；process ID不得作主要identity。
3. C++只有在Python runtime attested revision、C++ active revision、manifest expected revision及deployment／manifest ID全部一致後，才可把該session的32值frames包裝成可進入Phase1Pipeline的ReceiveEvent。
4. 缺少runtime attestation回`ConfigurationMissing`並拒絕production規劃及真實執行；任一revision或manifest ID不一致回`ConfigurationMismatch`，拒絕該session全部frames並清除三幀累積。
5. connection reset／reconnect或session ID改變時，舊attestation立即失效；每個新connection必須產生新的不可重用session ID並重新handshake，且三幀累積清空。即使process ID相同亦不得延續attestation或三幀累積；process ID改變則另記Diagnostic並同樣要求新session。
6. attestation格式無法解析、欄位矛盾回`InvalidConfiguration`；message schema不支援回`UnsupportedConfigurationVersion`。
7. 單一32值payload不能證明runtime revision一致，C++不得由payload推導Python實際revision。
8. P1-03契約及fake驗收尚未完成時，Phase 1能力本身未完成；若契約已完成但缺少可承載handshake的ProductionVisionTransportAdapter integration，production狀態為`IntegrationRequired`。`CalibrationRequired`不得表示transport缺失。

### 5.3 Offline Core and Production Transport Adapter

P1-03核心能力包含contract、versioned control-message schema與parser、32值data parser、
static manifest validation、runtime attestation validation、AttestedVisionSession狀態模型、
connection／session lifecycle抽象事件、identity綁定、session gate、失效／重置、
ReceiveEvent建立條件、具名Result／Diagnostic及transport-neutral adapter interface。
這些能力必須以fake byte／connection／session source及完整錯誤注入完全離線驗證，
不得開啟TCP Socket、執行production連線、依賴production SocketClient具體實作或修改
production application orchestration。

獨立application／infrastructure邊界`ProductionVisionTransportAdapter`只負責：

- 開啟、維持及關閉Operating-system Socket。
- 接收versioned control message與32值data frame。
- 維持transport framing與connection lifecycle。
- 將原始bytes及connection metadata交給P1-03定義的attestation／parser邊界。

它不得包含解析後的撞球規劃、三幀穩定、ShotBrain、相機補償、RobotController、
HRSDK、DO、fallback或其他硬體／策略邏輯，也不得成為Phase1Pipeline／ShotBrain核心依賴。
P1-03只定義它必須遵守的interface與contract；具體adapter整合不屬於P1-03離線核心驗收。

若repository已有adapter，後續application integration只做P1-03 interface相容性驗證；
若沒有，它是production operation的外部整合前置依賴。未整合／未驗收時Phase 1仍可
離線完成，但production狀態為`IntegrationRequired`。此責任不新增主要ticket，也不得
移入P2-03；`CalibrationRequired`只保留給校正／envelope缺失。

## 6. Dynamic Balls and Static Pockets

### 6.1 動態球

- 1 至 9 號球與母球是每幀動態觀測。
- 它們的 presence 與位置進入三幀穩定流程。
- 任一不合法動態球 frame 使穩定累積完全重置。

### 6.2 袋口觀測

- 六袋欄位為 wire 相容資料，不是規劃權威。
- Parser 將其輸出為 `ObservedPocketSet`。
- 規劃使用的袋口中心、類型、外向法線、virtual target、`corridorHalfWidthMm`與入射角參數由 `TableGeometryConfig/PocketModel`唯一管理；wire袋口Point不得暗中取代這些靜態幾何設定。
- ObservedPocketSet只可用於診斷顯示、啟動一致性檢查、校正工具輸入或未核准calibration candidate；不得直接建立、覆寫或更新正式規劃用TableGeometryConfig。
- 若用於校正，唯一允許的生命週期為`ObservedPocketSet → calibration candidate → validation → human or authorized approval → versioned TableGeometryConfig`。
- TableGeometryConfig只能來自已核准校正流程、版本化設定或具有revision的部署／校正產物；未核准觀測不得成為規劃權威。
- wire 袋口不參與動態球 presence、median 或穩定重置。
- 啟動／診斷一致性檢查啟用時，完整的 ObservedPocketSet 可與 TableGeometryConfig 比較。
- 偏差超過具名容差時回 `TableGeometryMismatch`；單幀袋口缺失只代表無法完成該次一致性檢查。
- TableGeometryConfig 本身缺失或六袋不完整時禁止規劃。

## 7. ReceiveEvent and Freshness

wire 維持 32 值，不新增 sender sequence 或 timestamp。每次完整 newline receive completion 由 C++ 接收邊界包裝：

```text
ReceiveEvent {
  strictly increasing local event id,
  monotonic receive timestamp,
  attested non-reusable session ID,
  Python process ID as Diagnostic metadata,
  attestation/manifest reference,
  raw payload
}
```

規則：

- 三幀必須來自三個不同且嚴格遞增的本地 event ID。
- 三幀必須屬於同一個有效AttestedVisionSession；跨session event不得累積。
- 相鄰有效事件間隔不得大於 `maxInterFrameIntervalMs`。
- timeout、連線重建、session ID改變、attestation失效、Parser失敗或動態球不穩定會清空累積；process ID相同不得豁免重置。
- 相同座標 payload 可以是合法靜止畫面，不得因內容相同而拒絕。
- 本契約無法偵測 sender 重送舊payload；「fresh」只能表示新的本地receive事件，不能表示新的相機曝光。
- `maxInterFrameIntervalMs`缺失時回 `ConfigurationMissing`；非正或超限時回 `InvalidConfiguration`。

## 8. Parser and Single-Frame Results

概念輸出分成：

- `ParsedVisionFrame`：dynamic ball optionals 與 ObservedPocketSet；僅代表wire合法。
- `ValidatedDynamicFrame`：dynamic ball座標已通過finite、`TableObservationBounds`及必要資料驗證。
- `SingleFrameResult`：`Valid(success value)`或`Rejected(Diagnostic metadata)`。

Result不變量：

- `Valid`必須帶有ValidatedDynamicFrame success value。
- `Rejected`不得帶有success value。
- 成功與失敗都可以帶有Diagnostic metadata。
- Diagnostic不得被轉型或視為合法Point、ValidatedDynamicFrame、
  StableTableState、Candidate或Plan，也不得包含partial／fallback成功值。

必要資料：母球與至少一顆編號球。六袋規劃必要性由 TableGeometryConfig 滿足，不由每幀wire presence滿足。

`TableObservationBounds`只表示active TableFrame校正可接受的輸入觀測範圍，不是Phase 1規劃用的`PlayableBallCenterRegion`、`PocketCaptureCorridor`或`RailReflectionRegion`，不得用它預先判定進袋路徑可行性。

## 9. Configuration

必要設定：

- Deployment Calibration Manifest及受支援的schema version
- Python expected、C++ active及manifest expected static calibration revisions
- 每個runtime connection的有效RuntimeCalibrationAttestation與AttestedVisionSession
- active TableFrame definition及revision
- TableFrame實體原點、三軸及Pocket／Physical Rail ID mappings
- `TableObservationBounds`
- maximum wire frame bytes
- `maxInterFrameIntervalMs`
- 可選 pocket consistency tolerance與enable flag

缺少必要值回`ConfigurationMissing`；非有限、負值、範圍反轉或不一致revision回`InvalidConfiguration／ConfigurationMismatch`。

## 10. Testing Decisions

- Parser seam：`raw wire + receive metadata → SingleFrameResult`。
- Attested Session seam：`fake control/data bytes + fake connection/session events + manifest/config → attestation/session/parser Results`。
- 覆蓋恰好32值、31／33值、空token、尾隨字元、單邊／成對sentinel、NaN、Infinity、超長frame。
- 覆蓋袋口缺失不重置動態球、TableGeometry缺失禁止規劃、袋口一致性超限。
- 本契約測試ReceiveEvent metadata的建立與單一event驗證；event stream、
  NeedMoreEvents、timeout、三幀累積及connection reset屬於Phase1Pipeline seam。
- 驗證TableFrame軸單位、正交、右手性、Pocket／Rail ID mapping與revision mismatch。
- 覆蓋三方revision一致、Python revision缺失、C++ revision缺失、manifest缺失、任一revision不一致及manifest schema不支援。
- 覆蓋runtime attestation成功、缺失、revision／manifest ID不一致、control schema不支援、connection reset、session ID改變、process ID重用／改變及重新handshake；證明session ID是主要identity且相同process ID不能延續舊session。
- 證明manifest Python expected欄位與單一32值payload都不能作runtime evidence，也不得繞過session attestation gate。
- 覆蓋ObservedPocketSet只形成未核准候選；未經validation與授權核准不得建立或更新正式TableGeometryConfig。
- P1-03核心測試全部使用fake byte／connection／session source離線執行，不開production Socket。ProductionVisionTransportAdapter只在獨立application／infrastructure整合中驗證其轉交契約，不屬於P1-03離線驗收，且不執行Phase1Pipeline、ShotBrain或硬體流程。

## 11. Out of Scope

- Python相機演算法的實作。
- sender-side frame ID／timestamp擴充。
- TableFrame→Base0轉換。
- 撞球候選、評分、Robot Pose或硬體控制。

## 12. Further Notes

若未來擴充wire加入sender sequence與timestamp，必須建立版本化協定；不得靜默改變32值欄位順序。
