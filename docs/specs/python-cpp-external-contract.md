# Python–C++ External Contract

## 文件資訊

- 狀態：Approved
- Final Workflow Verification：PASS（2026-08-01）
- 規格版本：1.0
- 上層權威：`billiards-system-refactor-master-spec.md`
- 唯一權威範圍：既有32值wire、Robot Base0 planar XY、本地connection／shot-cycle freshness、球與袋口觀測、Parser與單幀結果

## 1. Problem Statement

現有 newline CSV 可以傳送球與袋口座標，但沒有 frame ID、timestamp 或 frame metadata；袋口又同時被當成靜態桌面幾何和每幀動態物件。舊 Parser 以範圍門檻判斷 missing，會讓單邊 sentinel、非有限值或重送資料進入規劃。

## 2. Solution

保留既有32值wire相容性，將精確sentinel限制在Parser邊界；由C++接收端在CameraPose settle後開啟的shot-cycle capture window內，為每個完整newline frame建立本地ReceiveEvent。動態球與六袋wire XY都進入同一cycle的三幀一致性流程；當次穩定wire pocket center是規劃的唯一袋口中心來源，TableGeometryConfig只提供拓撲、法線、capture／exit與安全幾何設定。

## 3. User Stories

1. 作為 Python 開發者，我要維持既有 32 值封包，避免不必要的同步部署。
2. 作為 C++ 開發者，我要在單一邊界消除 sentinel，使下游只看到 optional 或具名錯誤。
3. 作為演算法測試者，我要能把同一cycle內穩定的動態球與六袋觀測組成一致的規劃輸入。
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

所有座標單位為毫米，並且已由Python轉換到同一個active Robot Base0平面XY。32值不包含Robot Z或姿態。

### 4.3 Sentinel

- 只有精確 `x == -9999.0 && y == -9999.0` 表示該欄位缺失。
- 只有 X 或只有 Y 等於 `-9999.0` 時，整個 frame 為 `InvalidFormat`。
- 不得使用 `< -9000`、`<= -9000` 或其他範圍門檻。
- Parser 成功後不得保留 sentinel；缺失值轉成 `nullopt`。

### 4.4 數值驗證

- 每個 present token 必須完整解析為有限 double。
- NaN、Infinity、溢位、空白外的尾隨內容或 token 數錯誤皆拒絕。
- Parser不執行相機補償、桌面bounds、三幀穩定、選球、旋轉、平移或任何robot frame轉換；成功Point的XY數值必須保持wire原值。

## 5. Robot Base0 Planar XY Contract

- Python負責把pixel經鏡頭校正、homography、固定offset與殘差補償轉成Robot Base0平面XY毫米。TableFrame若存在，只是Python內部校正過程，不得越過Python–C++ wire contract。
- active Base0 planar calibration必須以不可含糊的實體描述唯一定義：
  - 實體原點。
  - `+X`實體方向、`+Y`實體方向及離開桌面平面的`+Z`實體方向。
  - 三軸皆為有限單位向量且互相正交。
  - `+X cross +Y = +Z`，形成右手座標系。
  - wire Pocket ID 1至6各自對應的實體袋口。
  - Physical Rail ID 1至6各自對應的實體庫邊與順序。
  - calibration revision。
- 32值wire不攜帶frame metadata或revision。V1不以新的Python control handshake、sender frame ID、sender timestamp或runtime attestation作production gate；部署校正版本仍由既有版本化設定與受控驗收管理。
- 缺少任一必要軸、原點、ID mapping或revision時回`ConfigurationMissing`。
- 軸非有限、非單位、非正交、左右手錯誤或ID mapping矛盾時回
  `InvalidConfiguration`；可觀測的Python部署設定與C++ configured revision不一致時回
  `ConfigurationMismatch`。V1沒有runtime attestation，因此不得宣稱此比較能證明或偵測
  正在執行的Python process實際載入revision。
- C++收到的成功Point已是Base0平面XY。Phase 1及Phase 2不得執行TableFrame→Base0、pixel→XY、Homography、CameraCompensator、camera offset correction或第二次平面座標映射。

### 5.1 V1 Deployment Calibration

- Base0原點、軸、Pocket／Rail ID mapping及geometry revision必須以既有版本化設定或受控部署產物管理；缺失回`ConfigurationMissing`，矛盾或非法回`InvalidConfiguration／ConfigurationMismatch`。
- static deployment資料只能證明設定檔內容，不能證明Python runtime實際載入版本；V1不得宣稱可偵測這類runtime mismatch。
- 因V1刻意保留既有32值wire，runtime attestation不是啟用規劃的必要條件，也不得創建主要ticket或要求Python同步改協定。

### 5.2 V1 Offline Core and Production Transport

P1-03核心能力包含既有32值contract、最大frame長度、newline framing、嚴格data parser、
exact sentinel、finite validation、clean disconnect／transport error區分、本地ReceiveEvent、
connection／shot-cycle identity、cycle reset、stale-buffer protection及具名Result／Diagnostic。
這些能力使用fake byte／connection／cycle source完全離線驗證，不開TCP Socket。

既有`SocketClient + BilliardApp`是唯一production transport／capture-window整合owner：

- `SocketClient`開啟、維持及關閉Operating-system Socket，接收newline-delimited 32值frame並維持framing與connection lifecycle。
- `BilliardApp`在CameraPose成功、motion stopped與settle完成後，要求丟棄舊buffer、重置穩定累積並開啟新的shot-cycle capture window。
- 只有window開啟後收到、屬於目前connection與cycle的完整frame可形成ReceiveEvent。
- 兩者只把bytes與本地metadata交給P1-03 parser／session邊界；不得包含ShotBrain、相機補償、HRSDK或DO。

不得新增第二個Socket client、`ProductionVisionTransportAdapter.cpp`或平行application owner。
production integration未完成時回`IntegrationRequired`；此工作屬P1-03原地整合子工作，
不新增主要ticket且不得移入P2-03。

### 5.3 Future Hardening / Optional Future Protocol

未來可另案評估`RuntimeCalibrationAttestation`、`AttestedVisionSession`、versioned control
message、sender frame ID或sender timestamp。它們必須使用可與32值data frame無歧義區分
的版本化協定，且需Python／C++同步部署；本節不構成V1 requirement、production blocker、
acceptance gate或十二個能力切片中的工作。static manifest的Python declared欄位亦不得被誤稱為runtime evidence。

## 6. Dynamic Balls and Wire Pockets

### 6.1 動態球

- 1 至 9 號球與母球是每幀動態觀測。
- 它們的 presence 與位置進入三幀穩定流程。
- 任一不合法動態球 frame 使穩定累積完全重置。

### 6.2 袋口觀測與規劃來源

- Index 20–31的六袋XY必須由Parser依固定Pocket ID輸出，且參與目前shot cycle的規劃資料生命週期。
- 正常production規劃使用三個有效event得到的穩定wire pocket center；這是當次規劃唯一的pocket center來源。
- 每袋都必須present、finite、ID固定，並以三幀median與`pocketStabilityToleranceMm`（或後續明確核准的唯一方法）驗證一致；缺失、跳動或mismatch拒絕並重置本cycle累積。
- 不得把單一不穩定frame的pocket center與Stable動態球混用。
- `TableGeometryConfig/PocketModel`擁有Pocket ID／type、Physical Rail topology、inward／outward normal、effective rail、capture corridor、exit、collision margin、virtual-target offset／entry-angle參數與geometry revision；它可保存expected pocket positions作容差驗證，但不得用static center取代目前stable wire center。
- VirtualPocketTarget與PocketCaptureCorridor須以本cycle stable wire entrance center結合版本化PocketModel參數建立。
- `pocketStabilityToleranceMm`或必要geometry設定缺失回`ConfigurationMissing`；非法值或拓撲／方向矛盾回`InvalidConfiguration`；wire center相對expected position超限回`TableGeometryMismatch`並拒絕本cycle。

## 7. ReceiveEvent and Freshness

wire 維持 32 值，不新增 sender sequence 或 timestamp。每次完整 newline receive completion 由 C++ 接收邊界包裝：

```text
ReceiveEvent {
  strictly increasing local event id,
  monotonic receive timestamp,
  local connection identity,
  local shot-cycle identity,
  raw payload
}
```

規則：

- 三幀必須來自三個不同且嚴格遞增的本地 event ID。
- 三幀必須屬於同一個connection identity與shot-cycle identity，且全部在CameraPose settle、stale-buffer flush與累積reset完成後才接收；跨connection／cycle event不得累積。
- 相鄰有效事件間隔不得大於 `maxInterFrameIntervalMs`。
- timeout、disconnect、reconnect、Parser失敗、球或袋口不穩定會清空累積並關閉／重開相應cycle gate；舊buffer內容不得被重新包裝為新cycle event。
- 相同座標 payload 可以是合法靜止畫面，不得因內容相同而拒絕。
- 本契約無法偵測 sender 重送舊payload；「fresh」只能表示新的本地receive事件，不能表示新的相機曝光。
- `maxInterFrameIntervalMs`缺失時回 `ConfigurationMissing`；非正或超限時回 `InvalidConfiguration`。

## 8. Parser and Single-Frame Results

概念輸出分成：

- `ParsedVisionFrame`：dynamic ball optionals與固定ID的六袋optionals；僅代表wire合法。
- `ValidatedVisionFrame`：dynamic ball與六袋座標已通過finite、`TableObservationBounds`及必要資料驗證。
- `SingleFrameResult`：`Valid(success value)`或`Rejected(Diagnostic metadata)`。

Result不變量：

- `Valid`必須帶有ValidatedVisionFrame success value。
- `Rejected`不得帶有success value。
- 成功與失敗都可以帶有Diagnostic metadata。
- Diagnostic不得被轉型或視為合法Point、ValidatedVisionFrame、
  StableTableState、Candidate或Plan，也不得包含partial／fallback成功值。

單幀不設必要資料：1～9號編號球、母球與六袋皆可全部或部分缺失；只要其他單幀驗證合法（欄位數、數值格式、sentinel pair、observation bounds），仍產生`ValidatedVisionFrame SUCCESS`，缺失值一律以`nullopt`表示。母球與袋口是否收斂為必要值都不在單幀parser層判定。

母球與袋口是否收斂為必要值下放給`ThreeEventStability`：母球與同一袋口都需要目前3事件滑動視窗內全部出現、且互相在各自容差內（母球用`STABLE_FRAME_TOLERANCE_MM`、袋口用`POCKET_STABILITY_TOLERANCE_MM`）才能定案——兩者都是`StableTableState`的必要非optional欄位；缺席或不一致時整批`Stable`判定失敗（母球`BallMoved`、袋口`PocketMoved`），但不reset視窗，改為`NeedMoreEvents`繼續滑動等待收斂——與編號球的「單物件缺席只影響該物件、不拖垮整批」不同，母球或袋口未收斂時整批仍不能送出`Stable`。

`TableObservationBounds`只表示active Base0 planar calibration可接受的輸入觀測範圍，不是Phase 1規劃用的`PlayableBallCenterRegion`、`PocketCaptureCorridor`或`RailReflectionRegion`，不得用它預先判定進袋路徑可行性。

## 9. Configuration

必要設定：

- active Base0 planar calibration definition及revision
- Base0平面實體原點、X／Y軸及Pocket／Physical Rail ID mappings
- `TableObservationBounds`
- maximum wire frame bytes
- `maxInterFrameIntervalMs`
- `pocketStabilityToleranceMm`
- 本地connection identity與shot-cycle identity的產生／重置規則

缺少必要值回`ConfigurationMissing`；非有限、負值、範圍反轉或不一致revision回`InvalidConfiguration／ConfigurationMismatch`。

## 10. Testing Decisions

- Parser seam：`raw wire + receive metadata → SingleFrameResult`。
- Cycle/session seam：`fake data bytes + fake connection/cycle events + config → session/parser Results`。
- 覆蓋恰好32值、31／33值、空token、尾隨字元、單邊／成對sentinel、NaN、Infinity、超長frame。
- 覆蓋六袋任一缺失／非finite／ID錯置會拒絕frame並重置、三幀袋口median、袋口跳動與expected position一致性超限。
- 本契約測試ReceiveEvent metadata的建立與單一event驗證；event stream、
  NeedMoreEvents、timeout、三幀累積及connection reset屬於Phase1Pipeline seam。
- 驗證Base0平面X／Y軸單位、正交、方向、Pocket／Rail ID mapping與revision mismatch。
- 覆蓋CameraPose settle前frame丟棄、flush後才開capture window、cycle reset、嚴格遞增本地event ID、timeout、disconnect／reconnect與stale-buffer不能跨cycle累積。
- 證明同payload可形成不同本地event，但V1無法證明它來自不同Python曝光。
- 覆蓋stable wire pocket center是唯一planning center，TableGeometry expected center只能驗證不能取代。
- P1-03核心測試全部使用fake byte／connection／cycle source離線執行，不開production Socket。既有SocketClient與BilliardApp的production integration只在獨立application／infrastructure整合中驗證其轉交契約，不屬於P1-03離線驗收，且不執行ShotBrain或硬體流程。

## 11. Out of Scope

- Python相機演算法的實作。
- sender-side frame ID／timestamp擴充。
- RuntimeCalibrationAttestation、AttestedVisionSession與versioned control-message protocol；只屬Future Hardening。
- C++中的pixel→XY、TableFrame→Base0或任何第二次平面座標轉換。
- 撞球候選、評分、Robot Pose或硬體控制。

## 12. Further Notes

若未來擴充wire加入sender sequence與timestamp，必須建立版本化協定；不得靜默改變32值欄位順序。
