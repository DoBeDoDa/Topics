# BilliardApp 主流程實作計畫

狀態：暫定方案，尚未實作
專案：`DoBeDoDa/Topics`
核心模組：`src/BilliardApp.h/.cpp`

本文件記錄目前已由使用者確認的 shot-cycle 主流程、暫定競賽時間參數、安全失敗處理，以及實作時必須同步的跨模組契約。未收到明確的「開始實作」授權前，不得依本文件修改程式碼。

## 1. 目標

`BilliardApp`只負責協調以下既有模組，不重新實作撞球幾何、姿態數學或HRSDK底層功能：

```text
Start輸入
→ Robot準備與CameraPose
→ Socket/Vision freshness
→ ThreeEventStability
→ Algorithm ranked ShotPlan
→ MotionPlanner姿態搜尋
→ RobotController執行
→ 擊球後安全上升
→ 回準備姿態
→ WaitingForStart
```

## 2. 已確認與暫定參數

### 2.1 已確認

- 一個Start最多只允許一次實際擊球。
- 暫時以鍵盤`H`／`h`作為Start。
- 按下`H`立即生效，不需要再按Enter。
- 必須收到`H`放開事件後，才能接受下一次按下；長按不得連續啟動多輪。
- `P`可作為「只回準備姿態、不開始擊球」的人工功能，但不是每輪開始前的必要步驟。
- 準備姿態暫定為Joint pose：

```text
(0, 0, 90, 0, 0, -90)
```

- Robot不在準備姿態時按`H`，若狀態已知且安全，可先回準備姿態，再自動繼續同一輪。
- CameraPose到擊球點正上方不再經過`TRANSIT_JOINT`。
- 擊球後必須重新讀取actual pose。
- 第一個擊球後Robot motion必須保持actual X/Y/A/B/C，只沿已核准Base0 `+Z`以LIN上升。
- 擊球後的安全高度使用打擊前`safeApproachPose`的Z高度。
- 安全上升並確認停止後，以PTP回準備姿態，不直接回CameraPose。
- Python在CameraPose階段斷線時，可於同一輪內自動等待重連。
- Python重連不得重置競賽計時。
- 實體Start按鈕尚未完成；DI編號與active-high／active-low定義尚未確認，本輪不得猜測實裝。

### 2.2 暫定競賽參數

```text
shotDeadlineMs = 15000
minimumExecutionReserveMs = 5000
planningRetryCutoffMs = 10000
```

- `15000 ms`來自「超過15秒未出桿視為犯規」的比賽規則。
- `5000 ms`是人工可調／研究初值，不是固定機械規格；之後需用實機最慢執行時間校正。
- production deadline必須使用`std::chrono::steady_clock`等單調時鐘，不得使用系統日期時間、累加Sleep時間或可被校時影響的clock；測試必須能注入fake monotonic clock，不得真的等待10／15秒。
- 計時從接受`H`／Start的瞬間開始，包含必要的準備姿態移動、CameraPose、拍照、規劃、Robot移動與氣動。
- Push以DO1 Extend開始作為出桿時點。
- Pull以DO2 Retract開始作為出桿時點；Pull的DO1 pre-extend不是實際出桿。
- 找到安全可執行方案後立即執行，不等待10秒用完。
- 前10秒內允許自動重拍、重置三次累積並重新規劃。
- 剩餘時間少於5秒時不得再開始新的重拍／重算。
- `minimumExecutionReserveMs`是開始整段擊球執行的硬門檻：在任何mode-dependent pneumatic action或前往`safeApproachPose`的第一個motion之前，剩餘時間必須`>= 5000 ms`。Pull必須在DO1 pre-extend之前檢查；不足時不得改變氣動狀態或開始擊球動作，應在狀態已知安全時回準備姿態並結束本輪。
- Robot動作已經開始後，不得只因競賽時間超過15秒就粗暴abort；Robot安全優先。

## 3. 核准主流程

```text
WaitingForStart
→ H按下
→ 建立本輪唯一shot-cycle identity並開始15秒計時
→ 確認Robot connected、已停止、DO1/DO2 OFF、非UnknownUnsafe
→ 若不在準備姿態：PTP到準備姿態並確認停止
→ PTP到CameraPose並確認停止
→ Camera settle
→ 依同一shot deadline建立／確認Python連線
→ drain Winsock kernel receive queue並reset本地frame accumulator
→ reset ThreeEventStability與本輪PlanningResult
→ 開啟本輪capture window
→ 接收三筆本輪新資料
→ Algorithm產生ranked ShotPlan
→ 依母球XY與擊球方向決定唯一Push/Pull
→ MotionPlanner使用真實HRSDK checks搜尋A/B姿態
→ 同一ShotPlan內只在明確NotReachable時嘗試下一組A/B
→ 依第8.1節核准原因決定是否嘗試下一ranked ShotPlan
→ 確認剩餘時間>=minimumExecutionReserveMs
→ Pull only：確認DO狀態已知且全OFF，DO1 Extend並readback，等待direction-change delay
→ PTP直接到safeApproachPose
→ 讀取actual approach pose
→ motion_check_lin(actual approach, strikeReadyPose)
→ LIN到strikeReadyPose
→ confirmStopped
→ Push：DO1 Extend後DO2 Retract；Pull：DO2 Retract
→ 確認氣動完成且DO1/DO2 OFF
→ 讀取post-strike actual pose
→ 建立safe-lift target：保留actual X/Y/A/B/C，Z=safeApproachPose.z
→ motion_check_lin(actual, safe-lift target)
→ LIN安全上升
→ confirmStopped
→ PTP到準備姿態
→ confirmStopped
→ CycleCompleted
→ WaitingForStart
```

### 3.1 Shot-cycle identity唯一權威

- 接受H時只配置一次非零`activeShotCycleIdentity`，execution runtime、ReceiveEventFactory、PlanningSourceAudit與ExecutionPlan全部使用同一值。
- 不得依賴`OfflineExecutionRuntime::nextCycleIdentity`與`BilliardApp::nextShotCycleIdentity`兩個計數器「剛好同步」；實作時必須合併成一個authoritative allocator，或明確由同一個已配置identity傳入兩邊。
- 同一輪重拍、capture-window restart與Python reconnect不得增加shot-cycle identity。
- Python reconnect只更新`ConnectionIdentity`；shot-cycle identity保持不變。
- 本輪完成、無方案安全結束或fail closed後，下一次有效H才配置下一個shot-cycle identity。
- identity耗盡／wrap成0必須fail closed，不得重用舊identity。

## 4. 鍵盤控制

### 4.1 `H`／`h`：Start

- 使用Windows console的key-down／key-up事件，不使用`getline()`。
- 第一次有效key-down建立一輪Start。
- 同一次長按產生的repeat事件全部忽略。
- 收到key-up後才重新arm下一次Start。
- 一輪執行期間收到的其他`H`事件不得排隊成下一輪Start。

### 4.2 `P`／`p`：只回準備姿態

- 只允許在`WaitingForStart`且未進入shot cycle時使用。
- 按下後只執行安全前置確認與PTP到準備姿態，不前往CameraPose、不拍照、不規劃、不擊球。
- `P`不能清除`UnknownUnsafe`，也不能取代人工復原。
- H/P要求Robot從目前姿態PTP回準備姿態時，按鍵同時代表操作員已目視確認「目前姿態至準備姿態」的實體路徑無障礙；程式的停止、DO OFF與target reachability檢查不能宣稱整條PTP路徑安全。

### 4.3 H按下時不在準備姿態

H已代表操作員授權完整shot cycle，因此不強制要求先按P。允許：

```text
H
→ 確認Robot停止、DO全OFF、非UnknownUnsafe
→ PTP到準備姿態
→ confirmStopped
→ 自動繼續CameraPose與本輪流程
```

任一確認失敗即停止，不得跳過準備姿態或繼續擊球。

## 5. 自動重拍／重算

自動重試仍屬同一個Start與同一個shot cycle，不得產生第二次實際擊球。

```text
本輪三筆資料
→ NoPotCandidate／NoLegalContact，或核准的candidate-local可行性拒絕已耗盡
→ Robot保持CameraPose
→ flush/drain舊資料
→ reset本輪累積
→ 使用相同shot-cycle identity重開capture window
→ 收三筆新資料
→ 重新規劃
```

規則：

- 只在elapsed `< 10000 ms`時開始新一輪重拍。
- 一旦找到安全方案且剩餘時間`>= minimumExecutionReserveMs`，立即進入執行；不足時不得開始Pull pre-extend或任何擊球區motion。
- 到達`10000 ms`仍沒有安全方案時，不強制出桿；在狀態已知安全的前提下回準備姿態，等待下一次H。
- `NoPlanReason::NoPotCandidate`與`NoPlanReason::NoLegalContact`可在10秒規劃時段內重拍。
- `NoPlanReason::NoEligibleTarget`代表沒有合法目標，直接安全結束本輪，不重拍。
- `NoPlanReason::InvalidBrainConfiguration`與`NoPlanReason::NumericalPlanningFailure`必須fail closed，不重拍。
- SDK、設定、數值或UnknownUnsafe錯誤不得包裝成NoPlan重試。

## 6. Python斷線恢復

### 6.1 啟動與初次連線

- `BilliardApp::initialize()`只做本地設定與生命週期初始化，不得在接受H/P之前無限等待Python連線。
- `P`是Robot準備姿態的人工功能，不依賴Python是否已啟動。
- H開始本輪後，最遲在CameraPose capture階段建立／確認Python連線；連線等待使用同一個15秒shot deadline，不建立新的timeout時鐘。
- Python尚未連線不授權Robot離開CameraPose前往擊球區。

### 6.2 CameraPose階段斷線

只允許在Robot仍處於CameraPose、尚未開始前往擊球區時自動恢復：

```text
Vision socket斷線／timeout
→ Robot保持不動
→ shot deadline繼續計時，不歸零
→ 等待Python重新連線
→ 取得新的connection identity
→ read-only確認Robot仍停止在CameraPose
→ read-only確認DO1/DO2均為OFF
→ drain/reset Socket receive state
→ reset ThreeEventStability與PlanningResult
→ 以新connection identity、相同shot-cycle identity重開capture window
→ 重新累積三筆本輪資料
```

以下任一情況禁止自動繼續：

- Robot不在CameraPose。
- Robot尚未停止。
- DO1或DO2不是OFF。
- DO readback失敗。
- Robot／HRSDK連線中斷。
- RobotController已latch `UnknownUnsafe`。
- 已超過planning retry cutoff，沒有足夠的5秒執行預留時間。

到達`planningRetryCutoffMs`仍未重連時，停止重連，並read-only確認Robot仍在CameraPose、已停止且DO1/DO2均為OFF；全部通過才可PTP回準備姿態並安全結束本輪。若Robot位置已知但不在CameraPose，或DO readback明確顯示任一輸出ON，進入`ManualRecoveryRequired`；若姿態／停止／DO狀態因SDK或readback失敗而無法得知，進入`UnknownUnsafe`。兩者都不得移動。

其他恢復檢查失敗後不得自行移動到CameraPose或偷偷把DO改成安全狀態再繼續，應等待人工處理。

## 7. Push／Pull執行順序

### Push

```text
Move to strikeReadyPose
→ confirmStopped
→ DO1 Extend並readback
→ direction-change delay
→ DO2 Retract並readback，確認DO1 OFF
→ mechanism-completion wait
→ post-strike actual pose
→ safe lift
```

### Pull

```text
safe state與DO readback已知
→ DO1 Extend並readback
→ direction-change delay
→ Move to strikeReadyPose
→ confirmStopped
→ DO2 Retract並readback，確認DO1 OFF
→ mechanism-completion wait
→ post-strike actual pose
→ safe lift
```

- Push/Pull使用同一Tool1、同一Base0及同一套nominal strike-position calculation。
- 不得建立Tool2、第二TCP或模式fallback。
- Pull pre-extend成功後若retract未完成，必須進入ManualRecoveryRequired或UnknownUnsafe，不得回準備姿態。

## 8. 失敗分類

### 8.1 可嘗試下一候選

必須區分兩層：

- 同一個ShotPlan內的A/B姿態搜尋，只有hardware check明確回傳`NotReachable`／`false`時才能嘗試下一組A/B。
- 下一個ranked ShotPlan只有在`FixedForceEnvelopeRejected`、`NoAcceptedPoseCandidate`或execution preflight明確`NotReachable`時才能嘗試。
- 任何SDK、configuration、numerical、invalid-result或`UnknownUnsafe`不得利用此機制換姿態或換ShotPlan。

### 8.2 可在同一輪重新拍照／規劃

- `NoPlanReason::NoPotCandidate`或`NoPlanReason::NoLegalContact`。
- 所有candidate-local可行性拒絕已耗盡，且仍在10秒規劃時段內。
- CameraPose階段的Vision transport中斷，且完成核准恢復檢查。

### 8.3 Fail closed，不得換候選或繼續Robot motion

- configuration failure
- numerical failure
- SDK failure
- Robot／HRSDK disconnect
- DO readback failure
- pneumatic failure
- `UnknownUnsafe`

### 8.4 Post-strike failure

DO已觸發後若actual-pose讀取、安全上升或氣動復位未能確認：

- 不得回準備姿態。
- 不得假設Robot或推桿安全。
- 使用`ManualRecoveryRequired`或`UnknownUnsafe`。

## 9. 兩階段實作範圍

Config、ExecutionPlan、RobotController與BilliardApp互相依賴。為避免產生暫時編不過的中間版本、compatibility wrapper或重複review，本任務只分成兩個Phase。

### Phase 1 — Production完整同步

核准範圍：

- `src/BilliardConfig.h`
- `src/BilliardConfig.cpp`
- `src/MotionPlanner.h`
- `src/MotionPlanner.cpp`
- `src/RobotController.h`
- `src/RobotController.cpp`
- `src/BilliardApp.h`
- `src/BilliardApp.cpp`

Phase 1內部按以下三個檢查點順序修改，中間不建立commit、不要求每個檢查點都能獨立編譯——但**每個檢查點完成後必須STOP，交出當下累積的diff給Claude審查，確認無誤才能開始下一個檢查點**。理由：這個session至今每一次真正抓到安全性問題（`waitForMotion`負數狀態未latch、`MAX_TOTAL_POSE_CANDIDATES`組合爆炸、`confirmStopped()`結構性無法latch、`pulseOutput()`/`executePneumaticSequence()`把UnknownUnsafe降級等）都發生在兩個檔案以內的小範圍審查，且多半需要2-3輪來回才收斂；Phase 1一次涉及8個檔案、新的keyboard I/O模型、新的競賽計時系統與`ExecutionPlan` schema重構，範圍遠大於先前任何一輪，若review只在全部完成後做一次，風險與診斷難度都會顯著提高。三次checkpoint review不要求production build成功（可能還在中間、暫時編不過），只要求diff邏輯可審查；只有Phase 1整體完成後才需要production build成功。

#### 檢查點1 — Config與ExecutionPlan schema

- 新增版本化／核准的standby joint reference。
- 新增15秒deadline與5秒execution reserve設定；5秒標記人工可調研究值。
- 從production `ExecutionPlan`移除`transitJointReference`與`JointPtpToTransit`。
- 將safe-lift唯一權威改為`safeApproachPose.z`：移除舊的固定`safeLiftHeightMm`設定，以及只服務`actual.z + height`模型的`SafeLiftDerivationRule`／`SafeLiftDerivation`／`safeLiftRule`欄位與驗證；不得保留第二套高度來源。
- 新契約必須驗證safe-lift target保留post-strike actual X/Y/A/B/C、`target.z == plan.safeApproachPose.z`且`target.z > actual.z`；任一不成立即fail closed。
- 調整motion intent與stage contract數量及`ExecutionPlan::isValid()`。
- 不建立compatibility wrapper或第二套ExecutionPlan。

**檢查點1完成後STOP**，交出`src/BilliardConfig.h/.cpp`、`src/MotionPlanner.h/.cpp`的diff供review，通過後才進入檢查點2。

#### 檢查點2 — RobotController最小硬體能力

- 提供read-only的configured joint-pose確認能力。
- 提供read-only的DO1/DO2 OFF確認能力。
- Robot motion／state API保留完整`RobotAdapterStatus`；氣動API保留`RealPneumaticStatus::Completed/KnownSafeFailure/UnknownUnsafe`。BilliardApp不得把兩套結果壓成只有Success／Failure，也不得為此發明語意重疊的新錯誤enum。
- 不在BilliardApp直接呼叫HRSDK。
- 實體Start DI尚未確認，本輪不得新增猜測的DI編號或電位規則。

**檢查點2完成後STOP**，交出`src/RobotController.h/.cpp`的diff供review，通過後才進入檢查點3。

#### 檢查點3 — BilliardApp orchestration

- 實作H/P鍵盤edge gate。
- 建立以`std::chrono::steady_clock`為production authority、可由test seam注入fake clock的單一shot deadline，跨重拍與重連保持不變。
- 建立單一authoritative shot-cycle identity；移除execution／vision兩個計數器靠相同初值隱式同步的風險，capture restart與reconnect重用同一cycle identity。
- 移除`initialize()`在H/P可用之前無限等待Python的行為；P不依賴Vision，H在CameraPose階段依同一shot deadline處理初次連線／重連。
- 移除執行層TRANSIT_JOINT步驟。
- 實作CameraPose內重拍／重算。
- 實作Vision reconnect recovery gate。
- 擴充step結果，避免把UnknownUnsafe壓成一般Failure。
- 完成後回準備姿態；NoPlan／候選耗盡也只在已知安全時回準備姿態。

**檢查點3完成後STOP**，交出`src/BilliardApp.h/.cpp`的diff供review；通過後才進入下方Phase 1完成條件的整體production build驗證。

#### Phase 1完成條件

- 八個核准production檔案完成一致的schema migration。
- 各修改translation unit成功編譯。
- production integration build成功，所有artifact隔離在本輪專用build目錄。
- 不執行RealHardware擊球。
- grep確認production沒有`TRANSIT_JOINT`執行語意或舊ExecutionPlan欄位殘留。
- 輸出完整diff與安全審查報告，然後STOP等待review。

### Phase 2 — Tests migration與驗證

Phase 1通過review後才進行：

- 遷移`tests/p2_01_motion_planner_tests.cpp`。
- 遷移`tests/p2_02_execution_state_machine_tests.cpp`。
- 遷移`tests/p2_03_real_adapter_tests.cpp`。
- 更新其他實際受schema影響的既有測試。
- 補齊第10節列出的H/P、deadline、重拍、重連、safe lift與failure propagation情境。
- 完成targeted tests、production integration build與非硬體full regression。
- no-fire實機驗證必須另行取得使用者授權；真實Push/Pull擊球不包含在本Phase。

不得為了撐住舊test API而建立compatibility wrapper。Phase 2完成後輸出測試證據與完整diff，然後STOP等待最終review。

### Git commit／push規則

- Phase 1與Phase 2實作及review期間不得自行commit或push。
- 每個Phase只有在review通過且使用者明確授權`commit + push`後才能提交。
- 提交前必須重新確認remote為`https://github.com/DoBeDoDa/Topics.git`、分支為`main`，並逐一列出staged files。
- Phase 1 commit只納入上述八個核准production檔案；不得夾帶既有或無關dirty files。
- Phase 2 commit只納入核准的tests migration與當輪明確授權文件；不得夾帶其他工作區內容。
- push只允許`origin/main`；禁止推送至其他remote。

## 10. 必要測試情境

- H key-down立即Start，不需Enter。
- H長按只啟動一次，key-up後才可重新arm。
- active cycle期間排入console queue的H/P事件不得在回到WaitingForStart後自動啟動下一輪。
- P只回準備姿態，不啟動shot cycle。
- H在非準備姿態時先安全回準備姿態，再開始同一輪。
- 一個H最多只有一次實際氣動擊球。
- 第一個安全方案找到且剩餘時間仍符合5秒execution reserve時立即執行。
- 剩餘時間少於5秒時不得開始Pull pre-extend或前往safeApproachPose；執行已開始後不以deadline強制abort。
- 10秒內NoPlan可重新收三筆並重算。
- deadline測試使用fake monotonic clock，不實際等待10／15秒。
- 第10秒仍無方案時不強制擊球。
- `NoPotCandidate`／`NoLegalContact`可重拍；`NoEligibleTarget`直接結束；config／numerical failure不得重拍。
- 重連不重置15秒deadline。
- Vision reconnect使用新connection identity、相同shot-cycle identity。
- 重拍、capture restart與reconnect都不增加shot-cycle identity；下一次有效H才增加一次。
- 重連時CameraPose不符、Robot未停或DO非OFF均禁止繼續。
- 第10秒仍未重連時，只有read-only安全確認全部通過才可回standby，否則ManualRecoveryRequired／UnknownUnsafe。
- production執行沒有TRANSIT_JOINT命令。
- CameraPose直接PTP到safeApproachPose。
- safe lift保留post-strike actual X/Y/A/B/C，Z等於safeApproachPose.z。
- safe lift確認後PTP回standby，不回CameraPose。
- explicit NotReachable可考慮下一姿態／候選。
- SDK/config/numerical/UnknownUnsafe不得換候選。
- Pull pre-extend後retract失敗不得返回standby。

### 10.1 驗證方式分類

單元測試的目的是保護少數高風險安全契約，不是把上面24項行為描述逐條變成
獨立大型測試案例。自動測試（tests/p2_02、p2_03）集中保護以下6項核心契約：

1. **一個H最多一次實際氣動擊球**（對應第6項）——
   `runOfflineSingleCycle`/`runRealSingleCycle`結構上每次呼叫只執行一次
   `runPneumatic`；p2_02/p2_03全部成功路徑案例的`do1On`呼叫次數斷言為1。
2. **UnknownUnsafe後零further motion**（貫穿第16、17、23項的安全核心）——
   p2_02、p2_03對pneumatic/actualPose/safe-lift-path/Pull-prepare各個
   UnknownUnsafe分支，都斷言零actual-pose read、零LIN、零PTP、零
   standby-return。
3. **Pull retract失敗不得返回standby**（對應第24項）——
   `runtime.state == ManualRecoveryRequired`且明確驗證不繼續standby return。
4. **safe lift使用actual pose且只改Z**（對應第20項）——
   LIN起點＝actual pose、目標保留X/Y/A/B/C、Z=safeApproachPose.z，
   check LIN→move LIN→confirm stopped→return standby的嚴格順序都有斷言。
5. **deadline不足不開始擊球，執行已開始不因deadline中止**（對應第7、8項）——
   Push/Pull兩種模式都驗證reserve不足時零pre-extend/零safeApproach motion；
   另有獨立案例證明執行開始後deadline中途過期不會中止已開始的cycle。
6. **reconnect不得混用connection／shot-cycle identity**（對應第13、14、15項
   的核心風險）——deadline累加（不因重連歸零）有完整自動測試；
   connectionIdentity配發器（`LocalConnectionLifecycle`）本身有直接測試，
   但這**不是**端到端測試，run()實際wiring靠程式碼審查，見下方第14項。

其餘18項不個別建立獨立大型測試，逐項分類如下（不重複列出測試內容細節）：

- **既有測試已涵蓋同一段程式碼路徑**（第1–5、9–12、19、21、22項）：
  H/P鍵盤edge gate、PreparationReturn、NoPlan重試/安全結束分類、
  CameraPose直接PTP、candidate NotReachable換下一個——都在p2_02、p2_03
  既有的state-machine/candidate-search table驅動測試裡，不需要另外
  逐條再開一個測試案例。NoLegalContact與NoPotCandidate共用同一段
  default-continue程式碼，「精確收三筆」是p1_04既有範圍，不重複測。
- **程式碼審查佐證**（第14項run()端wiring、第15項cycleIdentity不可變、
  第16項Robot未停分支、第18項TRANSIT_JOINT、第23項候選搜尋期間
  preflightExecution自身latch的UnknownUnsafe）：結構上由型別系統或
  已移除的符號保證，建構獨立測試fixture的成本明顯超過驗證價值。
- **實機no-fire驗證**（第17項10秒cutoff與read-only確認的合併時序）：
  單元測試分別驗證了AllSafe/ManualRecoveryRequired/UnknownUnsafe三個
  分支各自成立，但「剛好卡在10秒」的合併時序屬於第11節no-fire驗證範圍。

## 11. 實機驗證順序

1. 純鍵盤event測試，不連Robot。
2. Fake adapter驗證state transition、deadline與錯誤傳遞。
3. Production compile/link，但不啟動真實擊球。
4. no-fire：H/P、standby、CameraPose與直接safe-approach PTP，使用低速並人工監看。
5. no-fire：actual-pose vertical LIN safe lift與standby return。
6. DO readback測試，不接觸球。
7. Push/Pull實機擊球必須另外取得使用者授權，不可由本計畫自動進行。

## 12. 尚未確認／延後項目

- 實體Start按鈕的DI index。
- 按鈕按下時DI是`1`還是`0`。
- 實體按鈕debounce與release-to-rearm時間。
- `minimumExecutionReserveMs = 5000`是否足以涵蓋最慢CameraPose至實際出桿時間；需實測後調整。
- 準備姿態與CameraPose、safeApproach之間的真實PTP路徑仍須以低速no-fire方式驗證；target reachable不等於整條PTP path已證明安全。

## 13. 明確不修改

- `Algorithm.*`
- `BilliardPhysics.*`
- `VisionDataParser.*`
- `SocketClient.*`
- `TableState.h`
- `TargetSelector.*`
- `MathUtils.*`
- Python V1 32-value newline-delimited CSV protocol

不得新增frame ID、timestamp、handshake或其他Python protocol metadata。
