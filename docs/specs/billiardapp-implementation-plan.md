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
- 計時從接受`H`／Start的瞬間開始，包含必要的準備姿態移動、CameraPose、拍照、規劃、Robot移動與氣動。
- Push以DO1 Extend開始作為出桿時點。
- Pull以DO2 Retract開始作為出桿時點；Pull的DO1 pre-extend不是實際出桿。
- 找到安全可執行方案後立即執行，不等待10秒用完。
- 前10秒內允許自動重拍、重置三次累積並重新規劃。
- 剩餘時間少於5秒時不得再開始新的重拍／重算。
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
→ drain Winsock kernel receive queue並reset本地frame accumulator
→ reset ThreeEventStability與本輪PlanningResult
→ 開啟本輪capture window
→ 接收三筆本輪新資料
→ Algorithm產生ranked ShotPlan
→ 依母球XY與擊球方向決定唯一Push/Pull
→ MotionPlanner使用真實HRSDK checks搜尋A/B姿態
→ 只在明確NotReachable時嘗試下一姿態或下一ranked candidate
→ PTP直接到safeApproachPose
→ 讀取actual approach pose
→ motion_check_lin(actual approach, strikeReadyPose)
→ LIN到strikeReadyPose
→ confirmStopped
→ 執行Push或Pull
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
→ NoPlan或所有候選明確NotReachable
→ Robot保持CameraPose
→ flush/drain舊資料
→ reset本輪累積
→ 使用相同shot-cycle identity重開capture window
→ 收三筆新資料
→ 重新規劃
```

規則：

- 只在elapsed `< 10000 ms`時開始新一輪重拍。
- 一旦找到安全方案立即進入Robot執行。
- 到達`10000 ms`仍沒有安全方案時，不強制出桿；在狀態已知安全的前提下回準備姿態，等待下一次H。
- `NoEligibleTarget`等確定沒有合法目標的結果不應無限重算；應結束本輪。
- SDK、設定、數值或UnknownUnsafe錯誤不得包裝成NoPlan重試。

## 6. Python斷線恢復

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

失敗後不得自行移動到CameraPose或偷偷把DO改成安全狀態再繼續，應等待人工處理。

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

只有：

- 明確`NotReachable`
- `FixedForceEnvelopeRejected`
- `NoAcceptedPoseCandidate`

### 8.2 可在同一輪重新拍照／規劃

- 安全的NoPlan。
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

Phase 1內部仍按以下順序修改與自我核對，但中間不建立commit、不要求逐點停下review；全部production contract同步並成功編譯後再統一STOP送審。

#### 檢查點1 — Config與ExecutionPlan schema

- 新增版本化／核准的standby joint reference。
- 新增15秒deadline與5秒execution reserve設定；5秒標記人工可調研究值。
- 從production `ExecutionPlan`移除`transitJointReference`與`JointPtpToTransit`。
- 將safe-lift契約由固定`actual.z + safeLiftHeightMm`改為`safeApproachPose.z`。
- 調整motion intent與stage contract數量及`ExecutionPlan::isValid()`。
- 不建立compatibility wrapper或第二套ExecutionPlan。

#### 檢查點2 — RobotController最小硬體能力

- 提供read-only的configured joint-pose確認能力。
- 提供read-only的DO1/DO2 OFF確認能力。
- 保留`Success/NotReachable/KnownSafeFailure/UnknownUnsafe`資訊給BilliardApp。
- 不在BilliardApp直接呼叫HRSDK。
- 實體Start DI尚未確認，本輪不得新增猜測的DI編號或電位規則。

#### 檢查點3 — BilliardApp orchestration

- 實作H/P鍵盤edge gate。
- 建立單一shot deadline，跨重拍與重連保持不變。
- 移除執行層TRANSIT_JOINT步驟。
- 實作CameraPose內重拍／重算。
- 實作Vision reconnect recovery gate。
- 擴充step結果，避免把UnknownUnsafe壓成一般Failure。
- 完成後回準備姿態；NoPlan／候選耗盡也只在已知安全時回準備姿態。

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
- P只回準備姿態，不啟動shot cycle。
- H在非準備姿態時先安全回準備姿態，再開始同一輪。
- 一個H最多只有一次實際氣動擊球。
- 第一個安全方案找到後立即執行。
- 10秒內NoPlan可重新收三筆並重算。
- 第10秒仍無方案時不強制擊球。
- 重連不重置15秒deadline。
- Vision reconnect使用新connection identity、相同shot-cycle identity。
- 重連時CameraPose不符、Robot未停或DO非OFF均禁止繼續。
- production執行沒有TRANSIT_JOINT命令。
- CameraPose直接PTP到safeApproachPose。
- safe lift保留post-strike actual X/Y/A/B/C，Z等於safeApproachPose.z。
- safe lift確認後PTP回standby，不回CameraPose。
- explicit NotReachable可考慮下一姿態／候選。
- SDK/config/numerical/UnknownUnsafe不得換候選。
- Pull pre-extend後retract失敗不得返回standby。

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
