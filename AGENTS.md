# Codex 專案協作規則

本檔案適用於整個 repository。Codex 在任何電腦 clone 本專案後，都必須先閱讀並遵守以下規則。

## 溝通與動工規則

1. 修改任何程式碼或專案檔案前，先向使用者確認修改範圍、必要參數、安全條件與驗證方式。
2. 使用者明確回覆可以開始後才能修改；尚未確認的需求不得自行假設或實裝。
3. 產生修改後，必須主動審查邏輯漏洞、錯誤處理與安全風險，修正後再進行編譯或適當的驗證。
4. 回報時清楚區分：已完成、尚未完成、仍待使用者確認。
5. 每五次有效的使用者提問後，附上「如何讓問題問得更好」的具體建議。

## Git 與使用者檔案

1. 所有經使用者確認的程式碼修改，都提交並推送至：
   `https://github.com/DoBeDoDa/Topics.git`
2. 預設分支為 `main`。
3. 禁止推送至：
   `https://github.com/team-reflect/reflect-open.git`
4. 提交前必須確認 remote URL、目前分支與 staged files，避免夾帶無關修改。
5. 工作區內既有的未提交修改屬於使用者；除非使用者明確授權，不得覆寫、還原或提交。
6. 下列使用者檔案或資料夾不得自行刪除或提交：
   - `History/`
   - `debug_frame.png`
   - `黑白棋盤.docx`
7. `src/test_cueball.cpp` 若只有使用者既有的輸出換行修改，不得擅自覆寫或納入其他提交。

## 已確認的機械手臂與座標原則

1. 手臂型號為 HIWIN RA605-GC。
2. 目前使用 `Tool1`，球桿尖端設定為 Tool1 TCP。
3. 目前使用 `Base0`。
4. 正式送出笛卡兒目標前，程式應主動設定並確認 Tool1 與 Base0。
5. Python送入C++的32值座標已是Robot Base0平面`X、Y`毫米；C++不得再次執行pixel轉換、Homography、相機補償、TableFrame→Base0或第二次平面映射。
6. Robot `Z`由`MotionPlanner`使用人工校正的Strike Z生成；A、B只可在版本化且人工核准的小區間內搜尋，C由Base0平面擊球方向產生，再由HRSDK adapter依已驗證規則轉成控制器角度表示。
7. HRSDK 的 `ptp_pos(HROBOT, mode, pose)` 第二個參數是運動 mode，不是 Tool 編號。Tool 編號必須使用 `set_tool_number()` 或 `RobotController::setToolNumber()` 設定。
8. 即使 TCP 的 `X、Y、Z` 不變，改變 `RX、RY、RZ` 仍可能造成法蘭與關節大幅移動，因此送出動作前仍須檢查可達性與安全。
9. V1維持既有newline-delimited 32值CSV，不得自行要求Python新增control handshake、runtime attestation、sender frame ID或timestamp；freshness由既有`BilliardApp + SocketClient`在CameraPose settle後以本地shot-cycle gate、舊buffer flush及累積reset管理。
10. 一個`StartRequested`只允許一個完整shot cycle；完成後返回`WaitingForStart`，不得自動規劃或擊發下一球。

## 已確認的程式職責方向

1. `BilliardConfig.cpp/.h`：集中保存可調參數、Tool/Base 編號及姿態設定。
2. `MathUtils.cpp/.h`：負責旋轉矩陣、座標軸、矩陣組合與角度轉換等純數學功能。
3. `MotionPlanner.cpp/.h`：根據擊球線與設定值建立手臂目標姿態及移動計畫。
4. `TargetSelector.cpp/.h`：唯一負責從穩定桌面狀態中選出最低號存在的 1～9 號合法目標球；沒有編號球時回報 `NoEligibleTarget`，不得負責袋口評分、Robot Pose 或硬體控制。
5. `Algorithm.cpp/.h`：負責對已選定目標球生成與比較 DirectPot／KickPot 候選、策略決策、共同評分與確定性選擇；不得重新選擇較高號目標球，不處理 HRSDK 或機械手臂姿態。
6. `BilliardApp.cpp/.h`：只負責整合視覺生命週期、規劃結果、移動計畫與手臂執行流程，不實作個別撞球幾何或硬體底層細節。
7. `RobotController.cpp/.h`：封裝 HRSDK 連線、Tool/Base 設定、診斷與運動／DO 命令。
8. 重構優先原地修改上述既有 cpp／class；概念名稱不得用來建立同功能的第二套 Socket、Parser、Algorithm、MotionPlanner、RobotController、BilliardApp、MathUtils 或 BilliardPhysics。Socket、Parser、Algorithm、MotionPlanner、RobotController、BilliardApp、MathUtils或BilliardPhysics。

## 尚未確認，禁止假設或實裝

下列項目必須取得使用者的實機確認或明確數值後才能修改程式：

1. `a、b、c` 的旋轉順序。
2. 固定參考姿態的`A、B`人工校正基準、核准搜尋範圍與step。
3. HIWIN將核心Pose的`A、B、C`轉成`RX、RY、RZ`的確切順序、角度範圍與奇異點處理規則。
4. `motion_reachable()`失敗時除核准A／B小區間搜尋以外的安全策略。
5. 鏡頭畸變校正功能的實裝範圍與校正資料。
6. Tool1中的實體球桿forward axis；不得在實機校正前假設為局部`+X`。
7. Base0 `+Z`是否為實體安全上方；P2-03 no-fire確認前不得啟用真實擊發後safe lift。

## 動作安全

1. 不得因 `motion_reachable()` 回傳可達，就假設整條 PTP 路徑安全；它只代表目標姿態的可達性判定。
2. LIN 路徑需另外使用 `motion_check_lin()` 檢查。
3. 未確認姿態轉換、座標正負方向或實機安全前，先做計算與輸出診斷，不得直接送出手臂移動命令。
4. 測試動作應使用低速、安全高度、人工確認與可立即停止的環境。
5. 擊球及氣動完成後，必須重新讀取actual pose；第一個Robot motion保持該actual pose的X／Y／A／B／C不變，以通過`motion_check_lin()`的垂直LIN沿已確認安全的Base0 `+Z`上升至安全高度。不得以planned pose代替；確認到達前不得PTP返回拍照點，氣動狀態未知時不得移動。
