# 撞球機械手臂系統重構 Master Spec

## 文件資訊

- 狀態：Approved for phased implementation
- 規格版本：1.1
- 建立日期：2026-07-30
- Repository：`C:\Users\User\Desktop\Topics-main-git`
- 基準分支：`main`
- 基準 Commit：`b278db0`
- Phase 1 詳細規格：[`phase-1-math-geometry-refactor-spec.md`](phase-1-math-geometry-refactor-spec.md)

本文件整合完整 Grill Me 訪談結果與基準 Commit 的實際程式碼。它是後續階段的總體約束，不授權任何未列入當期 Phase 的機械手臂動作、HRSDK 連線或氣壓輸出。

## 1. 系統背景

本系統使用 HIWIN RA605-GC 機械手臂、Base 0 與 Tool 1，結合 Python 視覺、C++ 撞球策略、姿態規劃、HRSDK 運動控制及氣壓推桿完成九號球擊球。

目前程式已包含：

- Python 視覺端持續傳送 32 個 CSV 座標值。
- C++ 端解析母球、1 至 9 號球及 6 個袋口。
- `TargetSelector` 選擇最低號目標球及袋口。
- `Algorithm` 建立直球或反彈球瞄準點。
- `MotionPlanner` 產生手臂笛卡兒姿態。
- `RobotController` 封裝 HRSDK。
- `BilliardApp` 串接視覺、策略、規劃與實際動作。

目前主要問題不是單一函式錯誤，而是純數學、視覺 sentinel、撞球物理、機械姿態與硬體控制的語意互相滲透。重構必須分階段完成，禁止一次改寫所有子系統。

## 2. Repository 基準與現況

掃描時確認：

- `main` 正確追蹤 `origin/main`。
- Remote fetch/push 均為 `https://github.com/DoBeDoDa/Topics.git`。
- `main` 與 `origin/main` ahead 0、behind 0。
- 工作樹乾淨。
- Repository 沒有既有 `docs/specs` 結構、CMakeLists 或單元測試框架。
- `.vscode/tasks.json` 使用 MSVC `cl.exe`，但沒有 `/std:c++17`。
- `README.md` 目前只有簡短專案名稱。

## 3. Grill Me 已確認結論

### 3.1 MathUtils

`MathUtils` 只保留純數學與通用幾何責任：

- 有限值檢查。
- 二維向量建立。
- 向量長度。
- 點距離。
- 向量正規化。
- 二維方向角。
- 向量夾角。
- 必要的矩陣、座標軸及角度轉換。

以下責任必須移出或移除：

- 相機補償移至 `CameraCompensator`。
- 移除 `getTiltOffset()`。
- 移除 `MotionProfile::tiltRyDeg` 的混合語意。
- 移除 `YAW_OFFSET_DEG`。

零向量、NaN、Infinity、運算溢位或退化幾何不得回傳 0 度、原始點或看似成功的預設值。

### 3.2 視覺資料

- Socket 協定維持 32 個 CSV 值及換行框架。
- `-9999.0` 只保留為 wire protocol sentinel，且只接受精確成對
  `x == -9999.0 && y == -9999.0`。
- Parser 必須立即將精確成對 sentinel 轉成 `std::nullopt`；只有單軸等於
  sentinel 時整幀為 `Invalid`。
- 不使用 `-9000.0` 或任何範圍門檻判斷 missing。其他小於 `-9000.0` 的有限
  座標仍是 present point，交由補償後桌面範圍驗證拒絕。
- Sentinel 不得進入 MathUtils、BilliardPhysics、Algorithm 或 MotionPlanner。
- NaN、Infinity、格式錯誤、極端值與桌面外座標必須 fail closed。
- 規劃前需要連續 3 幀穩定資料。
- 三幀 presence pattern 必須完全一致；任一球或袋口出現／消失即為
  `Unstable`。
- 每個存在物件分別取三個 X 與三個 Y 的中位數；三幀中每個點到其中位數點的
  歐氏距離皆不得超過 `stableFrameToleranceMm`。
- 穩定成功後只使用中位數座標產生核心 `TableState`。
- 任一單幀處理結果不是 `Valid` 時不得組成三幀輸入，也不得產生穩定結果。
- 3 幀穩定容差尚待實測，不得猜測；缺少時回 `ConfigurationMissing`。

### 3.3 幾何與碰撞

- 零長度路徑是 `Invalid`，不是 `Clear`。
- 路徑檢查使用 `Clear`、`Blocked`、`Invalid` 三態。
- 鬼球點、鏡射點、交點、正規化、方向角與夾角必須顯式表示失敗。
- 球路安全距離為「球直徑加可配置安全餘量」。
- 正式安全餘量尚待實測，不得猜測。
- `checkRoute` 必須先驗證路徑與參數；驗證成功且障礙列表為空時回
  `Clear`，退化路徑或無效參數仍回 `Invalid`。
- 有效且非退化輸入下，Algorithm 現有直球／反彈策略排序不得在 Phase 1 改變。

### 3.4 機械手臂與工具

Cartesian Pose 格式：

```text
X, Y, Z, RX, RY, RZ
```

已確認：

- 使用 Tool 1 與 Base 0。
- Tool 1 TCP 以氣壓推桿完全縮回時標定。
- Tool 1 相對法蘭只有平移，沒有旋轉偏移。
- Tool `+X` 是擊球方向。
- Tool `-X` 是退桿方向。
- 擊球點是推桿縮回時 Tool 1 TCP 到達的位置。
- 水平且工具朝下的基準 RX 為 `-180°`。
- 水平擊球的基準 RY 為 `0°`。
- RZ 不是固定值。
- 同一擊球候選中，TCP 的 X、Y、Z、RZ 固定。
- 搜尋 RX／RY 時不重新計算 TCP XYZ。
- TCP 距母球中心的基準距離為 50 mm。
- 擊球 Z 由實機手動調整。

擊球方向與 RZ 定義：

```text
shotDirection = aimTarget - cueBall
shotRzDeg = atan2(shotDirection.y, shotDirection.x)
```

不得使用 RX、RY 或 HRSDK Euler 值反推二維擊球方向。不得對 RZ 加固定補償。

### 3.5 姿態搜尋

暫定搜尋範圍：

```text
RX：-180° 至 -175°
RY：0° 至 -5°
步距：1°
```

搜尋規則：

1. 先固定 RX=`-180°`，依序搜尋 RY。
2. 無有效姿態時，再逐步搜尋 RX=`-179°` 至 `-175°`。
3. `RX<-180°` 在實測前禁止啟用。
4. 不得把 `RX=-185°` 自動改寫成 `+175°`。
5. 不得只因 Euler 角數學等價就假設關節組態與路徑等價。

### 3.6 建議運動流程

未來目標流程：

```text
Joint PTP 到 transitJoint
→ Cartesian PTP 到 safeApproachPose
→ 讀取實際目前姿態
→ motion_check_lin(actualPose, strikeTcpPose)
→ 短距離 LIN 到 strikeTcpPose
→ 手臂停止
→ 氣壓擊發及收回
→ Joint PTP 直接回 CAMERA_JOINT
```

LIN 檢查失敗時不得自動改用未驗證的 PTP 下降。

### 3.7 自動比賽

系統分為：

- `CompetitionAuto`
- `ManualDiagnostic`

CompetitionAuto 使用接在電腦上的按鈕模擬鍵盤 `H`：

- 使用按下沿觸發。
- 必須釋放按鍵後才接受下一次觸發。
- 回拍照點後等待對手，不立即規劃。
- H 按下後清除舊 Socket 資料。
- 只接受觸發後重新累積的 3 幀穩定資料。
- 視覺或幾何無效可以重新取像。
- 手臂警報不得自動忽略。
- 警報排除後，必須重新按 H 並從取像開始。

### 3.8 氣壓

- DO1：擊出。
- DO2：收回。
- 電磁閥會保持切換後狀態。
- DO1 與 DO2 不得同時 ON。
- DO2 完成收回後必須 OFF。
- DO2 OFF 後手臂才可返回拍照點。
- 命令脈衝與機構完成等待必須使用不同參數。
- 正式時間尚待實測。

第一輪實機擊球測試：

- 氣壓電源供應器不通電。
- 程式可以執行 DO 邏輯。
- 實際氣壓閥由人工操作。
- 只測試單顆球。
- 不執行正式連續比賽。

## 4. 現有程式碼與需求的衝突

### 4.1 Safety Critical

1. `MathUtils::getAngleBetweenVectors()` 在任一零向量時回傳 `0.0`，可能把無效幾何當成完美夾角。
2. `MathUtils::getVectorAngle(0, 0)` 透過 `atan2` 產生看似合法的 0 度。
3. `BilliardPhysics::isPathBlocked()` 在零長度路徑時回傳 `false`，語意等同 `Clear`。
4. `getGhostBall()`、`getPerpendicularTarget()` 與 `getSlantedBankTarget()` 在退化情況回傳輸入點，製造假成功。
5. `VisionDataParser` 沒有拒絕所有非有限值；Infinity 可被標記為 detected。
6. `MotionProfile::tiltRyDeg` 同時被當成 Pose RY 與三角函數傾角。
7. `getTiltOffset()` 對姿態 RY 使用 sin/cos；輸入 `-180°` 時會反轉 XY 方向。
8. `MotionPlanner` 將 `getTiltOffset()` 的 Z 結果丟棄。
9. `BilliardApp` 在 `motion_check_lin()` 失敗或回報不可達後仍明確執行 PTP。
10. Algorithm 對不安全的反彈路線仍建立「強制開火」決策。
11. 手臂運動期間 Python 持續送資料；目前返回拍照點後可能消費舊影像。

### 4.2 設定衝突

目前 `PRODUCTION_MOTION`：

```text
RX = 0°
RY = -180°
```

已確認模型：

```text
RX = -180°
RY = 0° 起始
```

目前設定不可視為已通過實機驗證的正式姿態。

Repository 的 `AGENTS.md` 仍將「Tool +Y 或 -Y」、「固定參考姿態」及「沿 Tool Y 位移 50 mm」列為未確認。完整 Grill Me 訪談已明確確認 Tool `+X`／`-X`、RX=`-180°`、RY=`0°`基準及50 mm TCP距離，因此本規格採用最新訪談結論。`AGENTS.md` 的文字同步應在後續取得授權時獨立處理；Euler旋轉順序等未確認項目仍維持未確認。

### 4.3 數值與建置

- `0.001` 同時比較 mm、mm²及交點行列式。
- `-9000.0` 門檻會把合法解析但桌面外的有限值誤判為 missing；Phase 1 必須
  移除該門檻，只保留精確成對 `-9999.0`。
- 距離使用 `sqrt(dx*dx+dy*dy)`，可能不必要地溢位。
- 建置未指定 C++17。
- 沒有純數學測試目標。

## 5. 模組責任與架構邊界

### 5.1 目標模組

| 模組 | 目標責任 |
|---|---|
| MathUtils | 純數學、有限值、向量、角度、矩陣 |
| CameraCompensator | 僅執行單點的現有線性相機殘差補償 |
| VisionDataParser | 嚴格解析 32 欄 CSV、精確成對 sentinel 轉 `ParsedVisionFrame` 中的 optional |
| VisionFrameProcessor | 持有／使用 CameraCompensator，執行單幀補償、必要資料與桌面範圍驗證，成功後產生 `ProcessedVisionFrame` |
| StableFrameValidator | 對三個單幀 Valid 結果執行 presence、中位數及距離驗證，成功後唯一產生 `TableState` |
| TargetSelector | 只根據三幀穩定後的 `TableState` 選球；不得持有、建構或呼叫 CameraCompensator |
| BilliardPhysics | 鬼球、碰撞、鏡射及交點 |
| ShotCandidateValidator | 組合幾何結果，拒絕無效候選 |
| Algorithm | 直球／反彈策略與目標選擇 |
| MotionPlanner | 建立姿態候選與動作意圖，不呼叫 HRSDK |
| HiwinOrientationAdapter | 將已確認的姿態模型轉成 HRSDK Euler |
| RobotController | HRSDK、Tool/Base、reachable、LIN、PTP及警報 |
| BilliardApp | 流程協調與狀態機 |
| PneumaticController | DO1／DO2互斥狀態機 |

### 5.2 依賴方向

```text
SocketClient
→ VisionDataParser
→ ParsedVisionFrame
→ VisionFrameProcessor
    └─ CameraCompensator
→ ProcessedVisionFrame（已補償且單幀 Valid）
→ StableFrameValidator
→ TableState（三幀中位數、穩定且已驗證的 Base0 座標）
→ TargetSelector
→ Algorithm / BilliardPhysics
→ ShotCandidateValidator
→ MotionPlanner
→ BilliardApp
→ RobotController / PneumaticController
→ HRSDK / digital outputs
```

MathUtils 不得依賴 BilliardConfig、CameraCompensator、Algorithm、MotionPlanner 或 HRSDK。

## 6. 資料流

### 6.1 視覺資料流

```text
TCP newline frame
→ 嚴格解析 32 tokens
→ 非有限值檢查
→ 成對 sentinel 轉 std::nullopt
→ ParsedVisionFrame
→ VisionFrameProcessor
→ 逐點 CameraCompensator
→ 補償後 Base0 桌面範圍及必要資料驗證
→ ProcessedVisionFrame
→ StableFrameValidator（三個單幀 Valid 結果）
→ presence pattern 一致性
→ 每個存在物件的 X/Y 中位數與歐氏距離驗證
→ TableState（中位數座標）
→ TargetSelector
```

解析、補償、範圍與穩定驗證是不同責任。任何階段失敗都不得產生部分可用的核心球桌狀態。

固定階段型別不得互換：

- `ParsedVisionFrame`：已解析 sentinel 與有限值，但尚未補償。
- `ProcessedVisionFrame`：已補償並通過單幀必要資料與桌面範圍驗證。
- `TableState`：只由 `StableFrameValidator` 三幀穩定成功後產生。

`TargetSelector` 不得 include、持有、建構、注入或呼叫 `CameraCompensator`；
其唯一視覺核心輸入是 `TableState`。

### 6.2 擊球資料流

```text
cueBall + aimTarget
→ shotDirection
→ 正規化二維擊球方向
→ shotRzDeg
→ 固定 strikeTcpPose XYZ/RZ
→ RX/RY候選
→ reachable及路徑驗證
→ 動作執行
```

RX／RY 是姿態候選，不是二維擊球方向來源。

Phase 1 完成後 `MotionProfile` 的欄位與順序固定為：

```cpp
struct MotionProfile {
    double strikeZ;
    double safeZ;
    double rxDeg;
    double ryDeg;
    double standoffExtraMm;
};
```

`moveBackMm` 必須移除；PRODUCTION_MOTION 與 TEST_MOTION 只做舊值逐欄遷移，
不得改變現有 RX／RY 數值。現有 PRODUCTION_MOTION 尚未通過實機驗證，
編譯成功不得解讀為姿態安全。

## 7. 安全原則

1. 所有無效輸入 fail closed。
2. 不以假角度、假座標或預設點繼續規劃。
3. `motion_reachable()` 只證明終點可達，不證明 PTP 路徑安全。
4. LIN 必須使用 `motion_check_lin()`。
5. LIN 檢查失敗不可改用 PTP 強制下降。
6. 未確認 Euler 規則前，不實作矩陣轉 HRSDK Euler。
7. 未校正的安全參數不得填入猜測值。
8. Tool 1／Base 0 必須在正式笛卡兒命令前主動設定並確認。
9. 改變 RX／RY 即使 TCP 不變，也必須視為可能造成大幅關節動作。
10. 任何自動擊發前，所有幾何、姿態、路徑及氣壓狀態都必須有效。
11. 有效資料下的策略相容，不代表不安全候選可以強制執行。
12. 所有「status + optional payload」Result 必須符合雙向不變量：success
    status 必須有值，任何非 success status 必須為 `std::nullopt`；禁止矛盾組合。
13. Phase 1 Implement 與測試不得執行 `main`、`calibrate`、`test_cueball`
    或任何連結 RobotController／HRSDK／數位輸出的程式。
14. Phase 1 只允許執行兩個不依賴 HRSDK 的離線測試；編譯成功不代表目前姿態
    可安全上機。

## 8. 自動比賽狀態機概要

Phase 5 應使用明確 enum state，不得以零散 bool 表示：

```text
Initializing
→ ReturningToCamera
→ WaitingForStart
→ FlushingVision
→ CollectingStableFrames
→ SelectingTarget
→ PlanningShot
→ SearchingPose
→ MovingToTransit
→ MovingToApproach
→ ValidatingLinearPath
→ MovingToStrike
→ Firing
→ Retracting
→ ReturningToCamera
```

失敗分支：

- 視覺格式／穩定失敗：回 `CollectingStableFrames`。
- 幾何候選失敗：嘗試下一候選；全部失敗後重新取像。
- 姿態不可達：嘗試下一姿態或路線。
- HRSDK錯誤、警報、動作逾時：進入 `FaultLatched`。
- `FaultLatched` 只能由人工排除，之後回 `WaitingForStart`，並要求新的 H 按鍵。

## 9. 分階段計畫

### Phase 1：MathUtils、CameraCompensator與安全幾何

依賴：

- C++17編譯器。
- 本文件及 Phase 1 詳細規格。
- 不依賴HRSDK、手臂或氣壓。

修改範圍：

- MathUtils、Point、CameraCompensator。
- ParsedVisionFrame、ProcessedVisionFrame、TableState、VisionDataParser、
  VisionFrameProcessor、StableFrameValidator、TargetSelector。
- BilliardPhysics及必要Algorithm呼叫端。
- MotionPlanner與BilliardConfig僅做新API及固定五欄 `MotionProfile` 的必要相容遷移。
- BilliardApp與test_cueball僅處理新失敗結果。
- 建置設定與兩個純離線測試執行檔。

驗收：

- C++17。
- `phase1_core_tests`與`phase1_algorithm_regression_tests`均不include或link
  HRSDK、RobotController、SocketClient或數位輸出程式。
- Sentinel不進核心幾何。
- 只有`StableFrameValidator`可產生`TableState`。
- 三幀presence、中位數及距離規則具有離線fixture測試。
- 所有退化幾何顯式失敗。
- 有效輸入的直球／反彈決策回歸結果不變。
- `MotionProfile`欄位固定為strikeZ、safeZ、rxDeg、ryDeg、standoffExtraMm，
  並移除moveBackMm。
- 不執行main、calibrate或test_cueball。

回滾：

- 單一Phase提交。
- 回滾整個Phase提交即可恢復原API與呼叫端。
- 不回滾或覆寫Phase開始前的使用者變更。

禁止跨越：

- 不實作H按鈕。
- 不實作DO狀態機。
- 不改寫HRSDK運動。
- 不執行姿態搜尋。
- 不改Algorithm策略排序。
- 不執行任何連結RobotController／HRSDK或數位輸出的程式。

### Phase 2：MotionPlanner、CartesianPose與姿態候選

依賴：

- Phase 1完成。
- `STRIKE_TCP_Z_MM`可保持未校正，但正式動作必須被禁用。
- Euler文件未確認時只能產生語意姿態候選，不能建立未證實的矩陣轉Euler。

預計檔案：

- `Point.h`或新 `CartesianPose.h`
- `BilliardConfig.h/.cpp`
- `MotionPlanner.h/.cpp`
- `BilliardApp.h/.cpp`
- 新 `HiwinOrientationAdapter.h/.cpp`，初期可只有明確blocked介面
- MotionPlanner純測試

驗收：

- 核心不傳遞笛卡兒 `std::array<double,6>`。
- strike TCP的XYZ/RZ在RX／RY候選間固定。
- 搜尋範圍與順序符合規格。
- MotionPlanner不包含HRSDK。
- 沒有實際運動。

回滾：

- 回滾Phase 2提交，Phase 1純幾何仍可獨立保留。

禁止跨越：

- 不自動角度wrap。
- 不假設Euler順序。
- 不呼叫motion_reachable或運動命令。

### Phase 3：RobotController安全運動流程

依賴：

- Phase 2。
- Needs Documentation項目完成。
- 安全接近Z與測試環境完成硬體確認。

預計檔案：

- `RobotController.h/.cpp`
- `BilliardApp.h/.cpp`
- `BilliardConfig.h/.cpp`
- 運動診斷測試程式

驗收：

- Tool 1／Base 0設定及讀回確認。
- 每個候選終點使用reachable。
- LIN使用實際起點執行motion_check_lin。
- LIN檢查失敗不送PTP下降。
- 動作錯誤立即fail closed。
- 低速、無氣壓測試通過。

回滾：

- 關閉新運動流程，恢復ManualDiagnostic計算輸出模式。
- 可回滾Phase 3而保留前兩Phase。

禁止跨越：

- 不通電氣壓。
- 不啟用CompetitionAuto。
- 不把endpoint reachable視為path safe。

### Phase 4：DO1／DO2氣壓狀態機

依賴：

- Phase 3。
- DO編號及保持型閥行為已確認。
- 脈衝與等待時間完成硬體驗證。

預計檔案：

- 新 `PneumaticController.h/.cpp`
- `RobotController.h/.cpp`
- `BilliardConfig.h/.cpp`
- `BilliardApp.h/.cpp`
- mock digital-output tests

驗收：

- DO1、DO2永不同時ON。
- DO1完成後OFF。
- DO2完成收回後OFF。
- DO2 OFF前禁止回拍照點。
- 逾時或錯誤轉安全狀態。
- 以mock驗證完整時序。

回滾：

- 設定中停用automatic pneumatic。
- 返回人工氣壓操作。

禁止跨越：

- 未量測時間不得通電。
- 第一輪整合測試不連續擊球。

### Phase 5：CompetitionAuto、H按鈕與視覺週期

依賴：

- Phase 1至4。
- 3幀穩定容差已校正。
- 舊影像清除策略完成整合測試。
- 自動擊發前必須有獨立安全閘門，禁止執行Algorithm目前的「強制開火」不安全候選。

預計檔案：

- `BilliardApp.h/.cpp`
- `SocketClient.h/.cpp`
- 新 `StartTrigger.h/.cpp`
- 新 `CompetitionStateMachine.h/.cpp`
- StableFrameValidator與CompetitionAuto整合相關檔案

驗收：

- H按下沿只觸發一回合。
- 長按不重複觸發。
- 等待對手期間持續排空舊影像。
- H後只接受新3幀。
- 視覺失敗可重試。
- HRSDK警報進FaultLatched。
- 警報後必須新H重新開始。

回滾：

- 關閉CompetitionAuto，使用ManualDiagnostic。

禁止跨越：

- 不允許不安全候選因比賽自動化而強制執行。
- 不使用警報前的舊計畫或舊影像。

### Phase 6：低速實機驗證與參數校正

依賴：

- 前述Phase通過離線及mock測試。
- 安全環境、急停、人工監看。

涉及檔案：

- `BilliardConfig.h/.cpp`
- 測試與驗證紀錄文件
- 必要的診斷程式，不直接改策略

驗收順序：

1. 不連氣壓的計算與reachable診斷。
2. 低速、無氣壓運動。
3. RX=`-180°`、RY=`0°`基準。
4. RY依序至`-5°`。
5. RX依序至`-175°`。
6. 桌面邊界姿態與Joint PTP回程。
7. 氣壓電源不通電，程式DO時序演練。
8. 人工氣壓單球擊打。
9. 所有結果合格後才能討論自動氣壓及連續比賽。

回滾：

- 恢復上一組已驗證設定。
- 停用CompetitionAuto與automatic pneumatic。
- 返回ManualDiagnostic。

禁止跨越：

- 不跳過姿態增量測試。
- 不直接從未驗證姿態進入自動連續擊球。

### 後續獨立策略工作

`SafetyShotPlanner` 必須與目前直球／反彈策略分離。九號球安全球必須先碰最低號球；是否要求碰庫由比賽規則設定，目前訪談結論為不強制。此工作不屬於Phase 1，且需要獨立規格。

## 10. 未決事項

### Needs Documentation

| 項目 | 阻擋階段 | 說明 |
|---|---|---|
| HRSDK Euler旋轉順序 | Phase 2/3 | 禁止自行假設 |
| HRSDK Euler角度範圍 | Phase 2/3 | 包含RX及RZ邊界 |
| HRSDK奇異點規則 | Phase 2/3 | 包含腕部組態 |
| `-180°/+180°`等價姿態行為 | Phase 2/3 | 禁止自動wrap |
| `motion_check_lin`完整保證範圍 | Phase 3 | 不等同碰撞檢查 |

### Needs Hardware Validation

| 項目 | 阻擋階段 | 說明 |
|---|---|---|
| `STRIKE_TCP_Z_MM` | Phase 3/6 | 使用者依實際調整 |
| 安全接近Z | Phase 3/6 | 短LIN起點 |
| 3幀穩定容差 | Phase 5 | 單位mm |
| 球路安全餘量 | 正式策略／自動模式 | 單位mm |
| 庫邊安全餘量 | 後續反彈重構 | 單位mm |
| Base 0固定桌面幾何 | 後續反彈重構 | 六段有效庫邊與袋口 |
| DO1／DO2脈衝 | Phase 4 | 命令時間 |
| 氣缸完成等待 | Phase 4 | 機構時間 |
| 全桌Joint PTP回CAMERA_JOINT | Phase 3/6 | 連桿與工具掃掠 |
| RX／RY實際擊球效果 | Phase 6 | 高度、跳球、桌面間隙 |
| RX小於-180° | 未排程 | 未實測前禁用 |

缺少上述值時不得填猜測值。任何需要該值的production路徑必須明確回報 `ConfigurationMissing` 或維持功能停用。

## 11. 實機驗證計畫

### 11.1 前置條件

- Tool 1與Base 0已設定並讀回確認。
- 正式姿態轉換已有文件依據。
- 急停可立即使用。
- 速度使用低速設定。
- 氣壓電源保持斷電，除非進入人工單球擊球步驟。
- 桌面、球桿、法蘭及關節掃掠區有人工監看。

### 11.2 記錄項目

- 計畫Pose與實際Pose。
- 實際關節角。
- reachable與motion_check_lin結果。
- 每段動作SDK code、時間與警報。
- Tool X實際方向。
- RX／RY對擊球高度及法蘭位置的影響。
- PTP回程最小桌面與球桿間隙。
- 單球擊打的方向、接觸高度、跳球與桌面風險。

### 11.3 通過條件

- 無未預期翻腕。
- 無法蘭、球桿或連桿碰撞。
- LIN檢查與實際短LIN行為一致。
- RZ保持母球到aimTarget方向。
- 固定strike TCP XYZ不因候選搜尋被修改。
- RY範圍內沒有不可接受的跳球或桌面接觸。

## 12. 回滾策略

1. 每個Phase獨立提交，不混合跨Phase功能。
2. 每個Phase先通過離線測試才合併下一Phase。
3. 以整個Phase提交為最小回滾單位。
4. 安全設定使用feature flag或模式切換：
   - `CompetitionAuto`可退回`ManualDiagnostic`。
   - automatic pneumatic可退回人工控制。
   - 實際運動可退回計算及診斷輸出。
5. 不得使用`git reset --hard`覆寫使用者資料。
6. 不得刪除或提交`History/`、`debug_frame.png`、`黑白棋盤.docx`。
7. `src/test_cueball.cpp`的使用者既有輸出換行不得被無關修改覆寫。

## 13. 全系統驗收標準

- 每個Phase符合自身驗收條件。
- C++核心明確使用C++17。
- Sentinel只存在wire protocol。
- 所有幾何失敗都能傳遞到動作閘門。
- 有效輸入下Algorithm策略回歸相容。
- 不安全或Invalid候選永不觸發動作。
- 姿態、運動、氣壓與自動比賽各自具有明確狀態及錯誤處理。
- 未校正值不會被預設猜測值取代。
- HRSDK、氣壓與實機測試只在對應Phase授權後執行。
