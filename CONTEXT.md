# Robot Billiards System

本文件統一撞球視覺、規劃與 HIWIN 執行流程的專案用語。它只定義領域語言；實作狀態、參數與操作方式請見 `docs/project-overview.md`。

## Language

### Frames and calibration

**Base0**:
HIWIN 控制器中的共同機器人基準座標系；正式 32 值資料的 X、Y 與機械手臂目標都以此座標系表達。
_Avoid_: Robot frame、世界座標、桌面座標

**Tool1**:
正式擊球使用的工具座標系，其原點應是球桿尖端 TCP。
_Avoid_: 球桿座標、Cue tool

**Tool2**:
獨立 RGB→Base0 校正實驗使用的工具座標系；目前約定其原點與 RGB optical center 重合、軸向與 RGB optical frame 對齊。
_Avoid_: 相機工具、Camera TCP

**RGB Optical Frame**:
Gemini 2 XL 彩色鏡頭的右手光學座標系，`+X` 向影像右、`+Y` 向影像下、`+Z` 朝鏡頭前方場景。
_Avoid_: Camera frame、ROS sensor frame

**Base0 Planar Calibration**:
把影像觀測轉成 Base0 平面 X、Y 毫米的版本化校正定義；目前正式外部契約由 Python 擁有此轉換。
_Avoid_: Homography、相機補償、桌面轉換

**RGB-only Calibration**:
只使用選定 RGB profile 的內參與畸變資料建立 RGB 像素射線的校正方式，不啟用或依賴 Depth stream、D2C 或深度外參。
_Avoid_: RGB-D calibration、Depth-assisted calibration

**Ground Truth**:
以獨立方式量得、用來比較 RGB→Base0 計算結果的球心 Base0 座標。
_Avoid_: 校正點、預測值、reference guess

### Vision and shot planning

**Vision Frame**:
一筆 newline 結尾、恰含 32 個數值的 Python→C++ 觀測資料，依序包含 1～9 號球、母球與六個袋口的 Base0 平面 X、Y。
_Avoid_: Packet、YOLO frame、image frame

**ReceiveEvent**:
C++ 接收邊界為一筆完整 Vision Frame 加上的本地事件編號、接收時間、連線身分與 shot-cycle 身分。
_Avoid_: Camera frame ID、曝光序號、sender timestamp

**Shot Cycle**:
從單次 `StartRequested` 開始，經拍照、穩定觀測、規劃與允許的執行後回到等待狀態的一次完整生命週期。
_Avoid_: Loop、round、continuous mode

**StableTableState**:
同一 shot cycle 內三個合法 ReceiveEvent 通過球與袋口一致性檢查後形成的穩定桌面狀態。
_Avoid_: TableState、single-frame state、latest frame

**Shot Brain**:
由既有 `TargetSelector`、`Algorithm` 與 `BilliardPhysics` 共同承擔的純規劃能力。
_Avoid_: ShotBrain.cpp、第二套演算法、AI planner

**DirectPot**:
母球先碰目標球、目標球不碰庫直接進入指定袋口的候選球路。
_Avoid_: Direct shot、直球

**KickPot**:
母球先碰一次庫邊，再碰目標球並使目標球進袋的候選球路。
_Avoid_: Bank shot、反彈球、顆星

**ShotPlan**:
通過目標資格、幾何、碰撞、袋口與評分規則後，可供 Phase 2 消費的唯一成功規劃值。
_Avoid_: Candidate、fallback plan、diagnostic plan

**NoPlan**:
合法完成規劃但沒有可接受候選的具名結果，不含假座標、假姿態或退而求其次的成功值。
_Avoid_: Error、default shot、fallback shot

### Robot execution and safety

**ExecutionPlan**:
由有效 ShotPlan、版本化運動校正與執行政策建立的完整姿態、路徑與安全前置條件集合。
_Avoid_: MotionProfile、pose list、robot command

**PlanningTest**:
只允許計算、診斷與 fake/offline 驗證，不授權真實機械手臂或氣動擊球的執行模式。
_Avoid_: Dry run、simulation mode、safe mode

**RealHardware**:
只有在所有硬體校正與安全驗收通過後，才可使用真實 HRSDK 與氣動輸出的執行模式。
_Avoid_: Production、live mode、auto mode

**Safe Lift**:
擊球及氣動完成後，從重新讀取的 actual pose 保持 X、Y、A、B、C 不變，只沿已驗證安全的 Base0 `+Z` 執行的第一段 LIN 上升。
_Avoid_: Retract、return motion、planned lift

**UnknownUnsafe**:
無法確認機械手臂、通訊或氣動輸出實體安全狀態的終端結果；進入後禁止任何後續移動。
_Avoid_: Generic error、timeout、recoverable failure
