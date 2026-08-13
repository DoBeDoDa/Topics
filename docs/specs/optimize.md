輸入 / 狀態
SocketClient          ✅ Freeze
VisionDataParser      ✅ Freeze
TableState            ✅ Freeze
        ↓
TargetSelector        ✅ Freeze
        ↓
BilliardPhysics       ✅ Freeze
        ↓
Algorithm             ✅ Freeze
        ↓
MotionPlanner         ✅ Freeze
        ↓
RobotController       ✅ Freeze
        ↓
BilliardApp           ← 現在

0	MathUtils.h/.cpp
共用純數學工具	已完成，Freeze，不回頭改

1	SocketClient.h/.cpp
從 Python/網路把一輪資料可靠收進來	已完成，Freeze；只做 integration smoke check

2	VisionDataParser.h/.cpp
把收到的 CSV/32 值轉成 C++ 可用資料	已完成，Freeze；只做 integration smoke check

3	TableState.h/.cpp

或實際 state 實作	把單次輸入整理成可信的球桌狀態	已完成，Freeze（含ReceiveEventFactory/ThreeEventStability）；只做 integration smoke check

4	TargetSelector.h/.cpp

從桌上球選出最低號合法目標球	已完成，Freeze；只做 integration smoke check

5	BilliardConfig.h/.cpp
全系統設定與版本化參數	已完成，Freeze；VISION_OBSERVATION_BOUNDS/TABLE_GEOMETRY保持nullopt待實測

6	BilliardPhysics.h/.cpp	Ghost、碰撞、袋口、庫邊、反射幾何
已完成，Freeze；只做 integration smoke check

7	Algorithm.h/.cpp
產生 Direct/Kick、評分、排序、選打法	已完成，Freeze；只做 integration smoke check

8	MotionPlanner.h/.cpp
把「這球怎麼打」轉成機械手臂應到的姿態與運動計畫	已完成，Freeze；A/B總搜尋上限10（對稱網格實際最多9組），0.1°由stepADeg/stepBDeg設定

9	RobotController.h/.cpp
HRSDK、Tool/Base、reachability、LIN/PTP、DO 等硬體介面	已完成，Freeze；3項修正（waitForMotion兩階段、timeout latch、connect() guard）已實作並獨立驗證

10	BilliardApp.h/.cpp
把 Socket→規劃→Motion→Robot 整個 shot cycle 串起來	下一個從這裡開始重新審查（orchestration review）

11	main.cpp
啟動與最外層組裝	最小化審查，不塞業務邏輯

12	tests/*
對完成後的 production contract 做 regression	最後統一 migration / 補 coverage / full regression

## 下一步：RobotController module audit

主審查檔案（本輪若需要修改，也只先考慮這兩個）：

- `src/RobotController.h`
- `src/RobotController.cpp`

唯讀核對 contract／caller／測試：

- `src/MotionPlanner.h/.cpp`：`ExecutionPlan`、姿態與路徑檢查契約。
- `src/BilliardConfig.h/.cpp`：Tool1、Base0、ABC mapping、timeout、DO與實機授權設定。
- `src/BilliardApp.h/.cpp`：preflight、Push/Pull順序、actual-pose safe lift與錯誤傳播。
- `src/MathUtils.h/.cpp`、`include/HRSDK.h`：確認角度轉換及HRSDK API signature沒有第二套解讀。
- `tests/p2_03_real_adapter_tests.cpp`、`tests/p2_02_execution_state_machine_tests.cpp`：只讀現有安全回歸；tests migration留到最後統一處理。

建議審查順序：

1. 連線／斷線、handle與`unknownUnsafeLatched`生命週期。
2. 正式動作前Tool1／Base0設定及readback確認。
3. ABC↔HRSDK RX/RY/RZ唯一mapping，確認Push/Pull不會重複加180°。
4. `motion_reachable`、`motion_check_lin`、PTP/LIN與timeout/abort的fail-closed邊界。
5. DO1/DO2互斥、readback、Push/Pull pulse順序及UnknownUnsafe／ManualRecovery傳播。
6. 擊球後actual pose讀回，以及只保留actual X/Y/A/B/C、沿核准Base0 +Z做LIN safe lift。
7. 跨模組追蹤：RealHardware A/B搜尋需要hardware-backed checks；責任應由RobotController提供安全adapter能力、BilliardApp負責注入，不在RobotController建立第二套MotionPlanner或自行換ShotPlan。
