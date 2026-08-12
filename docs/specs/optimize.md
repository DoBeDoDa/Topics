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
MotionPlanner
        ↓
RobotController
        ↓
BilliardApp

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
全系統設定與版本化參數	下一個：做一致性/安全性審查；不要隨便改已被 frozen planning 使用的 contract

6	BilliardPhysics.h/.cpp	Ghost、碰撞、袋口、庫邊、反射幾何
已完成，Freeze；只做 integration smoke check

7	Algorithm.h/.cpp
產生 Direct/Kick、評分、排序、選打法	已完成，Freeze；只做 integration smoke check

8	MotionPlanner.h/.cpp
把「這球怎麼打」轉成機械手臂應到的姿態與運動計畫	下一個重要的大模組

9	RobotController.h/.cpp
HRSDK、Tool/Base、reachability、LIN/PTP、DO 等硬體介面	MotionPlanner contract 穩定後正式審查

10	BilliardApp.h/.cpp
把 Socket→規劃→Motion→Robot 整個 shot cycle 串起來	最後做 orchestration review

11	main.cpp
啟動與最外層組裝	最小化審查，不塞業務邏輯

12	tests/*
對完成後的 production contract 做 regression	最後統一 migration / 補 coverage / full regression
