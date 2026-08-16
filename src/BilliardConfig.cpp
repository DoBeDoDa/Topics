// 定義機械手臂連線、視覺連線、球桌幾何、演算法、Tool/Base、
// 關節點位及正式／測試動作參數。

//P1 = (-619.212, 246.817);
//P2 = (10.788, 246.817);
//P3 = (640.788, 246.817);
//P4 = (640.788, 876.817);
//P5 = (10.788, 876.817);
//P6 = (-619.212, 876.817);
//擊球z=218;

#include "BilliardConfig.h"

#include <algorithm>
#include <cmath>

namespace BilliardConfig {

bool StandbyJointReference::isValid() const noexcept
{
    return calibrationRevision.has_value() && !calibrationRevision->empty() &&
        std::all_of(jointDeg.begin(), jointDeg.end(), [](double value) {
            return std::isfinite(value);
        });
}

bool ShotCycleTimingConfig::isValid() const noexcept
{
    return shotDeadlineMs > 0 && minimumExecutionReserveMs > 0 &&
        planningRetryCutoffMs > 0 && shotDeadlineMs >= planningRetryCutoffMs &&
        (shotDeadlineMs - planningRetryCutoffMs) >= minimumExecutionReserveMs;
}

// [人工設定]：部署環境改變時由人修改
// [需實測]  ：必須由實際球桌、相機或機械手臂量測/標定後才能填
// [固定規格]：通常不需要改，除非硬體或協定改變
// [暫勿啟用]：目前規格尚未核准，保持 nullopt
// [Legacy]  ：舊流程遺留，目前正式架構不應使用

// ============================================================
// 1. 連線設定
// ============================================================

// [人工設定]
// HIWIN 機械手臂控制器 IP。
// 換控制器、網段或控制器 IP 時才需要修改。
// 實際值應由控制器網路設定確認。
const char* const ARM_IP = "192.168.0.1";

// [人工設定]
// Python 視覺程式所在電腦的 IP。
// 127.0.0.1 表示 Python 與 C++ 在同一台電腦。
// 若未來 Python 放到另一台電腦，需改成那台電腦的 LAN IP。
const char* const VISION_SERVER_IP = "127.0.0.1";

// [人工設定]
// Python → C++ 傳送 32 值 CSV 所使用的 TCP Port。
// Python 與 C++ 必須完全一致。
// Port 本身不需要量測，只需要雙方設定一致。
const int VISION_SERVER_PORT = 12345;

// [人工設定 / Legacy或其他校正用途]
// 校正服務使用的 Port。
// 如果目前正式流程沒有另外啟動 calibration server，
// 不要誤認為 Phase 1 的 32-value socket port。
const int CALIBRATION_SERVER_PORT = 12347;


// ============================================================
// 2. Robot 基本控制設定
// ============================================================

// [人工設定]
// 機械手臂一般運動速度比例。
// 20 通常代表控制器速度比例 20%，實際語意需依 HRSDK/控制器確認。
// 真機初次測試建議使用低速；提高前需要確認安全。
const int NORMAL_SPEED_RATIO = 100;

// [人工設定 / 實機確認]
// 使用控制器中的 Tool 編號。
// 若控制器 Tool 設定改變，此值必須同步修改。
const int TOOL_NUMBER = 1;

// [人工設定 / 實機確認]
// 使用的 Robot Base 編號。
// 目前 Python 輸出的 XY 與 Robot motion 都以 Base0 為共同座標系。
// 若改 Base，Python 校正也必須重新建立，不能只改這個數字。
const int BASE_NUMBER = 0;

const int START_BUTTON_DI_INDEX = 1;

// [人工設定 / Phase 2 硬體確認]
// 氣動擊球使用的 Digital Output 編號。
// 實際 DO wiring、DO1/DO2 定義仍需依控制器接線確認。
// Phase 1 演算法本身不應依賴此值。
const int PNEUMATIC_OUTPUT = 1;


// ============================================================
// 3. 撞球基本尺寸與舊動作參數
// ============================================================

// [需實測]
// 實際使用撞球的直徑，單位 mm。
// 應使用游標卡尺量測實際球，而不是單純猜規格。
// Phase 1 幾何真正權威應以 TableGeometryConfig 中
// ballRadiusMm / ballDiameterMm 為準，避免出現兩套來源。
const double BALL_DIAMETER_MM = 49.52;

// [需實測]
// 機械手臂 RZ / C 軸瞄準角與實際球桿 forward direction
// 之間的固定角度偏移。
// 必須在 Tool1 forward axis 與控制器姿態定義確認後才能標定。
// 若目前尚未做實機方向校正，不應自行猜值。
const double YAW_OFFSET_DEG = 0.0;

// [人工設定]
// 當兩個幾何點太靠近時，拒絕 normalize / 建立擊球方向的最小距離。
// 主要用於避免接近零長度向量造成不穩定。
// 此值屬演算法安全門檻，不是球桌尺寸。
// 修改前應確認 Math / Motion 使用位置。
const double MIN_AIM_DISTANCE_MM = 3.0;

// [需實測 / Robot規格確認]
// 允許使用的最大手臂工作半徑。
// 不應只按照理論 reach 填值；需考慮實際 Tool、姿態與安全空間。
// Phase 2 最終仍應依 motion_reachable / path check 判斷。
const double MAX_REACH_RADIUS_MM = 800.0;


// ============================================================
// 4. Vision / 32-value 外部資料契約
// ============================================================


// 單筆 newline-delimited 32-value CSV 允許的最大 byte 數。
// 需根據實際 Python 格式、數字小數位數及合理上限核准後填入。
// 這是防止異常超長 frame 的安全限制，不是座標值限制。
const std::optional<std::size_t> VISION_MAX_FRAME_BYTES = 2048;



// C++ 等待一筆完整 newline frame 的最長時間，單位 ms。
// 需要根據 Python 實際 frame 傳送頻率及網路狀況測試後決定。
// 太短會誤判 timeout；太長會讓失效連線反應太慢。
const std::optional<unsigned long> VISION_RECEIVE_TIMEOUT_MS = 10000;

// [需實測 / 標定]
// Python 傳入 Robot Base0 XY 時允許的合法觀測範圍。
// 這不是球桌 playable region，而是「合理可能收到的 Base0 XY」。
// 需要根據實際相機視野＋Base0 校正範圍取得。
// 尚未核准前保持 nullopt。
const std::optional<AxisAlignedBounds2D> VISION_OBSERVATION_BOUNDS =
    AxisAlignedBounds2D{
        -800.0,  // minX
         800.0,  // maxX
         10.0,  // minY
        1000.0   // maxY
    };


// ============================================================
// 5. 三幀穩定性設定
// ============================================================


// 同一顆球在連續三次有效偵測中，
// 每一幀相對三幀中位數允許的最大位置偏差，單位 mm。
// 建議先讓球完全靜止，記錄 20～30 次 Vision Base0 XY 抖動，
// 再依真實 noise 決定，而不是猜 1 mm、5 mm 等值。
const std::optional<double>
    STABLE_FRAME_TOLERANCE_MM = 10.0;


// 六個袋口在三次偵測中允許的最大位置偏差，單位 mm。
// 袋口理論上固定，因此可利用實際 YOLO + Homography 抖動資料標定。
// 同樣不可直接猜值。
const std::optional<double>
    POCKET_STABILITY_TOLERANCE_MM = 10.0;


// 相鄰兩筆有效 ReceiveEvent（現在是Python累積後送出的Logical Frame，
// 不是每次YOLO都送一筆）最大允許時間間隔，單位 ms。
// [使用者2026-08-16核准放寬] 1000->5000：Python端一個Logical Frame要
// 累積多張YOLO偵測都收斂才送出（見python/capture_accumulator.py），
// YOLO本身若有閃爍/漏偵測，Logical Frame之間的間隔可能遠超過1秒，
// 原本1000ms會讓ThreeEventStability一直TimedOut重置、永遠湊不滿三幀。
// 超過此值代表三筆資料時間距離太遠，不應被視為同一次穩定觀測。
const std::optional<unsigned long>
    MAX_INTER_FRAME_INTERVAL_MS = 5000;


// ============================================================
// 6. 球桌 / 袋口 / 庫邊完整幾何設定
// ============================================================
// [已核准 2026-08] 使用者提供實測值。physicalPlayingSurface刻意與
// VISION_OBSERVATION_BOUNDS使用同一組邊界（使用者確認）。
// cushionInsetMm為保守暫定值（未實測緩衝墊壓縮量），與collisionMarginMm
// 是兩個不同用途的欄位：collisionMarginMm是泛用碰撞安全緩衝，
// cushionInsetMm只用在Kick一次碰庫反彈點計算。
const std::optional<TableGeometryConfig>
    TABLE_GEOMETRY = TableGeometryConfig{
        "table-2026-08-v1",
        AxisAlignedBounds2D{-800.0, 800.0, 10.0, 1000.0},
        24.76,  // ballRadiusMm = 49.52 / 2
        49.52,  // ballDiameterMm
        5.0,    // collisionMarginMm
        {{
            {RailId::Rail1, PocketId::Pocket1, PocketId::Pocket2, 8.0, 8.0, 3.0},
            {RailId::Rail2, PocketId::Pocket2, PocketId::Pocket3, 8.0, 8.0, 3.0},
            {RailId::Rail3, PocketId::Pocket3, PocketId::Pocket4, 8.0, 8.0, 3.0},
            {RailId::Rail4, PocketId::Pocket4, PocketId::Pocket5, 8.0, 8.0, 3.0},
            {RailId::Rail5, PocketId::Pocket5, PocketId::Pocket6, 8.0, 8.0, 3.0},
            {RailId::Rail6, PocketId::Pocket6, PocketId::Pocket1, 8.0, 8.0, 3.0}
        }}
    };

// ============================================================
// 7. 一次碰庫 Kick 幾何
// ============================================================

// [需實驗 / 標定]
// 一次母球碰庫 Kick 的限制與數值容差。
// 尚未核准前保持 nullopt。
//
// maxKickRailAngleDeg
//   [需實驗]
//   允許的最大碰庫角。
//   太極端的入射角即使理想鏡射幾何成立，實際擊球也可能不可靠。
//
// reflectionDirectionTolerance
//   [數值 / 實驗]
//   實際計算出的反射方向與理想鏡射方向允許的誤差。
//
// reflectionAngleToleranceDeg
//   [數值 / 實驗]
//   入射角與反射角允許的角度誤差。
//
const std::optional<KickGeometryConfig>
    KICK_GEOMETRY = KickGeometryConfig{
        /* maxKickRailAngleDeg */          65.0,
        /* reflectionDirectionTolerance */ 2.0,
        /* reflectionAngleToleranceDeg */  1e-4
    };

// ============================================================
// 8. Phase 1 評分設定
// ============================================================

// [目前已定義的實驗權重 / 尚需人工重新核准]
// 五項成本的初始實驗權重（pocketEntryAngle項目已隨舊Pocket model移除，
// 其餘raw weight沿用先前數值，未重新分配，總和非1.00亦可，
// 實際排名使用raw/rawSum正規化）。
//
// 0.30 kickPenalty
//   是否使用一次碰庫。
//   Direct = 0，Kick = 1。
//   使 Direct 具有較強但非絕對的偏好。
//
// 0.30 cuttingAngle
//   切球角成本。
//   切球角越大通常成本越高。
//
// 0.20 totalDistance
//   母球路徑 + 目標球路徑總距離成本。
//   路徑越長，正規化成本越高。
//
// 0.10 clearanceRisk
//   與障礙球的安全淨空成本。
//   越接近障礙物，成本越高。
//
// 0.05 kickRailAngleRisk
//   Kick 碰庫角風險。
//   Direct 對此項通常沒有 Kick 成本。
//
const ScoringWeights INITIAL_EXPERIMENTAL_SCORING_WEIGHTS = {
    0.35,  // Kick penalty：偏好不碰庫的 Direct
    0.30,  // Cutting angle：切球角成本
    0.20,  // Total distance：總路徑長度成本
    0.10,  // Clearance：障礙球安全淨空成本
    0.05,  // Kick rail angle：碰庫角風險
    };


// [需人工核准]
// 完整 ScoringConfig。
// 權重已經有初始實驗值，但其他正規化範圍尚未核准。
//
// effectiveWeightSumTolerance
//   [數值容差]
//   六個 effective weights 正規化後總和接近 1.0 的允許誤差。
//
// maxCutAngleDeg
//   [需實驗]
//   切球角正規化所使用的上限。
//   不是單純「超過就一定不能打」，須依現有 Scoring Spec 語意使用。
//
// minDistanceMm / maxDistanceMm
//   [需依球桌尺寸設定]
//   路徑距離成本的正規化範圍。
//   應由實際球桌尺度與可行 path 分布決定。
//
// preferredClearanceMm
//   [需實驗]
//   認為較理想的障礙球安全淨空距離。
//   需考慮球直徑、collision margin、視覺誤差。
//
// tieEpsilon
//   [人工設定 / 數值容差]
//   兩候選總成本接近多少時視為近似平手，
//   再進入 deterministic tie-break。
//   不應設太大，否則會改變原本 scoring 意義。
//
// planningMode
//   [人工設定]
//   PotOnly：正常 production 模式。
//       沒有進球候選 → NoPlan(NoPotCandidate)。
//
//   ManualResearch：研究模式。
//       只有所有 Pot 都失敗後，才允許嘗試 LegalContact。
//       不等於 SafetyShot；real hardware 預設不可直接執行。
//
const std::optional<ScoringConfig>
    SCORING_CONFIG = ScoringConfig{
        /* rawWeights */
        INITIAL_EXPERIMENTAL_SCORING_WEIGHTS,

        /* effectiveWeightSumTolerance */
        1e-9,

        /* maxCutAngleDeg */
        90.0,

        /* minDistanceMm */
        0.0,

        /* maxDistanceMm */
        1400.0,  // TODO: 依實際 P1~P6 球桌尺度與一次 Kick 最大合理總路徑設定

        /* preferredClearanceMm */
        70.0,  // TODO: 必須 > ballDiameterMm + collisionMarginMm

        /* tieEpsilon */
        1e-9,

        /* planningMode */
        PlanningMode::ManualResearch
    };


// ============================================================
// 9. Base0 平面標定版本
// ============================================================

// [需人工設定 / 標定]
// Python pixel→Base0 XY 所使用的版本化校正 ID。
// 必須與目前實際部署的 Python calibration 相符。
//
// 例如未來可以是：
// "base0-table-2026-08-v1"
//
// 但名稱必須由你實際建立的標定版本決定，不能現在隨便填。
// 32-value wire 本身不帶 revision，所以這是部署管理資訊。
const std::optional<std::string>
    BASE0_PLANAR_CALIBRATION_REVISION = "base0-planar-v1";


// ============================================================
// 10. Shot Brain 組合設定
// ============================================================

// [程式組合值，不直接量測]
// 將 Base0 calibration revision、Kick geometry、Scoring config
// 集中交給 Algorithm / Shot Brain。
// 任一必要值缺少時應 fail closed，不應自行 fallback。
const BrainConfig BRAIN_CONFIG = {
    BASE0_PLANAR_CALIBRATION_REVISION,
    KICK_GEOMETRY,
    SCORING_CONFIG
};


// ============================================================
// 12. Robot / Camera 動作等待時間
// ============================================================

// [需實測]
// Robot 到達 CameraPose 並停止後，等待相機與畫面穩定的時間。
// 800 ms 應透過實際影像測試確認：
// 手臂停止後相機震動、曝光、自動對焦等是否已穩定。
const unsigned long CAMERA_SETTLE_MS = 1000;

// [人工設定 / 安全參數]
// Robot motion命令送出後，確認控制器離開停止狀態的最長等待時間。
const unsigned long MOTION_START_CONFIRMATION_TIMEOUT_MS = 1000;

// [人工設定 / 安全參數]
// 單次 Robot motion 最長允許等待時間。
// 超過即判定 timeout。
// 必須大於正常最慢動作時間，但不可無限等待。
const unsigned long MOTION_TIMEOUT_MS = 60000;

// [人工設定]
// 查詢 Robot motion state 的 polling 間隔。
// 太小會增加控制器通訊負擔；太大會降低停止反應速度。
const unsigned long MOTION_POLL_INTERVAL_MS = 50;

// [需實測]
// clearAlarm前確認馬達已真正斷電（get_motor_state==0）的最長等待時間。
// set_motor_state(0)回傳成功只代表指令已送出，控制器實際斷電可能有延遲；
// 若clearAlarm在馬達尚未真正OFF前送出，會被控制器拒絕（sdkCode=300，
// 即使get_alarm_code查無alarm）。逾時後仍照常嘗試clearAlarm，讓原本的
// sdkCode錯誤路徑處理（不無限等待）。500ms為初始保守值，應透過實機
// 量測set_motor_state實際斷電延遲後調整。
const unsigned long MOTOR_OFF_CONFIRMATION_TIMEOUT_MS = 500;

// 推桿後方障礙檢查：母球中心沿執行方向反方向延伸Lback=ballRadiusMm+
// BACK_OBSTACLE_EXTRA_MM的有限線段，若其他球中心到此線段的最短距離
// <= ballRadiusMm+BACK_OBSTACLE_LATERAL_MARGIN_MM就拒絕該執行方向
// （怕推桿實際伸出時先撞到後方障礙球）。這不是完整Tool掃掠體積模型，
// 之後量到實際pusherRadiusMm應改用該值取代這裡的簡化門檻。
const double BACK_OBSTACLE_EXTRA_MM = 1.0;
// [使用者2026-08-16確認] 側向margin 5mm。
const double BACK_OBSTACLE_LATERAL_MARGIN_MM = 5.0;

// [演算法研究值，非實測]
const double CUE_BALL_CONTACT_ONLY_ANGULAR_STEP_DEG = 15.0;

// [演算法研究值，非實測]
const double LEGAL_CONTACT_GRAZING_ANGULAR_STEP_DEG = 15.0;

// [演算法研究值，非實測]
const double LEGAL_CONTACT_KICK_GRAZING_ANGULAR_STEP_DEG = 15.0;


// ============================================================
// 13. CameraPose
// ============================================================

// [需人工教導 / 實機量測]
// 拍照位置的六軸 Joint angle。
// 必須由 HIWIN 教導器實際教導，確認：
//
// 1. 相機完整看見球桌。
// 2. 手臂沒有遮住球桌。
// 3. 不接近奇異點。
// 4. 從其他允許姿態移動至此位置的路徑安全。
//
// 若相機、支架、Robot Base、球桌位置改變，通常需要重新教導。
const std::array<double, 6> CAMERA_JOINT = {
    -3.792,
    -26.792,
    55.334,
    9.579,
    -27.804,
    -97.606
};

// [人工可調 / 實驗參數]
// 六軸都在此誤差內時，視為已位於CameraPose，不重送相同PTP命令。
const double CAMERA_JOINT_TOLERANCE_DEG = 0.2;

// [已核准 / 人工可調研究值]
// standby姿態專用容差，獨立於CameraPose；初始值與CAMERA_JOINT_TOLERANCE_DEG
// 相同純屬巧合（同屬controller停止震動量級），未來各自實測校正時可分開調整。
const double STANDBY_JOINT_TOLERANCE_DEG = 0.2;

// [已核准 / 人工可調研究值]
// Vision reconnect gate每次重試連線之間的等待間隔。
const unsigned long VISION_RECONNECT_POLL_INTERVAL_MS = 200;


// ============================================================
// 14. Standby（準備姿態）Pose
// ============================================================

// [已核准]
// Robot準備姿態，H/P流程PTP的唯一權威joint reference。
// CameraPose到擊球點正上方（safeApproachPose）不再經過任何中繼pose；
// 此姿態只在WaitingForStart前置確認、P人工功能，以及擊球後safe lift
// 完成時使用。
// jointDeg數值為使用者已於計畫2.1節確認。calibrationRevision由使用者於
// Phase 2核准為"standby-joint-2026-08-v1"，解除checkpoint 1的fail-closed
// 狀態；之後若重新教導準備姿態，必須同步核准新版號，不得沿用舊字串。
const StandbyJointReference STANDBY_JOINT_REFERENCE = {
    std::string{"standby-joint-2026-08-v1"},
    {0.0, 0.0, 90.0, 0.0,-27.804,-90.0}
};


// ============================================================
// 15. Production Motion Profile
// ============================================================

// [全部屬實機標定 / Phase 2]
// 正式機械手臂擊球相關參數。
// Phase 1 球路演算法測試不需要使用這組值。
const MotionProfile PRODUCTION_MOTION = {

    // [需實測]
    // 真正擊球時 Tool1 TCP / 球桿尖端所需的 Z 高度。
    // 必須依桌面高度、球半徑、球桿中心高度及 Tool1 TCP 實測。
    -220.0,

    // [需實測]
    // 正式擊球前使用的安全／預備 Z 高度。
    // 名稱與 Phase 2 真正「post-strike safe lift Z」要避免混淆。
    // 必須確認 Base0 +Z 的實體方向與安全上方後才能正式啟用。
    -180.0,

    // [需實機標定]
    // Robot pose 中對應的 RX / A 類姿態設定。
    // 不能只依數學猜測；需確認 HRSDK 姿態定義。
    0.0,

    // [需實機標定]
    // 球桿主要傾斜姿態。
    // 目前 -180 是否正確必須以實際 Tool orientation / cue direction 驗證。
    -180.0,

    // [目前應為 0]
    // 舊的擊球後退距離。
    // 目前核准流程不允許用 XY backward retreat 取代垂直 safe lift。
    0.0,

    // [需實驗]
    // 額外 standoff distance。
    // 若目前未核准，維持 0，不自行增加。
    0.0
};


// ============================================================
// 16. Test Motion Profile
// ============================================================

// [測試專用]
// test_cueball.cpp 等 Robot 測試使用。
// 不應影響正式 Phase 1 Algorithm。
// 所有值仍需在低速、安全高度、人工監看下確認。
const MotionProfile TEST_MOTION = {

    // [需人工確認]
    // 測試時使用較高的 Z，不下降到實際擊球高度。
    -180.0,

    // [需人工確認]
    // 測試模式的安全高度。
    -150.0,

    // [需人工確認]
    // 測試 Robot 姿態 RX。
    -180.0,

    // [需人工確認]
    // 測試 Robot 姿態 RY / tilt。  
    0.5,

    // [保持 0，除非另行核准]
    // 測試 backward motion。
    0.0,

    // [保持 0，除非另行核准]
    // 測試 standoff。
    0.0
};

// ============================================================
// 17. P2-01 Motion Planning Calibration
// ============================================================

// [暫勿啟用 / 需完整人工校正]
// Strike Z、ready gap、A/B核准範圍、C offset、Tool1 local +X及
// safe approach/lift尚未共同驗收前保持nullopt，MotionPlanner必須fail closed。
// strikePositionBiasMm初始值固定為0.0 mm；非零值只由人工調適。
// pullModeMinBottomDistanceMm初始研究值為300.0 mm，不是固定機械規格。
// 球桌往下方向已確認為Base0 Y-，且Robot正對球桌長邊、無平面旋轉偏移。
// [已核准 2026-08] 使用者提供實測/決定值。
// [使用者2026-08核准放寬] fixedForceEnvelope不再用距離/切球角度/反彈角度
// 限制是否執行：氣動力道固定、不可調，這些幾何量對力道是否足夠沒有意義。
// 範圍已放寬到validEnvelopeLimits允許的上限（見下方fixedForceEnvelope）。
// productionLegalContactFallbackAuthorized=true：只在所有進袋候選都試過、
// 確認沒有可行方案時，才允許退而求其次打防守安全球；
// legalContactExecutionAuthorized維持false：不允許在有進袋機會時主動選擇
// 防守球。
const std::optional<MotionPlanningConfig> MOTION_PLANNING_CONFIG = [] {
    MotionPlanningConfig config;
    config.calibrationRevision = "motion-2026-08-v1";
    config.base0PlanarCalibrationRevision = BASE0_PLANAR_CALIBRATION_REVISION;
    config.cueForwardAxisCalibrationRevision = "tool-axis-2026-08-v1";
    config.strikeZMm = -225.0;
    config.safeApproachZMm = -170.0;
    config.readyGapMm = 0.0;
    config.strikePositionBiasMm = 20.0;
    config.pullModeMinBottomDistanceMm = 300.0;
    config.a0Deg = -179.0;
    config.b0Deg = -2.0;
    // [使用者2026-08核准放寬] A軸搜尋窗5°->10°：原窗口在部分shot方向下
    // 找不到可用姿態（NoAcceptedPoseCandidate）。41*13=533候選，未超過
    // MAX_TOTAL_POSE_CANDIDATES=1000。
    config.deltaADeg = 15.0;
    config.deltaBDeg = 3.0;
    config.stepADeg = 0.5;
    config.stepBDeg = 0.5;
    config.searchOrder = PoseSearchOrder::AThenB;
    config.axisOffsetOrder = AxisOffsetOrder::LowerThenHigher;
    config.tieBreak = PoseTieBreak::FirstInApprovedSearchOrder;
    // [待實測] Tool1的C=0參考方向與Base0角度0參考方向間的固定offset；
    // 使用者確認Tool1相對法蘭無ABC安裝偏差，但這是軟體角度參考系的獨立
    // 校正，暫定0待實機量測後修正。
    config.cToolOffsetDeg = 0.0;
    config.cueForwardAxisTool = std::array<double, 3>{1.0, 0.0, 0.0};
    config.maxCueDirectionErrorDeg = 0.5;
    config.directionUnitTolerance = 1e-8;
    config.executionPolicyRevision = "real-hardware-v1";
    config.policyMode = ExecutionPolicyMode::RealHardware;
    config.legalContactExecutionAuthorized = false;
    config.productionLegalContactFallbackAuthorized = true;
    // [使用者2026-08核准放寬] 氣動力道固定、不可調，距離／切球角度／
    // 反彈角度對執行力道沒有影響，故不再用這些幾何量限制是否執行；
    // 範圍放寬到validEnvelopeLimits／FixedForceEnvelopeEvaluation::isValid()
    // 允許的上限（角度180°為validAngle上限，距離用大值取代原本武斷值），
    // 讓這組檢查對任何幾何上合法的shot都會通過。
    config.fixedForceEnvelope = FixedForceEnvelopeConfig{
        "force-envelope-2026-08-v1",
        FixedForceEnvelopeLimits{true, 0.0, 1000000.0, 180.0, std::nullopt},
        FixedForceEnvelopeLimits{true, 0.0, 1000000.0, 180.0, 180.0},
        FixedForceEnvelopeLimits{true, 0.0, 1000000.0, std::nullopt, std::nullopt},
        FixedForceEnvelopeLimits{true, 0.0, 1000000.0, std::nullopt, 180.0},
        // CueBallContactOnly：沒有目標球，切球角度／反彈角度概念不適用。
        FixedForceEnvelopeLimits{true, 0.0, 1000000.0, std::nullopt, std::nullopt}
    };
    // [使用者2026-08-16實測確認] 500ms曾經觀察到DO訊號已經送出、但推桿
    // 機構實際還沒到位，一度改成1000ms對齊安全值；但1000ms一輪氣動序列
    // （伸出脈衝+方向切換延遲+退回脈衝+機構完成等待）合計4秒，在15秒
    // 賽制deadline下太久，使用者實機重測後確認改回500ms。這兩份設定
    // 本來就必須完全一致（見RobotController.cpp
    // validateRealExecutionConfiguration的timingMatches檢查）。
    config.pneumaticTimingProfile = PneumaticTimingProfileReference{
        "pneumatic-timing-v1", 500, 500, 500};
    config.tool1ControllerCalibrationRevision = "tool1-controller-v1";
    config.railHuggingTriggerDistanceMm = 30.0;  // [使用者2026-08-16確認] 3公分
    config.railHuggingReadyGapMm = 60.0;  // 使用者提供初始值，之後會調整
    return config;
}();
const ExecutionPolicyMode PRODUCTION_RUNTIME_MODE = ExecutionPolicyMode::RealHardware;

// ============================================================
// 18. P2-03 Real Hardware Authorization / Calibration
// ============================================================

// ABC↔HRSDK RX/RY/RZ、唯一Tool1/Base0 revision、Base0 +Z安全方向與
// DO1/DO2 timing尚未完成受控實機核准；不得以測試值代替production值。
// ============================================================
// 18. P2-03 Real Hardware Authorization / Calibration
// ============================================================

// 真機 motion / DO 尚未完整驗收前保持停用。
const std::optional<RealHardwareExecutionConfig>
    REAL_HARDWARE_EXECUTION_CONFIG =
        RealHardwareExecutionConfig{

            // 1. 真機執行授權版本
            /* authorizationRevision */
            "real-hardware-v1",

            // 2. 真機總開關
            /* realHardwareExecutionEnabled */
            true,

            // 3. Robot Base 編號
            /* baseNumber */
            0,

            // 4. Tool1 編號
            /* tool1Number */
            1,

            // 5. Base0 controller calibration 版本
            // RobotController::validateRealExecutionConfiguration要求這裡
            // 跟ExecutionPlan帶的base0PlanarCalibrationRevision逐字相同，
            // 統一沿用同一顆常數避免兩處各自維護、字串再次走鐘。
            /* base0CalibrationRevision */
            BASE0_PLANAR_CALIBRATION_REVISION,

            // 6. Tool1 controller / TCP calibration 版本
            /* tool1ControllerCalibrationRevision */
            "tool1-controller-v1",

            // 7. A/B/C → HRSDK RX/RY/RZ 映射
            // [已核准] 使用者確認：只要正確選到Tool1（activateConfiguredToolAndBase
            // 已切換），HRSDK的RX/RY/RZ座標定義本身就與此codebase的A/B/C語意一致，
            // 不需換位置、不需反號、無零點偏移。
            /* angleMapping */
            HrSdkAngleMappingConfig{
                /* calibrationRevision */
                "abc-hrsdk-mapping-v1",

                /* rxRyRzSources */
                {
                    RobotAngleComponent::A,  // 已核准：Tool1下A對應RX
                    RobotAngleComponent::B,  // 已核准：Tool1下B對應RY
                    RobotAngleComponent::C   // 已核准：Tool1下C對應RZ
                },

                /* scales */
                {
                    1.0,  // 已核准：方向一致，不需反號
                    1.0,
                    1.0
                },

                /* offsetsDeg */
                {
                    0.0,  // 已核准：無零點偏移
                    0.0,
                    0.0
                }
            },

            // 8. Safe-Up 校正版本
            /* safeUpCalibrationRevision */
            "safe-up-v1",

            // 9. 執行政策要求的 Tool1 calibration
            /* requiredTool1CalibrationRevision */
            "tool1-controller-v1",

            // 10. 執行政策要求的 ABC mapping
            /* requiredAbcMappingRevision */
            "abc-hrsdk-mapping-v1",

            // 11. 執行政策要求的 Safe-Up calibration
            /* requiredSafeUpCalibrationRevision */
            "safe-up-v1",

            // 12. 是否已實機確認 Base0 +Z 是安全抬升方向
            /* base0PositiveZSafeConfirmed */
            true,

            // 13. Striker 伸出 DO
            // [已核准] 使用者已依實際接線確認：DO1為伸出。
            /* extendDoIndex */
            1,

            // 14. Striker 收回 DO
            // [已核准] 使用者已依實際接線確認：DO2為收回。
            /* retractDoIndex */
            2,

            // 15. 已核准的氣動 timing
            // [使用者2026-08-16實機重測確認] 500ms；見BilliardConfig.cpp
            // MOTION_PLANNING_CONFIG.pneumaticTimingProfile旁的完整說明。
            /* approvedTimingProfile */
            PneumaticTimingProfileReference{
                /* calibrationRevision */
                "pneumatic-timing-v1",

                /* pneumaticPulseMs */
                500,

                /* directionChangeDelayMs */
                500,

                /* mechanismCompletionWaitMs */
                500
            },

            // 16. 競賽用實體Start按鈕DI index
            // [待確認接線] 使用者稱為DI1，暫定index=1；正式比賽前務必在
            // 控制箱上實際觸發一次確認對應到這裡的index，接錯不會有編譯期
            // 錯誤，只會表現成「按鈕沒反應」或誤觸別的DI。
            /* startDigitalInputIndex */
            1
        };

// ============================================================
// 19. Shot-Cycle競賽計時
// ============================================================

// [依比賽規則 / 已核准]
// shotDeadlineMs：15000ms，超過15秒未出桿視為犯規，計時從接受H／Start瞬間開始。
// minimumExecutionReserveMs：5000ms，剩餘時間低於此值不得開始執行（Pull
// pre-extend或前往safeApproachPose）。
// planningRetryCutoffMs：10000ms，超過此值仍無方案時安全結束，不強制擊球。
const ShotCycleTimingConfig SHOT_CYCLE_TIMING = {150000, 50000, 100000};

}  // namespace BilliardConfig
