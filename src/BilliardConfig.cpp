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

namespace BilliardConfig {

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
const int NORMAL_SPEED_RATIO = 20;

// [人工設定 / 實機確認]
// 使用控制器中的 Tool 編號。
// 目前使用 Tool 1，且 Tool1 TCP 應設定在球桿尖端。
// 若控制器 Tool 設定改變，此值必須同步修改。
const int TOOL_NUMBER = 1;

// [人工設定 / 實機確認]
// 使用的 Robot Base 編號。
// 目前 Python 輸出的 XY 與 Robot motion 都以 Base0 為共同座標系。
// 若改 Base，Python 校正也必須重新建立，不能只改這個數字。
const int BASE_NUMBER = 0;

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
const double MAX_REACH_RADIUS_MM = 850.0;


// ============================================================
// 4. Vision / 32-value 外部資料契約
// ============================================================


// 單筆 newline-delimited 32-value CSV 允許的最大 byte 數。
// 需根據實際 Python 格式、數字小數位數及合理上限核准後填入。
// 這是防止異常超長 frame 的安全限制，不是座標值限制。
const std::optional<std::size_t> VISION_MAX_FRAME_BYTES = 1024;



// C++ 等待一筆完整 newline frame 的最長時間，單位 ms。
// 需要根據 Python 實際 frame 傳送頻率及網路狀況測試後決定。
// 太短會誤判 timeout；太長會讓失效連線反應太慢。
const std::optional<unsigned long> VISION_RECEIVE_TIMEOUT_MS = 2000;

// [需實測 / 標定]
// Python 傳入 Robot Base0 XY 時允許的合法觀測範圍。
// 這不是球桌 playable region，而是「合理可能收到的 Base0 XY」。
// 需要根據實際相機視野＋Base0 校正範圍取得。
// 尚未核准前保持 nullopt。
const std::optional<AxisAlignedBounds2D>
    VISION_OBSERVATION_BOUNDS =
        AxisAlignedBounds2D{
            -750,
            750,
            150,
            1000
        };


// ============================================================
// 5. 三幀穩定性設定
// ============================================================


// 同一顆球在連續三次有效偵測中，
// 每一幀相對三幀中位數允許的最大位置偏差，單位 mm。
// 建議先讓球完全靜止，記錄 20～30 次 Vision Base0 XY 抖動，
// 再依真實 noise 決定，而不是猜 1 mm、5 mm 等值。
const std::optional<double>
    STABLE_FRAME_TOLERANCE_MM = 7.0;


// 六個袋口在三次偵測中允許的最大位置偏差，單位 mm。
// 袋口理論上固定，因此可利用實際 YOLO + Homography 抖動資料標定。
// 同樣不可直接猜值。
const std::optional<double>
    POCKET_STABILITY_TOLERANCE_MM = 7.0;


// 相鄰兩筆有效 ReceiveEvent 最大允許時間間隔，單位 ms。
// 應根據相機 FPS、YOLO inference 時間與 socket 實際傳送週期決定。
// 超過此值代表三筆資料時間距離太遠，不應被視為同一次穩定觀測。
const std::optional<unsigned long>
    MAX_INTER_FRAME_INTERVAL_MS = 500;


// ============================================================
// 6. 球桌 / 袋口 / 庫邊完整幾何設定
// ============================================================
const std::optional<TableGeometryConfig>
    TABLE_GEOMETRY =
        TableGeometryConfig{

            // 1. 球桌幾何標定版本
            "REPLACE_TABLE_GEOMETRY_REVISION",

            // 2. 球桌實體 playing-surface 邊界，Robot Base0 XY
            // 注意：這不是球心可移動邊界。
            // 程式之後會自動向內縮 ballRadiusMm。
            AxisAlignedBounds2D{
                -750.0,   // minX
                 750.0,   // maxX
                 150.0,   // minY
                1000.0    // maxY
            },

            // 3. 球半徑 mm
            24.76,

            // 4. 球直徑 mm
            49.52,

            // 5. 碰撞額外安全距離 mm
            3.0,

            // 6. 六段實體庫邊
            // segment端點不在此寫死，改由PocketId於每輪Vision週期查
            // StableTableState.pockets即時取得（見BilliardPhysics::resolveTableGeometry）。
            std::array<PhysicalRailConfig, 6>{{

                // Rail1：P1 左下角 → P2 下中袋
                PhysicalRailConfig{
                    RailId::Rail1,
                    PocketId::Pocket1,
                    PocketId::Pocket2,
                    Vector2D{0.0, 1.0},
                    35.0,   // startExclusionMm（須≥ballRadiusMm，否則corner袋口端點會落在playableRegion外）
                    35.0,   // endExclusionMm
                    0.0     // cushionInsetMm（非0會讓checkEffectiveRailForReflection必定失敗，見P1-xx討論）
                },

                // Rail2：P2 下中袋 → P3 右下角
                PhysicalRailConfig{
                    RailId::Rail2,
                    PocketId::Pocket2,
                    PocketId::Pocket3,
                    Vector2D{0.0, 1.0},
                    35.0,
                    35.0,
                    0.0
                },

                // Rail3：P3 右下角 → P4 右上角
                PhysicalRailConfig{
                    RailId::Rail3,
                    PocketId::Pocket3,
                    PocketId::Pocket4,
                    Vector2D{-1.0, 0.0},
                    35.0,
                    35.0,
                    0.0
                },

                // Rail4：P4 右上角 → P5 上中袋
                PhysicalRailConfig{
                    RailId::Rail4,
                    PocketId::Pocket4,
                    PocketId::Pocket5,
                    Vector2D{0.0, -1.0},
                    35.0,
                    35.0,
                    0.0
                },

                // Rail5：P5 上中袋 → P6 左上角
                PhysicalRailConfig{
                    RailId::Rail5,
                    PocketId::Pocket5,
                    PocketId::Pocket6,
                    Vector2D{0.0, -1.0},
                    35.0,
                    35.0,
                    0.0
                },

                // Rail6：P6 左上角 → P1 左下角
                PhysicalRailConfig{
                    RailId::Rail6,
                    PocketId::Pocket6,
                    PocketId::Pocket1,
                    Vector2D{1.0, 0.0},
                    35.0,
                    35.0,
                    0.0
                }

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
    KICK_GEOMETRY = std::nullopt;

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
    SCORING_CONFIG = std::nullopt;


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
    BASE0_PLANAR_CALIBRATION_REVISION = std::nullopt;


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
// 11. 舊 Camera Compensation 參數
// ============================================================

// [Legacy / 正式 Phase 1 不應使用]
// Python 現在負責：
// pixel → lens correction → homography/calibration → Robot Base0 XY。
//
// 因此正式 C++ 不得再使用下面這些數值做第二次 XY compensation。
// 如果舊 test_cueball.cpp 還引用，只能視為 legacy/test code。
// 不應為現在 Phase 1 Vision integration 再人工調整它們。

const double CAMERA_OFFSET_X_MM = 0.0;
const double CAMERA_OFFSET_Y_MM = 0.0;

const double CAMERA_REFERENCE_X_MM = -400.0;
const double CAMERA_REFERENCE_Y_MM = 600.0;

const double CAMERA_COMPENSATION_KX = 0.0;
const double CAMERA_COMPENSATION_KY = 0.0;


// ============================================================
// 12. Robot / Camera 動作等待時間
// ============================================================

// [需實測]
// Robot 到達 CameraPose 並停止後，等待相機與畫面穩定的時間。
// 800 ms 應透過實際影像測試確認：
// 手臂停止後相機震動、曝光、自動對焦等是否已穩定。
const unsigned long CAMERA_SETTLE_MS = 800;

// [需實測]
// Robot 到 Transit pose 後的額外穩定等待時間。
// 應依實際手臂停止震動時間確認。
const unsigned long TRANSIT_SETTLE_MS = 500;

// [人工設定 / 安全參數]
// 單次 Robot motion 最長允許等待時間。
// 超過即判定 timeout。
// 必須大於正常最慢動作時間，但不可無限等待。
const unsigned long MOTION_TIMEOUT_MS = 60000;

// [人工設定]
// 查詢 Robot motion state 的 polling 間隔。
// 太小會增加控制器通訊負擔；太大會降低停止反應速度。
const unsigned long MOTION_POLL_INTERVAL_MS = 50;


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
    0.0,
    -11.049,
    28.921,
    0.0,
    -15.574,
    -90.0
};


// ============================================================
// 14. Transit Pose
// ============================================================

// [需人工教導 / 實機確認]
// CameraPose 與擊球區域之間使用的中繼 Joint pose。
// 必須確認整條移動路徑不撞球桌、相機、氣管或其他設備。
// 不可因為單一 pose reachable 就宣稱 PTP path 安全。
const std::array<double, 6> TRANSIT_JOINT = {
    -5.0,
    -53.0,
    8.0,
    -3.62,
    -46.497,
    0.0
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
    -216.0,

    // [需實測]
    // 正式擊球前使用的安全／預備 Z 高度。
    // 名稱與 Phase 2 真正「post-strike safe lift Z」要避免混淆。
    // 必須確認 Base0 +Z 的實體方向與安全上方後才能正式啟用。
    -160.0,

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
    -140.0,

    // [需人工確認]
    // 測試模式的安全高度。
    -150.0,

    // [需人工確認]
    // 測試 Robot 姿態 RX。
    -10.0,

    // [需人工確認]
    // 測試 Robot 姿態 RY / tilt。
    0.0,

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
const std::optional<MotionPlanningConfig> MOTION_PLANNING_CONFIG = std::nullopt;
const ExecutionPolicyMode PRODUCTION_RUNTIME_MODE = ExecutionPolicyMode::PlanningTest;

// ============================================================
// 18. P2-03 Real Hardware Authorization / Calibration
// ============================================================

// ABC↔HRSDK RX/RY/RZ、唯一Tool1/Base0 revision、Base0 +Z安全方向與
// DO1/DO2 timing尚未完成受控實機核准；不得以測試值代替production值。
const std::optional<RealHardwareExecutionConfig>
    REAL_HARDWARE_EXECUTION_CONFIG = std::nullopt;

}
