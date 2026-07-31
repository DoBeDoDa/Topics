# Phase 1：MathUtils、CameraCompensator與安全幾何重構規格

> **狀態：Superseded（僅供歷史追溯）**
> 本文件的 P1-01／P1-02 背景可用於追溯既有成果，但其 CameraCompensator、
> VisionFrameProcessor 補償責任、舊 Phase／ticket 切分及策略相容條款，已由
> `billiards-system-refactor-master-spec.md`、`python-cpp-external-contract.md`、
> `phase-1-shot-brain-spec.md` 與 `phase-2-shot-executor-spec.md` 取代。
> 後續 To Tickets 與實作不得以本文件覆蓋 active specs。

## 文件資訊

- 狀態：Superseded；不得供新 tickets 或實作使用
- 規格版本：1.1
- 建立日期：2026-07-30
- 基準 Commit：`b278db0`
- 上層規格：[`billiards-system-refactor-master-spec.md`](../../specs/billiards-system-refactor-master-spec.md)

## 1. 問題陳述

目前核心數學API會將無效或退化輸入轉換成看似合法的數值：

- 零向量夾角回傳0度。
- `(0,0)`方向角得到0度。
- 零長度路徑被當成未阻擋。
- 無法建立鬼球或鏡射點時回傳輸入點。
- Parser使用`bool detected + Point`，無效Point仍可被誤用。
- 相機補償直接處理sentinel。
- `0.001`跨量綱使用。

這些行為會讓無效視覺或幾何進入Algorithm及MotionPlanner。Phase 1的目的，是建立不依賴HRSDK的安全數學與幾何基礎。

## 2. 修改目標

1. MathUtils只保留純數學。
2. 相機補償移至CameraCompensator，數學公式不變。
3. Wire sentinel在Parser邊界轉成`std::optional<Point>`。
4. 所有可能失敗的幾何使用optional或具名狀態。
5. 路徑檢查使用`Clear/Blocked/Invalid`。
6. 移除`getTiltOffset()`及必要呼叫。
7. 移除`YAW_OFFSET_DEG`的必要呼叫；因現值為0，有效輸入行為應保持。
8. 將`tiltRyDeg`改為僅表示機械姿態的`ryDeg`，不在Phase 1實作姿態搜尋。
9. 建立`ParsedVisionFrame`、`ProcessedVisionFrame`與`TableState`三個不可混用的階段型別。
10. 以三個單幀Valid fixture完成presence、中位數及距離穩定驗證。
11. 建立兩個C++17離線測試執行檔，不載入HRSDK。
12. 有效、有限、非退化輸入下，Algorithm既有策略輸出保持相容。

## 3. 修改範圍

### 3.1 新增

- `src/CameraCompensator.h`
- `src/CameraCompensator.cpp`
- `src/VisionFrameProcessor.h`
- `src/VisionFrameProcessor.cpp`
- `src/GeometryResults.h`
- `src/StableFrameValidator.h`
- `src/StableFrameValidator.cpp`
- `tests/phase1_core_tests.cpp`
- `tests/phase1_algorithm_regression_tests.cpp`

如實作時需要極小型無外部依賴測試工具，可新增：

- `tests/TestHarness.h`

### 3.2 修改

- `src/Point.h`
- `src/MathUtils.h`
- `src/MathUtils.cpp`
- `src/BilliardConfig.h`
- `src/BilliardConfig.cpp`
- `src/TableState.h`
- `src/VisionDataParser.h`
- `src/VisionDataParser.cpp`
- `src/TargetSelector.h`
- `src/TargetSelector.cpp`
- `src/BilliardPhysics.h`
- `src/BilliardPhysics.cpp`
- `src/Algorithm.h`
- `src/Algorithm.cpp`
- `src/MotionPlanner.h`
- `src/MotionPlanner.cpp`
- `src/BilliardApp.cpp`
- `src/test_cueball.cpp`
- `.vscode/tasks.json`

### 3.3 不需修改

- `src/RobotController.h/.cpp`
- `src/SocketClient.h/.cpp`
- `src/calibrate.cpp`
- `src/main.cpp`
- `python/*`
- `HRSDK.h`
- `include/HRSDK.h`

## 4. 明確不處理的範圍

Phase 1禁止：

- H按鈕與CompetitionAuto狀態機。
- 3幀Socket收集狀態機。
- DO1／DO2控制。
- HRSDK連線、reachable、LIN、PTP或警報流程改寫。
- RX／RY姿態搜尋執行。
- `CartesianPose`全面遷移。
- 50 mm正式strike TCP運動模型。
- `STRIKE_TCP_Z_MM`實機值。
- 旋轉矩陣轉HRSDK Euler。
- Algorithm直球／反彈策略排序變更。
- 強制開火與防守球策略重寫。
- 正式手臂或氣壓實機測試。
- 為未來姿態轉換預先發明沒有現有呼叫端的HRSDK矩陣／Euler API。
- 執行`main`、`calibrate`、`test_cueball`或任何連結
  `RobotController`／HRSDK／數位輸出的程式。

通用`Matrix3x3`等純數學能力未來仍屬於MathUtils，但只有在Phase 2提出具體、與HRSDK語意解耦且可單元測試的需求後才新增。

Phase 1只允許執行`phase1_core_tests`與
`phase1_algorithm_regression_tests`。編譯主程式或診斷程式成功只代表介面
一致，不代表目前姿態或路徑可安全上機。

## 5. 現有API與問題

| 現有API／型別 | 問題 |
|---|---|
| `Point applyCameraCompensation(Point)` | MathUtils依賴BilliardConfig並理解sentinel |
| `double getDistance(...)` | 使用平方和sqrt，未定義非有限輸入 |
| `double getLength(...)` | 未定義非有限輸入及溢位 |
| `double getAngleBetweenVectors(...)` | 零向量回傳0度 |
| `double getVectorAngle(...)` | `(0,0)`得到假方向 |
| `Vector2D getVector(...)` | 相減溢位沒有失敗型別 |
| `Offset3D getTiltOffset(...)` | 混合機械姿態與幾何傾角 |
| `DetectedPoint{bool, Point}` | 無效Point仍存在 |
| `bool isPathBlocked(...)` | `false`同時代表Clear及Invalid |
| `Point getGhostBall(...)` | 退化時回傳目標球 |
| `Point getSlantedBankTarget(...)` | 退化時回傳輸入點 |
| `bool getIntersection(..., out)` | 無法區分NoIntersection與Invalid |

## 6. 新資料型別

### 6.1 基礎型別

`Point`與`Vector2D`保留mm單位：

```cpp
struct Point {
    double x;
    double y;
};

struct Vector2D {
    double x;
    double y;
};
```

新增軸對齊範圍：

```cpp
struct AxisAlignedBounds2D {
    double minX;
    double maxX;
    double minY;
    double maxY;
};
```

`Offset3D`在移除`getTiltOffset()`且確認沒有其他使用者後移除。

### 6.2 三個視覺階段型別

以optional取代DetectedPoint，並以三個不同的具名型別區分處理階段：

```cpp
struct ParsedVisionFrame {
    std::array<std::optional<Point>, 9> objectBalls;
    std::optional<Point> cueBall;
    std::array<std::optional<Point>, 6> pockets;
};

struct ProcessedVisionFrame {
    std::array<std::optional<Point>, 9> objectBalls;
    std::optional<Point> cueBall;
    std::array<std::optional<Point>, 6> pockets;
};

struct TableState {
    std::array<std::optional<Point>, 9> objectBalls;
    std::optional<Point> cueBall;
    std::array<std::optional<Point>, 6> pockets;
};
```

語意：

- `ParsedVisionFrame`是`VisionDataParser`的唯一成功輸出。
- `ParsedVisionFrame`中的`std::nullopt`只表示wire protocol的完整座標對未偵測。
- `ParsedVisionFrame`中的`Point`已完成語法與有限值檢查，但尚未完成相機補償、桌面範圍及必要資料驗證。
- `ProcessedVisionFrame`中的存在點均已完成相機補償，且整幀已通過必要資料與
  Base0桌面範圍驗證；它只代表單幀Valid，不代表三幀穩定。
- `TableState`只由`StableFrameValidator`在三幀穩定成功後產生；其中存在點
  是三幀各軸中位數座標。
- `ParsedVisionFrame`與`ProcessedVisionFrame`不得傳入`TargetSelector`或
  `Algorithm`。
- `TableState`是`TargetSelector`唯一可接受的視覺核心輸入。
- 核心幾何函式只接受已驗證的實際`Point`，不接受optional或sentinel。

固定資料流：

```text
VisionDataParser
→ ParsedVisionFrame
→ VisionFrameProcessor
→ ProcessedVisionFrame
→ StableFrameValidator
→ TableState
→ TargetSelector
```

### 6.3 幾何結果

`src/GeometryResults.h`：

```cpp
enum class PathStatus {
    Clear,
    Blocked,
    Invalid
};

enum class IntersectionStatus {
    Intersects,
    NoIntersection,
    Invalid
};

struct IntersectionResult {
    IntersectionStatus status;
    std::optional<Point> point;
};
```

不變量：

- `Intersects`時`point`必須有值。
- `NoIntersection`或`Invalid`時`point`必須為`std::nullopt`。

### 6.4 Result通用不變量

所有同時含status與`std::optional` payload的Result一律遵守：

- success狀態必須有payload。
- 所有非success狀態的payload必須是`std::nullopt`。
- 禁止success搭配`std::nullopt`。
- 禁止failure／invalid／unstable／configuration狀態攜帶payload。
- 建構Result的函式必須集中建立一致組合；呼叫端不得容忍或修補矛盾組合。

## 7. MathUtils新API

建議公開API：

```cpp
namespace BilliardMath {

inline constexpr double PI = 3.14159265358979323846;

bool isFinite(Point point) noexcept;
bool isFinite(Vector2D vector) noexcept;

std::optional<Vector2D> getVector(
    Point start,
    Point end
) noexcept;

std::optional<double> getLength(
    Vector2D vector
) noexcept;

std::optional<double> getDistance(
    Point first,
    Point second
) noexcept;

std::optional<Vector2D> normalize(
    Vector2D vector
) noexcept;

std::optional<double> getVectorAngleDeg(
    Vector2D vector
) noexcept;

std::optional<double> getAngleBetweenVectorsDeg(
    Vector2D first,
    Vector2D second
) noexcept;

}
```

### 7.1 單位與範圍

- `Point`與`Vector2D`長度單位：mm。
- `getLength`與`getDistance`輸出：mm。
- `normalize`輸出：無因次單位向量。
- `getVectorAngleDeg`輸出：度，範圍`[-180°, 180°]`，遵循`std::atan2`；負X軸預期為`+180°`。
- `getAngleBetweenVectorsDeg`輸出：度，範圍`[0°, 180°]`。

### 7.2 失敗條件

所有optional API在以下情況回傳`std::nullopt`：

- 任一輸入NaN或Infinity。
- 中間相減或運算結果不是有限值。
- 最終數值不是有限值。

另外：

- `normalize`：長度為0。
- `getVectorAngleDeg`：長度為0。
- `getAngleBetweenVectorsDeg`：任一向量長度為0。

MathUtils只判斷數學上的零向量。相機精度或撞球業務所需的最小有效長度由
`VisionFrameProcessor`、`StableFrameValidator`、`ShotCandidateValidator`
或BilliardPhysics設定處理，不在純MathUtils內硬編碼mm容差。

### 7.3 實作要求

- 長度使用`std::hypot`。
- 夾角cosine使用`std::clamp(value, -1.0, 1.0)`。
- optional失敗不得被轉換成0度。
- 函式不修改全域狀態。
- 可證明不拋例外的純函式使用`noexcept`。
- MathUtils不得include `BilliardConfig.h`。

## 8. CameraCompensator API

### 8.1 參數

```cpp
struct CameraCompensationParameters {
    double offsetXmm;
    double offsetYmm;
    double referenceXmm;
    double referenceYmm;
    double compensationKx;
    double compensationKy;
};
```

### 8.2 類別

```cpp
class CameraCompensator {
public:
    explicit CameraCompensator(
        CameraCompensationParameters parameters
    ) noexcept;

    bool isConfigured() const noexcept;

    std::optional<Point> compensate(
        Point rawPoint
    ) const noexcept;
};
```

第一行註解應直接說明：「將視覺座標套用既有線性殘差補償；不負責sentinel解析。」

### 8.3 固定公式

```text
compensatedX =
    rawX
    + CAMERA_OFFSET_X_MM
    + CAMERA_COMPENSATION_KX
      * (rawX - CAMERA_REFERENCE_X_MM)

compensatedY =
    rawY
    + CAMERA_OFFSET_Y_MM
    + CAMERA_COMPENSATION_KY
      * (rawY - CAMERA_REFERENCE_Y_MM)
```

Phase 1不得更改此公式。

### 8.4 失敗條件

回傳`std::nullopt`：

- rawPoint非有限。
- 任一補償參數非有限。
- 補償中間值或輸出非有限。

CameraCompensator不接受sentinel。呼叫者只能對有值的optional呼叫。

CameraCompensator只負責單點轉換，不認識`ParsedVisionFrame`、
`ProcessedVisionFrame`、`TableState`或`TargetSelector`。整幀逐點補償由
`VisionFrameProcessor`統一協調。

## 9. VisionDataParser契約

### 9.1 Wire格式

順序維持現有32值：

```text
b1x,b1y,...,b9x,b9y,bwx,bwy,p1x,p1y,...,p6x,p6y
```

Phase 1不得修改Python協定。

### 9.2 嚴格解析

建議API：

```cpp
enum class VisionParseStatus {
    Success,
    InvalidFieldCount,
    InvalidToken,
    NonFiniteValue,
    InvalidSentinelPair
};

struct VisionParseResult {
    VisionParseStatus status;
    std::optional<ParsedVisionFrame> frame;
};

VisionParseResult parseVisionFrame(
    std::string_view payload
);
```

不變量：

- `Success`時`frame`必須有值。
- 其他任何status時`frame`必須為`std::nullopt`。

Parser必須：

1. 精確取得32個token。
2. 拒絕空token。
3. 每個token必須被完整解析，不接受`1.2abc`。
4. 拒絕NaN與Infinity。
5. 拒絕數值轉換overflow。
6. 只有`x == -9999.0 && y == -9999.0`時才轉成`std::nullopt`。
7. 只有一個軸精確等於`-9999.0`時回`InvalidSentinelPair`，整幀失敗。
8. 其他小於`-9000.0`的有限數值不得視為missing；Parser保留為present
   `Point`，後續由`VisionFrameProcessor`的桌面範圍驗證拒絕。
9. Parse失敗時不得產生partial `ParsedVisionFrame`。

Wire protocol只定義一個精確sentinel常數：

```cpp
inline constexpr double MISSING_COORDINATE_SENTINEL = -9999.0;
```

不得定義`MISSING_COORDINATE_LIMIT`，也不得以`< -9000.0`、`<= -9000.0`
或近似比較判斷missing。`MISSING_COORDINATE_SENTINEL`只允許存在於
Parser／wire protocol層。

### 9.3 必要資料

Parser只負責語法、數值及sentinel轉換。必要球資料由下一層驗證：

- cueBall必須存在。
- 至少一顆目標球必須存在。
- 目前策略所需袋口必須存在。

缺少必要資料時整幀不可進入Algorithm。

## 10. 單幀處理與三幀穩定API

Phase 1只建立可由fixture呼叫的純邏輯，不串接Socket、H按鈕或
CompetitionAuto。

### 10.1 VisionFrameProcessor

建議API：

```cpp
struct VisionProcessingConfig {
    std::optional<AxisAlignedBounds2D> tableBoundsMm;
};

enum class SingleFrameStatus {
    Valid,
    CompensationFailed,
    MissingRequiredData,
    OutsideTableBounds,
    Invalid,
    ConfigurationMissing
};

struct SingleFrameResult {
    SingleFrameStatus status;
    std::optional<ProcessedVisionFrame> frame;
};

class VisionFrameProcessor {
public:
    SingleFrameResult process(
        const ParsedVisionFrame& frame
    ) const noexcept;
};
```

Result不變量：

- `SingleFrameStatus::Valid`時`frame`必須有值。
- 其他所有status時`frame`必須為`std::nullopt`。

責任與順序：

1. 驗證`tableBoundsMm`存在、有限且`minX < maxX`、`minY < maxY`；缺少時
   回`ConfigurationMissing`，內容無效時回`Invalid`。
2. 對`ParsedVisionFrame`中每一個存在點呼叫`CameraCompensator`。
3. 任一存在點補償失敗時回`CompensationFailed`，不得產生partial frame。
4. 驗證補償後每一個存在點皆有限且在Base0桌面範圍內；否則回
   `OutsideTableBounds`或`Invalid`。
5. 驗證cue ball、至少一顆目標球及目前策略所需袋口；缺少時回
   `MissingRequiredData`。
6. 只有全部成功時產生`ProcessedVisionFrame`並回`Valid`。

`VisionFrameProcessor`可以持有或注入`CameraCompensator`；
`TargetSelector`不得持有、建構、注入或呼叫`CameraCompensator`。

### 10.2 StableFrameValidator

建議API：

```cpp
struct StableFrameConfig {
    std::optional<double> stableFrameToleranceMm;
};

enum class StableFrameStatus {
    Stable,
    Unstable,
    InvalidInput,
    InvalidConfiguration,
    ConfigurationMissing
};

struct StableFrameResult {
    StableFrameStatus status;
    std::optional<TableState> tableState;
};

class StableFrameValidator {
public:
    StableFrameResult validate(
        const std::array<ProcessedVisionFrame, 3>& frames
    ) const noexcept;
};
```

Result不變量：

- `StableFrameStatus::Stable`時`tableState`必須有值。
- 其他所有status時`tableState`必須為`std::nullopt`。

呼叫邊界：

- 只有`SingleFrameResult::status == SingleFrameStatus::Valid`且其`frame`
  有值時，該frame才可加入三幀輸入。
- 任一單幀結果不是`Valid`、缺少payload或違反Result不變量時，不得呼叫
  `validate()`，也不得產生穩定結果。
- Phase 1不負責從Socket收集三幀；測試直接傳入三個
  `ProcessedVisionFrame` fixture。

### 10.3 三幀穩定演算法

`StableFrameValidator::validate()`依下列固定順序執行：

1. `stableFrameToleranceMm`缺失時回`ConfigurationMissing`。
2. tolerance非有限或小於0時回`InvalidConfiguration`。
3. 防禦性檢查三幀所有存在點皆有限；否則回`InvalidInput`。
4. 比較三幀所有9顆object ball、cue ball及6個袋口的presence pattern。
5. 任一對應物件在三幀間出現或消失時回`Unstable`。
6. 對每個三幀皆存在的物件，分別計算三個X值的中位數與三個Y值的中位數，
   得到`medianPoint`。
7. 使用`std::hypot(framePoint.x - medianPoint.x,
   framePoint.y - medianPoint.y)`計算每一幀位置到中位數點的歐氏距離。
8. 任一距離非有限時回`InvalidInput`；任一距離大於
   `stableFrameToleranceMm`時回`Unstable`。距離等於tolerance視為通過。
9. 對三幀皆缺失的物件保留`std::nullopt`；對三幀皆存在且通過的物件使用
   `medianPoint`。
10. 所有物件通過後，以中位數座標建立唯一的`TableState`並回`Stable`。

X與Y必須分別取中位數，禁止取「某一幀整個Point」代替，也禁止用平均數。
`TableState`不得由Parser、VisionFrameProcessor、TargetSelector或其他呼叫端
直接建構為視覺輸入。

## 11. BilliardPhysics新API

建議API：

```cpp
struct CollisionParameters {
    double ballDiameterMm;
    double clearanceMarginMm;
    double minimumSegmentLengthMm;
    double segmentParameterTolerance;
    double parallelSinTolerance;
};

PathStatus checkPath(
    Point start,
    Point end,
    Point obstacle,
    const CollisionParameters& parameters
) noexcept;

PathStatus checkRoute(
    Point start,
    Point end,
    const std::vector<Point>& obstacles,
    const CollisionParameters& parameters
) noexcept;

std::optional<Point> getPerpendicularTarget(
    Point base,
    Point edgeA,
    Point edgeB,
    double backwardDistanceMm,
    double minimumEdgeLengthMm
) noexcept;

std::optional<Point> getGhostBall(
    Point destination,
    Point targetBall,
    double ballDiameterMm,
    double minimumDirectionLengthMm
) noexcept;

std::optional<Point> getSlantedBankTarget(
    Point input,
    Point railA,
    Point railB,
    double minimumRailLengthMm
) noexcept;

IntersectionResult getIntersection(
    Point rayStart,
    Point rayTarget,
    Point segmentA,
    Point segmentB,
    double minimumDirectionLengthMm,
    double segmentParameterTolerance,
    double parallelSinTolerance
) noexcept;
```

具體命名可依現有風格微調，但不得退回bool或假Point。

### 11.1 PathStatus精確語意

`Clear`：

- 所有輸入有限。
- 參數有效。
- 路徑長度大於最小有效長度。
- 障礙中心到有限線段的最短距離大於阻擋門檻。

`Blocked`：

- 幾何有效。
- 最短距離小於或等於：

```text
ballDiameterMm + clearanceMarginMm
```

`Invalid`：

- 任一座標非有限。
- 路徑長度不足。
- ballDiameter不是有限正值。
- clearanceMargin不是有限非負值。
- 任何必要容差缺失、非有限或不符合其有效範圍。

`checkRoute`合併規則：

1. 先驗證start、end及全部`CollisionParameters`，不得先以障礙列表為空提早回傳。
2. 路徑退化、座標非有限或參數無效時回`Invalid`，即使障礙列表為空亦相同。
3. 路徑與參數有效且障礙列表為空時回`Clear`。
4. 非空列表中任一障礙非有限或障礙檢查為`Invalid`，整條route為`Invalid`。
5. 否則任一障礙為`Blocked`，route為`Blocked`。
6. 全部障礙為`Clear`時回`Clear`。

### 11.2 IntersectionResult

`Intersects`：

- 輸入有效。
- 射線與有限segment存在唯一、容差內有效交點。

`NoIntersection`：

- 輸入有效，但射線平行不重合、交點在射線後方，或在segment範圍外。

`Invalid`：

- 非有限輸入。
- 射線方向長度不足。
- segment長度不足。
- 兩線重合而無唯一交點。
- 容差參數無效。

## 12. Algorithm與呼叫端遷移

### 12.1 原則

- 有效輸入下不改策略順序、門檻與文字以外的決策結果。
- 幾何`Invalid`時不得建立替代Point。
- Phase 1不修正「安全路徑受阻，強制開火」策略；但Invalid幾何不得到達該分支。
- Master Spec已將強制開火列為CompetitionAuto前的Safety Critical阻擋項。

### 12.2 建議結果

```cpp
enum class ShotDecisionStatus {
    Success,
    InvalidGeometry
};

struct ShotDecisionResult {
    ShotDecisionStatus status;
    std::optional<ShotDecision> decision;
    std::string diagnostic;
};
```

不變量：

- `Success`時decision有值。
- `InvalidGeometry`時decision為nullopt。

### 12.3 TargetSelector

- API只接受`const TableState&`或語意等價的已補償、已驗證核心型別。
- 不得include、持有、建構、注入或呼叫`CameraCompensator`。
- 不得接受`ParsedVisionFrame`或`ProcessedVisionFrame`，也不得自行處理
  sentinel、相機補償或三幀穩定。
- 依策略需求對`std::optional<Point>`做存在性檢查，例如跳過已落袋球。
- 夾角optional無值時跳過該袋口候選。
- 全部袋口候選無效時回報錯誤。
- 不將無效夾角替換成0。

### 12.4 MotionPlanner必要相容遷移

Phase 1只為了移除MathUtils舊API而修改：

- `getVector()`、`getLength()`與角度改處理optional。
- 移除`YAW_OFFSET_DEG`加法；現值為0，正常輸入行為不變。
- 移除`getTiltOffset()`呼叫；目前production及test的`moveBackMm`皆為0，正常輸入位置不變。
- `MotionProfile::tiltRyDeg`改名為`ryDeg`，只表示Pose RY。
- 移除不再使用且現值為0的`moveBackMm`。
- 保留現階段standoff與ready/strike運動結構，留待Phase 2正式重寫。
- 不在Phase 1套用50 mm新strike TCP公式。
- 不在Phase 1更改RX／RY數值或執行搜尋。

Phase 1完成後型別及欄位順序固定為：

```cpp
struct MotionProfile {
    double strikeZ;
    double safeZ;
    double rxDeg;
    double ryDeg;
    double standoffExtraMm;
};
```

#### PRODUCTION_MOTION欄位遷移

| 舊欄位順序 | 舊值 | 新欄位順序 | 新值 | 規則 |
|---|---:|---|---:|---|
| 1. strikeZ | -216.0 | 1. strikeZ | -216.0 | 原值保留 |
| 2. safeZ | -160.0 | 2. safeZ | -160.0 | 原值保留 |
| 3. rxDeg | 0.0 | 3. rxDeg | 0.0 | Phase 1不得改值 |
| 4. tiltRyDeg | -180.0 | 4. ryDeg | -180.0 | 只改名，不改值 |
| 5. moveBackMm | 0.0 | 移除 | — | 不得留下placeholder |
| 6. standoffExtraMm | 0.0 | 5. standoffExtraMm | 0.0 | 往前補位，原值保留 |

對應新aggregate initializer必須為：

```cpp
const MotionProfile PRODUCTION_MOTION = {
    -216.0,  // strikeZ
    -160.0,  // safeZ
    0.0,     // rxDeg
    -180.0,  // ryDeg
    0.0      // standoffExtraMm
};
```

#### TEST_MOTION欄位遷移

| 舊欄位順序 | 舊值 | 新欄位順序 | 新值 | 規則 |
|---|---:|---|---:|---|
| 1. strikeZ | -140.0 | 1. strikeZ | -140.0 | 原值保留 |
| 2. safeZ | -150.0 | 2. safeZ | -150.0 | 原值保留 |
| 3. rxDeg | -10.0 | 3. rxDeg | -10.0 | Phase 1不得改值 |
| 4. tiltRyDeg | 180.0 | 4. ryDeg | 180.0 | 只改名，不改值 |
| 5. moveBackMm | 0.0 | 移除 | — | 不得留下placeholder |
| 6. standoffExtraMm | 0.0 | 5. standoffExtraMm | 0.0 | 往前補位，原值保留 |

對應新aggregate initializer必須為：

```cpp
const MotionProfile TEST_MOTION = {
    -140.0,  // strikeZ
    -150.0,  // safeZ
    -10.0,   // rxDeg
    180.0,   // ryDeg
    0.0      // standoffExtraMm
};
```

欄位移除後必須逐欄核對兩個initializer，不得只刪除第五個數字而造成
`standoffExtraMm`或姿態欄位錯位。

現有`PRODUCTION_MOTION`的RX／RY與已確認的預期基準不同，且尚未通過實機
驗證。Phase 1只保留現值、確保編譯及API語意，不宣告其可安全實機使用。

### 12.5 test_cueball

- 遷移MathUtils與VisionFrameProcessor新API。
- 策略資料流不得自行散落CameraCompensator呼叫。
- 不重寫姿態搜尋。
- 不執行測試程式。
- 保留使用者既有輸出換行，不做無關格式化。

## 13. 相容策略

### 13.1 保留

- Python wire format。
- 32欄順序。
- 有效輸入下相機補償公式。
- 有效輸入下直球／反彈策略排序。
- 現有HRSDK公開封裝。

### 13.2 不保留

以下危險行為不得提供legacy wrapper：

- 零向量回傳0度。
- 無效路徑回傳Clear。
- 無法計算時回傳輸入Point。
- sentinel直接通過MathUtils。

### 13.3 未校正設定

桌面bounds、穩定幀容差及球路安全餘量使用optional設定：

- 測試注入明確fixture。
- production缺少時回`ConfigurationMissing`或`Invalid`。
- 不以0或其他數值假裝正式校正值。
- 若需要0安全餘量做回歸測試，只能在測試fixture中明確指定。

## 14. 數值容差

禁止使用單一全域epsilon處理不同量綱。

| 名稱語意 | 單位 | 所屬層 |
|---|---|---|
| minimumVectorLengthMm | mm | 幾何／業務驗證 |
| minimumSegmentLengthMm | mm | BilliardPhysics |
| dimensionlessComparisonTolerance | 無因次 | 一般浮點比較 |
| segmentParameterTolerance | 無因次 | t/u範圍 |
| parallelSinTolerance | 無因次 | 平行判定 |
| stableFrameToleranceMm | mm | StableFrameValidator |
| ballPathClearanceMarginMm | mm | BilliardPhysics設定 |

規則：

- MathUtils不硬編碼相機或撞球mm容差。
- 所有容差必須有限。
- 長度及安全餘量不得為負。
- 線段參數容差及平行容差必須在規格化有效範圍。
- 正式未校正值維持missing。

## 15. 測試策略

### 15.1 框架

Repository目前沒有測試框架。Phase 1建立兩個互相獨立、不含外部套件的
C++17測試執行檔：

```text
phase1_core_tests
  source: tests/phase1_core_tests.cpp
  modules:
    MathUtils
    CameraCompensator
    VisionDataParser
    VisionFrameProcessor
    StableFrameValidator
    BilliardPhysics

phase1_algorithm_regression_tests
  source: tests/phase1_algorithm_regression_tests.cpp
  modules:
    all phase1_core_tests modules
    TargetSelector
    Algorithm
    required pure data types
```

兩個執行檔共同要求：

- 不include或link `HRSDK.h`、HRSDK library、`RobotController`、
  `SocketClient`或任何數位輸出程式。
- 不開Socket。
- 不呼叫Sleep。
- 不連機械手臂。
- 不操作數位輸出。
- 非0 exit code表示失敗。
- 不得以main、calibrate或test_cueball充當測試入口。
- Phase 1 Implement期間只允許執行這兩個離線測試執行檔。

### 15.2 距離與向量

| 輸入 | 預期結果 | 失敗狀態 | API | 類型 |
|---|---|---|---|---|
| `(0,0)→(1,0)` | vector `(1,0)`、length `1` | 無 | getVector/getLength | 單元 |
| `(0,0)→(0,1)` | `(0,1)`、`1` | 無 | 同上 | 單元 |
| `(0,0)→(-1,0)` | `(-1,0)`、`1` | 無 | 同上 | 單元 |
| `(0,0)→(0,-1)` | `(0,-1)`、`1` | 無 | 同上 | 單元 |
| 四象限斜向量 | 正確hypot長度 | 無 | getLength | 單元 |
| `(0,0)`向量 | length `0`；normalize無值 | `nullopt`僅normalize | getLength/normalize | 單元 |
| 極小非零有限向量 | 純MathUtils可正規化 | 無 | normalize | 單元 |
| 極小向量搭配業務minimum | 幾何層拒絕 | `Invalid` | BilliardPhysics | 單元 |
| NaN分量 | 無結果 | `nullopt` | 所有optional Math API | 單元 |
| Infinity分量 | 無結果 | `nullopt` | 同上 | 單元 |
| `1e150`級有限向量 | 有限hypot | 無 | getLength | 單元 |
| 相減溢位 | 無結果 | `nullopt` | getVector/getDistance | 單元 |

### 15.3 方向角

| 向量 | 預期 | 失敗狀態 | API | 類型 |
|---|---|---|---|---|
| `(1,0)` | `0°` | 無 | getVectorAngleDeg | 單元 |
| `(0,1)` | `90°` | 無 | 同上 | 單元 |
| `(-1,0)` | `180°` | 無 | 同上 | 單元 |
| `(0,-1)` | `-90°` | 無 | 同上 | 單元 |
| `(1,1)` | `45°` | 無 | 同上 | 單元 |
| `(-1,1)` | `135°` | 無 | 同上 | 單元 |
| `(-1,-1)` | `-135°` | 無 | 同上 | 單元 |
| `(1,-1)` | `-45°` | 無 | 同上 | 單元 |
| `(0,0)` | 無結果 | `nullopt` | 同上 | 單元 |
| NaN／Infinity | 無結果 | `nullopt` | 同上 | 單元 |

### 15.4 向量夾角

| 輸入 | 預期 | 失敗狀態 | API | 類型 |
|---|---|---|---|---|
| 同方向 | `0°` | 無 | getAngleBetweenVectorsDeg | 單元 |
| 反方向 | `180°` | 無 | 同上 | 單元 |
| 垂直 | `90°` | 無 | 同上 | 單元 |
| 任一零向量 | 無結果 | `nullopt` | 同上 | 單元 |
| 任一非有限 | 無結果 | `nullopt` | 同上 | 單元 |
| 近同向浮點邊界 | 有限且在`[0,180]` | 無 | 同上 | 單元 |

近同向測試必須覆蓋cosine clamp到`[-1,1]`。

### 15.5 CameraCompensator

測試參數只能是fixture，不代表production校正值。

| 輸入 | 預期 | 失敗狀態 | API | 類型 |
|---|---|---|---|---|
| 一般有限座標 | 依固定公式 | 無 | compensate | 單元 |
| raw等於reference | 只加固定offset | 無 | compensate | 單元 |
| 只有X offset | X正確、Y不變 | 無 | compensate | 單元 |
| 只有Y offset | Y正確、X不變 | 無 | compensate | 單元 |
| Kx/Ky非0 | 比例補償正確 | 無 | compensate | 單元 |
| 缺失optional | 呼叫端保持nullopt，不呼叫compensate | 無資料 | 邊界整合 | 整合 |
| NaN／Infinity raw | 無結果 | `nullopt` | compensate | 單元 |
| 非有限參數 | 未配置／無結果 | `isConfigured=false` | constructor/compensate | 單元 |
| 補償溢位 | 無結果 | `nullopt` | compensate | 單元 |

### 15.6 VisionFrameProcessor

| 輸入 | 預期 | 失敗狀態 | API | 類型 |
|---|---|---|---|---|
| 所有必要點合法 | 所有存在點完成補償並產生ProcessedVisionFrame | Valid | process | 整合 |
| 任一點補償失敗 | payload為nullopt | CompensationFailed | process | 整合 |
| 任一補償後點超出bounds | payload為nullopt | OutsideTableBounds | process | 整合 |
| cueBall缺失 | payload為nullopt | MissingRequiredData | process | 整合 |
| tableBounds缺失 | payload為nullopt | ConfigurationMissing | process | 單元 |
| tableBounds無效 | payload為nullopt | Invalid | process | 單元 |
| 非必要object ball缺失 | 保留對應nullopt，其餘資料正常 | Valid | process | 整合 |

另以依賴邊界測試或編譯檢查確認：

- `TargetSelector.h/.cpp`不include `CameraCompensator.h`。
- `TargetSelector`沒有CameraCompensator成員、建構參數或呼叫。
- `TargetSelector`只接受`TableState`，不接受`ParsedVisionFrame`或
  `ProcessedVisionFrame`。

### 15.7 StableFrameValidator

以下測試直接使用三個符合單幀Valid契約的`ProcessedVisionFrame` fixture，
不涉及Socket或等待：

| 輸入 | 預期 | 失敗狀態 | API | 類型 |
|---|---|---|---|---|
| 三幀presence一致且位置相同 | 以相同座標產生TableState | Stable | validate | 單元 |
| 三幀presence一致且在tolerance內 | 各物件使用X/Y中位數 | Stable | validate | 單元 |
| 任一presence bit不同，且三幀仍各自符合單幀Valid契約 | payload為nullopt | Unstable | validate | 單元 |
| 任一點距中位數大於tolerance | payload為nullopt | Unstable | validate | 單元 |
| 所有距離等於tolerance | 產生TableState | Stable | validate | 單元 |
| tolerance缺失 | payload為nullopt | ConfigurationMissing | validate | 單元 |
| tolerance為負或非有限 | payload為nullopt | InvalidConfiguration | validate | 單元 |
| 任一輸入點非有限 | payload為nullopt | InvalidInput | validate | 單元 |
| 三幀皆缺失的非必要物件 | TableState對應欄位為nullopt | Stable | validate | 單元 |
| X/Y中位數來自不同幀 | 使用各軸中位數，不選任一整點 | Stable | validate | 單元 |

另測試pipeline邊界：

- object ball、cue ball或袋口的出現／消失若仍保有三個單幀Valid結果，
  `StableFrameValidator`必須回`Unstable`。
- presence變動若同時使任一幀缺少必要資料，該幀先由
  `VisionFrameProcessor`回非Valid；不得組成三幀輸入，也不得產生
  `StableFrameResult::Stable`。
- 任一`SingleFrameResult`不是`Valid`、payload缺失或status/payload矛盾時，
  不得呼叫`validate()`。

### 15.8 PathStatus

| 輸入 | 預期 | 失敗狀態 | API | 類型 |
|---|---|---|---|---|
| 障礙遠離線段 | `Clear` | 無 | checkPath | 單元 |
| 障礙距離小於直徑 | `Blocked` | 無 | checkPath | 單元 |
| 距離等於直徑，margin=0 fixture | `Blocked` | 無 | checkPath | 單元 |
| margin>0 fixture使原clear變blocked | `Blocked` | 無 | checkPath | 單元 |
| start=end | `Invalid` | 退化路徑 | checkPath | 單元 |
| 任一Point非有限 | `Invalid` | 無效輸入 | checkPath | 單元 |
| margin<0或非有限 | `Invalid` | 無效設定 | checkPath | 單元 |
| 有效路徑、有效參數、空障礙列表 | `Clear` | 無 | checkRoute | 單元 |
| 退化路徑、空障礙列表 | `Invalid` | 退化路徑 | checkRoute | 單元 |
| 無效參數、空障礙列表 | `Invalid` | 無效設定 | checkRoute | 單元 |
| route含一個Invalid障礙 | `Invalid` | 無效障礙 | checkRoute | 單元 |
| route含Blocked及Invalid | `Invalid`優先 | 無效幾何 | checkRoute | 單元 |

### 15.9 鬼球、鏡射與交點

| 輸入 | 預期 | 失敗狀態 | API | 類型 |
|---|---|---|---|---|
| 有效袋口／目標球 | 正確鬼球點 | 無 | getGhostBall | 單元 |
| 袋口=目標球 | 無結果 | `nullopt` | getGhostBall | 單元 |
| railA=railB | 無結果 | `nullopt` | bank APIs | 單元 |
| 有效唯一交點 | `Intersects`及Point | 無 | getIntersection | 單元 |
| 平行不相交 | `NoIntersection` | 無解 | getIntersection | 單元 |
| 重合直線 | `Invalid` | 無唯一解 | getIntersection | 單元 |
| 交點在線段外 | `NoIntersection` | 無解 | getIntersection | 單元 |
| 非有限輸入 | `Invalid` | 無效輸入 | getIntersection | 單元 |

### 15.10 Parser邊界

| 輸入 | 預期 | 失敗狀態 | API | 類型 |
|---|---|---|---|---|
| 32個合法值 | 成功ParsedVisionFrame | 無 | parse | 單元 |
| 31個值 | 失敗且output不變 | 欄位不足 | parse | 單元 |
| 33個值 | 失敗且output不變 | 欄位過多 | parse | 單元 |
| 成對`-9999` | 對應optional為nullopt | 無 | parse | 單元 |
| 單軸sentinel | 整幀失敗 | sentinel pair錯誤 | parse | 單元 |
| 成對`-9998` | present Point，不是missing | 無；後續範圍拒絕 | parse | 單元 |
| 任一其他小於`-9000`的有限值 | present Point，不是missing | 無；後續範圍拒絕 | parse | 單元 |
| NaN | 整幀失敗 | 非有限 | parse | 單元 |
| Infinity | 整幀失敗 | 非有限 | parse | 單元 |
| `1.0abc` | 整幀失敗 | token未完整解析 | parse | 單元 |
| 空token | 整幀失敗 | 格式錯誤 | parse | 單元 |
| 小於`-9000`的有限座標 | Parser成功、Processor拒絕 | OutsideTableBounds | Parser+Processor | 整合 |
| 一般補償後桌面外 | 整幀失敗 | OutsideTableBounds | Parser+Processor | 整合 |
| cueBall缺失 | 不得進Algorithm | MissingRequiredData | Processor | 整合 |

### 15.11 有效策略回歸

建立固定、有限、非退化fixture：

- 直球路徑。
- 直球受阻轉反彈。
- 現有安全反彈路徑。

比較Phase 1前後：

- `best_aim_target`
- `strategy_name`
- `angle_deg`
- `direct_path_blocked`
- `bank_route_safe`

除浮點容差內差異外結果必須相同。這些測試屬於
`phase1_algorithm_regression_tests`，不允許連HRSDK。

## 16. 建置修改

所有MSVC task加入：

```text
/std:c++17
```

GCC task加入：

```text
-std=c++17
```

主程式build加入`CameraCompensator.cpp`、`VisionFrameProcessor.cpp`、
`StableFrameValidator.cpp`及Phase 1必要新來源檔，但Phase 1期間不得執行產物。

新增兩組完全獨立的build/run task：

#### phase1_core_tests

只編譯／link：

- `tests/phase1_core_tests.cpp`
- MathUtils
- CameraCompensator
- VisionDataParser
- VisionFrameProcessor
- StableFrameValidator
- BilliardPhysics
- 必要純資料型別

#### phase1_algorithm_regression_tests

只編譯／link：

- `tests/phase1_algorithm_regression_tests.cpp`
- `phase1_core_tests`所需的核心模組
- TargetSelector
- Algorithm
- 必要純資料型別

兩組task共同禁止：

- include或link HRSDK。
- 編譯或link RobotController、SocketClient、BilliardApp及任何數位輸出程式。
- 將main、calibrate或test_cueball設為測試入口。
- 在run task中啟動上述任何非Phase 1離線程式。

## 17. 驗收條件

Phase 1完成必須同時滿足：

1. 專案相關task明確使用C++17。
2. MathUtils不include BilliardConfig。
3. MathUtils不包含相機或姿態偏移。
4. `getTiltOffset()`不存在。
5. `YAW_OFFSET_DEG`不再被任何C++呼叫端使用。
6. `tiltRyDeg`名稱不存在。
7. `moveBackMm`名稱不存在，MotionProfile固定為五欄指定順序。
8. PRODUCTION_MOTION與TEST_MOTION逐欄遷移，RX／RY值不變且aggregate
   initializer沒有錯位。
9. 只有精確成對`-9999.0`轉missing；單軸sentinel令整幀Invalid。
10. `MISSING_COORDINATE_LIMIT`不存在，且沒有以`-9000.0`門檻判斷missing。
11. 其他小於`-9000.0`的有限座標由Parser保留、由範圍驗證拒絕。
12. `ParsedVisionFrame`、`ProcessedVisionFrame`與`TableState`三個階段型別
    不得互換。
13. 只有單幀Valid結果含`ProcessedVisionFrame`；其他單幀status payload為nullopt。
14. 只有`StableFrameValidator::Stable`可產生`TableState`。
15. 三幀presence一致、中位數及歐氏距離演算法通過完整fixture測試。
16. tolerance缺失回ConfigurationMissing，不填猜測值。
17. `TargetSelector`只接受TableState，且沒有CameraCompensator header、成員、
    建構參數或呼叫。
18. 所有status+optional Result符合success有值、非success為nullopt的不變量。
19. 零向量角度無結果。
20. 零長度路徑為Invalid。
21. `checkRoute`先驗證路徑與參數；有效空列表為Clear，無效空列表為Invalid。
22. `phase1_core_tests`與`phase1_algorithm_regression_tests`全部通過。
23. 有效Algorithm回歸fixture結果相容，且未修改策略排序。
24. 兩個測試執行檔均未include或link HRSDK、RobotController、SocketClient、
    BilliardApp或數位輸出程式。
25. Phase 1期間未執行main、calibrate、test_cueball或任何HRSDK／控制程式。
26. 主程式、calibrate及test_cueball僅允許編譯驗證，不得執行。
27. 未連線、未送運動、未操作DO。
28. 未填入任何尚待硬體校正的production數值。
29. 現有PRODUCTION_MOTION明確標記尚未通過實機驗證；編譯成功不代表安全上機。

## 18. 回滾方式

1. Phase 1使用單一獨立提交或一組明確標記的連續提交。
2. API、呼叫端與測試必須一起回滾，禁止只回滾MathUtils造成ABI／編譯不一致。
3. CameraCompensator新檔可隨Phase提交整體移除。
4. Python協定沒有修改，不需要跨語言回滾。
5. 不得使用`git reset --hard`。
6. 不得覆寫Phase開始前的使用者修改。
7. `src/test_cueball.cpp`只回滾Phase 1必要API遷移，不觸及使用者原有輸出換行。

## 19. Implement執行順序

1. 建立兩個C++17離線測試task及最小測試harness，先驗證其依賴圖不含
   HRSDK、RobotController、SocketClient、BilliardApp或數位輸出。
2. 新增GeometryResults與基礎型別，建立所有status+optional Result不變量測試。
3. 建立`ParsedVisionFrame`、`ProcessedVisionFrame`與`TableState`三階段型別。
4. 重構MathUtils並先在`phase1_core_tests`完成MathUtils單元測試。
5. 建立CameraCompensator及核心測試。
6. 嚴格化VisionDataParser：只接受精確成對`-9999.0`，移除
   `MISSING_COORDINATE_LIMIT`，只輸出`ParsedVisionFrame`。
7. 建立VisionFrameProcessor，使其只在單幀Valid時輸出
   `ProcessedVisionFrame`。
8. 建立StableFrameValidator，完成三幀presence、各軸中位數、歐氏距離與
   ConfigurationMissing fixture測試。
9. 重構BilliardPhysics結果型別，加入checkRoute空列表順序測試。
10. 將MotionProfile改為固定五欄，逐欄遷移PRODUCTION_MOTION及TEST_MOTION，
    移除moveBackMm並核對initializer。
11. 遷移TargetSelector與Algorithm，確認TargetSelector只接受TableState且不
    依賴CameraCompensator，加入有效策略回歸測試。
12. 遷移MotionPlanner必要MathUtils呼叫，不改完整運動流程、RX／RY值或姿態搜尋。
13. 遷移BilliardApp與test_cueball的編譯介面，但不得執行。
14. 更新主程式build及兩個Phase 1離線測試task。
15. 執行且只執行`phase1_core_tests`。
16. 執行且只執行`phase1_algorithm_regression_tests`。
17. 可編譯main、calibrate及test_cueball以確認介面，但不得執行任何產物。
18. 執行程式碼審查，確認沒有HRSDK、Socket或DO測試副作用，並再次確認編譯
    成功未被宣告為實機安全。

## 20. Phase 1未決事項

以下不阻擋API、純邏輯或fixture測試，但阻擋正式production啟用：

### Needs Hardware Validation

- Base 0桌面bounds。
- 3幀穩定容差。
- 球路安全餘量。
- 最小業務向量／線段長度容差。
- 平行及交點容差的正式校正策略。

缺少上述值時必須fail closed，不得由Implement自行填值。
