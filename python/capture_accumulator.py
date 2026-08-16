"""Accumulate per-object observations across raw images within one
START_CAPTURE window, until every tracked object resolves to a stable
state, then emit exactly one existing-format 32-value Logical Frame.

Two layers of stability exist in this system:
  - This module: several raw YOLO images -> one Logical Frame.
  - C++ ThreeEventStability (see src/TableState.h): three Logical Frames ->
    one StableTableState.
This module only produces the first layer; it must never itself decide the
table is stable enough for shot planning.
"""

import math

from detection_filter import CaptureRejected, order_six_by_position
from vision_payload import (
    BALL_CLASS_COUNT,
    FIELD_COUNT,
    MISSING_COORDINATE,
    POCKET_COUNT,
    box_center,
)

CUE_CLASS_ID = BALL_CLASS_COUNT - 1  # 9

# [可調參數，非量測值] 一顆球（或母球）要連續幾次觀測到「有」才視為
# present收斂。
PRESENT_STABLE_COUNT = 3

# [可調參數，非量測值] 一顆編號球要連續幾次觀測到「沒有」才視為absent
# 收斂；刻意比PRESENT_STABLE_COUNT更保守——實際比賽中，擊球可能意外
# 碰進非目標的其他號球，不一定是最低號的球缺席，任何一顆球都可能
# 突然從畫面消失。如果absent判定跟present一樣快，一顆只是短暫被遮擋/
# 漏偵測、其實還在桌上的球，可能被誤判成「確定不在」，讓Phase1誤以為
# 安全、規劃出一條其實會撞到那顆球的路徑。母球（allow_absent=False）
# 不受這個常數影響——母球永遠不允許收斂成absent。
ABSENT_STABLE_COUNT = 5

# 沿用舊名稱給袋口cluster穩定度使用（袋口不會消失，只有「這個位置夠不
# 夠穩定可信」的問題，跟球的present/absent語意不同，維持原本3幀）。
OBSERVATION_STABLE_COUNT = PRESENT_STABLE_COUNT

# [可調參數，非量測值] 收斂視窗內，同一物件Base0 XY允許的最大偏移，單位mm。
OBSERVATION_POSITION_TOLERANCE_MM = 10.0

# [可調參數，非量測值] 未標記袋口候選要多接近（像素距離）才視為同一個
# cluster；袋口幾乎不會移動，這裡刻意跟球的容差分開設定，單位px。
POCKET_CLUSTER_TOLERANCE_PX = 20.0

# [可調參數，非量測值] 單一Logical Frame最多嘗試幾張影像才放棄這一輪；
# 呼叫端決定放棄後的行為（通常是繼續累積或回報錯誤），這個模組本身
# 不強制中止capture window。
MAX_IMAGES_PER_LOGICAL_FRAME = 90


def _project_point_or_none(pixel_point, projector):
    try:
        x, y, z = map(float, projector.project(*pixel_point))
    except Exception:
        return None
    if not all(math.isfinite(value) for value in (x, y, z)):
        return None
    if x == MISSING_COORDINATE or y == MISSING_COORDINATE:
        return None
    return (x, y)


def _project_box_or_none(box, projector):
    if box is None:
        return None
    return _project_point_or_none(box_center(box), projector)


class _ObjectAccumulator:
    """單一物件（Ball1-9或母球）的觀測視窗，運作在Base0 mm座標。
    allow_absent=False（母球）時，連續Absent不會收斂，只會持續等待，
    因為母球在正式打球流程中必須存在，缺席不是合法的最終狀態。
    present用較短視窗（PRESENT_STABLE_COUNT）保持反應速度；absent用
    較長視窗（ABSENT_STABLE_COUNT）更保守，避免短暫遮擋/漏偵測就被
    誤判成「確定不在桌上」。"""

    def __init__(self, allow_absent):
        self._allow_absent = allow_absent
        self._window = []
        self._capacity = (
            max(PRESENT_STABLE_COUNT, ABSENT_STABLE_COUNT)
            if allow_absent else PRESENT_STABLE_COUNT
        )

    def add(self, base0_point):
        # base0_point是(x, y)或None（這張影像沒偵測到這個物件）。
        self._window.append(base0_point)
        if len(self._window) > self._capacity:
            self._window.pop(0)

    def resolved(self):
        """回傳("present", (x, y))、("absent", None)或None（尚未收斂）。
        混合present/absent或全present但位置不穩定都回傳None，絕不
        擅自收斂成任何一種確定狀態。"""
        if (self._allow_absent and len(self._window) >= ABSENT_STABLE_COUNT and
                all(point is None
                    for point in self._window[-ABSENT_STABLE_COUNT:])):
            return ("absent", None)

        if len(self._window) < PRESENT_STABLE_COUNT:
            return None
        recent = self._window[-PRESENT_STABLE_COUNT:]
        if any(point is None for point in recent):
            return None
        anchor = recent[0]
        for point in recent[1:]:
            distance = math.hypot(point[0] - anchor[0], point[1] - anchor[1])
            if distance > OBSERVATION_POSITION_TOLERANCE_MM:
                return None
        xs = sorted(point[0] for point in recent)
        ys = sorted(point[1] for point in recent)
        mid = len(recent) // 2
        return ("present", (xs[mid], ys[mid]))


class _PocketClusterAccumulator:
    """跨影像累積未標記的原始袋口像素中心，聚成穩定的六個袋口位置。
    運作在像素空間，才能沿用DetectionFilter既有的P1-P6排序規則（該規則
    定義在像素座標，不是Base0），只有解出六個穩定位置後才轉投影+排序。"""

    def __init__(self):
        self._clusters = []  # [{"centroid": (x, y), "count": n}, ...]

    def add(self, pixel_point):
        for cluster in self._clusters:
            centroid = cluster["centroid"]
            if math.hypot(
                    pixel_point[0] - centroid[0],
                    pixel_point[1] - centroid[1]) <= POCKET_CLUSTER_TOLERANCE_PX:
                count = cluster["count"]
                cluster["centroid"] = (
                    (centroid[0] * count + pixel_point[0]) / (count + 1),
                    (centroid[1] * count + pixel_point[1]) / (count + 1))
                cluster["count"] = count + 1
                return
        self._clusters.append({"centroid": pixel_point, "count": 1})

    def resolved_pixel_centers(self):
        """六個穩定cluster的像素中心都收斂時回傳6個(x, y)；否則None。
        次數不足六個「穩定」cluster（count>=OBSERVATION_STABLE_COUNT）時
        視為尚未收斂——不會用不足六個的候選硬湊出P1-P6。"""
        stable = [c for c in self._clusters if c["count"] >= OBSERVATION_STABLE_COUNT]
        if len(stable) < POCKET_COUNT:
            return None
        stable.sort(key=lambda c: (-c["count"], c["centroid"][1], c["centroid"][0]))
        return [cluster["centroid"] for cluster in stable[:POCKET_COUNT]]


class CaptureWindowAccumulator:
    """一個START_CAPTURE window內的完整累積狀態。resolve一次就送一個
    Logical Frame、reset、獨立開始累積下一個，直到STOP_CAPTURE為止。"""

    def __init__(self):
        self.reset()

    def reset(self):
        self._balls = [_ObjectAccumulator(allow_absent=True)
                        for _ in range(CUE_CLASS_ID)]
        self._cue = _ObjectAccumulator(allow_absent=False)
        self._pockets = _PocketClusterAccumulator()
        self.images_fed = 0

    def feed(self, raw, projector):
        """raw是DetectionFilter.extract_raw()的結果；projector提供
        project(u, v) -> (base0_x, base0_y, base0_z)。單一物件這一輪投影
        失敗只讓那個物件視為這一輪沒觀測到，不中止整個capture window。"""
        self.images_fed += 1
        for class_id in range(CUE_CLASS_ID):
            box = raw.balls.get(class_id)
            self._balls[class_id].add(_project_box_or_none(box, projector))
        self._cue.add(_project_box_or_none(raw.balls.get(CUE_CLASS_ID), projector))
        for hole in raw.raw_holes:
            self._pockets.add(box_center(hole))

    def resolved_coordinates(self, projector):
        """全部物件都收斂時回傳既有32值coordinates tuple；否則None。
        六個袋口在像素空間解出穩定位置、排序成P1..P6後，才在這裡逐一
        投影成Base0；若某個已幾何收斂的袋口在這一步投影失敗，代表真正的
        幾何/校正問題，回報CaptureRejected由呼叫端決定重置範圍，不在這裡
        悄悄跳過。"""
        ball_results = [accumulator.resolved() for accumulator in self._balls]
        if any(result is None for result in ball_results):
            return None
        cue_result = self._cue.resolved()
        if cue_result is None or cue_result[0] != "present":
            return None
        pocket_pixel_centers = self._pockets.resolved_pixel_centers()
        if pocket_pixel_centers is None:
            return None
        ordered_pixel_centers = order_six_by_position(
            pocket_pixel_centers, lambda point: point)

        coordinates = [MISSING_COORDINATE] * FIELD_COUNT
        for class_id, (status, point) in enumerate(ball_results):
            if status == "present":
                coordinates[class_id * 2] = point[0]
                coordinates[class_id * 2 + 1] = point[1]
        coordinates[CUE_CLASS_ID * 2] = cue_result[1][0]
        coordinates[CUE_CLASS_ID * 2 + 1] = cue_result[1][1]

        for pocket_index, pixel_center in enumerate(ordered_pixel_centers):
            base0 = _project_point_or_none(pixel_center, projector)
            if base0 is None:
                raise CaptureRejected(
                    f"P{pocket_index + 1} resolved a stable pixel cluster but "
                    "geometry projection failed"
                )
            field_index = 20 + pocket_index * 2
            coordinates[field_index] = base0[0]
            coordinates[field_index + 1] = base0[1]
        return tuple(coordinates)

    def timed_out(self):
        return self.images_fed >= MAX_IMAGES_PER_LOGICAL_FRAME
