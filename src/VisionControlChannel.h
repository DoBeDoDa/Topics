// Python視覺capture window的控制通道：C++->Python送出
// START_CAPTURE/STOP_CAPTURE純文字控制訊息，走既有SocketClient TCP連線
// 的同一條線（Python同一個socket上讀控制行、寫Logical Frame資料行）。
// 刻意獨立於VisionDataParser.cpp——那裡只解析32值CSV資料幀，不處理
// 控制協定，兩者不共用剖析邏輯，也不共用失敗語意。
#pragma once

#include "SocketClient.h"
#include "TableState.h"

enum class VisionControlSendStatus {
    Success,
    InvalidCycleIdentity,
    SendFailed
};

// cycleIdentity==0視為不合法（ShotCycleIdentity的保留值），直接回
// InvalidCycleIdentity，不嘗試送出。
[[nodiscard]] VisionControlSendStatus sendStartCapture(
    SocketClient& visionClient,
    ShotCycleIdentity cycleIdentity);

[[nodiscard]] VisionControlSendStatus sendStopCapture(
    SocketClient& visionClient,
    ShotCycleIdentity cycleIdentity);
