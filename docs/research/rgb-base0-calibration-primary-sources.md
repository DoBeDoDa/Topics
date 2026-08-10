# RGB optical pixel → HIWIN Base0：官方資料研究紀錄

日期：2026-08-10

範圍：Orbbec Gemini 2 XL、Orbbec SDK v1.10.18、HIWIN RA605/HRSDK 姿態標示。

目的：只記錄可由官方一手資料支持的內容，並清楚隔離使用者暫時核准的假設。

## 結論

1. Gemini RGB optical frame 採右手座標：`+X` 影像右、`+Y` 影像下、`+Z` 朝鏡頭前方場景。這是 optical frame，不是 ROS 非 optical sensor frame。
2. SDK v1 的 `VideoStreamProfile::getDistortion()` 官方註解稱為 Brown distortion model；`OBCameraDistortion` 公開欄位順序是 `k1..k6,p1,p2`。公開 profile 回傳型別沒有攜帶可唯一決定手寫 OpenCV 方程式的 model enum，因此工具不手動重排係數，改用官方 SDK inverse projection。
3. `CoordinateTransformHelper::calibration2dTo3dUndistortion` 官方定義會做 undistortion，輸入 2D pixel、depth mm 與 source/target sensor。以 COLOR→COLOR、合成正深度 1 mm 得到 ray scale，再正規化，是本工具的受驗證用法；同時用 1000 mm、回投影及 COLOR XY tables 交叉檢查，任何不一致即停止。1 mm 絕不是量測深度。
4. HIWIN 公開資料可支持 A/RX 對 X 軸、B/RY 對 Y 軸、C/RZ 對 Z 軸，以及角度單位 degree；找到的公開 HRSDK/communication manuals 不足以唯一決定 absolute pose 的 intrinsic/extrinsic、active/passive、矩陣相乘順序、角域與奇異點規則，也沒有找到官方 ABC→rotation-matrix utility。
5. 因此 `R=Rz(C)*Ry(B)*Rx(A)` 只能標成 `user_approved_temporary`，不能描述成 HIWIN 官方規則。

## Orbbec 官方來源

- [Orbbec ROS 2 coordinate systems and TF](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/4_application_guide/coordinate_and_tf.html)：camera optical coordinate system 的 X/Y/Z 方向。
- [OrbbecSDK_ROS2 optical-frame transform source](https://github.com/orbbec/OrbbecSDK_ROS2/blob/main/orbbec_camera/src/ob_camera_node.cpp#L2469-L2524)：SDK optical translation 轉成 ROS robot frame，以及 per-stream optical frame 發布邏輯。
- [Gemini 2 XL official product page](https://www.orbbec.com/products/stereo-vision-camera/gemini-2-xl/)：機型與官方產品資料入口。
- [Orbbec SDK v1 `VideoStreamProfile::getDistortion`](https://github.com/orbbec/OrbbecSDK/blob/main/include/libobsensor/hpp/StreamProfile.hpp#L65-L72)：官方註解 `Brown distortion model`。
- [Orbbec SDK v1 camera calibration types](https://github.com/orbbec/OrbbecSDK/blob/main/include/libobsensor/h/ObTypes.h#L351-L455)：RGB intrinsic、`OBCameraDistortion` 欄位與 calibration parameter 定義。
- [Orbbec `CoordinateTransformHelper` API](https://orbbec.github.io/OrbbecSDK/doc/api/English/classob_1_1CoordinateTransformHelper.html)：2D→3D undistortion、3D→2D 與 XY table API。
- [Orbbec SDK v1 Pipeline API](https://orbbec.github.io/OrbbecSDK/doc/api/English/classob_1_1Pipeline.html)：依 configured profile 取得 `OBCalibrationParam`。
- [Official Transformation sample](https://github.com/orbbec/OrbbecSDK/blob/main/examples/cpp/Sample-Transformation/Transformation.cpp)：以 sensor type 初始化 XY tables 的官方範例。

本機安裝檢查：

- 安裝路徑：`C:\Program Files\OrbbecSDK 1.10.18\SDK`
- 官方 GitHub installer asset 與本機下載檔 SHA-256 相同：`9FA35BE61849245984D17DA32E215326BFD37CD1064FDD63EE6E49E7513BCD24`
- 建置時與執行時皆檢查 SDK version 為 `1.10.18`。

## HIWIN 官方來源與限制

- [HIWIN Software Development Kit manual](https://www.hiwinsupport.com/download/tech_doc/mar/Software_Development_Kit-%28E%29.pdf)：HRSDK Cartesian pose 使用六值 `{X,Y,Z,A,B,C}`，`get_current_position` 回傳 current absolute Cartesian coordinate；`ptp_pos` 的第二參數是 motion mode，不是 Tool number。
- [HIWIN Robot Communication manual](https://www.hiwinsupport.com/download/tech_doc/mar/Robot_Communication_Manual-%28E%29.pdf)：A=RX、B=RY、C=RZ，orientation 使用 degree 尺度（通訊整數單位為 0.001 degree）。

公開文件沒有提供足以推出唯一 rotation matrix 的完整 absolute-pose 慣例。Jog 畫面或沿 RX/RY/RZ 的增量旋轉，只能證明使用者如何沿所選座標軸 jog，不能反推 absolute ABC pose 的矩陣乘法順序。因此工具必須：

- 把使用者核准的 Z‑Y‑X 公式寫入每份輸出。
- 明確標示來源為暫定使用者規則。
- 在實機多點 ground truth 通過前，禁止主程式或運動控制使用。

## 實裝決策更新

本研究最初建議使用需要完整 `OBCalibrationParam` 的 SDK inverse projection；Gemini 2 XL 實機的 Color/Depth profile 無法取得 matched camera param 後，使用者核准改為 RGB-only。現行決策與驗收邊界以 `docs/specs/rgb-only-calibration-spec.md` 為準：只讀取選定 Color profile 的 K/D，使用明確版本化且標為工程假設的 Brown rational inverse solver，並要求實機 ground truth。下列原則中「Python 只做 YOLO」與其他座標／安全邊界仍有效，但舊的 SDK helper／XY-table 交叉檢查已被取代。

- C++：Orbbec v1.10.18 camera/profile/calibration/ray、HRSDK 靜止姿態讀取、座標矩陣與 Base0 平面交點。
- Python：只執行既有 Ultralytics YOLO model，不控制相機、不做座標轉換。
- RGB frame：只接受原始 1280×720 MJPG，不允許 resize、crop、mirror、flip、rotation 或 letterbox 後的座標。
- Ray：完全不使用 Depth；以 `rgb_brown_rational_v1` 反解、有限迭代、奇異／發散拒絕與 ≤0.25 px forward round-trip fail closed。此映射是待 ground truth 驗證的工程假設，不是官方方程已證實。
- 平面：依使用者 2026-08-10 更新的球徑 44.5 mm，球中心使用 `Z_table + 22.25 mm`；lambda 只由 Base0 plane intersection 計算。
- 驗收：至少 6 點、RMS XY ≤3 mm、每點 ≤5 mm；影像位置系統性趨勢以可記錄、可調整的 Pearson correlation gate 操作化，預設絕對值 0.7，並明確聲明不是官方規格。
