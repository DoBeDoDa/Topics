import cv2
import os
import time
import socket
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# ==========================================
# [參數設定區] 標籤代號與對應顏色
# ==========================================
label_map = {
    0: "b1", 1: "b2", 2: "b3", 3: "bw", 
    4: "p1", 5: "p2", 6: "p3", 7: "p4", 8: "p5", 9: "p6"
}

color_map = {
    0: (0, 255, 255), 1: (255, 0, 0), 2: (0, 0, 255), 3: (255, 255, 255),
    4: (128, 0, 128), 5: (0, 165, 255), 6: (0, 128, 0), 
    7: (0, 0, 128), 8: (0, 0, 0), 9: (0, 255, 255)
}

# ==========================================
# 🎯 [核心修改區] 無限多點 Homography 透視轉換
# ==========================================
def get_perspective_matrix():
    """
    [嚴格檢查重點]
    1. 必須使用 cv2.findHomography 取代原本的 cv2.getPerspectiveTransform。
    2. cam_points 與 table_points 的「數量」必須完全一樣。
    3. cam_points 的第 N 個座標，必須精準對應 table_points 的第 N 個物理座標。
    """
    
    # 📝 1. 填入你用放大鏡取樣出來的「超多」像素座標 (X, Y)
    # [請在此處無腦貼上你取樣的像素座標]
    cam_points = np.float32([

        [155, 61],   # 第 1 點 (例如 p2 袋口)
        [728, 65],   # 第 2 點 (例如 p5 袋口)
        [99, 637],   # 第 3 點 (例如 p1 袋口)
        [725, 662],  # 第 4 點 (例如 p4 袋口)
        # 👇 在下方繼續新增你的取樣點
        [1049, 276],    #1
        [1078, 343],    #2
        [1045, 555],    #3
        [1009, 480],    #4
        [999, 395],    #5
        [955, 344],   #6
        [963, 585],    #7
        [903, 524],    #8
        [940, 451],    #9
        [930, 400],    #10
        [872, 374],    #11
        [879, 324],    #12
        [841, 175],    #13
        [897, 630],    #14
        [822, 585],    #15
        [797, 514],    #16
        [850, 439],    #17
        [787, 462],    #18
        [770, 399],    #19
        [809, 362],    #20
        [737, 370],    #21
        [765, 290],    #22
        [740, 558],    #23
        [691, 455],    #24
        [665, 395],    #25
        [681, 332],    #26
        [652, 534],    #27
        [605, 477],    #28
        [546, 424],    #29
        [570, 361],    #30
        [590, 596],    #31
        [545, 521],    #32
        [478, 610],    #33
        [470, 545],    #34
        [482, 466],    #35
        [472, 393],    #36
        [452, 218],    #37
        [364, 621],    #38
        [362, 541],    #39
        [354, 472],    #40
        [399, 402],    #41
        [233, 601],    #42
        [261, 536],    #43
        [262, 460],    #44
        [95, 598],     #45
        [159, 534],    #46
        [190, 465],    #47
        [233, 420],    #48
        [310, 406],    #49
        [351, 371]     #50

    ])

    # 📝 2. 填入與上方嚴格對應的「實體物理座標 (mm)」
    # ⚠️ [嚴格檢查]：既然 p1 是你的 (0,0) 原點，請確保這裡所有物理坐標都是以 p1 為基準量測出來的！
    table_points = np.float32([
        [-753.128, 920.343],  # 對應上方第 1 點
        [-123.041, 935.936],  # 對應上方第 2 點
        [-753.128, 300.343],  # 對應上方第 3 點
        [-123.041, 285.936],  # 對應上方第 4 點
        # 👇 在下方繼續新增對應的物理座標
        [209.355, 678.979],     #1
        [232.005, 609.044],    #2
        [191.680, 394.457],    #3
        [169.642, 469.361],    #4
        [161.546, 559.416],    #5
        [119.387, 611.272],    #6
        [118.738, 364.380],    #7
        [61.489, 424.45],    #8
        [101.093, 499.958],    #9
        [93.605, 552.8],    #10
        [33.002, 574.854],    #11
        [43.764, 629.244],    #12
        [6.387, 792.965],    #13
        [49.478, 326.332],     #14
        [-2.562, 365.882],    #15
        [-27.070, 434.885],    #16
        [30.425, 510.537],    #17
        [-34.733, 485.186],    #18
        [-51.676, 548.555],    #19
        [-10.001, 587.702],    #20
        [-85.480, 578.003],    #21
        [-55.179, 661.177],    #22
        [-87.083, 390.648],    #23
        [-135.174, 491.203],    #24
        [-162.824, 552.302],    #25
        [-144.686, 616.405],    #26
        [-176.810, 411.993],    #27
        [-224.787, 466.836],    #28
        [-284.34, 519.480],    #29
        [-261.972, 587.280],    #30
        [-238.587, 353.897],    #31
        [-284.844, 422.411],    #32
        [-351.500, 339.361],    #33
        [-358.629, 399.105],    #34
        [-349.993, 477.707],    #35
        [-363.858, 553.035],    #36
        [-388.575, 737.336],    #37
        [-458.893, 325.884],    #38
        [-464.806, 402.283],    #39
        [-479.183, 471.338],    #40
        [-437.578, 542.357],    #41
        [-588.985, 344.193],    #42
        [-567.555, 406.646],    #43
        [-572.078, 483.038],    #44
        [-721.592, 346.772],    #45
        [-664.206, 407.464],    #46
        [-640.740, 479.017],    #47
        [-601.276, 522.381],    #48 
        [-524.687, 535.797],    #49
        [-487.323, 574.579]     #50
    ])

    # 🛡️ [嚴格檢查一] 防呆阻擋：數量不對等直接中斷程式
    if len(cam_points) != len(table_points):
        raise ValueError(f"\n[嚴重錯誤] 像素點有 {len(cam_points)} 個，但實體物理點只有 {len(table_points)} 個！請檢查是否少貼了哪一行！")

    # 🛡️ [嚴格檢查二] 點數太少防呆
    if len(cam_points) < 4:
        raise ValueError("\n[嚴重錯誤] 透視轉換最少需要 4 個點！")

# 🚀 將 cv2.RANSAC, 5.0 改成 cv2.LMEDS
    # LMEDS 不需要設定誤差範圍，它會自己找出這 50 個點中最完美的核心轉換率
    matrix, mask = cv2.findHomography(cam_points, table_points, cv2.LMEDS)
    
    return matrix

def cam_to_arm(x_cam, y_cam, matrix):
    # 這裡的 perspectiveTransform 可以直接吃 findHomography 吐出來的矩陣，完全不用改！
    pt = np.array([[[x_cam, y_cam]]], dtype=np.float32)
    pt_arm = cv2.perspectiveTransform(pt, matrix)
    return round(pt_arm[0][0][0], 1), round(pt_arm[0][0][1], 1)

# ==========================================
# [主程式區] 影像擷取、辨識與通訊服務
# ==========================================
def start_yolo_service(model_path="best.pt", port=12345):
    
    # 啟動時先算好這組神級矩陣
    M = get_perspective_matrix()
    
    if not os.path.exists(model_path):
        print(f"[系統錯誤] 找不到 YOLO 模型檔案: {model_path}")
        return
    
    print("[系統狀態] 正在載入 YOLO 模型...")
    model = YOLO(model_path)

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', port))
    server_socket.listen(1)
    
    print(f"[網路狀態] 等待 C++ 控制端連線中 (Port: {port})...")
    conn, addr = server_socket.accept()
    print(f"[網路狀態] 連線成功！來源 IP: {addr}")

    print("[硬體狀態] 正在啟動 Intel RealSense D435i 模組...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    try:
        pipeline.start(config)
        print("[系統狀態] D435i 硬體啟動成功，進入視覺伺服主迴圈。")
    except Exception as e:
        print(f"[硬體錯誤] 無法啟動 D435i: {e}")
        return
    
    last_print_time = 0 

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame: continue

            frame = np.asanyarray(color_frame.get_data())
            annotated_frame = frame.copy()

            results = model(frame, conf=0.3, verbose=False)

            best_boxes = {}
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                if cls_id not in best_boxes:
                    best_boxes[cls_id] = box

            # C++ 端要求沒有球時傳送 -9999.0
            coords = [-9999.0] * 20            
            display_data = {}

            for cls_id in range(10):
                short_name = label_map[cls_id]
                box_color = color_map.get(cls_id, (0, 255, 0))
                
                if cls_id in best_boxes:
                    box = best_boxes[cls_id]
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                    
                    arm_x, arm_y = cam_to_arm(cx, cy, M)
                    
                    coords[cls_id * 2] = arm_x
                    coords[cls_id * 2 + 1] = arm_y

                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), box_color, 2)
                    cv2.circle(annotated_frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                    cv2.putText(annotated_frame, short_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
                    display_data[short_name] = f"{arm_x:>7.1f}, {arm_y:>7.1f}"
                else:
                    display_data[short_name] = "   --- ,    --- "

            current_time = time.time()
            if current_time - last_print_time > 0.5:
                os.system('cls' if os.name == 'nt' else 'clear')
                print("=====================================================")
                print(" [D435i] 機器人視覺絕對座標面板 (單位: mm, 原點: p1)")
                print("=====================================================")
                print(f" [1號球 b1] {display_data['b1']}   |   [左上 p1] {display_data['p1']}")
                print(f" [2號球 b2] {display_data['b2']}   |   [中上 p2] {display_data['p2']}")
                print(f" [3號球 b3] {display_data['b3']}   |   [右上 p3] {display_data['p3']}")
                print(f" [母  球 bw] {display_data['bw']}   |   [左下 p4] {display_data['p4']}")
                print(f"                              |   [中下 p5] {display_data['p5']}")
                print(f"                              |   [右下 p6] {display_data['p6']}")
                print("=====================================================")
                print(" 系統提示: 請在相機影像視窗按下 'q' 結束程式")
                last_print_time = current_time

            msg = ",".join(map(str, coords))
            try:
                conn.sendall((msg + "\n").encode('utf-8'))
            except: 
                print("\n[網路警告] C++ 端已斷線，終止視覺伺服服務。")
                break 

            cv2.imshow("Direct Arm Vision (Intel RealSense)", annotated_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q') or key == 27: 
                break

    finally:
        print("[系統狀態] 正在關閉相機與通訊埠...")
        pipeline.stop()
        cv2.destroyAllWindows()
        conn.close()
        server_socket.close()

if __name__ == "__main__":
    start_yolo_service()