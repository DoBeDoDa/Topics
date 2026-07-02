import cv2
import socket
import numpy as np
from ultralytics import YOLO
import os
import time

# ==========================================
# [相機設定] 設定您所使用的相機 USB Index
# ==========================================
CAMERA_INDEX = 1  # 依使用者表示，設定 1 是對的
# ==========================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
DEFAULT_MODEL_PATH = os.path.join(ROOT_DIR, "bin", "best.pt")
CALIBRATION_TXT_PATH = os.path.join(ROOT_DIR, "calibrated_points.txt")

def start_calibration_service(model_path=None, port=12347):
    # 啟動 TCP Socket 伺服器
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', port))
    server_socket.listen(1)
    print(f"\n[網路狀態] 等待 C++ 標定端連線中 (Port: {port})...")
    conn, addr = server_socket.accept()
    print(f"[網路狀態] 連線成功！來源 IP: {addr}")

    # 設定非阻塞模式以實現流暢的 OpenCV 畫面更新
    conn.setblocking(False)

    # 啟動 USB 相機 (Orbbec Gemini 2 XL)
    print(f"[硬體狀態] 正在啟動 USB 相機 (Index: {CAMERA_INDEX})...")
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"[硬體錯誤] 無法開啟相機索引 {CAMERA_INDEX}！")
        conn.close()
        server_socket.close()
        return
    
    # 設定解析度
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    print(f"[硬體狀態] 相機啟動成功，進入標定工作流。")

    # 棋盤格設定：偵測 8x5 的角點（9x6格）
    pattern_size = (8, 5)

    try:
        round_count = 1
        while True:
            # 1. 等待 C++ 端發送 START_DETECTION 指令才啟動辨識
            print("\n[相機] 等待 C++ 端發送 START_DETECTION 指令 (請在 C++ 按下 Enter)...")
            socket_buffer = ""
            start_detection_received = False
            
            while not start_detection_received:
                ret, frame = cap.read()
                if ret:
                    overlay = frame.copy()
                    cv2.rectangle(overlay, (0, 0), (1280, 70), (0, 0, 0), -1)
                    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
                    cv2.putText(frame, "Waiting for C++ to start detection...", (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    cv2.imshow("Calibration Tool (Orbbec Camera)", frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == ord('Q') or key == 27:
                    print("\n[系統] 使用者結束程式。")
                    return
                
                try:
                    data = conn.recv(1024).decode('utf-8')
                    if data:
                        socket_buffer += data
                        if "START_DETECTION" in socket_buffer:
                            start_detection_received = True
                except BlockingIOError:
                    pass
                except ConnectionResetError:
                    print("[網路錯誤] C++ 斷開連線。")
                    return

            print(f"\n=========================================")
            print(f"  開始第 {round_count} 輪標定偵測...")
            print(f"  請確保黑白棋盤格 (8x5角點) 完整呈現在畫面中")
            print(f"=========================================")

            corners_refined = None
            
            # Phase 1: 偵測階段，必須成功偵測到完整棋盤格
            while True:
                ret, frame = cap.read()
                if not ret: continue
                annotated_frame = frame.copy()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # 偵測棋盤格角點
                found, corners = cv2.findChessboardCorners(gray, pattern_size, None)

                if found:
                    # 亞像素級精細化角點
                    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                     criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                    
                    # 繪製偵測到的棋盤格角點
                    cv2.drawChessboardCorners(annotated_frame, pattern_size, corners, found)
                    
                    # 標示出 Corner A (紅圈，起點 0) 與 Corner B (藍圈，首列終點 7)
                    ptA = tuple(map(int, corners[0][0]))
                    ptB = tuple(map(int, corners[7][0]))
                    
                    cv2.circle(annotated_frame, ptA, 10, (0, 0, 255), -1)
                    cv2.putText(annotated_frame, "A (Origin)", (ptA[0] + 15, ptA[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    cv2.circle(annotated_frame, ptB, 10, (255, 0, 0), -1)
                    cv2.putText(annotated_frame, "B (X-Axis)", (ptB[0] + 15, ptB[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

                    cv2.putText(annotated_frame, "Status: Chessboard Corners LOCKED!", (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    corners_refined = corners
                else:
                    cv2.putText(annotated_frame, "Status: Searching Chessboard (8x5 corners)...", (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                cv2.imshow("Calibration Tool (Orbbec Camera)", annotated_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == ord('Q') or key == 27:
                    print("\n[系統] 使用者結束程式。")
                    return

                # 若成功鎖定且已偵測到角點，則跳出 Phase 1 進入標定
                if found:
                    print("\n[成功] 已鎖定黑白棋盤格角點！")
                    break

            # 發送 START_CALIBRATION 控制信號給 C++ 手臂端
            print("[網路] 發送標定信號 START_CALIBRATION 至 C++...")
            try:
                try:
                    while conn.recv(1024): pass
                except BlockingIOError:
                    pass
                conn.sendall(b"START_CALIBRATION\n")
            except Exception as e:
                print(f"[網路錯誤] 無法發送控制信號至 C++: {e}")
                break

            # Phase 2: 連續畫面更新，同時非阻塞接收 C++ 手臂傳送的 2 個角點座標
            arm_coords = {}
            current_target_idx = 0
            socket_buffer = ""
            targets = ["ptA", "ptB"]
            target_names = ["Corner A (Red Circle)", "Corner B (Blue Circle)"]

            while current_target_idx < 2:
                ret, frame = cap.read()
                if not ret: continue
                annotated_frame = frame.copy()

                # 即時在畫面上繪製鎖定的角點標記
                if corners_refined is not None:
                    cv2.drawChessboardCorners(annotated_frame, pattern_size, corners_refined, True)
                    ptA = tuple(map(int, corners_refined[0][0]))
                    ptB = tuple(map(int, corners_refined[7][0]))
                    cv2.circle(annotated_frame, ptA, 10, (0, 0, 255), -1)
                    cv2.putText(annotated_frame, "A (Origin)", (ptA[0] + 15, ptA[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.circle(annotated_frame, ptB, 10, (255, 0, 0), -1)
                    cv2.putText(annotated_frame, "B (X-Axis)", (ptB[0] + 15, ptB[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

                # 繪製半透明提示遮罩以突顯操作提示
                overlay = annotated_frame.copy()
                cv2.rectangle(overlay, (0, 0), (1280, 80), (0, 0, 0), -1)
                cv2.addWeighted(overlay, 0.6, annotated_frame, 0.4, 0, annotated_frame)

                # 在螢幕上顯示清晰的操作工作流提示
                target_ball_name = target_names[current_target_idx]
                guide_text1 = f"Step {current_target_idx + 1}/2: Move robot arm to [{target_ball_name}]"
                guide_text2 = "After positioning, press [Enter] in the C++ terminal window."
                cv2.putText(annotated_frame, guide_text1, (30, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(annotated_frame, guide_text2, (30, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

                cv2.imshow("Calibration Tool (Orbbec Camera)", annotated_frame)
                cv2.waitKey(1)

                # 非阻塞接收 C++ 傳送的手臂座標
                try:
                    data = conn.recv(1024).decode('utf-8')
                    if data:
                        socket_buffer += data
                        while "\n" in socket_buffer:
                            line, socket_buffer = socket_buffer.split("\n", 1)
                            parts = line.strip().split(',')
                            if len(parts) == 3:
                                ball_name = parts[0]
                                arm_x = float(parts[1])
                                arm_y = float(parts[2])
                                if ball_name == targets[current_target_idx]:
                                    if arm_x < -9000.0 or arm_y < -9000.0:
                                        print(f" --> [錯誤] 棋盤基準點 {ball_name} 不能被跳過，標定中斷。")
                                        current_target_idx = 99 # 終止
                                    else:
                                        arm_coords[ball_name] = (arm_x, arm_y)
                                        print(f" --> [對齊記錄] {ball_name} 成功配對：手臂 ({arm_x}, {arm_y})")
                                        current_target_idx += 1
                except BlockingIOError:
                    pass
                except ConnectionResetError:
                    print("[網路錯誤] C++ 斷開連線。")
                    break

            if len(arm_coords) < 2:
                print("[錯誤] 標定中斷，未收集齊全 A、B 兩個基準點。")
                break

            # 3. 幾何推算 40 個角點的實際物理座標，並輸出結果
            print("\n=========================================")
            print(f"  第 {round_count} 輪標定數據計算中...")
            print("=========================================")

            x_A, y_A = arm_coords["ptA"]
            x_B, y_B = arm_coords["ptB"]

            # 計算 A 到 B 在手臂空間中的距離與角度
            dx = x_B - x_A
            dy = y_B - y_A
            distance_arm = np.sqrt(dx**2 + dy**2)
            theta = np.arctan2(dy, dx)

            # 由於 A 點到 B 點之間橫跨了 7 個格子 (橫向 8 個點)
            actual_square_size = 25.0
            print(f"[計算資訊] A-B 測量距離: {distance_arm:.2f} mm (以 2.5cm 方格估計理論值應為 175.0 mm)")
            print(f"[計算資訊] 測量推算單格邊長: {distance_arm / 7.0:.2f} mm")
            print(f"[計算資訊] 套用固定單格邊長: {actual_square_size:.2f} mm")
            print(f"[計算資訊] 旋轉角度: {np.degrees(theta):.2f} 度")

            cos_t = np.cos(theta)
            sin_t = np.sin(theta)

            cam_points_list = []
            table_points_list = []

            # 依序計算棋盤格上所有 40 個點對應的手臂座標
            for k in range(40):
                col = k % 8
                row = k // 8
                # 棋盤格局部空間座標
                x_board = col * actual_square_size
                y_board = row * actual_square_size

                # 剛體旋轉與平移 (Rotation + Translation) 轉換至手臂空間
                x_arm = x_A + (x_board * cos_t - y_board * sin_t)
                y_arm = y_A + (x_board * sin_t + y_board * cos_t)

                # 像素與物理座標配對
                cam_points_list.append([float(corners_refined[k][0][0]), float(corners_refined[k][0][1])])
                table_points_list.append([round(x_arm, 1), round(y_arm, 1)])

            print(f"\n[成功] 已自動產生全量 {len(table_points_list)} 個角點的對照數據！")

            # 4. 輸出並保存結果
            print("\n[Python 格式點位數據]（請將下方兩行內容貼至 robot.py 中的對應變數）：")
            print(f"DEFAULT_CAM_POINTS = np.float32({cam_points_list})")
            print(f"DEFAULT_TABLE_POINTS = np.float32({table_points_list})")
            
            # 計算 Homography 矩陣以便存檔備份
            matrix, _ = cv2.findHomography(np.float32(cam_points_list), np.float32(table_points_list), cv2.LMEDS)
            matrix_str = np.array2string(matrix, separator=', ')

            # 備份至 calibrated_points.txt
            with open(CALIBRATION_TXT_PATH, "a", encoding="utf-8") as f:
                f.write(f"\n--- Round {round_count} ({time.strftime('%Y-%m-%d %H:%M:%S')}) ---\n")
                f.write(f"DEFAULT_CAM_POINTS = np.float32({cam_points_list})\n")
                f.write(f"DEFAULT_TABLE_POINTS = np.float32({table_points_list})\n")
                f.write(f"# Calculated Homography Matrix:\n# {matrix_str}\n")
            print(f"\n[系統] 標定數據已追加寫入檔案 '{CALIBRATION_TXT_PATH}'")

            round_count += 1

    finally:
        print("[系統狀態] 正在釋放資源與關閉連線...")
        if 'cap' in locals() and cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        conn.close()
        server_socket.close()

if __name__ == "__main__":
    start_calibration_service()
