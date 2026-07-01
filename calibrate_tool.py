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

def start_calibration_service(model_path="best.pt", port=12347):
    # 載入 YOLO 模型
    if not os.path.exists(model_path):
        print(f"[系統錯誤] 找不到 YOLO 模型檔案: {model_path}")
        return
    print("[系統狀態] 正在載入 YOLO 模型...")
    model = YOLO(model_path)

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

    label_map = {0: "b1", 1: "b2", 2: "b3", 3: "bw"}
    required_labels = [0, 1, 2, 3] # b1, b2, b3, bw
    balls_order = ["bw", "b1", "b2", "b3"]
    ball_names = ["母球 (bw)", "1號球 (b1)", "2號球 (b2)", "3號球 (b3)"]

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
            print(f"  請確保母球 (bw)、1號球 (b1)、2號球 (b2)、3號球 (b3) 都在鏡頭視野內")
            print(f"=========================================")

            pixel_coords = {}
            
            # Phase 1: 偵測階段，必須同時偵測到全部 4 顆球
            while True:
                ret, frame = cap.read()
                if not ret: continue
                annotated_frame = frame.copy()

                # YOLO 偵測 (映射新模型分類 0:cue, 1:b1, 2:b2, 3:b3 至舊系統 0:b1, 1:b2, 2:b3, 3:bw)
                results = model(frame, conf=0.3, verbose=False)
                balls = {}
                for box in results[0].boxes:
                    c = int(box.cls[0])
                    if c not in balls:
                        balls[c] = box
                    elif box.conf[0] > balls[c].conf[0]:
                        balls[c] = box
                
                best_boxes = {}
                if 0 in balls: best_boxes[0] = balls[0]  # b1 (來自新 class 0)
                if 1 in balls: best_boxes[1] = balls[1]  # b2 (來自新 class 1)
                if 2 in balls: best_boxes[2] = balls[2]  # b3 (來自新 class 2)
                if 9 in balls: best_boxes[3] = balls[9]  # bw (來自新 class 9)

                current_detect = {}
                for cls_id in required_labels:
                    label_name = label_map[cls_id]
                    if cls_id in best_boxes:
                        box = best_boxes[cls_id]
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                        
                        current_detect[label_name] = (round(cx, 1), round(cy, 1))
                        
                        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.circle(annotated_frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                        cv2.putText(annotated_frame, label_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # 繪製畫面提示
                missing = [label_map[c] for c in required_labels if label_map[c] not in current_detect]
                if missing:
                    prompt_text = f"Status: Detecting balls... Missing: {', '.join(missing)}"
                    cv2.putText(annotated_frame, prompt_text, (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    print(f"\r[偵測中] 缺少球種: {', '.join(missing)}     ", end="", flush=True)
                else:
                    cv2.putText(annotated_frame, "Status: All 4 balls locked!", (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    print("\n[成功] 已同時偵測到全部 4 顆撞球！")
                    pixel_coords = current_detect
                    break

                cv2.imshow("Calibration Tool (Orbbec Camera)", annotated_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == ord('Q') or key == 27:
                    print("\n[系統] 使用者結束程式。")
                    return

            # 發送 START_CALIBRATION 控制信號給 C++ 手臂端
            print("[網路] 發送標定信號 START_CALIBRATION 至 C++...")
            try:
                # 先清空 Socket 接收緩衝區
                try:
                    while conn.recv(1024): pass
                except BlockingIOError:
                    pass
                conn.sendall(b"START_CALIBRATION\n")
            except Exception as e:
                print(f"[網路錯誤] 無法發送控制信號至 C++: {e}")
                break

            # Phase 2: 連續畫面更新，同時非阻塞接收 C++ 手臂座標
            arm_coords = {}
            current_target_idx = 0
            socket_buffer = ""

            while current_target_idx < 4:
                ret, frame = cap.read()
                if not ret: continue
                annotated_frame = frame.copy()

                # 即時更新背景 YOLO 辨識 (映射新模型分類 0:cue, 1:b1, 2:b2, 3:b3 至舊系統 0:b1, 1:b2, 2:b3, 3:bw)
                results = model(frame, conf=0.3, verbose=False)
                balls = {}
                for box in results[0].boxes:
                    c = int(box.cls[0])
                    if c not in balls:
                        balls[c] = box
                    elif box.conf[0] > balls[c].conf[0]:
                        balls[c] = box
                
                best_boxes = {}
                if 0 in balls: best_boxes[0] = balls[0]  # b1 (來自新 class 0)
                if 1 in balls: best_boxes[1] = balls[1]  # b2 (來自新 class 1)
                if 2 in balls: best_boxes[2] = balls[2]  # b3 (來自新 class 2)
                if 9 in balls: best_boxes[3] = balls[9]  # bw (來自新 class 9)

                for cls_id in required_labels:
                    label_name = label_map[cls_id]
                    if cls_id in best_boxes:
                        box = best_boxes[cls_id]
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (255, 255, 0), 2)
                        cv2.circle(annotated_frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                        cv2.putText(annotated_frame, label_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

                # 繪製半透明提示遮罩以突顯操作提示
                overlay = annotated_frame.copy()
                cv2.rectangle(overlay, (0, 0), (1280, 80), (0, 0, 0), -1)
                cv2.addWeighted(overlay, 0.6, annotated_frame, 0.4, 0, annotated_frame)

                # 在螢幕上顯示清晰的操作工作流提示
                target_ball_name = ball_names[current_target_idx]
                guide_text1 = f"Step {current_target_idx + 1}/4: Move robot arm to center of [{target_ball_name}]"
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
                                if ball_name == balls_order[current_target_idx]:
                                    if arm_x < -9000.0 or arm_y < -9000.0:
                                        print(f" --> [對齊記錄] {ball_name} 被跳過。")
                                    else:
                                        arm_coords[ball_name] = (arm_x, arm_y)
                                        print(f" --> [對齊記錄] {ball_name} 成功配對：像素 {pixel_coords[ball_name]} -> 手臂 ({arm_x}, {arm_y})")
                                    current_target_idx += 1
                except BlockingIOError:
                    pass
                except ConnectionResetError:
                    print("[網路錯誤] C++ 斷開連線。")
                    break

            if len(arm_coords) < 3:
                print(f"[錯誤] 標定中斷，至少需要收集 3 個有效點位（目前僅收集到 {len(arm_coords)} 個點），無法計算校正矩陣。")
                break

            # 3. 輸出並保存結果
            print("\n=========================================")
            print(f"  第 {round_count} 輪標定數據完成！")
            print("=========================================")
            
            valid_balls = [b for b in balls_order if b in arm_coords]
            cam_points_list = [pixel_coords[b] for b in valid_balls]
            table_points_list = [arm_coords[b] for b in valid_balls]

            print("\n[Python 格式點位數據]：")
            print(f"cam_points = np.float32({cam_points_list})")
            print(f"table_points = np.float32({table_points_list})")
            
            # 備份至 calibrated_points.txt
            with open("calibrated_points.txt", "a", encoding="utf-8") as f:
                f.write(f"\n--- Round {round_count} ({time.strftime('%Y-%m-%d %H:%M:%S')}) ---\n")
                f.write(f"cam_points = np.float32({cam_points_list})\n")
                f.write(f"table_points = np.float32({table_points_list})\n")
            print("\n[系統] 標定點位已寫入檔案 'calibrated_points.txt'")

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
