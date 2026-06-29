import cv2
import socket
import numpy as np
from ultralytics import YOLO
import os
import time

# ==========================================
# [相機設定] 設定您所使用的相機 USB Index
# ==========================================
CAMERA_INDEX = 2  # 如果讀取不到相機，請嘗試修改為 0, 1, 2...
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

    # 啟動 USB 相機 (例如 Orbbec Gemini 2 XL)
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

    try:
        round_count = 1
        while True:
            print(f"\n=========================================")
            print(f"  開始第 {round_count} 輪標定偵測...")
            print(f"  請確保母球 (bw)、1號球 (b1)、2號球 (b2)、3號球 (b3) 都在鏡頭視野內")
            print(f"=========================================")

            pixel_coords = {}
            
            # 1. 偵測階段：必須同時偵測到母球、1、2、3號球的中心像素座標
            while True:
                ret, frame = cap.read()
                if not ret: continue
                annotated_frame = frame.copy()

                results = model(frame, conf=0.3, verbose=False)
                
                # 追蹤每個類別的最佳外框
                best_boxes = {}
                for box in results[0].boxes:
                    cls_id = int(box.cls[0])
                    if cls_id in required_labels and cls_id not in best_boxes:
                        best_boxes[cls_id] = box

                # 標註畫面並記錄像素位置
                current_detect = {}
                for cls_id in required_labels:
                    label_name = label_map[cls_id]
                    if cls_id in best_boxes:
                        box = best_boxes[cls_id]
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                        
                        current_detect[label_name] = (round(cx, 1), round(cy, 1))
                        
                        # 繪製標記
                        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.circle(annotated_frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                        cv2.putText(annotated_frame, label_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # 終端機顯示目前缺少哪顆球
                missing = [label_map[c] for c in required_labels if label_map[c] not in current_detect]
                if missing:
                    print(f"\r[偵測中] 缺少球種: {', '.join(missing)}     ", end="", flush=True)
                else:
                    print("\n[成功] 已同時偵測到全部 4 顆撞球！")
                    pixel_coords = current_detect
                    break

                cv2.imshow("Calibration Detector (RealSense)", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("\n[系統] 使用者結束程式。")
                    return

            # 2. 座標配對與 C++ 手臂位置收集階段
            print("\n[網路] 發送標定信號 START_CALIBRATION 至 C++...")
            try:
                conn.sendall(b"START_CALIBRATION\n")
            except Exception as e:
                print(f"[網路錯誤] 無法發送控制信號至 C++: {e}")
                break

            arm_coords = {}
            # 接收 C++ 回傳的 4 個球點實體座標
            error_occurred = False
            for i in range(4):
                try:
                    data = conn.recv(1024).decode('utf-8')
                    if not data:
                        print("[網路錯誤] C++ 斷開連線。")
                        error_occurred = True
                        break
                    
                    # 格式: ball_name,X,Y
                    parts = data.strip().split(',')
                    if len(parts) == 3:
                        ball_name = parts[0]
                        arm_x = float(parts[1])
                        arm_y = float(parts[2])
                        arm_coords[ball_name] = (arm_x, arm_y)
                        print(f" --> [對齊記錄] {ball_name} 成功配對：像素 {pixel_coords[ball_name]} -> 手臂 ({arm_x}, {arm_y})")
                    else:
                        print(f"[錯誤] 接收到的資料格式不符：{data}")
                        error_occurred = True
                        break
                except Exception as e:
                    print(f"[錯誤] 接收手臂座標時發生異常: {e}")
                    error_occurred = True
                    break

            if error_occurred:
                break

            # 3. 儲存與列印結果
            print("\n=========================================")
            print(f"  第 {round_count} 輪標定數據完成！")
            print("=========================================")
            
            # 列印出程式碼格式的點位
            balls_order = ["bw", "b1", "b2", "b3"]
            cam_points_list = [pixel_coords[b] for b in balls_order]
            table_points_list = [arm_coords[b] for b in balls_order]

            print("\n[Python 格式點位數據]：")
            print(f"cam_points = np.float32({cam_points_list})")
            print(f"table_points = np.float32({table_points_list})")
            
            # 寫入文字檔備份
            with open("calibrated_points.txt", "a", encoding="utf-8") as f:
                f.write(f"\n--- Round {round_count} ({time.strftime('%Y-%m-%d %H:%M:%S')}) ---\n")
                f.write(f"cam_points = np.float32({cam_points_list})\n")
                f.write(f"table_points = np.float32({table_points_list})\n")
            print("\n[系統] 標定點位已寫入檔案 'calibrated_points.txt'")

            # 提示並等待 5 秒後進行下一輪
            print("\n5 秒後將回到初始狀態，開始新的一輪標定...")
            time.sleep(5)
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
