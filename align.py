import cv2
import socket
import numpy as np

# 建立空函式供 Trackbar 使用
def on_trackbar(val):
    pass

def start_align_service(port=12346):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', port))
    server_socket.listen(1)
    
    print(f"[視覺伺服] 通訊埠已就緒。等待 main.cpp 連線 (Port: {port})...")
    conn, addr = server_socket.accept()
    print(f"[視覺伺服] C++ 連線成功！啟動 OpenCV 動態輪廓辨識...")
    
    CAMERA_INDEX = 2 
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    
    if not cap.isOpened():
        print(f"[硬體錯誤] 無法開啟相機索引 {CAMERA_INDEX}！")
        conn.close()
        server_socket.close()
        return

    TARGET_WIDTH = 640
    TARGET_HEIGHT = 480
    FRAME_CENTER_X = TARGET_WIDTH // 2 

    # 建立視窗與動態調整滑桿 (預設值 120，最大 255)
    cv2.namedWindow("OpenCV Contour Align")
    cv2.createTrackbar("Threshold", "OpenCV Contour Align", 120, 255, on_trackbar)

    try:
        while True:
            ret, raw_frame = cap.read()
            if not ret: continue

            frame = cv2.resize(raw_frame, (TARGET_WIDTH, TARGET_HEIGHT), interpolation=cv2.INTER_AREA)
            annotated_frame = frame.copy()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (7, 7), 0)
            
            # 即時讀取滑桿數值作為二值化門檻
            current_threshold = cv2.getTrackbarPos("Threshold", "OpenCV Contour Align")
            _, thresh = cv2.threshold(blurred, current_threshold, 255, cv2.THRESH_BINARY)
            
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            best_center = None
            best_radius = 0
            
            # 放寬面積與半徑的篩選條件
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 50:  
                    (x, y), radius = cv2.minEnclosingCircle(cnt)
                    if 10 < radius < 250: 
                        if radius > best_radius:
                            best_radius = radius
                            best_center = (int(x), int(y))

            error_x = -9999.0 
            if best_center is not None:
                error_x = best_center[0] - FRAME_CENTER_X
                cv2.circle(annotated_frame, best_center, int(best_radius), (0, 255, 255), 2)
                cv2.circle(annotated_frame, best_center, 5, (0, 0, 255), -1)

            cv2.line(annotated_frame, (FRAME_CENTER_X, 0), (FRAME_CENTER_X, TARGET_HEIGHT), (0, 255, 0), 2)
            cv2.putText(annotated_frame, f"Error px: {error_x:.1f}", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            # 顯示二值化畫面於右上角，方便確認過濾效果
            thresh_resized = cv2.resize(thresh, (160, 120))
            thresh_colored = cv2.cvtColor(thresh_resized, cv2.COLOR_GRAY2BGR)
            annotated_frame[0:120, 640-160:640] = thresh_colored

            msg = f"{error_x}\n"
            try:
                conn.sendall(msg.encode('utf-8'))
            except:
                print("[視覺伺服] C++ 端已斷線。")
                break

            cv2.imshow("OpenCV Contour Align", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
            
    finally:
        cap.release()
        cv2.destroyAllWindows()
        conn.close()
        server_socket.close()

if __name__ == "__main__":
    start_align_service()