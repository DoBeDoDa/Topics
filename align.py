import cv2
import socket
import numpy as np

class ContourAligner:
    """處理影像的前處理、輪廓分析與圓心對齊偏差計算"""
    def __init__(self, target_width=640, target_height=480):
        self.target_width = target_width
        self.target_height = target_height
        self.frame_center_x = target_width // 2

    def process_frame(self, frame, threshold_val):
        """處理影像並尋找最適合的圓形輪廓，回傳標註影像、二值化影像與中心點偏差 px"""
        resized_frame = cv2.resize(frame, (self.target_width, self.target_height), interpolation=cv2.INTER_AREA)
        annotated_frame = resized_frame.copy()

        gray = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        
        _, thresh = cv2.threshold(blurred, threshold_val, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_center = None
        best_radius = 0
        
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
            error_x = best_center[0] - self.frame_center_x
            cv2.circle(annotated_frame, best_center, int(best_radius), (0, 255, 255), 2)
            cv2.circle(annotated_frame, best_center, 5, (0, 0, 255), -1)

        # 繪製中心對齊基準線與偏差文字
        cv2.line(annotated_frame, (self.frame_center_x, 0), (self.frame_center_x, self.target_height), (0, 255, 0), 2)
        cv2.putText(annotated_frame, f"Error px: {error_x:.1f}", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        # 於右上角嵌入缩小的二值化畫面
        thresh_resized = cv2.resize(thresh, (160, 120))
        thresh_colored = cv2.cvtColor(thresh_resized, cv2.COLOR_GRAY2BGR)
        annotated_frame[0:120, self.target_width-160:self.target_width] = thresh_colored

        return annotated_frame, error_x


class AlignSocketServer:
    """負責對齊通訊的 TCP Socket 伺服器"""
    def __init__(self, host="0.0.0.0", port=12346):
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.conn = None
        self.addr = None

    def start(self):
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        print(f"[視覺伺服] 通訊埠已就緒。等待 main.cpp 連線 (Port: {self.port})...")
        self.conn, self.addr = self.server_socket.accept()
        print(f"[視覺伺服] C++ 連線成功！啟動 OpenCV 動態輪廓辨識...")

    def send_error(self, error_x):
        if not self.conn:
            return False
        msg = f"{error_x}\n"
        try:
            self.conn.sendall(msg.encode('utf-8'))
            return True
        except socket.error:
            print("[視覺伺服] C++ 端連線中斷。")
            return False

    def close(self):
        if self.conn:
            self.conn.close()
            self.conn = None
        if self.server_socket:
            self.server_socket.close()


class AlignApp:
    """對齊校正系統的主控程式"""
    def __init__(self, camera_index=2, port=12346):
        self.camera_index = camera_index
        self.port = port
        self.aligner = ContourAligner()
        self.server = AlignSocketServer(port=port)
        self.cap = None

    def _on_trackbar(self, val):
        pass

    def run(self):
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            print(f"[硬體錯誤] 無法開啟相機索引 {self.camera_index}！")
            return

        cv2.namedWindow("OpenCV Contour Align")
        cv2.createTrackbar("Threshold", "OpenCV Contour Align", 120, 255, self._on_trackbar)

        try:
            self.server.start()
            while True:
                ret, raw_frame = self.cap.read()
                if not ret:
                    continue

                current_threshold = cv2.getTrackbarPos("Threshold", "OpenCV Contour Align")
                annotated_frame, error_x = self.aligner.process_frame(raw_frame, current_threshold)

                # 發送偏差值至 C++ 端
                if not self.server.send_error(error_x):
                    break

                cv2.imshow("OpenCV Contour Align", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            if self.cap:
                self.cap.release()
            cv2.destroyAllWindows()
            self.server.close()
            print("[視覺伺服] 對齊服務已完全關閉。")


if __name__ == "__main__":
    app = AlignApp()
    app.run()