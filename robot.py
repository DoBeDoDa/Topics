import cv2
import os
import time
import socket
import numpy as np
try:
    import pyrealsense2 as rs
except ImportError:
    rs = None
from ultralytics import YOLO

# ==========================================
# [相機設定] 設定您所使用的相機 USB Index
# ==========================================
CAMERA_INDEX = 1  # 如果讀取不到相機，請嘗試修改為 0, 1, 2...
# ==========================================

try:
    import torch
    import torch.nn as nn
    HAS_TORCH = True

    class CalibrationNet(nn.Module):
        def __init__(self):
            super(CalibrationNet, self).__init__()
            self.net = nn.Sequential(
                nn.Linear(2, 64),
                nn.ReLU(),
                nn.Linear(64, 64),
                nn.ReLU(),
                nn.Linear(64, 2)
            )
            
        def forward(self, x):
            return self.net(x)
except ImportError:
    HAS_TORCH = False
    class CalibrationNet:
        pass

# 📝 1. 像素座標 (X, Y) 預設對應點
DEFAULT_CAM_POINTS = np.float32([])

# 📝 2. 實體物理座標 (mm) 預設對應點
DEFAULT_TABLE_POINTS = np.float32([])

class BilliardDetector:
    """負責 YOLO 辨識與 Homography 物理座標轉換"""
    LABEL_MAP = {
        0: "b1", 1: "b2", 2: "b3", 3: "b4", 4: "b5", 
        5: "b6", 6: "b7", 7: "b8", 8: "b9", 9: "bw",
        10: "p1", 11: "p2", 12: "p3", 13: "p4", 14: "p5", 15: "p6"
    }

    COLOR_MAP = {
        0: (0, 255, 255),    # b1: 黃
        1: (255, 0, 0),      # b2: 藍
        2: (0, 0, 255),      # b3: 紅
        3: (0, 165, 255),    # b4: 橘
        4: (128, 0, 128),    # b5: 紫
        5: (0, 128, 0),      # b6: 綠
        6: (0, 0, 128),      # b7: 暗紅
        7: (0, 0, 0),        # b8: 黑
        8: (128, 128, 128),  # b9: 灰
        9: (255, 255, 255),  # bw: 白 (母球)
        10: (255, 0, 255),   # p1: 洋紅
        11: (255, 0, 255),   # p2: 洋紅
        12: (255, 0, 255),   # p3: 洋紅
        13: (255, 0, 255),   # p4: 洋紅
        14: (255, 0, 255),   # p5: 洋紅
        15: (255, 0, 255)    # p6: 洋紅
    }

    def __init__(self, model_path="best.pt", use_nn=False, nn_model_path="calibration_model.pth"):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"找不到 YOLO 模型檔案: {model_path}")
        print("[系統狀態] 正在載入 YOLO 模型...")
        self.model = YOLO(model_path)
        self.use_nn = use_nn
        
        if self.use_nn:
            if not HAS_TORCH:
                print("[系統警告] 偵測到未安裝 PyTorch。自動切換為傳統幾何校正模式。")
                self.use_nn = False
            elif not os.path.exists(nn_model_path):
                print(f"[系統警告] 找不到 PyTorch 模型檔案 '{nn_model_path}'。自動切換為傳統幾何校正模式。")
                self.use_nn = False
            else:
                print(f"[系統狀態] 正在載入 PyTorch 校正模型 '{nn_model_path}'...")
                try:
                    self.torch_device = torch.device("cpu")
                    checkpoint = torch.load(nn_model_path, map_location=self.torch_device, weights_only=False)
                    self.nn_model = CalibrationNet()
                    self.nn_model.load_state_dict(checkpoint['model_state'])
                    self.nn_model.eval()
                    
                    self.cam_mean = checkpoint['cam_mean']
                    self.cam_std = checkpoint['cam_std']
                    self.table_mean = checkpoint['table_mean']
                    self.table_std = checkpoint['table_std']
                    print("[系統狀態] PyTorch 校正模型載入成功。")
                except Exception as e:
                    print(f"[系統警告] 載入 PyTorch 模型時發生錯誤 ({e})。自動切換為傳統幾何校正模式。")
                    self.use_nn = False

        if not self.use_nn:
            print("[系統狀態] 正在使用傳統幾何校正模式...")
            self.matrix = self._get_perspective_matrix()

    def _get_perspective_matrix(self):
        cam_points = DEFAULT_CAM_POINTS
        table_points = DEFAULT_TABLE_POINTS

        if len(cam_points) < 4 or len(table_points) < 4:
            print("[系統警告] 標定點位小於 4 個點，透視矩陣將返回單位矩陣。請執行標定流程！")
            return np.eye(3, dtype=np.float32)

        if len(cam_points) != len(table_points):
            raise ValueError(f"[嚴重錯誤] 像素點有 {len(cam_points)} 個，但實體物理點只有 {len(table_points)} 個！")

        matrix, _ = cv2.findHomography(cam_points, table_points, cv2.LMEDS)
        return matrix

    def cam_to_arm(self, x_cam, y_cam):
        if self.use_nn:
            inp = np.array([x_cam, y_cam], dtype=np.float32)
            inp_scaled = (inp - self.cam_mean) / self.cam_std
            inp_tensor = torch.tensor(inp_scaled, dtype=torch.float32).to(self.torch_device)
            with torch.no_grad():
                pred_scaled = self.nn_model(inp_tensor).cpu().numpy()
            pred = pred_scaled * self.table_std + self.table_mean
            return round(float(pred[0]), 1), round(float(pred[1]), 1)
        else:
            pt = np.array([[[x_cam, y_cam]]], dtype=np.float32)
            pt_arm = cv2.perspectiveTransform(pt, self.matrix)
            return round(pt_arm[0][0][0], 1), round(pt_arm[0][0][1], 1)

    def detect(self, frame):
        """執行 YOLO 偵測並返回標註後的影像與座標列表"""
        results = self.model(frame, conf=0.3, verbose=False)
        
        # 分類收集偵測到的框 (新模型：0..8:Ball_1..9, 9:cue, 10:hole)
        balls = {}
        holes = []
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            if cls_id == 10:
                holes.append(box)
            else:
                if cls_id not in balls:
                    balls[cls_id] = box
                elif box.conf[0] > balls[cls_id].conf[0]:
                    balls[cls_id] = box

        # 映射回新系統的 class_id (0..8: b1..b9, 9: bw, 10..15: p1..p6)
        best_boxes = {}
        # 0..8: b1..b9 (來自新 class 0..8)
        for i in range(9):
            if i in balls:
                best_boxes[i] = balls[i]
        # 9: bw (來自新 class 9)
        if 9 in balls:
            best_boxes[9] = balls[9]

        # 將球洞以 X 座標進行排序，並分配至 p1~p6 (新 class 10~15)
        holes_sorted = sorted(holes, key=lambda b: float(b.xyxy[0][0]))
        for i, box in enumerate(holes_sorted[:6]):
            best_boxes[10 + i] = box

        coords = [-9999.0] * 32
        annotated_frame = frame.copy()
        display_data = {}

        for cls_id in range(16):
            short_name = self.LABEL_MAP[cls_id]
            box_color = self.COLOR_MAP.get(cls_id, (0, 255, 0))
            
            if cls_id in best_boxes:
                box = best_boxes[cls_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                
                arm_x, arm_y = self.cam_to_arm(cx, cy)
                coords[cls_id * 2] = arm_x
                coords[cls_id * 2 + 1] = arm_y

                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), box_color, 2)
                cv2.circle(annotated_frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                cv2.putText(annotated_frame, short_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
                display_data[short_name] = f"{arm_x:>7.1f}, {arm_y:>7.1f}"
            else:
                display_data[short_name] = "   --- ,    --- "

        return annotated_frame, coords, display_data


class USBCamera:
    """處理 USB 相機（如 Orbbec Gemini 2 XL）的啟動與影像讀取"""
    def __init__(self, camera_index=2, width=1280, height=720):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.cap = None
        self.is_started = False

    def start(self):
        if not self.is_started:
            print(f"[硬體狀態] 正在啟動 USB 相機 (Index: {self.camera_index})...")
            self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
            if not self.cap.isOpened():
                raise RuntimeError(f"[錯誤] 無法開啟相機索引 {self.camera_index}！")
            
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.is_started = True
            print(f"[系統狀態] USB 相機 (Index: {self.camera_index}) 啟動成功。")

    def get_frame(self):
        if not self.is_started or self.cap is None:
            return None
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def stop(self):
        if self.is_started:
            print("[硬體狀態] 正在關閉 USB 相機...")
            if self.cap:
                self.cap.release()
                self.cap = None
            self.is_started = False


class BilliardVisionServer:
    """負責與 C++ 控制端進行 TCP Socket 通訊"""
    def __init__(self, host='0.0.0.0', port=12345):
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.conn = None
        self.addr = None

    def start(self):
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        print(f"[網路狀態] 等待 C++ 控制端連線中 (Port: {self.port})...")
        self.conn, self.addr = self.server_socket.accept()
        print(f"[網路狀態] 連線成功！來源 IP: {self.addr}")

    def send_coords(self, coords):
        if not self.conn:
            return False
        msg = ",".join(map(str, coords)) + "\n"
        try:
            self.conn.sendall(msg.encode('utf-8'))
            return True
        except socket.error:
            print("\n[網路警告] C++ 端連線中斷。")
            return False

    def close(self):
        print("[網路狀態] 正在關閉通訊埠...")
        if self.conn:
            self.conn.close()
            self.conn = None
        if self.server_socket:
            self.server_socket.close()


class BilliardVisionApp:
    """整合相機、辨識與通訊的主應用程式"""
    def __init__(self, model_path="best.pt", port=12345, use_nn=False):
        self.detector = BilliardDetector(model_path, use_nn=use_nn)
        self.camera = USBCamera(camera_index=CAMERA_INDEX)
        self.server = BilliardVisionServer(port=port)

    def print_dashboard(self, display_data):
        os.system('cls' if os.name == 'nt' else 'clear')
        print("=====================================================")
        print(" [相機端] 機器人視覺絕對座標面板 (單位: mm, 原點: p1)")
        print("=====================================================")
        print(f" [1號球 b1] {display_data['b1']}   |   [左上 p1] {display_data['p1']}")
        print(f" [2號球 b2] {display_data['b2']}   |   [中上 p2] {display_data['p2']}")
        print(f" [3號球 b3] {display_data['b3']}   |   [右上 p3] {display_data['p3']}")
        print(f" [4號球 b4] {display_data['b4']}   |   [左下 p4] {display_data['p4']}")
        print(f" [5號球 b5] {display_data['b5']}   |   [中下 p5] {display_data['p5']}")
        print(f" [6號球 b6] {display_data['b6']}   |   [右下 p6] {display_data['p6']}")
        print(f" [7號球 b7] {display_data['b7']}")
        print(f" [8號球 b8] {display_data['b8']}")
        print(f" [9號球 b9] {display_data['b9']}")
        print(f" [母  球 bw] {display_data['bw']}")
        print("=====================================================")
        print(" 系統提示: 請在相機影像視窗按下 'q' 結束程式")

    def run(self):
        try:
            self.server.start()
            self.camera.start()

            last_print_time = 0
            while True:
                frame = self.camera.get_frame()
                if frame is None:
                    continue

                annotated_frame, coords, display_data = self.detector.detect(frame)

                # 每 0.5 秒更新一次面板資訊
                current_time = time.time()
                if current_time - last_print_time > 0.5:
                    self.print_dashboard(display_data)
                    last_print_time = current_time

                # 傳送座標至 C++ 手臂控制端
                if not self.server.send_coords(coords):
                    break

                cv2.imshow("Direct Arm Vision (Orbbec/USB Camera)", annotated_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == ord('Q') or key == 27:
                    break

        finally:
            self.camera.stop()
            self.server.close()
            cv2.destroyAllWindows()
            print("[系統狀態] 視覺服務已完全關閉。")


if __name__ == "__main__":
    import sys
    use_nn = "--nn" in sys.argv or "-n" in sys.argv
    app = BilliardVisionApp(use_nn=use_nn)
    app.run()