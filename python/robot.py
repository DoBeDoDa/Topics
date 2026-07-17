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
CAMERA_INDEX = 0  # 如果讀取不到相機，請嘗試修改為 0, 1, 2...
# ==========================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
DEFAULT_MODEL_PATH = os.path.join(ROOT_DIR, "bin", "best.pt")
DEFAULT_NN_MODEL_PATH = os.path.join(ROOT_DIR, "bin", "calibration_model.pth")

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
DEFAULT_CAM_POINTS = np.float32([[402.4846496582031, 306.7862548828125], [423.9533996582031, 306.79107666015625], [445.1630859375, 306.77569580078125], [466.3940124511719, 306.8249816894531], [487.54779052734375, 306.8326110839844], [508.680419921875, 306.7784118652344], [529.9215087890625, 306.76141357421875], [551.3934936523438, 306.69622802734375], [572.2357788085938, 306.788818359375], [402.71905517578125, 328.0751953125], [423.8086242675781, 328.0345458984375], [445.2142639160156, 328.0367736816406], [466.5150451660156, 327.9566345214844], [487.7117919921875, 327.9344177246094], [508.86395263671875, 327.94781494140625], [530.3446655273438, 327.90155029296875], [551.2516479492188, 327.93994140625], [572.4508056640625, 327.8527526855469], [403.437255859375, 349.0485534667969], [424.62603759765625, 348.97637939453125], [445.37225341796875, 349.0646667480469], [466.4974365234375, 349.0870056152344], [487.7868957519531, 349.0374755859375], [509.2010498046875, 348.89251708984375], [530.3377075195312, 348.8663024902344], [551.4425048828125, 348.8307189941406], [572.2265014648438, 348.8387451171875], [402.6304016113281, 370.07769775390625], [424.1826477050781, 369.7743225097656], [445.6600341796875, 369.7660827636719], [467.2594299316406, 370.0384826660156], [488.1153564453125, 370.1231689453125], [509.2713623046875, 369.9170227050781], [530.4446411132812, 369.911376953125], [551.377197265625, 369.9073486328125], [572.3927001953125, 369.851318359375], [403.3290710449219, 390.6513366699219], [424.8164978027344, 390.5455017089844], [446.2745361328125, 390.5926208496094], [467.37750244140625, 390.5093078613281], [487.8453063964844, 390.7455139160156], [509.04949951171875, 390.9241638183594], [530.373291015625, 390.9637451171875], [551.4972534179688, 390.88177490234375], [572.3555297851562, 391.10430908203125], [403.2723083496094, 411.9396667480469], [424.68853759765625, 411.8416442871094], [445.73382568359375, 411.7383117675781], [466.6059265136719, 411.819580078125], [487.99163818359375, 412.0515441894531], [509.1385803222656, 412.0892639160156], [529.6988525390625, 412.34222412109375], [551.1905517578125, 412.24456787109375], [572.3015747070312, 412.37249755859375]])

# 📝 2. 實體物理座標 (mm) 預設對應點
DEFAULT_TABLE_POINTS = np.float32([[-119.2, 689.5], [-94.5, 689.4], [-69.8, 689.3], [-45.1, 689.3], [-20.4, 689.2], [4.4, 689.2], [29.1, 689.1], [53.8, 689.0], [78.5, 689.0], [-119.3, 664.7], [-94.5, 664.7], [-69.8, 664.6], [-45.1, 664.6], [-20.4, 664.5], [4.3, 664.4], [29.0, 664.4], [53.7, 664.3], [78.4, 664.3], [-119.3, 640.0], [-94.6, 640.0], [-69.9, 639.9], [-45.2, 639.9], [-20.5, 639.8], [4.2, 639.7], [29.0, 639.7], [53.7, 639.6], [78.4, 639.6], [-119.4, 615.3], [-94.7, 615.3], [-70.0, 615.2], [-45.2, 615.1], [-20.5, 615.1], [4.2, 615.0], [28.9, 615.0], [53.6, 614.9], [78.3, 614.8], [-119.4, 590.6], [-94.7, 590.5], [-70.0, 590.5], [-45.3, 590.4], [-20.6, 590.4], [4.1, 590.3], [28.8, 590.2], [53.5, 590.2], [78.3, 590.1], [-119.5, 565.9], [-94.8, 565.8], [-70.1, 565.8], [-45.4, 565.7], [-20.6, 565.7], [4.1, 565.6], [28.8, 565.5], [53.5, 565.5], [78.2, 565.4]])

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

    def __init__(self, model_path=None, use_nn=False, nn_model_path=None):
        if model_path is None:
            model_path = DEFAULT_MODEL_PATH
        if nn_model_path is None:
            nn_model_path = DEFAULT_NN_MODEL_PATH

        if not os.path.exists(model_path):
            raise FileNotFoundError(f"找不到 YOLO 模型檔案: {model_path}")
        print("[系統狀態] 正在載入 YOLO 模型...")
        self.model = YOLO(model_path)
        self.use_nn = False # 依要求強制關閉 PyTorch，只使用黑白棋盤格校正
        
        if self.use_nn:
            if not HAS_TORCH:
                print("[系統警告] 偵測到未安裝 PyTorch。自動切換為黑白棋盤格校正模式。")
                self.use_nn = False
            elif not os.path.exists(nn_model_path):
                print(f"[系統警告] 找不到 PyTorch 模型檔案 '{nn_model_path}'。自動切換為黑白棋盤格校正模式。")
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
                    print(f"[系統警告] 載入 PyTorch 模型時發生錯誤 ({e})。自動切換為黑白棋盤格校正模式。")
                    self.use_nn = False

        if not self.use_nn:
            print("[系統狀態] 正在使用黑白棋盤格校正模式 (Homography 54 點法)...")
            self.matrix = self._get_perspective_matrix()

    def _get_perspective_matrix(self):
        cam_points = DEFAULT_CAM_POINTS
        table_points = DEFAULT_TABLE_POINTS

        if len(cam_points) != len(table_points):
            raise ValueError(f"[嚴重錯誤] 像素點有 {len(cam_points)} 個，但實體物理點只有 {len(table_points)} 個！")

        if len(cam_points) >= 4:
            print(f"[系統狀態] 偵測到 {len(cam_points)} 組黑白棋盤格點位，計算 Homography 轉換矩陣...")
            matrix, _ = cv2.findHomography(cam_points, table_points, cv2.LMEDS)
            return matrix
        elif len(cam_points) == 3:
            print("[系統狀態] 偵測到 3 組標定點位，使用仿射變換 (Affine Transform) 計算校正矩陣...")
            affine_matrix, _ = cv2.estimateAffine2D(cam_points, table_points)
            matrix = np.vstack([affine_matrix, [0, 0, 1]])
            return matrix
        else:
            print("[系統警告] 標定點位不足（至少需要 3 組點位），透視矩陣將返回單位矩陣。請執行標定流程！")
            return np.eye(3, dtype=np.float32)

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

        # 🎯 幾何路徑視覺化繪製邏輯
        target_cls = None
        for cls_id in range(9):
            if cls_id in best_boxes:
                target_cls = cls_id
                break
        
        if target_cls is not None and 9 in best_boxes:
            # 取得母球 (bw) 與目標球的像素中心與手臂物理座標
            box_bw = best_boxes[9]
            cx_bw = (box_bw.xyxy[0][0] + box_bw.xyxy[0][2]) / 2
            cy_bw = (box_bw.xyxy[0][1] + box_bw.xyxy[0][3]) / 2
            bw_arm = (coords[18], coords[19])

            box_tgt = best_boxes[target_cls]
            cx_tgt = (box_tgt.xyxy[0][0] + box_tgt.xyxy[0][2]) / 2
            cy_tgt = (box_tgt.xyxy[0][1] + box_tgt.xyxy[0][3]) / 2
            tgt_arm = (coords[target_cls * 2], coords[target_cls * 2 + 1])

            # 母球 -> 目標球 向量 (手臂座標系)
            v_cue = (tgt_arm[0] - bw_arm[0], tgt_arm[1] - bw_arm[1])

            # 遍歷所有偵測到的口袋
            pockets = []
            for p_cls in range(10, 16):
                if p_cls in best_boxes:
                    box_p = best_boxes[p_cls]
                    cx_p = (box_p.xyxy[0][0] + box_p.xyxy[0][2]) / 2
                    cy_p = (box_p.xyxy[0][1] + box_p.xyxy[0][3]) / 2
                    p_arm = (coords[p_cls * 2], coords[p_cls * 2 + 1])
                    v_pkt = (p_arm[0] - tgt_arm[0], p_arm[1] - tgt_arm[1])
                    
                    # 計算夾角
                    len_cue = np.hypot(v_cue[0], v_cue[1])
                    len_pkt = np.hypot(v_pkt[0], v_pkt[1])
                    if len_cue > 0.1 and len_pkt > 0.1:
                        cos_theta = (v_cue[0]*v_pkt[0] + v_cue[1]*v_pkt[1]) / (len_cue * len_pkt)
                        cos_theta = np.clip(cos_theta, -1.0, 1.0)
                        angle = np.degrees(np.arccos(cos_theta))
                    else:
                        angle = 999.0
                    pockets.append((p_cls, angle, (int(cx_p), int(cy_p))))

            if pockets:
                # 篩選出夾角最小的口袋
                best_pocket = min(pockets, key=lambda x: x[1])
                best_cls, min_angle, (cx_best, cy_best) = best_pocket

                # 1. 畫出 母球 -> 目標球 的擊球軌跡 (粗白線)
                cv2.line(annotated_frame, (int(cx_bw), int(cy_bw)), (int(cx_tgt), int(cy_tgt)), (255, 255, 255), 2, cv2.LINE_AA)
                
                # 2. 畫出 目標球 -> 所有口袋 的候選線路 (灰色細線)
                for p_cls, angle, (cx_p, cy_p) in pockets:
                    cv2.line(annotated_frame, (int(cx_tgt), int(cy_tgt)), (cx_p, cy_p), (140, 140, 140), 1, cv2.LINE_AA)
                    cv2.putText(annotated_frame, f"{angle:.1f} deg", (int((cx_tgt+cx_p)/2), int((cy_tgt+cy_p)/2)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

                # 3. 畫出 目標球 -> 最佳球洞 的主要軌跡 (綠色粗線)
                cv2.line(annotated_frame, (int(cx_tgt), int(cy_tgt)), (cx_best, cy_best), (0, 255, 0), 3, cv2.LINE_AA)
                cv2.circle(annotated_frame, (cx_best, cy_best), 15, (0, 255, 0), 2)
                cv2.putText(annotated_frame, f"TARGET p{best_cls-9} ({min_angle:.1f} deg)", (cx_best - 50, cy_best - 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

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
    def __init__(self, model_path=None, port=12345, use_nn=False):
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