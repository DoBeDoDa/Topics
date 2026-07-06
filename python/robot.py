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
DEFAULT_CAM_POINTS = np.float32([[468.58087158203125, 298.5911865234375], [490.2459411621094, 298.59881591796875], [511.6826477050781, 298.5861511230469], [533.2176513671875, 298.5194091796875], [554.6068725585938, 298.51055908203125], [576.2799072265625, 298.5653381347656], [597.7994995117188, 298.5148010253906], [619.4484252929688, 298.4812316894531], [641.1478881835938, 298.41888427734375], [469.0295715332031, 320.2224426269531], [490.50927734375, 320.2298583984375], [511.91375732421875, 320.272216796875], [533.3930053710938, 320.2184143066406], [554.9354248046875, 320.24383544921875], [576.4589233398438, 320.2237243652344], [597.7880859375, 320.23626708984375], [619.4236450195312, 320.16650390625], [641.1647338867188, 320.2159423828125], [469.52337646484375, 341.5296325683594], [490.6379089355469, 341.52874755859375], [512.1416625976562, 341.4966125488281], [533.573974609375, 341.508544921875], [555.13330078125, 341.5815734863281], [576.4155883789062, 341.57537841796875], [597.9124145507812, 341.52777099609375], [619.4679565429688, 341.6083984375], [641.14453125, 341.64752197265625], [469.7424621582031, 362.78570556640625], [491.0908203125, 362.8099365234375], [512.3317260742188, 362.9999084472656], [533.62744140625, 363.0743103027344], [555.1378173828125, 363.0434875488281], [576.415283203125, 363.1573181152344], [597.866455078125, 363.1734924316406], [619.5506591796875, 363.26226806640625], [641.1455078125, 363.3593444824219], [469.7260437011719, 384.38629150390625], [491.09429931640625, 384.36541748046875], [512.3741455078125, 384.4718322753906], [533.5946655273438, 384.48736572265625], [555.102294921875, 384.50543212890625], [576.4133911132812, 384.605224609375], [597.9973754882812, 384.71234130859375], [619.5693359375, 384.7166442871094], [641.0886840820312, 384.8052062988281], [469.46014404296875, 405.6719970703125], [490.80645751953125, 405.8514709472656], [512.1689453125, 405.86248779296875], [533.4918823242188, 406.007568359375], [554.9398193359375, 406.1745300292969], [576.470458984375, 406.2577819824219], [597.8119506835938, 406.405517578125], [619.4405517578125, 406.3743591308594], [641.0921630859375, 406.4854431152344]])

# 📝 2. 實體物理座標 (mm) 預設對應點
DEFAULT_TABLE_POINTS = np.float32([[-45.0, 694.1], [-20.3, 694.1], [4.4, 694.1], [29.1, 694.1], [53.8, 694.1], [78.6, 694.1], [103.3, 694.1], [128.0, 694.1], [152.7, 694.1], [-45.0, 669.4], [-20.3, 669.4], [4.4, 669.4], [29.1, 669.4], [53.9, 669.4], [78.6, 669.4], [103.3, 669.4], [128.0, 669.4], [152.7, 669.4], [-45.0, 644.7], [-20.3, 644.7], [4.4, 644.7], [29.1, 644.7], [53.9, 644.7], [78.6, 644.7], [103.3, 644.7], [128.0, 644.7], [152.7, 644.7], [-45.0, 619.9], [-20.3, 619.9], [4.4, 619.9], [29.1, 619.9], [53.9, 620.0], [78.6, 620.0], [103.3, 620.0], [128.0, 620.0], [152.7, 620.0], [-45.0, 595.2], [-20.3, 595.2], [4.4, 595.2], [29.1, 595.2], [53.9, 595.2], [78.6, 595.2], [103.3, 595.2], [128.0, 595.2], [152.7, 595.2], [-45.0, 570.5], [-20.3, 570.5], [4.4, 570.5], [29.1, 570.5], [53.9, 570.5], [78.6, 570.5], [103.3, 570.5], [128.0, 570.5], [152.7, 570.5]])

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