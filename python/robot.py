"""協調相機、YOLO、座標轉換、畫面繪製與 C++ TCP 通訊的視覺主程式。"""

import os
import socket
import time

import cv2
import numpy as np

from coordinate_transformer import CalibrationNet, HomographyCoordinateTransformer
from detection_filter import DetectionFilter
from pocket_selector import PocketSelector
from vision_renderer import VisionRenderer
from yolo_inference import YoloInference


CAMERA_INDEX = 0

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
DEFAULT_MODEL_PATH = os.path.join(ROOT_DIR, "bin", "best.pt")
DEFAULT_NN_MODEL_PATH = os.path.join(ROOT_DIR, "bin", "calibration_model.pth")

# 相機像素座標與手臂平面座標的 54 組校正點。
DEFAULT_CAM_POINTS = np.float32([[460.70050048828125, 298.6126403808594], [484.58441162109375, 298.5958251953125], [508.3777770996094, 298.6275329589844], [532.3826904296875, 298.69647216796875], [556.2919311523438, 298.7293395996094], [580.1995849609375, 298.80096435546875], [604.1380004882812, 298.7864990234375], [627.9979248046875, 298.7862243652344], [651.8408203125, 298.81854248046875], [460.36322021484375, 322.4401550292969], [484.1078796386719, 322.41796875], [508.1241760253906, 322.43011474609375], [532.13427734375, 322.47674560546875], [556.203369140625, 322.5574951171875], [580.1182250976562, 322.56951904296875], [603.9674682617188, 322.60650634765625], [628.0232543945312, 322.6125793457031], [651.8953857421875, 322.6688537597656], [460.4651794433594, 346.186767578125], [484.444091796875, 346.2602233886719], [507.759765625, 346.3084411621094], [531.7282104492188, 346.301513671875], [555.940185546875, 346.39385986328125], [579.8764038085938, 346.460693359375], [603.9918212890625, 346.43377685546875], [627.84716796875, 346.458740234375], [651.9368896484375, 346.4971008300781], [459.4992370605469, 369.86968994140625], [483.4936828613281, 369.73004150390625], [507.6581115722656, 369.85614013671875], [531.8362426757812, 370.3154296875], [555.720703125, 370.3399658203125], [579.7962036132812, 370.3904724121094], [603.7685546875, 370.42156982421875], [627.9190673828125, 370.4798583984375], [651.89697265625, 370.4890441894531], [459.20733642578125, 393.8043518066406], [483.4982604980469, 393.80426025390625], [507.6310729980469, 393.7657165527344], [531.6181030273438, 393.79498291015625], [555.3551025390625, 393.9565734863281], [579.4866333007812, 394.28924560546875], [603.6802368164062, 394.44354248046875], [627.7635498046875, 394.5285949707031], [651.9995727539062, 394.548095703125], [458.830322265625, 418.043701171875], [483.2038879394531, 418.10772705078125], [507.18353271484375, 418.146240234375], [530.8514404296875, 418.0980224609375], [555.1763916015625, 418.2758483886719], [579.3677978515625, 418.3940124511719], [603.4395141601562, 418.4772644042969], [627.8079223632812, 418.7021484375], [651.8318481445312, 418.8164367675781]])
DEFAULT_TABLE_POINTS = np.float32([[-219.8, 611.6], [-194.8, 611.1], [-169.8, 610.6], [-144.8, 610.1], [-119.8, 609.6], [-94.8, 609.1], [-69.8, 608.6], [-44.8, 608.1], [-19.8, 607.6], [-220.3, 586.6], [-195.3, 586.1], [-170.3, 585.6], [-145.3, 585.1], [-120.3, 584.6], [-95.3, 584.1], [-70.3, 583.6], [-45.3, 583.1], [-20.3, 582.6], [-220.8, 561.5], [-195.8, 561.0], [-170.8, 560.6], [-145.8, 560.1], [-120.8, 559.6], [-95.8, 559.1], [-70.8, 558.6], [-45.8, 558.1], [-20.8, 557.6], [-221.3, 536.5], [-196.3, 536.0], [-171.3, 535.5], [-146.3, 535.0], [-121.3, 534.6], [-96.3, 534.1], [-71.3, 533.6], [-46.3, 533.1], [-21.3, 532.6], [-221.8, 511.5], [-196.8, 511.0], [-171.8, 510.5], [-146.8, 510.0], [-121.8, 509.5], [-96.8, 509.0], [-71.8, 508.5], [-46.8, 508.1], [-21.8, 507.6], [-222.3, 486.5], [-197.3, 486.0], [-172.3, 485.5], [-147.3, 485.0], [-122.3, 484.5], [-97.3, 484.0], [-72.3, 483.5], [-47.3, 483.0], [-22.3, 482.5]])


class BilliardDetector:
    """整合推論、篩選、座標轉換、球袋選擇與畫面呈現。"""

    def __init__(self, model_path=None, use_nn=False, nn_model_path=None):
        model_path = model_path or DEFAULT_MODEL_PATH
        nn_model_path = nn_model_path or DEFAULT_NN_MODEL_PATH
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"找不到 YOLO 模型檔案: {model_path}")

        print("[系統狀態] 正在載入 YOLO 模型...")
        self.inference = YoloInference(model_path)
        self.filter = DetectionFilter()
        self.transformer = HomographyCoordinateTransformer(
            DEFAULT_CAM_POINTS,
            DEFAULT_TABLE_POINTS,
            use_nn=use_nn,
            nn_model_path=nn_model_path,
        )
        self.pocket_selector = PocketSelector()
        self.renderer = VisionRenderer()

    def detect(self, frame):
        results = self.inference.infer(frame)
        detections = self.filter.filter(results)
        coordinates = self.transformer.transform_detections(detections)
        pocket_selection = self.pocket_selector.select(detections, coordinates)
        annotated, display_data = self.renderer.render(
            frame,
            detections,
            coordinates,
            pocket_selection,
        )
        return annotated, coordinates, display_data


class USBCamera:
    """處理 USB 相機啟動與影像讀取。"""

    def __init__(self, camera_index=CAMERA_INDEX, width=1280, height=720):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.cap = None

    def start(self):
        if self.cap is not None:
            return
        print(f"[硬體狀態] 正在啟動 USB 相機 (Index: {self.camera_index})...")
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            self.cap = None
            raise RuntimeError(f"無法開啟相機索引 {self.camera_index}。")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

    def get_frame(self):
        if self.cap is None:
            return None
        success, frame = self.cap.read()
        return frame if success else None

    def stop(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


class BilliardVisionServer:
    """負責與 C++ 控制端進行 TCP Socket 通訊。"""

    def __init__(self, host="0.0.0.0", port=12345):
        self.host = host
        self.port = port
        self.server_socket = None
        self.connection = None
        self.address = None

    def start(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        print(f"[網路狀態] 等待 C++ 控制端連線中 (Port: {self.port})...")
        self.connection, self.address = self.server_socket.accept()
        print(f"[網路狀態] 連線成功！來源 IP: {self.address}")

    def send_coords(self, coordinates):
        if self.connection is None:
            return False
        message = ",".join(map(str, coordinates)) + "\n"
        try:
            self.connection.sendall(message.encode("utf-8"))
            return True
        except socket.error:
            print("\n[網路警告] C++ 端連線中斷。")
            return False

    def close(self):
        if self.connection is not None:
            self.connection.close()
            self.connection = None
        if self.server_socket is not None:
            self.server_socket.close()
            self.server_socket = None


class BilliardVisionApp:
    """視覺服務的流程協調入口。"""

    def __init__(self, model_path=None, port=12345, use_nn=False):
        self.detector = BilliardDetector(model_path, use_nn=use_nn)
        self.camera = USBCamera()
        self.server = BilliardVisionServer(port=port)

    @staticmethod
    def print_dashboard(display_data):
        os.system("cls" if os.name == "nt" else "clear")
        print("=====================================================")
        print(" [相機端] 機器人視覺絕對座標面板 (單位: mm)")
        print("=====================================================")
        for ball_id in range(1, 10):
            print(f" [{ball_id}號球 b{ball_id}] {display_data[f'b{ball_id}']}")
        print(f" [母球 bw] {display_data['bw']}")
        for pocket_id in range(1, 7):
            print(f" [球袋 p{pocket_id}] {display_data[f'p{pocket_id}']}")
        print("=====================================================")
        print(" 在相機影像視窗按下 q、Esc 結束程式")

    def run(self):
        try:
            self.server.start()
            self.camera.start()
            last_print_time = 0.0

            while True:
                frame = self.camera.get_frame()
                if frame is None:
                    continue

                annotated, coordinates, display_data = self.detector.detect(frame)
                current_time = time.time()
                if current_time - last_print_time > 0.5:
                    self.print_dashboard(display_data)
                    last_print_time = current_time

                if not self.server.send_coords(coordinates):
                    break

                cv2.imshow("Direct Arm Vision", annotated)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
        finally:
            self.camera.stop()
            self.server.close()
            cv2.destroyAllWindows()
            print("[系統狀態] 視覺服務已完全關閉。")


if __name__ == "__main__":
    import sys

    app = BilliardVisionApp(use_nn="--nn" in sys.argv or "-n" in sys.argv)
    app.run()
