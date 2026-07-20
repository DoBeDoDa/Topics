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
DEFAULT_CAM_POINTS = np.float32([[539.4574584960938, 273.9771423339844], [563.408203125, 274.0282897949219], [587.258056640625, 274.2308044433594], [611.1767578125, 274.26910400390625], [634.973876953125, 274.4761962890625], [658.7780151367188, 274.5926513671875], [682.6178588867188, 274.73382568359375], [706.4659423828125, 274.91400146484375], [730.3272705078125, 275.1044921875], [539.1395263671875, 297.6607666015625], [563.0631103515625, 297.6617431640625], [587.0230712890625, 297.7480773925781], [611.0182495117188, 298.0528259277344], [634.9120483398438, 298.17950439453125], [658.6991577148438, 298.3457946777344], [682.6703491210938, 298.4668884277344], [706.4855346679688, 298.5853271484375], [730.3463745117188, 298.71868896484375], [539.1505126953125, 321.4720764160156], [563.2211303710938, 321.53509521484375], [586.692138671875, 321.5364990234375], [610.7180786132812, 321.6429138183594], [634.7227783203125, 321.7799987792969], [658.7167358398438, 322.0538635253906], [682.643310546875, 322.19268798828125], [706.56005859375, 322.28680419921875], [730.4671020507812, 322.3661804199219], [538.3960571289062, 344.9567565917969], [562.52294921875, 344.9156799316406], [586.6207885742188, 345.1925354003906], [610.599609375, 345.6889343261719], [634.7178344726562, 345.6762390136719], [658.6275634765625, 345.8579406738281], [682.6795654296875, 345.8886413574219], [706.6198120117188, 346.2064208984375], [730.5857543945312, 346.3039245605469], [538.28759765625, 368.7532958984375], [562.4533081054688, 368.82403564453125], [586.4601440429688, 368.9473571777344], [610.5679321289062, 369.1602783203125], [634.58251953125, 369.2350769042969], [658.62060546875, 369.5900573730469], [682.6221313476562, 369.8980712890625], [706.6851806640625, 370.0789489746094], [730.7949829101562, 370.2236328125], [537.8944091796875, 392.9856262207031], [562.2763671875, 393.100341796875], [586.2830810546875, 393.30133056640625], [610.3876342773438, 393.3438415527344], [634.5194702148438, 393.5047912597656], [658.759765625, 393.5884094238281], [682.8538208007812, 393.6386413574219], [707.1918334960938, 393.93505859375], [731.1437377929688, 394.307373046875]])
DEFAULT_TABLE_POINTS = np.float32([[-138.1, 636.9], [-113.0, 636.5], [-87.9, 636.0], [-62.8, 635.5], [-37.8, 635.1], [-12.7, 634.6], [12.4, 634.2], [37.5, 633.7], [62.6, 633.2], [-138.5, 611.8], [-113.5, 611.4], [-88.4, 610.9], [-63.3, 610.5], [-38.2, 610.0], [-13.1, 609.5], [11.9, 609.1], [37.0, 608.6], [62.1, 608.2], [-139.0, 586.7], [-113.9, 586.3], [-88.8, 585.8], [-63.8, 585.4], [-38.7, 584.9], [-13.6, 584.5], [11.5, 584.0], [36.6, 583.5], [61.6, 583.1], [-139.5, 561.7], [-114.4, 561.2], [-89.3, 560.8], [-64.2, 560.3], [-39.1, 559.8], [-14.1, 559.4], [11.0, 558.9], [36.1, 558.5], [61.2, 558.0], [-139.9, 536.6], [-114.8, 536.1], [-89.8, 535.7], [-64.7, 535.2], [-39.6, 534.8], [-14.5, 534.3], [10.6, 533.8], [35.6, 533.4], [60.7, 532.9], [-140.4, 511.5], [-115.3, 511.0], [-90.2, 510.6], [-65.1, 510.1], [-40.1, 509.7], [-15.0, 509.2], [10.1, 508.8], [35.2, 508.3], [60.3, 507.8]])


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
