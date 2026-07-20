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
DEFAULT_CAM_POINTS = np.float32([[402.4846496582031, 306.7862548828125], [423.9533996582031, 306.79107666015625], [445.1630859375, 306.77569580078125], [466.3940124511719, 306.8249816894531], [487.54779052734375, 306.8326110839844], [508.680419921875, 306.7784118652344], [529.9215087890625, 306.76141357421875], [551.3934936523438, 306.69622802734375], [572.2357788085938, 306.788818359375], [402.71905517578125, 328.0751953125], [423.8086242675781, 328.0345458984375], [445.2142639160156, 328.0367736816406], [466.5150451660156, 327.9566345214844], [487.7117919921875, 327.9344177246094], [508.86395263671875, 327.94781494140625], [530.3446655273438, 327.90155029296875], [551.2516479492188, 327.93994140625], [572.4508056640625, 327.8527526855469], [403.437255859375, 349.0485534667969], [424.62603759765625, 348.97637939453125], [445.37225341796875, 349.0646667480469], [466.4974365234375, 349.0870056152344], [487.7868957519531, 349.0374755859375], [509.2010498046875, 348.89251708984375], [530.3377075195312, 348.8663024902344], [551.4425048828125, 348.8307189941406], [572.2265014648438, 348.8387451171875], [402.6304016113281, 370.07769775390625], [424.1826477050781, 369.7743225097656], [445.6600341796875, 369.7660827636719], [467.2594299316406, 370.0384826660156], [488.1153564453125, 370.1231689453125], [509.2713623046875, 369.9170227050781], [530.4446411132812, 369.911376953125], [551.377197265625, 369.9073486328125], [572.3927001953125, 369.851318359375], [403.3290710449219, 390.6513366699219], [424.8164978027344, 390.5455017089844], [446.2745361328125, 390.5926208496094], [467.37750244140625, 390.5093078613281], [487.8453063964844, 390.7455139160156], [509.04949951171875, 390.9241638183594], [530.373291015625, 390.9637451171875], [551.4972534179688, 390.88177490234375], [572.3555297851562, 391.10430908203125], [403.2723083496094, 411.9396667480469], [424.68853759765625, 411.8416442871094], [445.73382568359375, 411.7383117675781], [466.6059265136719, 411.819580078125], [487.99163818359375, 412.0515441894531], [509.1385803222656, 412.0892639160156], [529.6988525390625, 412.34222412109375], [551.1905517578125, 412.24456787109375], [572.3015747070312, 412.37249755859375]])
DEFAULT_TABLE_POINTS = np.float32([[-119.2, 689.5], [-94.5, 689.4], [-69.8, 689.3], [-45.1, 689.3], [-20.4, 689.2], [4.4, 689.2], [29.1, 689.1], [53.8, 689.0], [78.5, 689.0], [-119.3, 664.7], [-94.5, 664.7], [-69.8, 664.6], [-45.1, 664.6], [-20.4, 664.5], [4.3, 664.4], [29.0, 664.4], [53.7, 664.3], [78.4, 664.3], [-119.3, 640.0], [-94.6, 640.0], [-69.9, 639.9], [-45.2, 639.9], [-20.5, 639.8], [4.2, 639.7], [29.0, 639.7], [53.7, 639.6], [78.4, 639.6], [-119.4, 615.3], [-94.7, 615.3], [-70.0, 615.2], [-45.2, 615.1], [-20.5, 615.1], [4.2, 615.0], [28.9, 615.0], [53.6, 614.9], [78.3, 614.8], [-119.4, 590.6], [-94.7, 590.5], [-70.0, 590.5], [-45.3, 590.4], [-20.6, 590.4], [4.1, 590.3], [28.8, 590.2], [53.5, 590.2], [78.3, 590.1], [-119.5, 565.9], [-94.8, 565.8], [-70.1, 565.8], [-45.4, 565.7], [-20.6, 565.7], [4.1, 565.6], [28.8, 565.5], [53.5, 565.5], [78.2, 565.4]])


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
