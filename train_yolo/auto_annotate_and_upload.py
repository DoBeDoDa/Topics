import os
import glob
import sys
import cv2
import numpy as np
from ultralytics import YOLO

# ==========================================
# [Roboflow 設定參數]
# ==========================================
ROBOFLOW_API_KEY = "YOUR_API_KEY"         # 請在此填入您的 Roboflow Private API Key
WORKSPACE_ID = "s-workspace-jwrhi"       # 已根據您的網址自動填入
PROJECT_ID = "nineball-jbejf"           # 已根據您的網址自動填入

DATA_DIR = "yolo_data"
MODEL_PATH = "best.pt"
HISTORY_FILE = "uploaded_history.txt"

# 類別定義
CLASSES = {
    0: "b1",      # 黃色 1 號球
    1: "b2",      # 藍色 2 號球
    2: "b3",      # 紅色 3 號球
    3: "bw",      # 白色母球
    4: "pocket"   # 球洞
}

def install_and_import_roboflow():
    try:
        from roboflow import Roboflow
        return Roboflow
    except ImportError:
        print("[系統] 偵測到未安裝 roboflow SDK。正在進行自動安裝...")
        try:
            import subprocess
            subprocess.check_call([sys.executable, "-m", "pip", "install", "roboflow"])
            from roboflow import Roboflow
            print("[系統] roboflow 安裝成功！")
            return Roboflow
        except Exception as e:
            print(f"[錯誤] 自動安裝失敗: {e}")
            print("請手動執行: pip install roboflow")
            sys.exit(1)

class AutoAnnotator:
    def __init__(self):
        # 1. 載入 YOLO 模型
        self.model = None
        if os.path.exists(MODEL_PATH):
            try:
                print(f"[系統] 載入預訓練模型 '{MODEL_PATH}'...")
                self.model = YOLO(MODEL_PATH)
            except Exception as e:
                print(f"[警告] 無法載入 YOLO 模型 ({e})，將只使用 OpenCV 進行自動標註。")
        else:
            print("[系統] 未找到 'best.pt'，將完全使用 OpenCV 影像分析進行自動標註。")

        # 2. 初始化 Roboflow
        self.rf_project = None
        if ROBOFLOW_API_KEY != "YOUR_API_KEY":
            RoboflowClass = install_and_import_roboflow()
            try:
                print("[系統] 正在連線至 Roboflow...")
                rf = RoboflowClass(api_key=ROBOFLOW_API_KEY)
                self.rf_project = rf.workspace(WORKSPACE_ID).project(PROJECT_ID)
                print(f"[系統] Roboflow 連線成功！專案: {WORKSPACE_ID}/{PROJECT_ID}")
            except Exception as e:
                print(f"❌ [連線錯誤] 初始化 Roboflow 失敗: {e}")
                print("請檢查您的 API Key 是否正確。")
                sys.exit(1)
        else:
            print("\n=======================================================")
            print(" ❌ [配置錯誤] 請先填寫 'auto_annotate_and_upload.py' 中的 ROBOFLOW_API_KEY！")
            print("=======================================================\n")
            sys.exit(1)

        # 3. 讀取已上傳歷史紀錄
        self.uploaded_files = set()
        if os.path.exists(HISTORY_FILE):
            try:
                with open(HISTORY_FILE, "r", encoding="utf-8") as f:
                    for line in f:
                        self.uploaded_files.add(line.strip())
            except Exception as e:
                print(f"[警告] 無法讀取上傳紀錄檔: {e}")

    def calculate_iou(self, box1, box2):
        x_left = max(box1[0], box2[0])
        y_top = max(box1[1], box2[1])
        x_right = min(box1[2], box2[2])
        y_bottom = min(box1[3], box2[3])
        if x_right < x_left or y_bottom < y_top:
            return 0.0
        intersection_area = (x_right - x_left) * (y_bottom - y_top)
        box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
        box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union_area = box1_area + box2_area - intersection_area
        return intersection_area / union_area if union_area > 0 else 0.0

    def classify_color(self, crop):
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        h_mean = np.mean(hsv[:, :, 0])
        s_mean = np.mean(hsv[:, :, 1])
        v_mean = np.mean(hsv[:, :, 2])
        if v_mean < 45: return 4  # pocket
        if v_mean > 160 and s_mean < 75: return 3  # bw
        if 12 <= h_mean <= 40: return 0  # b1
        elif 85 <= h_mean <= 135: return 1  # b2
        elif h_mean <= 12 or h_mean >= 155: return 2  # b3
        return 3

    def auto_detect(self, img):
        h, w = img.shape[:2]
        detected_boxes = []

        # YOLO 推論
        if self.model is not None:
            try:
                results = self.model(img, conf=0.25, verbose=False)
                for box in results[0].boxes:
                    cls_id = int(box.cls[0])
                    if cls_id >= 4: cls_id = 4
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    detected_boxes.append({'class': cls_id, 'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2})
            except Exception as e:
                print(f"[警告] YOLO 預測錯誤: {e}")

        # OpenCV 霍夫圓檢測 (球)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=25, param1=50, param2=30, minRadius=12, maxRadius=35)
        opencv_proposals = []
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cx, cy, r = int(i[0]), int(i[1]), int(i[2])
                x1, y1 = max(0, cx - r), max(0, cy - r)
                x2, y2 = min(w, cx + r), min(h, cy + r)
                crop_box = img[max(0, cy-5):min(h, cy+5), max(0, cx-5):min(w, cx+5)]
                if crop_box.size > 0:
                    cls_id = self.classify_color(crop_box)
                    opencv_proposals.append({'class': cls_id, 'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2})

        # OpenCV 輪廓分析 (球洞)
        _, thresh = cv2.threshold(blurred, 40, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 300 < area < 8000:
                (cx, cy), r = cv2.minEnclosingCircle(cnt)
                x, y, cw, ch = cv2.boundingRect(cnt)
                if 0.7 < float(cw)/ch < 1.3:
                    rx = int(r)
                    opencv_proposals.append({'class': 4, 'x1': max(0, int(cx - rx)), 'y1': max(0, int(cy - rx)), 'x2': min(w, int(cx + rx)), 'y2': min(h, int(cy + rx))})

        # NMS 融合
        final_boxes = list(detected_boxes)
        for op in opencv_proposals:
            box_op = [op['x1'], op['y1'], op['x2'], op['y2']]
            is_duplicate = False
            for db in final_boxes:
                box_db = [db['x1'], db['y1'], db['x2'], db['y2']]
                if self.calculate_iou(box_op, box_db) > 0.4:
                    is_duplicate = True
                    break
            if not is_duplicate:
                final_boxes.append(op)

        return final_boxes

    def save_yolo_txt(self, img_path, boxes, h, w):
        txt_path = os.path.splitext(img_path)[0] + ".txt"
        lines = []
        for box in boxes:
            bx1, bx2 = sorted([box['x1'], box['x2']])
            by1, by2 = sorted([box['y1'], box['y2']])
            bx1, bx2 = max(0, min(bx1, w)), max(0, min(bx2, w))
            by1, by2 = max(0, min(by1, h)), max(0, min(by2, h))
            cx = ((bx1 + bx2) / 2.0) / w
            cy = ((by1 + by2) / 2.0) / h
            bw = float(bx2 - bx1) / w
            bh = float(by2 - by1) / h
            lines.append(f"{box['class']} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")
        with open(txt_path, "w", encoding="utf-8") as f:
            f.writelines(lines)
        return txt_path

    def run_pipeline(self):
        jpg_paths = sorted(glob.glob(os.path.join(DATA_DIR, "*.jpg")))
        if not jpg_paths:
            print(f"[錯誤] {DATA_DIR} 中找不到任何圖片！")
            return

        print(f"\n🚀 [開始自動批次標註與上傳] 總計 {len(jpg_paths)} 張圖片...")
        history_file_handle = open(HISTORY_FILE, "a", encoding="utf-8")
        
        success_count = 0
        skip_count = 0

        try:
            for idx, img_path in enumerate(jpg_paths):
                filename = os.path.basename(img_path)
                print(f"\n👉 [{idx+1}/{len(jpg_paths)}] 處理檔案: {filename}")

                if filename in self.uploaded_files:
                    print(f"   [跳過] 此影像已上傳過。")
                    skip_count += 1
                    continue

                img = cv2.imread(img_path)
                if img is None:
                    print("   ❌ 無法讀取影像！")
                    continue
                h, w = img.shape[:2]

                # 1. 檢查是否已有手動/本地標註檔，無則自動預測
                txt_path = os.path.splitext(img_path)[0] + ".txt"
                if os.path.exists(txt_path):
                    print("   [標記] 偵測到現有標籤檔，直接使用本地標籤。")
                else:
                    print("   [辨識] 正在自動偵測撞球與球洞...")
                    boxes = self.auto_detect(img)
                    txt_path = self.save_yolo_txt(img_path, boxes, h, w)
                    print(f"   [儲存] 已存檔標籤至 {os.path.basename(txt_path)} (偵測到 {len(boxes)} 個物件)")

                # 2. 上傳至 Roboflow
                try:
                    print("   [上傳] 正在傳送至 Roboflow...")
                    self.rf_project.single_upload(
                        image_path=img_path,
                        annotation_path=txt_path,
                        split="train",
                        is_prediction=False
                    )
                    # 紀錄成功
                    history_file_handle.write(filename + "\n")
                    history_file_handle.flush()
                    success_count += 1
                    print("   ✅ 上傳成功！")
                except Exception as e:
                    print(f"   ❌ 上傳失敗: {e}")

        finally:
            history_file_handle.close()

        print("\n=======================================================")
        print(" 🎉 [批次自動處理完成] 成果統計：")
        print(f"   - 成功上傳：{success_count} 張影像與標籤")
        print(f"   - 重複跳過：{skip_count} 張影像")
        print(f"   - 上傳歷史已更新於 '{HISTORY_FILE}'")
        print("=======================================================\n")

if __name__ == "__main__":
    annotator = AutoAnnotator()
    annotator.run_pipeline()
