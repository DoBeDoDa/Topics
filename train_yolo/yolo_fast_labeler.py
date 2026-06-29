import cv2
import os
import glob
import numpy as np
from ultralytics import YOLO

# ==========================================
# [設定參數]
# ==========================================
DATA_DIR = "yolo_data"
MODEL_PATH = "best.pt"
WINDOW_NAME = "YOLO Fast Labeler & Inspector"

# 類別定義與色彩
CLASSES = {
    0: "b1",      # 黃色 1 號球
    1: "b2",      # 藍色 2 號球
    2: "b3",      # 紅色 3 號球
    3: "bw",      # 白色母球
    4: "pocket"   # 球洞 (統一為同一類別)
}

COLOR_MAP = {
    0: (0, 255, 255),    # b1: 黃色
    1: (255, 0, 0),      # b2: 藍色
    2: (0, 0, 255),      # b3: 紅色
    3: (255, 255, 255),  # bw: 白色
    4: (255, 0, 255)     # pocket: 紫色
}

class YoloFastLabeler:
    def __init__(self, data_dir=DATA_DIR, model_path=MODEL_PATH):
        self.data_dir = data_dir
        self.image_paths = sorted(glob.glob(os.path.join(data_dir, "*.jpg")))
        if not self.image_paths:
            print(f"[警告] {data_dir} 中找不到任何 .jpg 圖片！")
        else:
            print(f"[系統] 找到 {len(self.image_paths)} 張待標註圖片。")

        # 載入 YOLO 模型 (選用)
        self.model = None
        if os.path.exists(model_path):
            try:
                print(f"[系統] 偵測到 '{model_path}'，正在載入預訓練模型以進行預標註...")
                self.model = YOLO(model_path)
                print("[系統] YOLO 模型載入成功。")
            except Exception as e:
                print(f"[警告] 無法載入 YOLO 模型 ({e})，將只使用 OpenCV 進行自動辨識。")
        else:
            print(f"[系統] 未找到 '{model_path}'，將完全使用 OpenCV 影像分析進行預標註。")

        self.current_idx = 0
        self.boxes = []  # 儲存當前圖片的標註框：[{'class': int, 'x1': int, 'y1': int, 'x2': int, 'y2': int}]
        self.selected_idx = None
        
        # 互動狀態變數
        self.is_drawing = False
        self.is_moving = False
        self.is_resizing = False
        self.resize_handle = None  # 'tl', 'tr', 'bl', 'br'
        self.drag_start = (0, 0)
        self.drag_offset = (0, 0)  # 用於移動標註框
        self.temp_box = None       # 用於手動繪製時的暫時框

        # 產生 dataset.yaml
        self.generate_dataset_yaml()

    def generate_dataset_yaml(self):
        """自動生成 dataset.yaml 供後續快速訓練 YOLO"""
        abs_path = os.path.abspath(self.data_dir).replace('\\', '/')
        yaml_content = f"""path: {abs_path}
train: .
val: .

names:
  0: b1
  1: b2
  2: b3
  3: bw
  4: pocket
"""
        yaml_path = os.path.join(self.data_dir, "dataset.yaml")
        try:
            with open(yaml_path, "w", encoding="utf-8") as f:
                f.write(yaml_content)
            print(f"[系統] 已生成/更新 YOLO 資料集設定檔: {yaml_path}")
        except Exception as e:
            print(f"[警告] 無法生成 dataset.yaml: {e}")

    def load_labels(self, img_path):
        """讀取已存在的 YOLO label 檔，若無則進行自動預標註"""
        txt_path = os.path.splitext(img_path)[0] + ".txt"
        img = cv2.imread(img_path)
        h, w = img.shape[:2]
        
        # 如果已經有標註檔案，優先讀取
        if os.path.exists(txt_path):
            boxes = []
            try:
                with open(txt_path, "r", encoding="utf-8") as f:
                    for line in f:
                        parts = line.strip().split()
                        if len(parts) == 5:
                            cls_id = int(parts[0])
                            # 防止類別越界 (將舊模型的 4-9 映射至 4: pocket)
                            if cls_id > 4:
                                cls_id = 4
                            cx, cy, bw, bh = map(float, parts[1:])
                            # 轉換回像素座標
                            x1 = int((cx - bw/2) * w)
                            y1 = int((cy - bh/2) * h)
                            x2 = int((cx + bw/2) * w)
                            y2 = int((cy + bh/2) * h)
                            boxes.append({'class': cls_id, 'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2})
                return boxes
            except Exception as e:
                print(f"[錯誤] 讀取標註檔 {txt_path} 失敗 ({e})，將重新自動辨識。")

        # 若無現有標註，使用自動辨識 (YOLO + OpenCV 混合)
        return self.auto_detect(img)

    def save_labels(self, img_path):
        """將當前標註框存成 YOLO 格式的 .txt 檔案"""
        if not self.boxes:
            # 如果清空了所有框，則刪除對應的 label 檔案
            txt_path = os.path.splitext(img_path)[0] + ".txt"
            if os.path.exists(txt_path):
                try:
                    os.remove(txt_path)
                except Exception as e:
                    print(f"[錯誤] 無法刪除空白標註檔 {txt_path}: {e}")
            return

        txt_path = os.path.splitext(img_path)[0] + ".txt"
        img = cv2.imread(img_path)
        h, w = img.shape[:2]

        lines = []
        for box in self.boxes:
            # 計算歸一化中心點與寬高
            bx1, bx2 = sorted([box['x1'], box['x2']])
            by1, by2 = sorted([box['y1'], box['y2']])
            
            # 限制在影像邊界內
            bx1 = max(0, min(bx1, w))
            bx2 = max(0, min(bx2, w))
            by1 = max(0, min(by1, h))
            by2 = max(0, min(by2, h))

            cx = ((bx1 + bx2) / 2.0) / w
            cy = ((by1 + by2) / 2.0) / h
            bw = float(bx2 - bx1) / w
            bh = float(by2 - by1) / h
            
            lines.append(f"{box['class']} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")

        try:
            with open(txt_path, "w", encoding="utf-8") as f:
                f.writelines(lines)
        except Exception as e:
            print(f"[錯誤] 無法儲存標註檔 {txt_path}: {e}")

    def auto_detect(self, img):
        """自動偵測：整合 YOLO (如果存在) 與 OpenCV 霍夫圓 + 顏色特徵分析"""
        h, w = img.shape[:2]
        detected_boxes = []

        # 1. 執行 YOLO 模型推論
        if self.model is not None:
            try:
                results = self.model(img, conf=0.25, verbose=False)
                for box in results[0].boxes:
                    cls_id = int(box.cls[0])
                    # 將 4-9 (p1-p6) 統一映射至 4 (pocket)
                    if cls_id >= 4:
                        cls_id = 4
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    detected_boxes.append({'class': cls_id, 'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2})
            except Exception as e:
                print(f"[警告] YOLO 預測時發生錯誤: {e}")

        # 2. OpenCV 輔助偵測 (如果 YOLO 漏失，或者完全使用 OpenCV)
        # 用 HoughCircles 尋找圓形球體
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        
        # 參數根據您的桌面與相機解析度微調，minDist 確保球不重疊
        circles = cv2.HoughCircles(
            blurred, 
            cv2.HOUGH_GRADIENT, 
            dp=1.2, 
            minDist=25, 
            param1=50, 
            param2=30, 
            minRadius=12, 
            maxRadius=35
        )

        opencv_proposals = []
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cx, cy, r = int(i[0]), int(i[1]), int(i[2])
                
                # 計算框邊界
                x1, y1 = cx - r, cy - r
                x2, y2 = cx + r, cy + r
                
                # 確保邊界合理
                x1 = max(0, x1)
                y1 = max(0, y1)
                x2 = min(w, x2)
                y2 = min(h, y2)

                # 提取圓形內部區域進行簡易色彩分類
                mask = np.zeros(gray.shape, dtype=np.uint8)
                cv2.circle(mask, (cx, cy), r - 2, 255, -1)
                crop_bgr = cv2.bitwise_and(img, img, mask=mask)
                
                # 取得圓心附近色彩
                crop_box = img[max(0, cy-5):min(h, cy+5), max(0, cx-5):min(w, cx+5)]
                if crop_box.size > 0:
                    cls_id = self.classify_color(crop_box)
                    opencv_proposals.append({'class': cls_id, 'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2})

        # 3. OpenCV 偵測暗色球洞 (contours)
        # 尋找大面積的暗色圓形區域
        _, thresh = cv2.threshold(blurred, 40, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 300 < area < 8000:
                # 計算外接圓與長寬比
                (cx, cy), r = cv2.minEnclosingCircle(cnt)
                x, y, cw, ch = cv2.boundingRect(cnt)
                aspect_ratio = float(cw) / ch
                
                # 球洞形狀通常很接近正圓
                if 0.7 < aspect_ratio < 1.3:
                    rx = int(r)
                    x1 = max(0, int(cx - rx))
                    y1 = max(0, int(cy - rx))
                    x2 = min(w, int(cx + rx))
                    y2 = min(h, int(cy + rx))
                    opencv_proposals.append({'class': 4, 'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2})

        # 4. 融合 YOLO 與 OpenCV 預測框 (利用重疊面積 NMS 避免重複)
        final_boxes = list(detected_boxes)  # 以 YOLO 優先
        
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

    def classify_color(self, crop):
        """根據 HSV 顏色空間快速將球分類 (0:b1黃, 1:b2藍, 2:b3紅, 3:bw白, 4:pocket洞)"""
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        h_mean = np.mean(hsv[:, :, 0])
        s_mean = np.mean(hsv[:, :, 1])
        v_mean = np.mean(hsv[:, :, 2])

        # 洞：極暗
        if v_mean < 45:
            return 4
        
        # 白母球：亮度高，飽和度低
        if v_mean > 160 and s_mean < 75:
            return 3

        # 根據 Hue 值判斷黃、藍、紅
        # 黃色 Hue 約 15 ~ 35
        # 藍色 Hue 約 95 ~ 130
        # 紅色 Hue 約 0~10 或 165~180
        if 12 <= h_mean <= 40:
            return 0  # b1: 黃
        elif 85 <= h_mean <= 135:
            return 1  # b2: 藍
        elif h_mean <= 12 or h_mean >= 155:
            return 2  # b3: 紅
        
        # 預設白色
        return 3

    def calculate_iou(self, box1, box2):
        """計算兩矩形框的重疊程度 (Intersection over Union)"""
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

    def mouse_callback(self, event, x, y, flags, param):
        """滑鼠互動邏輯：點擊選取、拖曳移動/縮放、拉取新框"""
        margin = 8  # 邊角檢測範圍 px
        
        if event == cv2.EVENT_LBUTTONDOWN:
            # 1. 優先檢查是否點在已選中框的四個角落，準備進行縮放 (Resize)
            if self.selected_idx is not None:
                box = self.boxes[self.selected_idx]
                x1, y1, x2, y2 = box['x1'], box['y1'], box['x2'], box['y2']
                
                # 角落定義
                handles = {
                    'tl': (x1, y1),
                    'tr': (x2, y1),
                    'bl': (x1, y2),
                    'br': (x2, y2)
                }
                
                for handle, (hx, hy) in handles.items():
                    if abs(x - hx) < margin and abs(y - hy) < margin:
                        self.is_resizing = True
                        self.resize_handle = handle
                        self.drag_start = (x, y)
                        return

            # 2. 檢查是否點擊在任何標註框內部，準備移動 (Move) 或切換選取
            clicked_idx = None
            for idx, box in enumerate(self.boxes):
                bx1, bx2 = sorted([box['x1'], box['x2']])
                by1, by2 = sorted([box['y1'], box['y2']])
                if bx1 <= x <= bx2 and by1 <= y <= by2:
                    clicked_idx = idx
                    break  
            
            if clicked_idx is not None:
                self.selected_idx = clicked_idx
                self.is_moving = True
                self.drag_start = (x, y)
                box = self.boxes[clicked_idx]
                self.drag_offset = (box['x1'] - x, box['y1'] - y, box['x2'] - x, box['y2'] - y)
            else:
                # 3. 點在空白處，準備手動繪製新標註框 (Draw)
                self.selected_idx = None
                self.is_drawing = True
                self.drag_start = (x, y)
                self.temp_box = [x, y, x, y]

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.is_resizing and self.selected_idx is not None:
                # 縮放標註框
                box = self.boxes[self.selected_idx]
                if self.resize_handle == 'tl':
                    box['x1'], box['y1'] = x, y
                elif self.resize_handle == 'tr':
                    box['x2'], box['y1'] = x, y
                elif self.resize_handle == 'bl':
                    box['x1'], box['y2'] = x, y
                elif self.resize_handle == 'br':
                    box['x2'], box['y2'] = x, y
            
            elif self.is_moving and self.selected_idx is not None:
                # 移動標註框
                box = self.boxes[self.selected_idx]
                box['x1'] = x + self.drag_offset[0]
                box['y1'] = y + self.drag_offset[1]
                box['x2'] = x + self.drag_offset[2]
                box['y2'] = y + self.drag_offset[3]

            elif self.is_drawing:
                # 繪製新框
                self.temp_box[2], self.temp_box[3] = x, y

        elif event == cv2.EVENT_LBUTTONUP:
            if self.is_resizing:
                self.is_resizing = False
                # 確保 x1 < x2 且 y1 < y2
                box = self.boxes[self.selected_idx]
                box['x1'], box['x2'] = sorted([box['x1'], box['x2']])
                box['y1'], box['y2'] = sorted([box['y1'], box['y2']])
                
            elif self.is_moving:
                self.is_moving = False
                box = self.boxes[self.selected_idx]
                box['x1'], box['x2'] = sorted([box['x1'], box['x2']])
                box['y1'], box['y2'] = sorted([box['y1'], box['y2']])

            elif self.is_drawing:
                self.is_drawing = False
                x1, y1, x2, y2 = self.temp_box
                x1, x2 = sorted([x1, x2])
                y1, y2 = sorted([y1, y2])
                
                # 面積過小不予保留
                if (x2 - x1) > 6 and (y2 - y1) > 6:
                    # 預設為最後選取的類別，若無則預設為 bw (類別 3)
                    default_cls = 3
                    self.boxes.append({'class': default_cls, 'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2})
                    self.selected_idx = len(self.boxes) - 1
                self.temp_box = None

        elif event == cv2.EVENT_RBUTTONDOWN:
            # 右鍵刪除點擊處的標註框
            clicked_idx = None
            for idx, box in enumerate(self.boxes):
                bx1, bx2 = sorted([box['x1'], box['x2']])
                by1, by2 = sorted([box['y1'], box['y2']])
                if bx1 <= x <= bx2 and by1 <= y <= by2:
                    clicked_idx = idx
                    break
            if clicked_idx is not None:
                self.boxes.pop(clicked_idx)
                self.selected_idx = None

    def run(self):
        if not self.image_paths:
            print("[錯誤] 無法啟動標註器，因為沒有圖片！")
            return

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)

        print("\n=======================================================")
        print(" [YOLO 快速標註與驗證工具] 已啟動！")
        print("-------------------------------------------------------")
        print(" 📌 滑鼠動作:")
        print("   - 左鍵點擊/拖曳框中心: 選取與「移動」標註框")
        print("   - 左鍵點擊框的四個角: 拖曳可「調整大小」")
        print("   - 空白處左鍵拖曳: 「手動新增」新標註框 (預設類別: bw)")
        print("   - 右鍵點擊框內: 「直接刪除」該標註框")
        print(" 📌 鍵盤快速鍵:")
        print("   - [Space] 或 [Enter] : 儲存當前標註，進入「下一張」")
        print("   - [B] 或 [b]         : 退回「上一張」影像")
        print("   - [C] 或 [c]         : 「清空」當前影像的所有標註")
        print("   - [D] 或 [d]         : 刪除當前「選中」的標註框")
        print("   - [R] 或 [r]         : 「重設」當前影像為系統自動預標註結果")
        print("   - [0], [1], [2], [3] : 切換選中框為 撞球 0:b1, 1:b2, 2:b3, 3:bw")
        print("   - [4]                : 切換選中框為 4:pocket (球洞)")
        print("   - [Esc] 或 [Q]/[q]    : 儲存當前標註並「關閉程式」")
        print("=======================================================")

        # 載入第一張圖片的標註
        self.boxes = self.load_labels(self.image_paths[self.current_idx])

        while True:
            img_path = self.image_paths[self.current_idx]
            frame = cv2.imread(img_path)
            if frame is None:
                print(f"[錯誤] 無法讀取圖片: {img_path}")
                self.current_idx = (self.current_idx + 1) % len(self.image_paths)
                continue

            h, w = frame.shape[:2]
            display_frame = frame.copy()

            # 繪製所有現有的標註框
            for idx, box in enumerate(self.boxes):
                x1, y1, x2, y2 = box['x1'], box['y1'], box['x2'], box['y2']
                cls_id = box['class']
                color = COLOR_MAP.get(cls_id, (0, 255, 0))
                label = CLASSES.get(cls_id, f"cls {cls_id}")

                # 判斷是否為選中狀態
                if idx == self.selected_idx:
                    # 畫選中框 (加粗)
                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 3)
                    # 在四個角畫小把手
                    r = 5
                    cv2.rectangle(display_frame, (x1-r, y1-r), (x1+r, y1+r), (0, 0, 255), -1)
                    cv2.rectangle(display_frame, (x2-r, y1-r), (x2+r, y1+r), (0, 0, 255), -1)
                    cv2.rectangle(display_frame, (x1-r, y2-r), (x1+r, y2+r), (0, 0, 255), -1)
                    cv2.rectangle(display_frame, (x2-r, y2-r), (x2+r, y2+r), (0, 0, 255), -1)
                else:
                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)

                # 寫上標記名稱
                cv2.putText(display_frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # 繪製正在拖曳中的暫時新框
            if self.is_drawing and self.temp_box:
                tx1, ty1, tx2, ty2 = self.temp_box
                cv2.rectangle(display_frame, (tx1, ty1), (tx2, ty2), (0, 255, 0), 1)
                cv2.putText(display_frame, "drawing...", (tx1, ty1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

            # 繪製上方資訊狀態欄
            overlay = display_frame.copy()
            cv2.rectangle(overlay, (0, 0), (w, 60), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.6, display_frame, 0.4, 0, display_frame)

            # 顯示進度與路徑資訊
            filename = os.path.basename(img_path)
            status_text = f"[{self.current_idx + 1} / {len(self.image_paths)}] File: {filename}"
            cv2.putText(display_frame, status_text, (20, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            # 顯示操作提示捷徑
            guide_text = "Space: Save & Next | B: Back | C: Clear | R: Auto-Reset | Esc: Quit"
            cv2.putText(display_frame, guide_text, (20, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # 顯示右側分類提示
            class_legend = "0:b1 1:b2 2:b3 3:bw 4:pocket"
            cv2.putText(display_frame, class_legend, (w - 280, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            if self.selected_idx is not None:
                sel_box = self.boxes[self.selected_idx]
                sel_label = CLASSES[sel_box['class']]
                sel_info = f"Selected: [{sel_label}] (Press 0-4 to change)"
                cv2.putText(display_frame, sel_info, (w - 380, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            cv2.imshow(WINDOW_NAME, display_frame)
            key = cv2.waitKey(30) & 0xFF

            # 鍵盤事件處理
            if key == ord(' ') or key == 13:  # Space 鍵 或 Enter 鍵
                # 儲存當前標註並切換至下一張
                self.save_labels(img_path)
                print(f"[儲存] 已存檔: {os.path.basename(img_path)}")
                
                self.current_idx += 1
                if self.current_idx >= len(self.image_paths):
                    print("\n🎉 恭喜！您已完成所有圖片的檢查與標註！")
                    self.current_idx = len(self.image_paths) - 1
                    
                self.selected_idx = None
                self.boxes = self.load_labels(self.image_paths[self.current_idx])

            elif key == ord('b') or key == ord('B'):
                # 回退到前一張圖片
                self.save_labels(img_path)
                self.current_idx -= 1
                if self.current_idx < 0:
                    self.current_idx = 0
                    print("[提示] 已經是第一張圖片。")
                self.selected_idx = None
                self.boxes = self.load_labels(self.image_paths[self.current_idx])

            elif key == ord('c') or key == ord('C'):
                # 清空當前圖片的標註
                self.boxes = []
                self.selected_idx = None
                print("[清除] 已清空當前圖片的標註。")

            elif key == ord('d') or key == ord('D') or key == 127:  # d/D 鍵 或 Del 鍵
                # 刪除選中的標註框
                if self.selected_idx is not None:
                    self.boxes.pop(self.selected_idx)
                    self.selected_idx = None
                    print("[刪除] 已刪除選中的標註框。")

            elif key == ord('r') or key == ord('R'):
                # 重設自動標註
                self.boxes = self.auto_detect(frame)
                self.selected_idx = None
                print("[重設] 已重新載入自動標註結果。")

            elif key in [ord('0'), ord('1'), ord('2'), ord('3'), ord('4')]:
                # 切換選中標註框的類別
                if self.selected_idx is not None:
                    cls_id = int(chr(key))
                    self.boxes[self.selected_idx]['class'] = cls_id
                    print(f"[類別切換] 已將選中框切換為 {CLASSES[cls_id]}")

            elif key == 27 or key == ord('q') or key == ord('Q'):  # Esc 鍵 或 Q 鍵
                # 儲存當前並退出
                self.save_labels(img_path)
                print(f"[儲存] 已存檔並關閉: {os.path.basename(img_path)}")
                break

        cv2.destroyAllWindows()
        print("[系統] 標註驗證程式已正常關閉。")

if __name__ == "__main__":
    labeler = YoloFastLabeler()
    labeler.run()
