from ultralytics import YOLO
import os
import torch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATASET_YAML = os.path.join(SCRIPT_DIR, "yolo_data", "dataset.yaml")

def main():
    # 檢查是否有安裝與支援 CUDA GPU，若無則自動切換至 CPU
    device = 0 if torch.cuda.is_available() else 'cpu'
    device_name = "NVIDIA GPU" if device == 0 else "CPU"
    print(f"[系統] 偵測到訓練裝置: {device_name}")

    # 1. 載入最新 YOLO26 輕量化預訓練模型 (首次執行會自動下載)
    # yolo26n.pt 是 2026 年最新旗艦邊緣運算模型，針對 CPU/GPU 速度進行了極大優化
    print("[系統] 正在載入 YOLO26n 預訓練模型...")
    model = YOLO("yolo26n.pt")

    # 2. 開始訓練 (使用優化過的快速訓練參數)
    print("\n=======================================================")
    print(" 🚀 [開始 YOLO26 快速訓練] ")
    print("=======================================================")
    model.train(
        data=DATASET_YAML,    # 資料集設定檔路徑
        epochs=150,           # 訓練輪數 (撞球任務約 100~150 輪即可收斂)
        imgsz=640,            # 影像解析度
        batch=16,             # 批次大小 (若顯存不足可改為 8，充足可改為 32)
        cache=True,           # 開啟記憶體快取 (RAM Cache)，徹底消除磁碟讀取瓶頸
        amp=True,             # 開啟混合精度訓練 (減少顯存使用並加速運算)
        workers=8,            # 數據載入執行緒
        device=device         # 自動裝置選擇 (GPU/CPU)
    )
    
    print("\n=======================================================")
    print(" 🎉 [訓練完成] ")
    print("   - 訓練產出的最佳權重檔案儲存在：")
    print("     runs/detect/train/weights/best.pt")
    print("   - 您可以將該檔案複製到根目錄替換原本的 best.pt 來使用新模型！")
    print("=======================================================\n")

if __name__ == "__main__":
    main()
