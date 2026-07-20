"""使用既有校正點訓練選用的神經網路座標轉換模型。"""

import os
import sys
import numpy as np

# Ensure PyTorch is installed
try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
except ImportError:
    print("[錯誤] 未能在目前環境中載入 PyTorch。請確認您是否使用虛擬環境執行此程式：")
    print("      .venv\\Scripts\\python train_calibration.py")
    sys.exit(1)

# Import calibration points and the model class from robot.py
try:
    from robot import DEFAULT_CAM_POINTS, DEFAULT_TABLE_POINTS, CalibrationNet
except ImportError:
    print("[錯誤] 無法從 robot.py 載入對應點與網路模型定義，請確保 robot.py 與此腳本在同一個資料夾。")
    sys.exit(1)

def train():
    print("==================================================")
    # 1. 準備數據
    cam_points = np.array(DEFAULT_CAM_POINTS, dtype=np.float32)
    table_points = np.array(DEFAULT_TABLE_POINTS, dtype=np.float32)
    
    print(f"[數據狀態] 載入共 {len(cam_points)} 組校正對應點。")
    
    # 計算平均值與標準差，用於進行特徵縮放 (Normalization)
    cam_mean = cam_points.mean(axis=0)
    cam_std = cam_points.std(axis=0)
    table_mean = table_points.mean(axis=0)
    table_std = table_points.std(axis=0)
    
    # 進行資料標準化，提升神經網路訓練收斂速度
    cam_scaled = (cam_points - cam_mean) / cam_std
    table_scaled = (table_points - table_mean) / table_std
    
    # 轉換成 PyTorch Tensors
    X_train = torch.tensor(cam_scaled, dtype=torch.float32)
    y_train = torch.tensor(table_scaled, dtype=torch.float32)
    
    # 2. 初始化模型、損失函數與優化器
    model = CalibrationNet()
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=0.01)
    
    # 3. 訓練循環
    epochs = 2000
    print(f"[訓練狀態] 正在使用 Adam 優化器進行 {epochs} 次迭代訓練...")
    
    for epoch in range(epochs):
        model.train()
        optimizer.zero_grad()
        
        # 前向傳播
        predictions = model(X_train)
        loss = criterion(predictions, y_train)
        
        # 反向傳播
        loss.backward()
        optimizer.step()
        
        if (epoch + 1) % 400 == 0:
            print(f"  - Epoch [{epoch + 1}/{epochs}], MSE Loss: {loss.item():.6f}")
            
    # 4. 儲存模型權重與標準化係數
    script_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.dirname(script_dir)
    model_save_path = os.path.join(root_dir, "bin", "calibration_model.pth")
    torch.save({
        'model_state': model.state_dict(),
        'cam_mean': cam_mean,
        'cam_std': cam_std,
        'table_mean': table_mean,
        'table_std': table_std
    }, model_save_path)
    
    print(f"[系統狀態] 模型已成功訓練並儲存至: {model_save_path}")
    print("==================================================")
    
    # 5. 驗證與誤差分析 (Sanity Check)
    model.eval()
    with torch.no_grad():
        pred_scaled = model(X_train).numpy()
    pred_table = pred_scaled * table_std + table_mean
    
    # 計算每個點的歐幾里得距離誤差 (mm)
    errors = np.linalg.norm(pred_table - table_points, axis=1)
    mean_error = np.mean(errors)
    max_error = np.max(errors)
    
    print(f"[訓練成效分析]")
    print(f"  - 平均對齊誤差: {mean_error:.3f} mm")
    print(f"  - 最大對齊誤差: {max_error:.3f} mm")
    print("==================================================")

if __name__ == "__main__":
    train()
