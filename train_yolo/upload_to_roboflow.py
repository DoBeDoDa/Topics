import os
import glob
import sys

# ==========================================
# [Roboflow 設定參數]
# 請在此填入您的 Roboflow 專案資訊
# ==========================================
ROBOFLOW_API_KEY = "YOUR_API_KEY"      # 填入您的 Roboflow Private API Key
WORKSPACE_ID = "YOUR_WORKSPACE_ID"      # 填入您的 Workspace ID
PROJECT_ID = "YOUR_PROJECT_ID"          # 填入您的 Project ID
DATA_DIR = "yolo_data"                  # 資料夾路徑
HISTORY_FILE = "uploaded_history.txt"   # 上傳紀錄檔，避免重複上傳

def install_and_import():
    try:
        from roboflow import Roboflow
        return Roboflow
    except ImportError:
        print("[系統提示] 偵測到未安裝 roboflow SDK。")
        print("正在嘗試自動安裝 roboflow 套件，請稍候...")
        try:
            import subprocess
            subprocess.check_call([sys.executable, "-m", "pip", "install", "roboflow"])
            from roboflow import Roboflow
            print("[系統] roboflow 安裝成功！")
            return Roboflow
        except Exception as e:
            print(f"[錯誤] 自動安裝失敗: {e}")
            print("請手動在終端機執行: pip install roboflow")
            sys.exit(1)

def main():
    # 檢查是否已填寫 API Key
    if ROBOFLOW_API_KEY == "YOUR_API_KEY" or WORKSPACE_ID == "YOUR_WORKSPACE_ID" or PROJECT_ID == "YOUR_PROJECT_ID":
        print("\n=======================================================")
        print(" ❌ [配置錯誤] 請先編輯 'upload_to_roboflow.py' 填寫您的專案資訊：")
        print("   - ROBOFLOW_API_KEY (Private API Key)")
        print("   - WORKSPACE_ID (工作區 ID)")
        print("   - PROJECT_ID (專案 ID)")
        print("=======================================================")
        print("💡 您可以在 Roboflow 專案頁面的 'API Keys' 或是網址中找到這些資訊。")
        return

    # 自動導入/安裝 Roboflow SDK
    RoboflowClass = install_and_import()

    # 讀取已上傳紀錄
    uploaded_files = set()
    if os.path.exists(HISTORY_FILE):
        try:
            with open(HISTORY_FILE, "r", encoding="utf-8") as f:
                for line in f:
                    uploaded_files.add(line.strip())
        except Exception as e:
            print(f"[警告] 無法讀取上傳紀錄檔: {e}")

    # 掃描 yolo_data 資料夾中的 .jpg 與 .txt 檔案
    print(f"[系統] 正在掃描 '{DATA_DIR}' 資料夾中已檢查的標註資料...")
    jpg_paths = glob.glob(os.path.join(DATA_DIR, "*.jpg"))
    
    to_upload = []
    for jpg_path in jpg_paths:
        base_name = os.path.splitext(jpg_path)[0]
        txt_path = base_name + ".txt"
        
        # 只有「同時有 JPG 圖片」與「已經標註檢查好的 TXT 文字檔」的資料才需要上傳
        if os.path.exists(txt_path):
            filename = os.path.basename(jpg_path)
            if filename not in uploaded_files:
                to_upload.append((jpg_path, txt_path, filename))

    if not to_upload:
        print("[系統提示] 沒有新標註好的影像需要上傳！(所有已標註的影像均已上傳，或尚未進行任何標註)")
        return

    print(f"[系統] 偵測到 {len(to_upload)} 張全新且已標註完畢的影像準備上傳。")
    print("[系統] 正在初始化 Roboflow 連線...")
    
    try:
        rf = RoboflowClass(api_key=ROBOFLOW_API_KEY)
        project = rf.workspace(WORKSPACE_ID).project(PROJECT_ID)
        print("[系統] 成功連線至 Roboflow 專案！開始上傳...")
    except Exception as e:
        print(f"❌ [連線錯誤] 初始化 Roboflow 失敗: {e}")
        print("請檢查您的 API Key、Workspace ID 與 Project ID 是否正確。")
        return

    success_count = 0
    history_file_handle = open(HISTORY_FILE, "a", encoding="utf-8")

    try:
        for idx, (jpg_path, txt_path, filename) in enumerate(to_upload):
            print(f"[{idx+1}/{len(to_upload)}] 正在上傳 {filename} 與標註檔...")
            try:
                # 使用 single_upload 進行上傳
                project.single_upload(
                    image_path=jpg_path,
                    annotation_path=txt_path,
                    split="train",  # 預設上傳至訓練集，上傳後可在網頁端重新分配或擴增
                    is_prediction=False
                )
                
                # 記錄上傳成功歷史，避免重複
                history_file_handle.write(filename + "\n")
                history_file_handle.flush()
                success_count += 1
                
            except Exception as e:
                print(f"  ❌ 上傳 {filename} 失敗: {e}")

    finally:
        history_file_handle.close()

    print("\n=======================================================")
    print(f" 🎉 上傳工作完成！")
    print(f"   - 成功上傳：{success_count} / {len(to_upload)} 張影像與標籤")
    print(f"   - 上傳歷史已記錄於 '{HISTORY_FILE}'")
    print("=======================================================")
    print("💡 現在您可以前往 Roboflow 網頁端進行資料擴增 (Augmentation) 與生成了！")

if __name__ == "__main__":
    main()
