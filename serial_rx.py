import serial
import os
from datetime import datetime

SERIAL_PORT = 'COM3' 
BAUD_RATE = 921600  

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MOCK_SD_DIR = os.path.join(BASE_DIR, "mock_sd_storage") 

if not os.path.exists(MOCK_SD_DIR):
    os.makedirs(MOCK_SD_DIR)

print(f"💽 虛擬 SD 卡已啟動，儲存位置: {MOCK_SD_DIR}")
print("正在等待戶外偵測傳回高畫質照片 (若有除錯訊息也會顯示於此)...")

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) # 加入微小 timeout 以利於迴圈持續監聽
        ser.reset_input_buffer()

        while True:
            # 逐行讀取 UART 資料
            line = ser.readline()
            
            if not line:
                continue
                
            # 1. 如果收到存檔指令，進入接收照片模式
            if b"==SAVE_IMG_START==" in line:
                size_line = ser.readline().strip()
                try:
                    img_size = int(size_line)
                except:
                    continue

                print(f"\n📥 偵測到動作！接收照片中 ({img_size/1024:.1f} KB)...", end=" ", flush=True)

                img_data = b""
                while len(img_data) < img_size:
                    chunk = ser.read(img_size - len(img_data))
                    if not chunk: break
                    img_data += chunk
                
                ser.read_until(b"==SAVE_IMG_END==")

                if len(img_data) == img_size:
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = os.path.join(MOCK_SD_DIR, f"capture_{timestamp}.jpg")
                    with open(filename, "wb") as f:
                        f.write(img_data)
                    print(f"已存檔: capture_{timestamp}.jpg")
                else:
                    print("資料傳輸不完整")
            
            # 2. 如果收到的是其他文字，直接印出來當作除錯訊息
            else:
                try:
                    text = line.decode('utf-8', errors='ignore').strip()
                    if text:
                        print(f"[ESP32] {text}")
                except:
                    pass

    except Exception as e:
        print(f"\n錯誤: {e}")
    finally:
        ser.close()

if __name__ == "__main__":
    main()