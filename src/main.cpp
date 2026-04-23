/* ==============================================================================
 * 專案名稱：ESP32-CAM 戶外低功耗大範圍監控系統 (雙模版)
 * 核心邏輯：Light Sleep 喚醒 -> 低畫質灰階比對 -> 若有動靜 -> 高畫質彩色拍照存檔 -> 繼續睡
 * ============================================================================== */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_camera.h"          // ESP32 相機核心驅動
#include "freertos/FreeRTOS.h"   // 即時作業系統核心，用於任務調度與延遲
#include "freertos/task.h"       // 處理 vTaskDelay 等時間控制
#include "driver/uart.h"         // 序列埠通訊驅動 (用來跟電腦 Python 溝通)
#include "esp_log.h"             // 日誌輸出
#include "esp_sleep.h"           // 睡眠模式控制 (Light Sleep / Deep Sleep)

// 引入 SD 卡所需的函式庫 (未來切換為實體卡時使用)
//#include "FS.h"
//#include "SD_MMC.h"

// ⚙️ 儲存模式切換開關 (巨集設定)
// 透過編譯器前置處理，決定要編譯哪一段程式碼。
// 將 0 改成 1 即可無痛切換為「實體 SD 卡模式」
// ==========================================
#define USE_REAL_SD_CARD 0  
// ==========================================

// --- 系統運作與影像辨識參數 ---
#define WAKEUP_INTERVAL_SEC 2    // 每次 Light Sleep 睡覺的時間 (秒)
const int width = 160;           // 低解析度 QQVGA 的寬度
const int height = 120;          // 低解析度 QQVGA 的高度
const int diff_threshold = 30;   // 像素差異門檻 (0~255)，大於 30 才算該像素有變動 (抗光影雜訊)
const float motion_limit = 0.15; // 變動面積門檻，當畫面中有 15% 以上的像素變動，才判定為「有移動」

/* --- 關鍵記憶體區塊 (RTC_DATA_ATTR) ---
 * 一般變數在 Light Sleep 或 Deep Sleep 醒來後可能會遺失或重新初始化。
 * 加上 RTC_DATA_ATTR 可以將變數存放在 RTC (實時時鐘) 記憶體中，
 * 確保 ESP32 睡醒後，這些資料(如開機狀態、照片編號)依然存在。
 */
RTC_DATA_ATTR static bool is_first_boot = true; 
RTC_DATA_ATTR static int pic_counter = 0; // 實體 SD 卡的照片檔名流水號計數器
uint8_t * prev_frame = NULL;              // 用來存放上一張「背景圖」的指標 (Light Sleep 會保留 SRAM，所以這不用放 RTC)

// 初始化相機的硬體腳位 (這些是 ESP32-CAM 模組固定的硬體接線，不需修改)
void setup_camera_pins(camera_config_t &config) {
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = 5; 
    config.pin_d1 = 18; 
    config.pin_d2 = 19; 
    config.pin_d3 = 21;
    config.pin_d4 = 36; 
    config.pin_d5 = 39; 
    config.pin_d6 = 34; 
    config.pin_d7 = 35;
    config.pin_xclk = 0; 
    config.pin_pclk = 22; 
    config.pin_vsync = 25; 
    config.pin_href = 23;
    config.pin_sccb_sda = 26; 
    config.pin_sccb_scl = 27;
    config.pin_pwdn = 32; 
    config.pin_reset = -1;
    config.xclk_freq_hz = 10000000; // 降低時脈可稍微省電並減少雜訊
}

// ---------------------------------------------------------
// 儲存模組：負責將拍好的高畫質照片寫入 SD 卡或傳給電腦
// ---------------------------------------------------------
void saveImageToStorage(camera_fb_t *fb) {
    if (!fb) return;

#if USE_REAL_SD_CARD
    // 【實體 SD 卡模式邏輯】
    pic_counter++; // 每次拍照編號 +1
    char filename[32];
    sprintf(filename, "/pic_%04d.jpg", pic_counter); // 格式化檔名為 /pic_0001.jpg
    
    printf("Saving to Real SD Card: %s...\n", filename);
    
    fs::FS &fs = SD_MMC;
    File file = fs.open(filename, FILE_WRITE); // 開啟檔案準備寫入
    if (file) {
        file.write(fb->buf, fb->len);          // 將影像二進位資料寫入
        file.close();
        printf("✅ Successfully saved to SD Card!\n");
    } else {
        printf("❌ ERROR: Failed to open file in writing mode\n");
    }

#else
    // 【虛擬 SD 卡模式邏輯 (UART 連接電腦 Python)】
    printf("Sending to Mock SD via UART...\n");
    const uart_port_t uart_num = UART_NUM_0;
    
    // 傳送通訊協定的開頭標記
    uart_write_bytes(uart_num, "==SAVE_IMG_START==\n", 19);
    
    // 傳送照片大小，讓 Python 知道要接收多少資料
    char size_str[32];
    int len = sprintf(size_str, "%u\n", fb->len); 
    uart_write_bytes(uart_num, size_str, len);
    
    // 傳送照片本體
    uart_write_bytes(uart_num, (const char*)fb->buf, fb->len);
    
    // 傳送通訊協定的結尾標記
    uart_write_bytes(uart_num, "==SAVE_IMG_END==\n", 17);
#endif
}

// ---------------------------------------------------------
// 高畫質拍照模組：當偵測到移動時才呼叫，最耗電的環節
// ---------------------------------------------------------
void takeHighResPhoto() {
    printf("Switching to High-Res JPEG...\n");
    
    // 1. 關閉剛才用來比對的低畫質相機
    esp_camera_deinit(); 
    
    // 2. 重新配置為 VGA (640x480) 高畫質、彩色 JPEG 模式
    camera_config_t config;
    setup_camera_pins(config);
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12; // 數字越小畫質越好，12 是一個畫質與檔案大小的良好平衡
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_PSRAM;

    if (esp_camera_init(&config) == ESP_OK) {
        /* 3. 暖機機制 (解決 AWB 白平衡偏綠 Bug)
         * 相機剛通電啟動時，自動曝光與自動白平衡需要時間收斂。
         * 如果立刻拍照，會得到一張綠色或曝光不正確的圖。
         * 這裡刻意連續抓取並丟棄 5 張廢圖，強迫感測器適應環境光線。
         */
        for(int i=0; i<5; i++) {
            camera_fb_t * d_fb = esp_camera_fb_get();
            if (d_fb) esp_camera_fb_return(d_fb);
        }

        // 4. 抓取最終完美的正式照片
        camera_fb_t * fb = esp_camera_fb_get();
        if (fb) {
            printf("High-Res Photo captured!\n");
            saveImageToStorage(fb);   // 呼叫儲存模組進行存檔
            esp_camera_fb_return(fb); // 釋放相機記憶體
        }
    } else {
        printf("ERROR: Failed to init JPEG camera.\n");
    }
    
    // 5. 拍完後立刻關閉相機，準備進入省電睡眠
    esp_camera_deinit(); 
}

// ---------------------------------------------------------
// 低功耗動態偵測模組：用最小的資源判斷畫面有無改變
// ---------------------------------------------------------
bool checkMotion() {
    // 1. 配置為 QQVGA (160x120) 極低畫質、灰階模式
    // 灰階圖沒有色彩資訊，檔案極小，比對速度極快，對 CPU 壓力最小
    camera_config_t config;
    setup_camera_pins(config);
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAMESIZE_QQVGA;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_PSRAM;

    bool motion_detected = false;

    if (esp_camera_init(&config) == ESP_OK) {
        camera_fb_t * fb = esp_camera_fb_get();
        if (fb && fb->len == width * height) {
            
            if (is_first_boot) {
                // 如果是系統第一次開機，直接把這張畫面存為「背景基準圖」，不做比對
                printf("First boot: Saving background image.\n");
                memcpy(prev_frame, fb->buf, fb->len);
            } else {
                // 如果不是第一次，就拿出記憶體中的舊圖 (prev_frame) 跟現在的新圖 (fb->buf) 比較
                int changed_pixels = 0;
                
                // 演算法優化：每次跳 3 個像素檢查 (i += 3)，犧牲微小精度換取 3 倍的比對速度
                for (int i = 0; i < fb->len; i += 3) {
                    // 計算亮度差異，大於 threshold 則判定該區域有動靜
                    if (abs((int)fb->buf[i] - (int)prev_frame[i]) > diff_threshold) {
                        changed_pixels++;
                    }
                }
                
                // 計算總變動比例 (變動像素 / 總檢查像素)
                float change_rate = (float)changed_pixels / (fb->len / 3);
                if (change_rate > motion_limit) {
                    printf("MOTION DETECTED! Change rate: %.2f%%\n", change_rate * 100);
                    motion_detected = true; // 觸發警報
                }
                
                // 無論有沒有偵測到，都把現在的畫面覆蓋成新的背景，這樣可以自動適應戶外光線的緩慢變化
                memcpy(prev_frame, fb->buf, fb->len);
            }
            esp_camera_fb_return(fb);
        } else {
            printf("ERROR: Grayscale frame size mismatch.\n");
        }
        esp_camera_deinit(); // 比對結束，立刻關閉相機省電
    } else {
        printf("ERROR: Failed to init Grayscale camera.\n");
    }
    
    return motion_detected; // 回傳給主迴圈，告訴它是否需要拍高畫質照片
}

// ---------------------------------------------------------
// 程式進入點 (主迴圈)
// ---------------------------------------------------------
extern "C" void app_main(void) {
    // 關閉不必要的系統除錯訊息，保持 UART 通訊乾淨
    // 因為 ESP32 預設會一直印出 WiFi 或系統底層的 Log，這會混進我們的照片資料裡導致破圖
    esp_log_level_set("*", ESP_LOG_NONE); 

    // =========================================================
    // --- 初始化 UART (設定與 Python 溝通的極限速度與封包格式) ---
    // =========================================================
    uart_config_t uart_config = {};
    
    // 1. baud_rate (鮑率)：講話的速度
    // 代表每秒傳輸的位元數 (bits per second)。標準是 115200。
    // 我們設定為 921600 是極高速度，因為一張照片動輒幾萬 bytes，速度不夠快會傳很久。
    uart_config.baud_rate = 921600; 
    
    // 2. data_bits (資料位元)：每句話的長度
    // 設定為 8_BITS 代表每一次的傳輸會打包 8 個 bits (正好是 1 個 Byte)。
    // 這是目前全世界最通用的標準，確保我們的照片二進位檔案能被原封不動地傳送。
    uart_config.data_bits = UART_DATA_8_BITS;
    
    // 3. parity (同位元檢查)：防呆機制
    // 可以設定為奇數或偶數檢查，用來確認這 8 bits 在傳輸過程中有沒有因為電磁干擾而算錯。
    // DISABLE (關閉) 是因為我們追求極致速度，且短距離傳輸出錯率低，省下檢查的負擔。
    uart_config.parity = UART_PARITY_DISABLE;
    
    // 4. stop_bits (停止位元)：句號
    // 告訴接收端 (電腦 Python)：「我這一個 Byte 傳完了」。
    // 設定為 1 個停止位元 (STOP_BITS_1) 是最有效率的做法；若是傳輸環境極度惡劣才會設為 2。
    uart_config.stop_bits = UART_STOP_BITS_1;
    
    // 5. flow_ctrl (硬體流量控制)：紅綠燈機制
    // 如果開啟，需要多接兩條實體線 (RTS/CTS) 來告訴對方「我處理不來了，先別傳」。
    // 我們的 ESP32-CAM 只有 TX 跟 RX 兩條線，所以必須設定為 DISABLE。
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    
    // 6. source_clk (時鐘來源)：
    // 告訴硬體要依據哪個內部時鐘來產生 921600 的頻率，DEFAULT 會自動選擇最穩定的系統頻率。
    uart_config.source_clk = UART_SCLK_DEFAULT;
    
    // --- 安裝與套用 UART 設定 ---
    // 安裝驅動程式。參數依序為：
    // (Port編號, RX緩衝區大小, TX緩衝區大小, EventQueue大小, EventQueue指標, 進入中斷的標誌)
    // 這裡我們給 RX (接收) 分配了 4096 bytes 的記憶體，避免電腦傳指令過來時漏接。
    uart_driver_install(UART_NUM_0, 4096, 0, 0, NULL, 0);
    // 正式將上面那一長串的 config 設定套用到 UART_NUM_0 (ESP32-CAM 預設的序列埠)
    uart_param_config(UART_NUM_0, &uart_config);

#if USE_REAL_SD_CARD
    // --- 實體 SD 卡初始化 ---
    printf("Initializing SD Card...\n");
    // SD_MMC 是一種比一般 SPI 模式更高速的記憶卡存取協定，是 ESP32-CAM 的預設硬體接法
    if (!SD_MMC.begin()) {
        printf("❌ ERROR: SD Card Mount Failed. Please check the card.\n");
    } else {
        printf("✅ SD Card Mounted successfully.\n");
    }
#endif

    // 為背景參考圖分配一塊專屬的 SRAM 記憶體空間 (160 * 120 bytes)
    if (prev_frame == NULL) {
        prev_frame = (uint8_t *)malloc(width * height);
    }

    // 剛上電的第一次，強制執行一次比對函數
    // 目的是：相機剛開機會抓到第一張圖，將它存入 prev_frame 成為「初始的乾淨背景」
    if (is_first_boot) {
        checkMotion();
        is_first_boot = false;
    }

    // =========================================================
    // --- 系統的生命週期迴圈 (不斷的醒來與睡覺) ---
    // =========================================================
    while (true) {
        printf("Waking up. Checking motion...\n");
        
        // 1. 執行低耗電偵測，若畫面像素差異過大，會回傳 true (有動靜)
        if (checkMotion()) {
            // 2. 啟動高耗電模式拍攝正式照片，並存入 SD 卡或傳給電腦
            takeHighResPhoto();
        }

        // 3. 準備睡覺
        printf("Going to Light Sleep for %d seconds...\n", WAKEUP_INTERVAL_SEC);
        
        // 【關鍵延遲】給 UART 100 毫秒的時間
        // 因為 printf 只是把字塞進 TX 緩衝區，硬體還需要時間把字透過傳輸線發出去。
        // 如果不加這 100 毫秒直接睡著，電腦上的字會被切斷或是變成亂碼。
        vTaskDelay(pdMS_TO_TICKS(100)); 
        
        // 4. 設定鬧鐘 
        // ESP32 的休眠計時器單位是「微秒 (Microseconds)」，所以秒數要乘以 100 萬 (1000000ULL)
        esp_sleep_enable_timer_wakeup(WAKEUP_INTERVAL_SEC * 1000000ULL);
        
        // 5. 正式進入輕度睡眠 (Light Sleep)
        // 此時 CPU 時脈會暫停，相機模組也會進入低功耗，整個系統的耗電量會大幅下降。
        // 當鬧鐘響起時，ESP32 不會重開機，而是直接從「這一行」醒過來，繼續往下跑，回到 while 迴圈的最上方。
        esp_light_sleep_start(); 
    }
}