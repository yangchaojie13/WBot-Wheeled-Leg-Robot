#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_camera.h"

// ================= 参数配置区 =================
const char* ssid = "17-6-302";
const char* password = "Qwe30268";

const char* rk3588_ip = "192.168.32.75"; // 你的 RK3588 的局域网 IP
const int udp_port = 8888;             // 接收端端口

WiFiUDP udp;

// ================= 摄像头引脚配置 (ESP32-S3 通用) =================
// ================= 摄像头引脚配置 (优信 ESP32-S3-N16R8 专属) =================
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5
#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM     13

void setup() {
  Serial.begin(115200);
  Serial.println("\n[启动] ESP32-S3 视觉中枢...");

  // 1. 连接 WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[网络] WiFi 已连接! IP: " + WiFi.localIP().toString());

  udp.begin(udp_port);

  // 2. 初始化摄像头
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  
  // 【关键修改 1】将分辨率从 VGA 降级到 QVGA (320x240)，大幅减少数据量
  config.frame_size = FRAMESIZE_VGA; 
  config.pixel_format = PIXFORMAT_JPEG;
  config.jpeg_quality = 10; 
  config.fb_count = 2; 

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[错误] 摄像头初始化失败，错误码: 0x%x\n", err);
    return;
  }
  Serial.println("[视觉] 摄像头就绪，准备开火！");
}

void loop() {
  // 1. 抓取一帧画面
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[错误] 抓取画面失败");
    return;
  }

  // 打印这帧画面的大小，让你心里有数
  Serial.printf("[发送] 抓拍成功! 照片大小: %d 字节\n", fb->len);

  // 2. 更加温柔的 UDP 分块引擎
  size_t chunk_size = 1024;
  for (size_t i = 0; i < fb->len; i += chunk_size) {
    size_t send_len = (fb->len - i < chunk_size) ? (fb->len - i) : chunk_size;
    
    udp.beginPacket(rk3588_ip, udp_port);
    // 这里我们将 PSRAM 中的数据写入 UDP 缓冲区
    udp.write(fb->buf + i, send_len);
    udp.endPacket();
    
    // 【关键修改 2】增加到 10 毫秒，强行给底层网卡足够的清空时间
    delay(10); 
  }

  // 3. 将显存交还给摄像头
  esp_camera_fb_return(fb);

  // 帧率控制：放慢到大概 3~5 帧每秒，先保证 100% 成功率
  delay(200); 
}