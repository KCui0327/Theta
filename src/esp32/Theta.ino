#include "esp_camera.h"
#include <WiFi.h>
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

const char *ssid = "blank";
const char *password = "blank";

// Pre-define client and server details
WiFiClient client;
const uint16_t port = 4010;
//const IPAddress serverIP("172.20.10.3");
const IPAddress serverIP("192.168.2.64");

void setup() {
  Serial.begin(115200);
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  
  // Optimize frame size and quality for speed
  config.frame_size = FRAMESIZE_VGA;  // Reduced from UXGA
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;  // Increased compression
  config.fb_count = 2;  // Double buffering

  if (psramFound()) {
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.fb_count = 1;
  }

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Optimize sensor settings
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);
  s->set_quality(s, 10);  // Increase compression
  s->set_brightness(s, 0);  // Reduce processing
  s->set_saturation(s, 0);
  s->set_contrast(s, 0);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);  // Disable WiFi power saving
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  // Initial connection to server
  client.connect(serverIP, port);
}

void loop() {
  static uint32_t lastFrameTime = 0;
  const uint32_t FRAME_INTERVAL = 33;
  
  if ((millis() - lastFrameTime) < FRAME_INTERVAL) {
    return;
  }

  if (!client.connected()) {
    client.connect(serverIP, port);
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (fb) {
    if (client.connected()) {
      // Send frame in chunks
      //const size_t chunkSize = 4096;
      const size_t chunkSize = 512;
      size_t remaining = fb->len;
      uint8_t* ptr = fb->buf;
      
      while (remaining > 0) {
        size_t toWrite = (remaining < chunkSize) ? remaining : chunkSize;
        size_t written = client.write(ptr, toWrite);
        if (written > 0) {
          ptr += written;
          remaining -= written;
        } 
        else {
          break;
        }
      }
      
      // Send END_OF_FRAME marker for server to use
      client.write((const uint8_t*)"END_OF_FRAME", 12);
    }
    esp_camera_fb_return(fb);
  }
  
  lastFrameTime = millis();
}