#include "esp_camera.h"
#include <WiFi.h>
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
#include <ESP32Servo.h>
#include <freertos/FreeRTOS.h>

const char *ssid = "erichome";
const char *password = "20031230";


// Define motor control pins
#define motor1Pin 13  // Motor 1 control pin (GPIO)
#define motor2Pin 14  // Motor 2 control pin (GPIO)

// Servo objects for each motor
Servo motor1;
Servo motor2;

// Control signal variables
volatile int controlSignalMotor1 = 0;  // 0 or 1 for motor 1
volatile int controlSignalMotor2 = 0;  // 0 or 1 for motor 2



// Pre-define client and server details
WiFiClient client;
const uint16_t port = 4010;
//const IPAddress serverIP("172.20.10.3");
const IPAddress serverIP("192.168.2.64");

// Task to control motor 1
void controlMotor1Task(void *pvParameters) {
  while (1) {
    if(controlSignalMotor1 == 1) {
      // Sweep the servo back and forth
      for (int pos = 0; pos <= 180; pos++) {
        motor1.write(pos);  // Move the servo to the specified position
        delay(15);           // Wait for the servo to reach the position
      }
      
      for (int pos = 180; pos >= 0; pos--) {
        motor1.write(pos);  // Move the servo to the specified position
        delay(15);           // Wait for the servo to reach the position
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);  // Delay for 50ms to manage CPU load
  }
}

// Task to control motor 2
void controlMotor2Task(void *pvParameters) {
  while (1) {
    if(controlSignalMotor2 == 1) {
      // Sweep the servo back and forth
      for (int pos = 0; pos <= 180; pos++) {
        motor2.write(pos);  // Move the servo to the specified position
        delay(15);           // Wait for the servo to reach the position
      }
      
      for (int pos = 180; pos >= 0; pos--) {
        motor2.write(pos);  // Move the servo to the specified position
        delay(15);           // Wait for the servo to reach the position
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);  // Delay for 50ms to manage CPU load
  }
}



void handleButton() {
  // Add your button task here
  Serial.println("Button pressed!");
  // Example: Take a high-res photo
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_UXGA);
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) {
    // Do something with the high-res image
    esp_camera_fb_return(fb);
  }
  // Return to streaming resolution
  s->set_framesize(s, FRAMESIZE_VGA);
}

void setup() {
  Serial.begin(115200);

  // Attach motors to the respective pins
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);

  // Start both tasks
  xTaskCreate(controlMotor1Task, "Control Motor 1", 1000, NULL, 1, NULL);
  xTaskCreate(controlMotor2Task, "Control Motor 2", 1000, NULL, 1, NULL);


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
  config.fb_count = 2;       // Double buffering

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
  s->set_quality(s, 10);    // Increase compression
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

  uint32_t currentTime = millis();


  if ((currentTime - lastFrameTime) < FRAME_INTERVAL) {
    return;
  }

  if (!client.connected()) {
    client.connect(serverIP, port);
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) {
    if (client.connected()) {
      // Send frame in chunks
      //const size_t chunkSize = 4096;
      const size_t chunkSize = 512;
      size_t remaining = fb->len;
      uint8_t *ptr = fb->buf;

      while (remaining > 0) {
        size_t toWrite = (remaining < chunkSize) ? remaining : chunkSize;
        size_t written = client.write(ptr, toWrite);
        //Serial.println("Sending written...");
        //Serial.println(written);
        //Serial.println("toWrite:");
        //Serial.println(toWrite);
        if (written > 0) {
          ptr += written;
          remaining -= written;
        } else {
          break;
        }
      }

      // Send END_OF_FRAME marker for server to use
      client.write((const uint8_t *)"END_OF_FRAME", 12);
      Serial.println("Sending END_OF_FRAME...");

      // Non-blocking read with timeout
      uint8_t recv_buf = -1;

      if (client.available()) {
        //int read_ret = client.read(&recv_buf, 1);
        byte incomingByte = client.read();
        Serial.println("incomingByte:");
        Serial.println(incomingByte);
        int incomingInt = (int)incomingByte;
        Serial.println("incomingInt:");
        Serial.println(incomingInt);
        ///Serial.println("recv_buf:");
        //Serial.println(recv_buf);


        // Process control signals
        if (incomingInt == 2) {
          controlSignalMotor1 = 1;
          controlSignalMotor2 = 0;
        } else if (incomingInt == 1) {
          controlSignalMotor1 = 0;
          controlSignalMotor2 = 1;
        } else if (incomingInt == 3) {
          controlSignalMotor1 = 1;
          controlSignalMotor2 = 1;
        } else {
          controlSignalMotor1 = 0;
          controlSignalMotor2 = 0;
        }
      }

      // Simulate control signal (you can replace this with real control logic)
      // Example: Toggle the control signals every second
      //controlSignalMotor1 = (controlSignalMotor1 == 1) ? 0 : 1;
      //controlSignalMotor2 = (controlSignalMotor2 == 1) ? 0 : 1;

      // Debugging the control signals
      Serial.print("Motor 1 Control Signal: ");
      Serial.println(controlSignalMotor1);
      Serial.print("Motor 2 Control Signal: ");
      Serial.println(controlSignalMotor2);

      //delay(1000);  // Simulate change in control signal every 1 second
    }
    esp_camera_fb_return(fb);
  }

  lastFrameTime = millis();
}