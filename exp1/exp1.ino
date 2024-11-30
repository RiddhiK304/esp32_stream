#include "esp_camera.h"
#include <WiFi.h>

// Replace with your network credentials
const char* ssid = "GetOutOfHere!!";        // Wi-Fi SSID
const char* password = "hotspotrrk"; // Wi-Fi Password

// Camera configuration
camera_config_t config;

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  
  // Set up camera configuration
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 13; // GPIO 13
  config.pin_d1 = 12; // GPIO 12
  config.pin_d2 = 14; // GPIO 14
  config.pin_d3 = 15; // GPIO 15
  config.pin_d4 = 2;  // GPIO 2
  config.pin_d5 = 4;  // GPIO 4
  config.pin_d6 = 5;  // GPIO 5
  config.pin_d7 = 18; // GPIO 18
  config.pin_xclk = 0; // GPIO 0
  config.pin_pclk = 22; // GPIO 22
  config.pin_vsync = 25; // GPIO 25
  config.pin_href = 23; // GPIO 23
  config.pin_sccb_sda = 26; // GPIO 26
  config.pin_sccb_scl = 27; // GPIO 27
  config.pin_reset = -1; // No reset pin
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Init with high specs
  if (psramFound()) {
    config.frame_size = FRAMESIZE_SVGA; // 800x600
    config.jpeg_quality = 12; // JPEG quality
    config.fb_count = 2; // Use two frames to reduce jitter
  } else {
    config.frame_size = FRAMESIZE_VGA; // 640x480
    config.jpeg_quality = 12; // JPEG quality
    config.fb_count = 1; // Use one frame
  }

  // Initialize the camera
  camera_index_init(&config);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("Connected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Put your main code here, to run repeatedly
}

// Function to initialize camera
void camera_index_init(camera_config_t *config) {
  esp_err_t err = esp_camera_init(config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}