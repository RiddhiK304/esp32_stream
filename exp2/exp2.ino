#include "esp_camera.h"
#include <WiFi.h>

// Select camera model
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#include "camera_pins.h"

// WiFi credentials
const char *ssid = "GetOutOfHere!!";
const char *password = "hotspotrrk";

// Pin Definitions
#define LED_PIN 4          // Using the flash LED (GPIO 4)
#define FAN_PIN 14         // Pin for Fan control (GPIO 14)
#define FAN_SPEED_PIN 15   // PWM control for Fan speed (GPIO 15)

// Variables for tracking blink and fan speed
int blinkCount = 0;
bool ledState = false;
bool fanState = false;
int fanSpeed = 0;  // 0-255 range for PWM speed control

void startCameraServer();

// Function to control the LED state
void toggleLED() {
  ledState = !ledState;
  digitalWrite(LED_PIN, ledState ? LOW : HIGH); // Inverted logic for flash LED
  
  if (ledState) {
    Serial.println("LED turned ON");
  } else {
    Serial.println("LED turned OFF");
  }
}

// Function to adjust fan speed
void adjustFanSpeed(int adjustment) {
  fanSpeed += adjustment * 25; // Increase or decrease fan speed by 25 (scaled for PWM)
  
  if (fanSpeed > 255) fanSpeed = 255;  // Max speed
  if (fanSpeed < 0) fanSpeed = 0;      // Min speed
  
  ledcWrite(0, fanSpeed); // Write PWM value to fan speed control pin
  
  Serial.print("Fan speed adjusted to: ");
  Serial.println(fanSpeed);
}

// Function to trigger emergency alert
void emergencyAlert() {
  fanState = false;  // Turn off the fan during an emergency
  digitalWrite(FAN_PIN, LOW);
  
  Serial.println("Emergency! Continuous blinking detected.");
  // Add additional emergency handling code here if needed
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Initialize GPIO pins
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // Turn off flash LED (inverted logic)
  
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW); // Ensure fan is off initially
  
  // Setup PWM for fan speed control
  ledcSetup(0, 5000, 8); // Channel 0, 5 kHz PWM, 8-bit resolution
  ledcAttachPin(FAN_SPEED_PIN, 0);
  ledcWrite(0, fanSpeed); // Start with fan speed at 0

  // Camera initialization
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
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // Camera init
  if (psramFound()) {
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // Initial adjustments
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // Flip it back
    s->set_brightness(s, 1);   // Increase brightness
    s->set_saturation(s, -2);  // Lower saturation
  }
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // Start camera server
  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  // Handle serial input for blink detection
  if (Serial.available() > 0) {
    String blinkMessage = Serial.readStringUntil('\n'); // Read until newline character

    // Remove any trailing carriage return or newline characters
    blinkMessage.trim();

    // Check if we got a blink message
    if (blinkMessage == "blink") {
      blinkCount++;
      Serial.println("Blink Detected");

      // Handle single blink: Toggle LED
      if (blinkCount == 1) {
        toggleLED();
        // Assuming blink lasts for 2 seconds
      }
      // Handle double blink: Increase fan speed
      else if (blinkCount == 2) {
        adjustFanSpeed(1);
        // Assuming blink lasts for 3 seconds
      }
      // Handle triple blink: Decrease fan speed
      else if (blinkCount == 3) {
        adjustFanSpeed(-1);
        // Assuming blink lasts for 3 seconds
      }
      // Handle emergency blink (4-5 blinks continuously)
      else if (blinkCount >= 4 && blinkCount <= 5) {
        emergencyAlert();
        // Assuming blink lasts for 4-5 seconds
      }

      // Reset blink count after processing
      blinkCount = 0;
    }
  }

  // Other code can be added here if needed

  delay(10); // Small delay to prevent overwhelming the CPU
}
