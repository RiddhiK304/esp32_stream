#include "esp_camera.h"
#include <WiFi.h>

// Select camera model
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#include "camera_pins.h"

#include <LiquidCrystal_I2C.h> // For LCD display
#include <Wire.h>

#include <driver/ledc.h>       // For PWM control on ESP32

// WiFi credentials
const char* ssid = "GetOutOfHere!!";
const char* password = "hotspotrrk";

// Pin Definitions
#define LED_PIN 2            // Pin for LED control
#define FAN_PIN 5            // Pin for Fan control (PWM pin)
#define FAN_SPEED_PIN 18     // PWM control for Fan speed

// Variables for tracking blink and fan speed
int blinkCount = 0;
bool ledState = false;
bool fanState = false;
int fanSpeed = 0;  // 0-255 range for PWM speed control

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x27, 16 columns and 2 rows

// Function declarations
void startCameraServer();
void setupLedFlash(int pin);
void toggleLED();
void adjustFanSpeed(int adjustment);
void emergencyAlert();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Setup LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Ready");

  // Set pin modes for LED and Fan
  pinMode(LED_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(FAN_SPEED_PIN, OUTPUT);

  // PWM setup for fan speed control
  int pwmChannel = 0;  // Use channel 0 for PWM
  int pwmFreq = 5000;  // Set frequency to 5kHz
  int pwmResolution = 8;  // Use 8-bit resolution (values from 0 to 255)

  // Initialize PWM for fan speed control
  ledcSetup(pwmChannel, pwmFreq, pwmResolution); // This is part of the driver/ledc.h for ESP32
  ledcAttachPin(FAN_SPEED_PIN, pwmChannel);

  // Camera configuration setup
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
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_UXGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (psramFound()) {
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }

  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  if (Serial.available() > 0) {
    String blinkMessage = Serial.readString();

    if (blinkMessage == "blink") {
      blinkCount++;
      Serial.println("Blink Detected");

      if (blinkCount == 1) {
        toggleLED();
        delay(2000);
      } else if (blinkCount == 2) {
        adjustFanSpeed(1);
        delay(3000);
      } else if (blinkCount == 3) {
        adjustFanSpeed(-1);
        delay(3000);
      } else if (blinkCount >= 4 && blinkCount <= 5) {
        emergencyAlert();
        delay(3000);
      }

      blinkCount = 0;
    }
  }
  delay(10000);
}

void toggleLED() {
  ledState = !ledState;
  digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  if (ledState) {
    lcd.print("LED: ON");
    Serial.println("LED Turned ON");
  } else {
    lcd.print("LED: OFF");
    Serial.println("LED Turned OFF");
  }
}

void adjustFanSpeed(int adjustment) {
  fanSpeed += adjustment * 25; // Increase or decrease fan speed by 25 (scaled for PWM)
  
  if (fanSpeed > 255) fanSpeed = 255;  // Max speed
  if (fanSpeed < 0) fanSpeed = 0;      // Min speed
  
  ledcWrite(0, fanSpeed);  // Apply the new fan speed to the PWM channel

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Fan Speed: ");
  lcd.print(fanSpeed);
  
  Serial.print("Fan Speed Adjusted: ");
  Serial.println(fanSpeed);
}

void emergencyAlert() {
  fanState = false;  // Turn off the fan during an emergency
  digitalWrite(FAN_PIN, LOW);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("EMERGENCY ALERT!");
  
  Serial.println("Emergency Alert Triggered");
}

void setupLedFlash(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}
