#include <Wire.h>
#include <LiquidCrystal_I2C.h> // Ensure you use an ESP32-compatible version
#include <driver/ledc.h>       // Include LEDC library for PWM control on ESP32

// I2C address 0x27 may vary depending on the module
LiquidCrystal_I2C lcd(0x27, 16, 2); // For 16x2 LCD

// Fan and LED control for ESP32
#define LED_PIN 2            // Pin for LED control
#define FAN_PIN 5            // Pin for Fan control
#define FAN_SPEED_PIN 18     // Pin for Fan speed control (PWM)

// PWM Configuration
const int pwmChannel = 0;
const int pwmFreq = 5000; // 5 kHz frequency
const int pwmResolution = 8; // 8-bit resolution

void setup() {
  Serial.begin(115200);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Ready");

  // Setup LED and Fan control
  pinMode(LED_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  
  // Setup PWM for fan speed control
  ledcSetup(pwmChannel, pwmFreq, pwmResolution); // Configure PWM
  ledcAttachPin(FAN_SPEED_PIN, pwmChannel); // Attach pin to PWM channel
}

void loop() {
  // Example: Toggle LED every 2 seconds
  digitalWrite(LED_PIN, HIGH); 
  delay(2000);
  digitalWrite(LED_PIN, LOW); 
  delay(2000);

  // Example: Change fan speed
  int fanSpeed = 128; // Set fan speed (0-255 range for 8-bit PWM)
  ledcWrite(pwmChannel, fanSpeed); // Adjust fan speed with PWM
}
