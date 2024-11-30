#include <LiquidCrystal_I2C.h> // For LCD display
#include <Wire.h>

// Pin Definitions for ESP32
#define LED_PIN 2            // Pin for LED control
#define FAN_PIN 5            // Pin for Fan control (PWM pin)
#define FAN_SPEED_PIN 18     // PWM control for Fan speed

// Variables for tracking blink and fan speed
int blinkCount = 0;
bool ledState = false;
bool fanState = false;
int fanSpeed = 0;  // 0-255 range for PWM speed control

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x27, 16 column and 2 row

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Set pin modes for LED and Fan
  pinMode(LED_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(FAN_SPEED_PIN, OUTPUT);

  // Initialize the LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Ready");
}

void loop() {
  // Assume the Python side will send blink detection status via serial communication
  // This will be handled in the Python script where the ESP32 will receive "blink" signals
  if (Serial.available() > 0) {
    String blinkMessage = Serial.readString();

    // Check if we got a blink message
    if (blinkMessage == "blink") {
      blinkCount++;
      Serial.println("Blink Detected");

      // Handle single blink: Turn on/off LED if blink lasts for 2 seconds
      if (blinkCount == 1) {
        toggleLED();
        delay(2000); // Simulating 2 seconds blink duration
      }

      // Handle double blink: Adjust fan speed (increase by 1)
      else if (blinkCount == 2) {
        adjustFanSpeed(1);
        delay(3000); // Simulating 3 seconds blink duration
      }

      // Handle triple blink: Adjust fan speed (decrease by 1)
      else if (blinkCount == 3) {
        adjustFanSpeed(-1);
        delay(3000); // Simulating 3 seconds blink duration
      }

      // Handle emergency blink (4-5 blinks continuously)
      else if (blinkCount >= 4 && blinkCount <= 5) {
        emergencyAlert();
        delay(3000); // Simulating 3 seconds blink duration
      }

      // Reset blink count after processing
      blinkCount = 0;
    }
  }
}

// Function to toggle the LED state
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

// Function to adjust fan speed
void adjustFanSpeed(int adjustment) {
  fanSpeed += adjustment * 25; // Increase or decrease fan speed by 25 (scaled for PWM)
  
  if (fanSpeed > 255) fanSpeed = 255;  // Max speed
  if (fanSpeed < 0) fanSpeed = 0;      // Min speed
  
  analogWrite(FAN_SPEED_PIN, fanSpeed);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Fan Speed: ");
  lcd.print(fanSpeed);
  
  Serial.print("Fan Speed Adjusted: ");
  Serial.println(fanSpeed);
}

// Function to trigger emergency alert (e.g., continuous blinking 4-5 times)
void emergencyAlert() {
  fanState = false;  // Turn off the fan during an emergency
  digitalWrite(FAN_PIN, LOW);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("EMERGENCY ALERT!");
  
  Serial.println("Emergency Alert Triggered");
}
