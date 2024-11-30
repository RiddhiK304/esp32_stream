void setup() {
  Serial.begin(115200);  // Start the Serial communication at 115200 baud rate
  while (!Serial) {
    ; // Wait for serial port to connect.
  }
  Serial.println("ESP32-CAM is ready and running!");
}

void loop() {
  Serial.println("Looping...");  // Print a message every second
  delay(1000);                    // Wait for 1 second
}