#include <WiFi.h>
#include <esp_camera.h>

const char* ssid = "GetOutOfHere!!";
const char* password = "hotspotrrk";

void setup() {
    Serial.begin(115200);
    Serial.println();

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to Wi-Fi...");
    }
    Serial.println("Connected to Wi-Fi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Start the camera server (uses built-in function from app_httpd.cpp)
    startCameraServer();
}

void loop() {
    // Do nothing; the server handles requests in the background
}

