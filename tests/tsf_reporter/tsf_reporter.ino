#include <WiFi.h>
#include <esp_mesh.h>

// Replace with your WiFi credentials
const char* ssid = "MarkIt-FC3C";
const char* password = "yzri@1220";
// NTP server and timezone
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0; // Adjust for your timezone
const int daylightOffset_sec = 0;
void setup() {
    // Initialize Serial Monitor
    Serial.begin(115200);
    delay(1000);
    // Connect to WiFi
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi");
}
void loop() {
    // Get current time
    int64_t tsf_time = esp_mesh_get_tsf_time();
    // Print the timestamp
    char timeString[30];
    sprintf(timeString, "%d", tsf_time);
    Serial.println(timeString);
    // Wait 100ms to print 10 times per second
    delay(100);
}