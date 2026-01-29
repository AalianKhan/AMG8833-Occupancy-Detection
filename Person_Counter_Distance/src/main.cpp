#include <Adafruit_AMG88xx.h>
#include <float.h>
#include <vl53l8cx.h>

#include "config.h"
#include "types.h"
#include "img_proc/img_proc.h"
#include "eeprom/eeprom_utils.h"
#include "mqtt/mqtt_setup.h"
#include "wifi/wifi_setup.h"

using namespace std;

// WiFi and MQTT Config
String ssid = "my_ssid";
String password = "my_password";
String mqtt_server = "my_mqtt_server";
String mqtt_username = "my_mqtt_username";
String mqtt_password = "my_mqtt_password";

// Constants for background extraction, blob detection, and tracking
float spikeThreshold = 1.75f;
const float thresholdStep = 50.0f;
const float minDistBetweenBlob = 4.0f;
const int maxArea = 9;
const int inactiveThreshold = -5;

// Global variables
WiFiClient espClient;
PubSubClient client(espClient);
float filteredPixels[64] = {0};
vector<Blob> blobs;
std::map<int, Centroid> activeCentroids, prevActiveCentroids;
int numOfPeople, prevNumOfPeople = 0;
IndexBank indexBank(10);
char timeStr[30];
float prevTemp = 0.0f;
float prevPixels[64] = {0};
String bootReason;

// Function declarations
void i2cScanner();
void personCounter(VL53L8CX_ResultsData *Result);

// Sensor configuration
#define LPN_PIN 0
#define DEV_I2C Wire

VL53L8CX sensor_vl53l8cx_top(&DEV_I2C, LPN_PIN);

bool EnableAmbient = false;
bool EnableSignal = false;
uint8_t res = VL53L8CX_RESOLUTION_8X8;
char report[256];
uint8_t status;

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  // // Clear EEPROM
  // for (int i = 0; i < EEPROM_SIZE; i++) {
  //   EEPROM.write(i, 0xFF);  // Writing 0xFF resets the memory
  // }
  // EEPROM.commit();  // Save changes
  // Serial.println("EEPROM cleared!");

  loadSettings();
  setup_wifi();
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  Serial.println("");
  client.setServer(mqtt_server.c_str(), 1883);
  client.setServer(mqtt_server.c_str(), 1883);
  client.setBufferSize(4096);
  client.setCallback(mqttCallback);
  Wire.begin();
  i2cScanner();
  sensor_vl53l8cx_top.begin();
  status = sensor_vl53l8cx_top.init();
  status = sensor_vl53l8cx_top.set_resolution(res);
  status = sensor_vl53l8cx_top.set_ranging_frequency_hz(30);

  // Start Measurements
  status = sensor_vl53l8cx_top.start_ranging();


  if (WiFi.status() == WL_CONNECTED) {
    // Wait for time to be set, retrying up to 5 times
    for (int retryCount = 0; time(nullptr) < 8 * 3600 * 2 && retryCount < 5; retryCount++) {
      delay(500);
      Serial.print(".");
    }
    // Send boot time
    time_t now = time(nullptr);
    struct tm* timeInfo = gmtime(&now); // Get UTC time

    strftime(timeStr, sizeof(timeStr), "%Y-%m-%dT%H:%M:%SZ", timeInfo);
  }

  // Get boot reason
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_POWERON: bootReason = "Power On"; break;
    case ESP_RST_EXT: bootReason = "External Reset"; break;
    case ESP_RST_SW: bootReason = "Software Reset"; break;
    case ESP_RST_PANIC: bootReason = "Panic Reset"; break;
    case ESP_RST_INT_WDT: bootReason = "Watchdog Reset"; break;
    case ESP_RST_TASK_WDT: bootReason = "Task Watchdog Reset"; break;
    case ESP_RST_WDT: bootReason = "Other Watchdog Reset"; break;
    case ESP_RST_DEEPSLEEP: bootReason = "Deep Sleep Reset"; break;
    case ESP_RST_BROWNOUT: bootReason = "Brownout Reset"; break;
    case ESP_RST_SDIO: bootReason = "SDIO Reset"; break;
    default: bootReason = "Unknown"; break;
  }

  // Publish boot reason
  publishData(BOOT_REASON_TOPIC, bootReason, true);

  setupServer();
  Serial.println("HTTP server started at " + WiFi.localIP().toString());
}

void loop() {
  unsigned long startTime = millis(); // Start measuring loop time
  server.handleClient();

  if (WiFi.status() != WL_CONNECTED) {
    server.handleClient();
  } else {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();

    // unsigned long startTime = millis(); // Start measuring loop time

    VL53L8CX_ResultsData Results;
    uint8_t NewDataReady = 0;

    do {
      status = sensor_vl53l8cx_top.check_data_ready(&NewDataReady);
    } while (!NewDataReady);

    if ((!status) && (NewDataReady != 0)) {
      status = sensor_vl53l8cx_top.get_ranging_data(&Results);
      personCounter(&Results);
    }
    if (status) {
      Serial.print("Error: ");
      Serial.println(status);
    }

    unsigned long loopTime = millis() - startTime; // Calculate loop time
    StaticJsonDocument<128> doc;
    doc["loopTime"] = loopTime;

    String jsonString;
    serializeJson(doc, jsonString);
    client.publish("esp32/loopTime", jsonString.c_str());
  }
}

// I2C Scanner Function
void i2cScanner() {
  Serial.println("Scanning for I2C devices...");
    for (byte address = 1; address < 127; ++address) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.printf("Found I2C device at 0x%02X\n", address);
        }
    }
    Serial.println("Scan complete.");
}

// Person Counder Logic
void personCounter(VL53L8CX_ResultsData *Result) {  

  // Extract distances from Result and store them in a 64-length array
  float distances[64];
  for (int i = 0; i < 64; i++) {
    distances[i] = Result->distance_mm[i];
  }

  // // Simple exponential smoothing, with different reactivity based on significant change for better step detection
  // bool significantChange = false;
  // for (int i = 0; i < 64; i++) {
  //   if (abs(pixels[i] - prevPixels[i]) > 1.50f) {
  //     significantChange = true;
  //     break;
  //   }
  // }

  // float reactivity = significantChange ? 0.1f : 0.75f;
  // for (int i = 0; i < 64; i++) {
  //   filteredPixels[i] = pixels[i] * (1 - reactivity) + prevPixels[i] * reactivity;
  // }

  // // Calculate frame statistics
  // float totalTemp = 0.0f, maxTemp = -FLT_MAX, minTemp = FLT_MAX;
  // for (int i = 0; i < 64; i++) {
  //   totalTemp += pixels[i];
  //   if (filteredPixels[i] > maxTemp) maxTemp = filteredPixels[i];
  //   if (filteredPixels[i] < minTemp) minTemp = filteredPixels[i];
  // }
  // float avgTemp = totalTemp / 64.0f;

  // // Dynamically calculate the minimum blob temperature threshold
  // float dynamicBlobTempThreshold = avgTemp + (maxTemp - avgTemp) * 0.5f; // 50% closer to maxTemp
  
  // Seperate object from current frame
  float object[64]  = {0};

  for (int i = 0; i < 64; i++) {
    if (distances[i] > 300 && distances[i] < 700) {
      object[i] = distances[i];
    }
  }
  
  // // Thresholding and object extraction
  // for (int i = 0; i < 64; i++) {
  //   float maxNeighborDiff = 0.0f;

  //   // Check neighbors (up, down, left, right)
  //   if (i >= 8) maxNeighborDiff = max(maxNeighborDiff, abs(distances[i] - distances[i - 8])); // Up
  //   if (i < 56) maxNeighborDiff = max(maxNeighborDiff, abs(distances[i] - distances[i + 8])); // Down
  //   if (i % 8 != 0) maxNeighborDiff = max(maxNeighborDiff, abs(distances[i] - distances[i - 1])); // Left
  //   if (i % 8 != 7) maxNeighborDiff = max(maxNeighborDiff, abs(distances[i] - distances[i + 1])); // Right

  //   // Apply threshold
  //   if (maxNeighborDiff > 500) {
  //     object[i] = distances[i];
  //   }
  // }

  // for (int i = 0; i < 64; i++)
  // {
  //   if (!(object[i] > 250 && object[i] < 700)) {
  //     object[i] = 0;
  //   }
    
  // }
  

  // fillSurroundedPixels(object, distances);


  // Simple blob detection
  blobs = simpleBlobDetector(object);
  
  // // Filter blobs based on dynamic temperature threshold
  // vector<Blob> validBlobs;
  // for (auto& blob : blobs) {
  //   // Calculate the temperature at the blob's centroid
  //   int centroidIndex = int(blob.y) * 8 + int(blob.x);
  //   float blobTemp = filteredPixels[centroidIndex];

  //   // Check if the blob's temperature meets the dynamic threshold
  //   if (blobTemp >= dynamicBlobTempThreshold) {
  //       validBlobs.push_back(blob);
  //   }
  // }

  // // Replace blobs with the filtered valid blobs
  // blobs = validBlobs;

  // Centroid tracking
  prevActiveCentroids = activeCentroids;
  activeCentroids = centroidTracker(blobs, activeCentroids);
  if (!activeCentroids.empty())
  {
    Serial.print("Number of active centroids: ");
    Serial.println(activeCentroids.size());
  }

  // Calculate temperature difference for each centroid
  for (auto& [id, centroid] : activeCentroids)
  {
    centroid.tempDiff = filteredPixels[int(centroid.y) * 8 + int(centroid.x)];
  }


  // Person Counting logic
  static int prevNumOfPeople = numOfPeople;

  for (const auto& [id, centroid] : activeCentroids) {
    if (prevActiveCentroids.count(id) > 0 
        && prevActiveCentroids[id].y > 3
        && centroid.y <= 3) {
      numOfPeople++;
      Serial.print("Number of people is: ");
      Serial.println(numOfPeople);
    }

    if (prevActiveCentroids.count(id) > 0 && numOfPeople > 0
        && prevActiveCentroids[id].y < 3 && centroid.y >= 3) {
      numOfPeople--;
      Serial.print("Number of people is: ");
      Serial.println(numOfPeople);
    }
  }

  if (numOfPeople != prevNumOfPeople) {
    publishData(PEOPLE_TOPIC, numOfPeople, false);
    prevNumOfPeople = numOfPeople;
  }

  // for (int i = 0; i < 64; i++) {
  //     prevPixels[i] = filteredPixels[i];
  // }

  // Prepare JSON data to send to Node-RED
  StaticJsonDocument<4096> doc;

  // Distance frame
  JsonArray distanceFrame = doc.createNestedArray("distanceFrame");
  for (int i = 0; i < 64; i++) {
      distanceFrame.add(round(distances[i] * 100.0) / 100.0);
  }

  // Object array
  JsonArray objectArray = doc.createNestedArray("object");
  for (int i = 0; i < 64; i++) {
    objectArray.add(round(object[i] * 100.0) / 100.0);
  }

  // Blobs
  JsonArray blobArray = doc.createNestedArray("blobs");
  for (auto& b : blobs) {
      JsonObject blobObj = blobArray.createNestedObject();
      blobObj["x"] = b.x;
      blobObj["y"] = b.y;
      blobObj["radius"] = b.radius;
  }

  // Centroids
  JsonArray centroidArray = doc.createNestedArray("centroids");
  for (auto& entry : activeCentroids) {
      JsonObject centroidObj = centroidArray.createNestedObject();
      centroidObj["id"] = entry.first;
      centroidObj["x"] = entry.second.x;
      centroidObj["y"] = entry.second.y;
      centroidObj["radius"] = entry.second.radius;
      centroidObj["tempDiff"] = round(entry.second.tempDiff * 100.0) / 100.0;
  }


  // doc["dynamicBlobTempThreshold"] = round(dynamicBlobTempThreshold * 100.0) / 100.0;

  // doc["significantChange"] = significantChange;

  // Send the data to Node-RED via MQTT
  String jsonString;
  serializeJson(doc, jsonString);
  client.publish("esp32/data", jsonString.c_str());

}