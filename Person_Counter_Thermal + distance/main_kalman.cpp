#include <Adafruit_AMG88xx.h>
#include <ArduinoOTA.h>

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
String mqtt_server = "1.1.1.1";
String mqtt_username = "my_mqtt_username";
String mqtt_password = "my_mqtt_password";

// Constants for background extraction, blob detection, and tracking
float bgOffset = 1.0f;
float timeConstant = 45.0f; // seconds
const float thresholdStep = 0.5f;
const float minDistBetweenBlob = 3.0f;
const int maxArea = 4;
const int inactiveThreshold = -5;

// Global variables
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_AMG88xx amg;
vector< vector<float> > frames;
float bg[64] = {0};
std::map<int, Centroid> activeCentroids, prevActiveCentroids;
int numOfPeople = 0;
IndexBank indexBank(10);
int framesCaptured = 0;
char timeStr[30];

float prevTemp = 0.0f;
int prevNumOfPeople = 0;
time_t lastBadFrameTime = 0;
String bootReason;

// Kalman filter parameters as constants for easy tuning
float initialEstimate = 17.0;  // Initial temperature estimate
float processNoise = 3.0;    // Process noise (increase for faster adaptation)
float measurementNoise = 5.0; // Measurement noise (increase to filter out sensor noise)

// Kalman filter array for all 64 pixels
KalmanFilter kalmanFilters[64] = {
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),

    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),

    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),

    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),

    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),

    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),

    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),

    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise),
    KalmanFilter(initialEstimate, processNoise, measurementNoise)
};

// Function declarations
void i2cScanner();
void personCounter();
float movingAvg(vector<vector<float>>&, int, int, int);

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

  // Wait for time to be set
  while (time(nullptr) < 8 * 3600 * 2) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  ArduinoOTA.begin();
  client.setServer(mqtt_server.c_str(), 1883);
  client.setServer(mqtt_server.c_str(), 1883);
  client.setBufferSize(4096);
  client.setCallback(mqttCallback);
  Wire.begin();
  i2cScanner();
  amg.begin();
  frames.reserve(5);
  for (int i = 0; i < 64; i++)
  {
    bg[i] = 30.0f;
  }

  if (WiFi.status() == WL_CONNECTED) {
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
  server.handleClient();
  ArduinoOTA.handle();

  if (WiFi.status() != WL_CONNECTED) {
    server.handleClient();
  } else {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();

    static long lastScan = 0;
    if (millis() - lastScan > 100) {
        personCounter();
        lastScan = millis();
    }
  }
  // Serial.println("Looping...");
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
void personCounter() {  
  float temp = amg.readThermistor();

  if (abs(temp - prevTemp) >= 0.5f) {
    publishData(TEMP_TOPIC, temp, false);
    prevTemp = temp;
  }

  float pixels[64];
  amg.readPixels(pixels); 

  // if (frames.size() < 4)
  // {
  //   frames.push_back(
  //       {
  //           pixels[0], pixels[1], pixels[2], pixels[3], pixels[4], pixels[5], pixels[6], pixels[7],
  //           pixels[8], pixels[9], pixels[10], pixels[11], pixels[12], pixels[13], pixels[14], pixels[15], 
  //           pixels[16], pixels[17], pixels[18], pixels[19], pixels[20], pixels[21], pixels[22], pixels[23], 
  //           pixels[24], pixels[25], pixels[26], pixels[27], pixels[28], pixels[29], pixels[30], pixels[31], 
  //           pixels[32], pixels[33], pixels[34], pixels[35], pixels[36], pixels[37], pixels[38], pixels[39],
  //           pixels[40], pixels[41], pixels[42], pixels[43], pixels[44], pixels[45], pixels[46], pixels[47],
  //           pixels[48], pixels[49], pixels[50], pixels[51], pixels[52], pixels[53], pixels[54], pixels[55],
  //           pixels[56], pixels[57], pixels[58], pixels[59], pixels[60], pixels[61], pixels[62], pixels[63]
  //       }
  //   );
  //   Serial.println("Capturing frames without calcs...");
  //   return;
  // }
  
  // // Delta Filter
  // const auto& latestFrame = frames.back();
  // for (int i = 0; i < 64; i++) {
  //   if (abs(pixels[i] - latestFrame[i]) >= 15.0f) {
  //     Serial.print("Significant change detected at pixel ");
  //     Serial.print(i);
  //     Serial.print(" changed by ");
  //     Serial.println(abs(pixels[i] - latestFrame[i]));
  //     Serial.println("Discarding frame");

  //     // Update last bad frame time
  //     lastBadFrameTime = time(nullptr);
  //     struct tm* timeInfo = gmtime(&lastBadFrameTime);
  //     strftime(timeStr, sizeof(timeStr), "%Y-%m-%dT%H:%M:%SZ", timeInfo);
  //     publishData(LAST_BAD_FRAME_TOPIC, timeStr, true);

  //     return;
  //   }
  // }

  // frames.push_back(
  //     {
  //         pixels[0], pixels[1], pixels[2], pixels[3], pixels[4], pixels[5], pixels[6], pixels[7],
  //         pixels[8], pixels[9], pixels[10], pixels[11], pixels[12], pixels[13], pixels[14], pixels[15], 
  //         pixels[16], pixels[17], pixels[18], pixels[19], pixels[20], pixels[21], pixels[22], pixels[23], 
  //         pixels[24], pixels[25], pixels[26], pixels[27], pixels[28], pixels[29], pixels[30], pixels[31], 
  //         pixels[32], pixels[33], pixels[34], pixels[35], pixels[36], pixels[37], pixels[38], pixels[39],
  //         pixels[40], pixels[41], pixels[42], pixels[43], pixels[44], pixels[45], pixels[46], pixels[47],
  //         pixels[48], pixels[49], pixels[50], pixels[51], pixels[52], pixels[53], pixels[54], pixels[55],
  //         pixels[56], pixels[57], pixels[58], pixels[59], pixels[60], pixels[61], pixels[62], pixels[63]
  //     }
  // );

  // // Moving Average Filter
  // float filteredPixels[64] = {0};
  // if (frames.size() > 3) {
  //   frames.erase(frames.begin());

  //   for (int i = 0; i < 64; i++) {
  //       filteredPixels[i] = round((0.8f * movingAvg(frames, i, 1, 3) +
  //                                  0.2f * frames[frames.size() - 3][i]) * 1000.0f) / 1000.0f;
  //   }
  // }  

  float filteredPixels[64];
  for (int i = 0; i < 64; i++) {
      kalmanFilters[i] = KalmanFilter(initialEstimate, processNoise, measurementNoise);
      filteredPixels[i] = kalmanFilters[i].update(pixels[i]);
  }

  // Seperate object from current frame
  float object[64]  = {0};
  
  for (int i = 0; i < 64; i++) {
      if (filteredPixels[i] > (bg[i] + bgOffset)) {
          object[i] = filteredPixels[i];
      }
      
  }

  // Simple blob detection
  vector<Blob> blobs = simpleBlobDetector(object);

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
    centroid.tempDiff = filteredPixels[int(centroid.y)*8 + int(centroid.x)] - (bg[int(centroid.y)*8 + int(centroid.x)] + bgOffset);
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
        && prevActiveCentroids[id].y <= 3 && centroid.y > 3) {
      numOfPeople--;
      Serial.print("Number of people is: ");
      Serial.println(numOfPeople);
    }
  }

  if (numOfPeople != prevNumOfPeople) {
    publishData(PEOPLE_TOPIC, numOfPeople, false);
    prevNumOfPeople = numOfPeople;
  }

  // Update background image, if not enough frames captured yet, use aggresive simple exponential smoothing
  if (framesCaptured < 100) {
    for (int i = 0; i < 64; i++) {
      bg[i] = filteredPixels[i] * (1 - 0.95) + bg[i] * 0.95;
    }
    framesCaptured++;
  } else
  {
    float reactivity = exp(-1.0 / (10.0 * timeConstant));
    for (int i = 0; i < 64; i++) {
      bg[i] = filteredPixels[i] * (1 - reactivity) + bg[i] * reactivity;
    }
  }

  // Prepare JSON data to send to Node-RED
  StaticJsonDocument<4096> doc;

  // Temperature frame
  JsonArray tempFrame = doc.createNestedArray("temperatureFrame");
  for (int i = 0; i < 64; i++) {
      tempFrame.add(round(filteredPixels[i] * 100.0) / 100.0);
  }

  // Background frame with offset
  JsonArray bgFrame = doc.createNestedArray("backgroundFrame");
  for (int i = 0; i < 64; i++) {
      bgFrame.add(round((bg[i] + bgOffset) * 100.0) / 100.0);
  }

  // Raw Pixels
  JsonArray rawPixels = doc.createNestedArray("rawPixels");
  for (int i = 0; i < 64; i++) {
      rawPixels.add(round(pixels[i] * 100.0) / 100.0);
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

  // Send the data to Node-RED via MQTT
  String jsonString;
  serializeJson(doc, jsonString);
  client.publish("esp32/data", jsonString.c_str());

}