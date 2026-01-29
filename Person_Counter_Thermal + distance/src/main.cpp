#include <Adafruit_AMG88xx.h>
#include <VL53L8CX.h>
#include <float.h>
#include <cstring>

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
const float thresholdStep = 0.25f;
const float minDistBetweenBlob = 3.0f;
const int maxArea = 4;
const int inactiveThreshold = -5;

// Add these constants after the existing constants
const float MIN_HUMAN_DISTANCE = 100.0f;   // 0.1m - minimum distance for human detection
const float MAX_HUMAN_DISTANCE = 900.0f;  // 0.9m - maximum distance for human detection
const float DISTANCE_THRESHOLD_REDUCTION = 0.4f; // Reduce thermal threshold by 40% when distance object detected
const float MIN_DISTANCE_CONFIDENCE = 200.0f; // Minimum distance reading reliability (mm)

// Global variables
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_AMG88xx amg;
VL53L8CX sensor_vl53l8cx_top(&Wire1, 0);
float filteredPixels[64] = {0};
vector<Blob> blobs;
std::map<int, Centroid> activeCentroids, prevActiveCentroids;
int numOfPeople, prevNumOfPeople = 0;
IndexBank indexBank(10);
char timeStr[30];
float prevTemp = 0.0f;
float prevPixels[64] = {0};
String bootReason;

// Global working arrays to avoid stack allocation
float distancePixels[64];
float alignedDistancePixels[64]; // Aligned distance data to match thermal FOV
float pixels[64];
float object[64];

// Function declarations
void i2cScanner();
void personCounter(VL53L8CX_ResultsData *Result);
void alignDistanceToThermal(float* distanceData, float* alignedDistance);

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
  Wire1.begin(19, 18);
  i2cScanner();
  amg.begin();
  sensor_vl53l8cx_top.begin();
  sensor_vl53l8cx_top.init();
  sensor_vl53l8cx_top.set_resolution(VL53L8CX_RESOLUTION_8X8);
  sensor_vl53l8cx_top.set_ranging_frequency_hz(10);
  sensor_vl53l8cx_top.start_ranging();

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
  static unsigned long lastProcessTime = 0;
  const unsigned long processInterval = 100; // 100ms = 10Hz
  
  server.handleClient();

  if (WiFi.status() != WL_CONNECTED) {
    server.handleClient();
  } else {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    
    // Check if it's time to process sensors (10Hz timing)
    unsigned long currentTime = millis();
    if (currentTime - lastProcessTime >= processInterval) {
      lastProcessTime = currentTime;
      
      VL53L8CX_ResultsData distanceData;
      uint8_t NewDataReady = 0;

      do {
        delay(10);
        sensor_vl53l8cx_top.check_data_ready(&NewDataReady);
      } while (!NewDataReady);

      if (NewDataReady != 0) {
        sensor_vl53l8cx_top.get_ranging_data(&distanceData);
        personCounter(&distanceData);
      }
    }
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
    for (byte address = 1; address < 127; ++address) {
        Wire1.beginTransmission(address);
        if (Wire1.endTransmission() == 0) {
            Serial.printf("Found I2C device at 0x%02X\n", address);
        }
    }
    Serial.println("Scan complete.");
} 

// Function to align VL53L8CX distance data (45° FOV) to AMG8833 thermal data (60° FOV)
void alignDistanceToThermal(float* distanceData, float* alignedDistance) {
  // Initialize aligned distance array
  for (int i = 0; i < 64; i++) {
    alignedDistance[i] = 0.0f;
  }
  
  // Scale factor: 45°/60° = 0.75
  // This means the distance data occupies 75% of the thermal data's FOV
  float scale = 0.75f;
  
  // Calculate the offset to center the distance data within the thermal FOV
  float baseOffset = (8.0f * (1.0f - scale)) / 2.0f; // Center the scaled image
  
  // Shift adjustment - positive values shift right, negative values shift left
  float horizontalShift = 0.0f; // Shift 1 pixel to the right
  float verticalShift = -1.0f;    // Shift 1 pixel up

  // Apply shifts to the offsets
  float xOffset = baseOffset + horizontalShift;
  float yOffset = baseOffset + verticalShift;
  
  // Bilinear interpolation to map distance pixels to thermal grid
  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 8; x++) {
      // Map thermal pixel coordinates to distance sensor coordinates
      float src_x = (x - xOffset) / scale;
      float src_y = (y - yOffset) / scale;
      
      // Check if the mapped coordinates are within the distance sensor bounds
      if (src_x >= 0 && src_x < 7 && src_y >= 0 && src_y < 7) {
        // Get the four surrounding pixels in the distance data
        int x0 = (int)src_x;
        int y0 = (int)src_y;
        int x1 = min(x0 + 1, 7);
        int y1 = min(y0 + 1, 7);
        
        // Calculate interpolation weights
        float wx = src_x - x0;
        float wy = src_y - y0;
        
        // Get the four corner values
        float val00 = distanceData[y0 * 8 + x0];
        float val01 = distanceData[y0 * 8 + x1];
        float val10 = distanceData[y1 * 8 + x0];
        float val11 = distanceData[y1 * 8 + x1];
        
        // Bilinear interpolation
        float val0 = val00 * (1 - wx) + val01 * wx;
        float val1 = val10 * (1 - wx) + val11 * wx;
        alignedDistance[y * 8 + x] = val0 * (1 - wy) + val1 * wy;
      }
      // Pixels outside the distance sensor FOV remain 0
    }
  }
} 

// Function to detect objects in distance data that could be humans
bool detectDistanceObjects(float* alignedDistance, bool* distanceObjectMask) {
    bool objectDetected = false;
    
    // Initialize mask
    for (int i = 0; i < 64; i++) {
        distanceObjectMask[i] = false;
    }
    
    // Detect objects within human distance range
    for (int i = 0; i < 64; i++) {
        float distance = alignedDistance[i];
        
        // Check if distance reading is valid and within human range
        if (distance > MIN_DISTANCE_CONFIDENCE && 
            distance >= MIN_HUMAN_DISTANCE && 
            distance <= MAX_HUMAN_DISTANCE &&
            distance != 0) {
            
            // Check for object consistency in neighborhood
            int validNeighbors = 0;
            int totalNeighbors = 0;
            
            // Check 3x3 neighborhood for consistency
            int row = i / 8;
            int col = i % 8;
            
            for (int dr = -1; dr <= 1; dr++) {
                for (int dc = -1; dc <= 1; dc++) {
                    int newRow = row + dr;
                    int newCol = col + dc;
                    
                    if (newRow >= 0 && newRow < 8 && newCol >= 0 && newCol < 8) {
                        int neighborIdx = newRow * 8 + newCol;
                        totalNeighbors++;
                        
                        float neighborDist = alignedDistance[neighborIdx];
                        if (neighborDist > MIN_DISTANCE_CONFIDENCE) {
                            // Check if neighbor distance is similar (within 30cm)
                            if (abs(neighborDist - distance) < 300.0f) {
                                validNeighbors++;
                            }
                        }
                    }
                }
            }
            
            // If at least 40% of neighbors are consistent, mark as object
            if (validNeighbors >= totalNeighbors * 0.4f) {
                distanceObjectMask[i] = true;
                objectDetected = true;
                
                // Also mark immediate neighbors to create object regions
                for (int dr = -1; dr <= 1; dr++) {
                    for (int dc = -1; dc <= 1; dc++) {
                        int newRow = row + dr;
                        int newCol = col + dc;
                        
                        if (newRow >= 0 && newRow < 8 && newCol >= 0 && newCol < 8) {
                            int neighborIdx = newRow * 8 + newCol;
                            float neighborDist = alignedDistance[neighborIdx];
                            
                            if (neighborDist > MIN_DISTANCE_CONFIDENCE && 
                                neighborDist >= MIN_HUMAN_DISTANCE && 
                                neighborDist <= MAX_HUMAN_DISTANCE &&
                                abs(neighborDist - distance) < 400.0f) {
                                distanceObjectMask[neighborIdx] = true;
                            }
                        }
                    }
                }
            }
        }
    }
    
    return objectDetected;
}

// Enhanced object extraction with distance assistance
void enhancedObjectExtraction(float* filteredPixels, float* alignedDistance, float* object, 
                            float avgTemp, float maxTemp, bool* distanceObjectMask) {
    // Clear the object array
    memset(object, 0, sizeof(float) * 64);
    
    // Calculate base dynamic threshold
    float baseDynamicThreshold = avgTemp + (maxTemp - avgTemp) * 0.5f;
    
    for (int i = 0; i < 64; i++) {
        float maxNeighborDiff = 0.0f;
        float currentThreshold = spikeThreshold;
        
        // If distance sensor detects an object at this location, reduce threshold
        if (distanceObjectMask[i]) {
            currentThreshold *= (1.0f - DISTANCE_THRESHOLD_REDUCTION);
            //Serial.printf("Distance object detected at pixel %d, reducing threshold to %.2f\n", i, currentThreshold);
        }
        
        // Check neighbors for edge detection
        if (i >= 8) maxNeighborDiff = max(maxNeighborDiff, abs(filteredPixels[i] - filteredPixels[i - 8])); // Up
        if (i < 56) maxNeighborDiff = max(maxNeighborDiff, abs(filteredPixels[i] - filteredPixels[i + 8])); // Down
        if (i % 8 != 0) maxNeighborDiff = max(maxNeighborDiff, abs(filteredPixels[i] - filteredPixels[i - 1])); // Left
        if (i % 8 != 7) maxNeighborDiff = max(maxNeighborDiff, abs(filteredPixels[i] - filteredPixels[i + 1])); // Right
        
        // Apply adaptive threshold
        if (maxNeighborDiff > currentThreshold) {
            object[i] = filteredPixels[i];
        }
        
        // Additional check: if distance object is present but thermal is weak, 
        // still include if temperature is above a minimum baseline
        else if (distanceObjectMask[i] && filteredPixels[i] > (avgTemp + 0.5f)) {
            object[i] = filteredPixels[i];
            // Serial.printf("Distance-assisted detection at pixel %d, temp: %.2f\n", i, filteredPixels[i]);
        }
    }
}

// Enhanced blob validation using distance data
vector<Blob> validateBlobsWithDistance(vector<Blob>& blobs, float* filteredPixels, 
                                      float* alignedDistance, bool* distanceObjectMask, 
                                      float dynamicBlobTempThreshold) {
    vector<Blob> validBlobs;
    
    for (auto& blob : blobs) {
        bool isValid = false;
        int centroidIndex = int(blob.y) * 8 + int(blob.x);
        float blobTemp = filteredPixels[centroidIndex];
        
        // Standard thermal validation
        if (blobTemp >= dynamicBlobTempThreshold) {
            isValid = true;
        }
        // Distance-assisted validation for weaker thermal signals
        else if (distanceObjectMask[centroidIndex]) {
            // Lower threshold for distance-confirmed objects
            float reducedThreshold = dynamicBlobTempThreshold * 0.7f; // 30% reduction
            if (blobTemp >= reducedThreshold) {
                isValid = true;
                Serial.printf("Distance-validated blob at (%.1f, %.1f) with temp %.2f\n", 
                            blob.x, blob.y, blobTemp);
            }
        }
        
        if (isValid) {
            validBlobs.push_back(blob);
        }
    }
    
    return validBlobs;
}

// Modified Person Counter Logic
void personCounter(VL53L8CX_ResultsData* distanceData) {
  // Use global arrays instead of local stack allocation
  for (int i = 0; i < 64; i++) {
    distancePixels[i] = distanceData->distance_mm[i];
  }
  
  // Align distance data to match thermal sensor FOV
  alignDistanceToThermal(distancePixels, alignedDistancePixels);

  // Detect distance objects that could be humans
  bool distanceObjectMask[64];
  bool distanceObjectsDetected = detectDistanceObjects(alignedDistancePixels, distanceObjectMask);

  float temp = amg.readThermistor();

  if (abs(temp - prevTemp) >= 0.5f) {
    publishData(TEMP_TOPIC, temp, false);
    prevTemp = temp;
  }

  amg.readPixels(pixels); 

  // Simple exponential smoothing, with different reactivity based on significant change for better step detection
  bool significantChange = false;
  for (int i = 0; i < 64; i++) {
    if (abs(pixels[i] - prevPixels[i]) > 1.50f) {
      significantChange = true;
      break;
    }
  }

  float reactivity = significantChange ? 0.1f : 0.75f;
  for (int i = 0; i < 64; i++) {
    filteredPixels[i] = pixels[i] * (1 - reactivity) + prevPixels[i] * reactivity;
  }

  // Calculate frame statistics
  float totalTemp = 0.0f, maxTemp = -FLT_MAX, minTemp = FLT_MAX;
  for (int i = 0; i < 64; i++) {
    totalTemp += pixels[i];
    if (filteredPixels[i] > maxTemp) maxTemp = filteredPixels[i];
    if (filteredPixels[i] < minTemp) minTemp = filteredPixels[i];
  }
  float avgTemp = totalTemp / 64.0f;

  // Dynamically calculate the minimum blob temperature threshold
  float dynamicBlobTempThreshold = avgTemp + (maxTemp - avgTemp) * 0.5f; // 50% closer to maxTemp
  
  // Enhanced object extraction using distance assistance
  enhancedObjectExtraction(filteredPixels, alignedDistancePixels, object, avgTemp, maxTemp, distanceObjectMask);

  fillSurroundedPixels(object, filteredPixels);

  // Simple blob detection
  blobs = simpleBlobDetector(object);
  
  // Enhanced blob validation using distance data
  vector<Blob> validBlobs = validateBlobsWithDistance(blobs, filteredPixels, alignedDistancePixels, 
                                                     distanceObjectMask, dynamicBlobTempThreshold);

  // Replace blobs with the validated blobs
  blobs = validBlobs;

  vector<Blob> blobsCache = blobs; 

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
        && prevActiveCentroids[id].y < 3
        && centroid.y >= 3) {
      numOfPeople++;
      Serial.print("Number of people is: ");
      Serial.println(numOfPeople);
    }

    if (prevActiveCentroids.count(id) > 0 && numOfPeople > 0
        && prevActiveCentroids[id].y >= 3 && centroid.y < 3) {
      numOfPeople--;
      Serial.print("Number of people is: ");
      Serial.println(numOfPeople);
    }
  }

  if (numOfPeople != prevNumOfPeople) {
    publishData(PEOPLE_TOPIC, numOfPeople, false);
    prevNumOfPeople = numOfPeople;
  }

  for (int i = 0; i < 64; i++) {
      prevPixels[i] = filteredPixels[i];
  }

  // Prepare JSON data to send to Node-RED
  StaticJsonDocument<4096> doc;

  // Temperature frame
  JsonArray tempFrame = doc.createNestedArray("temperatureFrame");
  for (int i = 0; i < 64; i++) {
      tempFrame.add(round(filteredPixels[i] * 100.0) / 100.0);
  }

  // Distance frame (aligned to thermal FOV)
  JsonArray distanceFrame = doc.createNestedArray("distanceFrame");
  for (int i = 0; i < 64; i++) {
      distanceFrame.add(round(alignedDistancePixels[i] * 100.0) / 100.0);
  }
  
  // // Distance object mask for debugging
  // JsonArray distanceMask = doc.createNestedArray("distanceObjectMask");
  // for (int i = 0; i < 64; i++) {
  //     distanceMask.add(distanceObjectMask[i] ? 1 : 0);
  // }

  // Object array
  JsonArray objectArray = doc.createNestedArray("object");
  for (int i = 0; i < 64; i++) {
    objectArray.add(round(object[i] * 100.0) / 100.0);
  }

  // Blobs
  JsonArray blobArray = doc.createNestedArray("blobs");
  for (auto& b : blobsCache) {
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

  // Frame statistics
  doc["dynamicBlobTempThreshold"] = round(dynamicBlobTempThreshold * 100.0) / 100.0;
  doc["significantChange"] = significantChange;
  doc["distanceObjectsDetected"] = distanceObjectsDetected;

  // Send the data to Node-RED via MQTT
  String jsonString;
  serializeJson(doc, jsonString);
  client.publish("esp32/data", jsonString.c_str());
}