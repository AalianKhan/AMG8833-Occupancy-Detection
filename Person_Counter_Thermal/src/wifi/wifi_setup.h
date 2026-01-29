#ifndef WIFI_SETUP_H
#define WIFI_SETUP_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <map>
#include <vector>
#include "types.h"


extern WebServer server;
extern String ssid;
extern float filteredPixels[64];
extern std::map<int, Centroid> activeCentroids;
extern std::vector<Blob> blobs;

void setup_wifi();
void startAPMode();
void setupServer();
void setupOTA();
void appendToSerialLog(const String& message);

#endif // WIFI_SETUP_H