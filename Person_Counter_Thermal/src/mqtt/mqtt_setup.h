#ifndef MQTT_SETUP_H
#define MQTT_SETUP_H

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>

extern PubSubClient client;
extern String mqtt_username;
extern String mqtt_password;
extern float bgOffset;
extern float prevTemp;
extern int numOfPeople;
extern char timeStr[30];
extern String bootReason;

void reconnect();
void sendDiscoveryPayloads();
void publishData(String topic, float value, bool retain = false);
void publishData(String topic, String value, bool retain = false);
void mqttCallback(char* topic, byte* payload, unsigned int length);

#endif // MQTT_SETUP_H