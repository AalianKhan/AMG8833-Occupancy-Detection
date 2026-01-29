#ifndef EEPROM_UTILS_H
#define EEPROM_UTILS_H

#include <Arduino.h>
#include <EEPROM.h>

extern String ssid;
extern String password;
extern String mqtt_server;
extern String mqtt_username;
extern String mqtt_password;

// Function declarations
void saveSettings(const String& ssid, const String& password, const String& mqtt_server, const String& mqtt_username, const String& mqtt_password);
void loadSettings();

#endif // EEPROM_UTILS_H