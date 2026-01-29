#include "eeprom_utils.h"
#include "config.h"

void saveSettings(const String& ssid, const String& password, const String& mqtt_server, const String& mqtt_username, const String& mqtt_password) {
    EEPROM.writeString(SSID_ADDR, ssid);
    EEPROM.writeString(PASSWORD_ADDR, password);
    EEPROM.writeString(MQTT_SERVER_ADDR, mqtt_server);
    EEPROM.writeString(MQTT_USERNAME_ADDR, mqtt_username);
    EEPROM.writeString(MQTT_PASSWORD_ADDR, mqtt_password);
    EEPROM.commit();
}

void loadSettings() {
    String new_ssid = EEPROM.readString(SSID_ADDR);
    String new_password = EEPROM.readString(PASSWORD_ADDR);
    String new_mqtt_server = EEPROM.readString(MQTT_SERVER_ADDR);
    String new_mqtt_username = EEPROM.readString(MQTT_USERNAME_ADDR);
    String new_mqtt_password = EEPROM.readString(MQTT_PASSWORD_ADDR);

    if (new_ssid.length() > 0) {
        ssid = new_ssid.c_str();
      }
      if (new_password.length() > 0) {
        password = new_password.c_str();
      }
      if (new_mqtt_server.length() > 0) {
        mqtt_server = new_mqtt_server.c_str();
      }
      if (new_mqtt_username.length() > 0) {
        mqtt_username = new_mqtt_username.c_str();
      }
      if (new_mqtt_password.length() > 0) {
        mqtt_password = new_mqtt_password.c_str();
      }
}