#ifndef CONFIG_H
#define CONFIG_H

#define DEVICE_NAME "esp32_person_counter"
#define SENSOR_TOPIC "homeassistant/sensor/" DEVICE_NAME
#define BUTTON_TOPIC "homeassistant/button/" DEVICE_NAME "/restart"
#define NUMBERS_TOPIC "homeassistant/number/" DEVICE_NAME

#define TEMP_TOPIC SENSOR_TOPIC "/thermistortemp"
#define PEOPLE_TOPIC SENSOR_TOPIC "/people"
#define BG_OFFSET_TOPIC NUMBERS_TOPIC "/bg_offset"
#define BG_OFFSET_CMD BG_OFFSET_TOPIC "/set"
#define BOOT_TIME_TOPIC SENSOR_TOPIC "/boot_time"
#define BOOT_REASON_TOPIC SENSOR_TOPIC "/boot_reason"
#define BUTTON_CMD_TOPIC BUTTON_TOPIC "/press"
#define IP_ADDRESS_TOPIC SENSOR_TOPIC "/ip_address"

#define EEPROM_SIZE 512
#define SSID_ADDR 0
#define PASSWORD_ADDR 100
#define MQTT_SERVER_ADDR 200
#define MQTT_USERNAME_ADDR 300
#define MQTT_PASSWORD_ADDR 400

#endif // CONFIG_H