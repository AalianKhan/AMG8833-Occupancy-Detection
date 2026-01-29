#include "mqtt_setup.h"
#include "config.h"

void setupMQTT(const char* mqtt_server) {
    client.setServer(mqtt_server, 1883);
    client.setBufferSize(2048);
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client", mqtt_username.c_str(), mqtt_password.c_str())) {
            Serial.println("connected");
            sendDiscoveryPayloads();
            client.subscribe(BG_OFFSET_CMD);
            client.subscribe(BUTTON_CMD_TOPIC);

            // Send current values
            publishData(TEMP_TOPIC, prevTemp, true);
            publishData(PEOPLE_TOPIC, numOfPeople, true);
            publishData(BOOT_TIME_TOPIC, timeStr, true);
            publishData(BG_OFFSET_TOPIC, spikeThreshold, true);
            publishData(BOOT_REASON_TOPIC, bootReason, true);
            publishData(IP_ADDRESS_TOPIC, WiFi.localIP().toString(), true);
        } else {
            Serial.println("failed, rc=" + String(client.state()) + " try again in 5 seconds");
            delay(5000);
        }
    }
}

void sendDiscoveryPayloads() {
    StaticJsonDocument<512> tempConfig;
    tempConfig["name"] = "Thermistor Temperature";
    tempConfig["dev_cla"] = "temperature";  // device_class
    tempConfig["stat_t"] = TEMP_TOPIC "/state";  // state_topic
    tempConfig["unit_of_meas"] = "°C";  // unit_of_measurement
    tempConfig["disp_prc"] = 2;  // display_precision
    tempConfig["val_tpl"] = "{{ value_json.value }}";  // value_template
    tempConfig["uniq_id"] = "esp32_temp_sensor";  // unique_id
    tempConfig["dev"]["ids"] = DEVICE_NAME;  // device->identifiers
    tempConfig["dev"]["name"] = "Person Counter";
    tempConfig["dev"]["mf"] = "Aalian Labs";  // manufacturer
    tempConfig["dev"]["mdl"] = "USBC";  // model
    tempConfig["dev"]["sw"] = "1.0";  // sw_version
    tempConfig["dev"]["hw"] = "1.0";  // hw_version
    
    StaticJsonDocument<512> peopleConfig;
    peopleConfig["name"] = "People Counter";
    peopleConfig["stat_t"] = PEOPLE_TOPIC "/state";
    peopleConfig["unit_of_meas"] = "People";
    peopleConfig["val_tpl"] = "{{ value_json.value }}";
    peopleConfig["ic"] = "mdi:account-multiple";  // icon
    peopleConfig["uniq_id"] = "esp32_people_counter";
    peopleConfig["dev"]["ids"] = DEVICE_NAME;
    peopleConfig["dev"]["name"] = "Person Counter";
    peopleConfig["dev"]["mf"] = "Aalian Labs";
    peopleConfig["dev"]["mdl"] = "USBC";
    peopleConfig["dev"]["sw"] = "1.0";
    peopleConfig["dev"]["hw"] = "1.0";
    
    StaticJsonDocument<512> bootTimeConfig;
    bootTimeConfig["name"] = "Last Boot Time";
    bootTimeConfig["state_topic"] = BOOT_TIME_TOPIC "/state";
    bootTimeConfig["device_class"] = "timestamp";
    bootTimeConfig["value_template"] = "{{ value_json.value }}";
    bootTimeConfig["unique_id"] = "person_counter_boot_time";
    bootTimeConfig["device"]["identifiers"] = DEVICE_NAME;
    bootTimeConfig["device"]["name"] = "Person Counter";
    bootTimeConfig["device"]["manufacturer"] = "Aalian Labs";
    bootTimeConfig["device"]["model"] = "USBC";
    bootTimeConfig["device"]["sw_version"] = "1.0";
    bootTimeConfig["device"]["hw_version"] = "1.0";

    StaticJsonDocument<512> bgOffsetConfig;
    bgOffsetConfig["name"] = "Background Offset";
    bgOffsetConfig["cmd_t"] = BG_OFFSET_CMD;  // command_topic
    bgOffsetConfig["stat_t"] = BG_OFFSET_TOPIC "/state";
    bgOffsetConfig["value_template"] = "{{ value_json.value }}";
    bgOffsetConfig["min"] = 0.0;
    bgOffsetConfig["max"] = 10.0;
    bgOffsetConfig["step"] = 0.25;
    bgOffsetConfig["unit_of_meas"] = "%";
    bgOffsetConfig["uniq_id"] = "person_counter_bg_offset";
    bgOffsetConfig["dev"]["ids"] = DEVICE_NAME;
    bgOffsetConfig["dev"]["name"] = "Person Counter";
    bgOffsetConfig["dev"]["mf"] = "Aalian Labs";
    bgOffsetConfig["dev"]["mdl"] = "USBC";
    bgOffsetConfig["dev"]["sw"] = "1.0";
    bgOffsetConfig["dev"]["hw"] = "1.0";
    
    StaticJsonDocument<512> buttonConfig;
    buttonConfig["name"] = "Restart ESP32";
    buttonConfig["command_topic"] = BUTTON_CMD_TOPIC;
    buttonConfig["unique_id"] = "person_counter_restart";
    buttonConfig["device"]["identifiers"] = DEVICE_NAME;
    buttonConfig["device"]["name"] = "Person Counter";
    buttonConfig["device"]["manufacturer"] = "Aalian Labs";
    buttonConfig["device"]["model"] = "USBC";
    buttonConfig["device"]["sw_version"] = "1.0";
    buttonConfig["device"]["hw_version"] = "1.0";

    StaticJsonDocument<512> bootReasonConfig;
    bootReasonConfig["name"] = "Boot Reason";
    bootReasonConfig["state_topic"] = BOOT_REASON_TOPIC "/state";
    bootReasonConfig["value_template"] = "{{ value_json.value }}";
    bootReasonConfig["unique_id"] = "person_counter_boot_reason";
    bootReasonConfig["device"]["identifiers"] = DEVICE_NAME;
    bootReasonConfig["device"]["name"] = "Person Counter";
    bootReasonConfig["device"]["manufacturer"] = "Aalian Labs";
    bootReasonConfig["device"]["model"] = "USBC";
    bootReasonConfig["device"]["sw_version"] = "1.0";
    bootReasonConfig["device"]["hw_version"] = "1.0";

    StaticJsonDocument<512> ipAddressConfig;
    ipAddressConfig["name"] = "Device IP Address";
    ipAddressConfig["state_topic"] = IP_ADDRESS_TOPIC "/state";
    ipAddressConfig["value_template"] = "{{ value_json.value }}";
    ipAddressConfig["unique_id"] = "person_counter_ip_address";
    ipAddressConfig["device"]["identifiers"] = DEVICE_NAME;
    ipAddressConfig["device"]["name"] = "Person Counter";
    ipAddressConfig["device"]["manufacturer"] = "Aalian Labs";
    ipAddressConfig["device"]["model"] = "USBC";
    ipAddressConfig["device"]["sw_version"] = "1.0";
    ipAddressConfig["device"]["hw_version"] = "1.0";

    char buffer[512];
    serializeJson(tempConfig, buffer);
    client.publish(TEMP_TOPIC "/config", buffer, true); // Retained message for discovery
  
    serializeJson(peopleConfig, buffer);
    client.publish(PEOPLE_TOPIC "/config", buffer, true); // Retained message for discovery

    serializeJson(bootTimeConfig, buffer);
    client.publish(BOOT_TIME_TOPIC "/config", buffer, true); // Retained message for discovery

    serializeJson(bgOffsetConfig, buffer);
    client.publish(BG_OFFSET_TOPIC "/config", buffer, true); // Retained message for discovery

    serializeJson(buttonConfig, buffer);
    client.publish(BUTTON_TOPIC "/config", buffer, true); // Retained message for discovery

    serializeJson(bootReasonConfig, buffer);
    client.publish(BOOT_REASON_TOPIC "/config", buffer, true); // Retained message for discovery

    serializeJson(ipAddressConfig, buffer);
    client.publish(IP_ADDRESS_TOPIC "/config", buffer, true); // Retained message for discovery

    Serial.println("Discovery payloads sent.");
}

void publishData(String topic, String value, bool retain) {
    StaticJsonDocument<128> Data;
    Data["value"] = value;
    char Buffer[128];
    serializeJson(Data, Buffer);
    client.publish((topic + "/state").c_str(), Buffer, retain);

    Serial.print("Published to ");
    Serial.print(topic + "/state");
    Serial.print(": ");
    Serial.println(Buffer);
}

void publishData(String topic, float value, bool retain) {
    StaticJsonDocument<128> Data;
    Data["value"] = round(value * 100.0) / 100.0;
    char Buffer[128];
    serializeJson(Data, Buffer);
    client.publish((topic + "/state").c_str(), Buffer, retain);

    Serial.print("Published to ");
    Serial.print(topic + "/state");
    Serial.print(": ");
    Serial.println(Buffer);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    payload[length] = '\0'; // Ensure it's a proper string
    String topicStr = String(topic);
    Serial.print("Message arrived [");
    Serial.print(topicStr);
    Serial.print("] ");
    Serial.write(payload, length);
    Serial.println();

    if (topicStr.equals(BG_OFFSET_CMD)) {
        float newValue = atof((char*)payload);
        if (newValue >= 0.0 && newValue <= 10.0) {
            spikeThreshold = newValue;
            Serial.print("Updated Background Offset: ");
            Serial.println(spikeThreshold);
            publishData(BG_OFFSET_TOPIC, spikeThreshold, true); // Publish updated value
        }
    } else if (topicStr.equals(BUTTON_CMD_TOPIC)) {
        Serial.println("Restarting ESP32...");
        delay(1000);
        ESP.restart();
    } else {
        Serial.print("Unknown command received. Topic: ");
        Serial.println(topicStr);
    }
}