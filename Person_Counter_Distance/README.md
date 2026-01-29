# Person Counter Project

## Overview
The Person Counter project is designed to count the number of people detected using a thermal sensor. It utilizes WiFi for connectivity, MQTT for messaging, and EEPROM for storing configuration settings. The project is structured to separate concerns into different modules for better organization and maintainability.

## Project Structure
```
Person_Counter
├── src
│   ├── main.cpp                # Entry point of the application
│   ├── wifi                    # WiFi setup module
│   │   ├── wifi_setup.cpp      # Implementation of WiFi setup functions
│   │   └── wifi_setup.h        # Header file for WiFi setup functions
│   ├── server                  # HTTP server module
│   │   ├── server_setup.cpp    # Implementation of server setup functions
│   │   └── server_setup.h      # Header file for server setup functions
│   ├── mqtt                    # MQTT setup module
│   │   ├── mqtt_setup.cpp      # Implementation of MQTT setup functions
│   │   └── mqtt_setup.h        # Header file for MQTT setup functions
│   ├── sensor                  # Sensor module
│   │   ├── sensor_setup.cpp     # Implementation of sensor setup functions
│   │   └── sensor_setup.h      # Header file for sensor setup functions
│   ├── eeprom                  # EEPROM utility module
│   │   ├── eeprom_utils.cpp    # Implementation of EEPROM utility functions
│   │   └── eeprom_utils.h      # Header file for EEPROM utility functions
│   ├── person_counter          # Person counting module
│   │   ├── person_counter.cpp   # Implementation of person counting logic
│   │   └── person_counter.h     # Header file for person counting functions
│   ├── utils                   # Utility functions
│   │   ├── i2c_scanner.cpp     # Implementation of I2C scanner function
│   │   └── i2c_scanner.h       # Header file for I2C scanner function
│   └── types                   # Data structures
│       └── types.h             # Definitions of data structures
├── platformio.ini              # PlatformIO configuration file
└── README.md                   # Project documentation
```

## Setup Instructions
1. **Clone the Repository**: Clone this repository to your local machine.
2. **Install PlatformIO**: Ensure you have PlatformIO installed in your development environment.
3. **Open the Project**: Open the project folder in PlatformIO.
4. **Build the Project**: Use the build command in PlatformIO to compile the project.
5. **Upload to Device**: Connect your ESP32 device and upload the firmware.

## Usage
- After uploading the firmware, the device will attempt to connect to the specified WiFi network.
- You can access the HTTP server by navigating to the device's IP address in a web browser.
- Use the web interface to configure WiFi and MQTT settings.
- The device will publish data to the specified MQTT topics, which can be monitored using an MQTT client.

## Contributing
Contributions are welcome! Please feel free to submit a pull request or open an issue for any enhancements or bug fixes.

## License
This project is licensed under the MIT License. See the LICENSE file for more details.

## References

https://www.researchgate.net/publication/315301224_Indoor_Human_Detection_Based_on_Thermal_Array_Sensor_Data_and_Adaptive_Background_Estimation