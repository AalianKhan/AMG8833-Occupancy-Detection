#include "wifi_setup.h"
#include "eeprom/eeprom_utils.h"
#include "config.h"
#include <ArduinoJson.h>

WebServer server(80);
String new_ssid, new_password, new_mqtt_server, new_mqtt_username, new_mqtt_password;

void setup_wifi() {
    delay(10);
    Serial.println("\nConnecting to " + String(ssid));
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected\nIP address: " + WiFi.localIP().toString());
    } else {
        Serial.println("\nFailed to connect to WiFi. Starting AP mode...");
        startAPMode();
    }
}

void startAPMode() {
    WiFi.softAP(DEVICE_NAME);
    Serial.println("AP mode started");
    Serial.println("AP SSID: " + WiFi.softAPSSID());
    Serial.println("AP IP address: " + WiFi.softAPIP().toString());
}

void setupServer() {
    server.on("/", HTTP_GET, []() {
        String html = "<style>"
                      "body { font-family: Arial, sans-serif; text-align: center; padding: 50px; }"
                      "input[type='text'], input[type='submit'] { font-size: 1.5em; padding: 10px; margin: 10px; width: 80%; }"
                      "input[type='submit'] { background-color: #4CAF50; color: white; border: none; cursor: pointer; }"
                      "input[type='submit']:hover { background-color: #45a049; }"
                      "button { font-size: 1.5em; padding: 10px; margin: 10px; background-color: #008CBA; color: white; border: none; cursor: pointer; }"
                      "button:hover { background-color: #005f73; }"
                      "</style>"
                      "<h1>WiFi and MQTT Configuration</h1>"
                      "<form action=\"/save\" method=\"POST\">"
                      "SSID: <input type=\"text\" name=\"ssid\" placeholder=\"" + String(ssid) + "\"><br>"
                      "Password: <input type=\"text\" name=\"password\" placeholder=\"" + String(password) + "\"><br>"
                      "MQTT Server: <input type=\"text\" name=\"mqtt_server\" placeholder=\"" + String(mqtt_server) + "\"><br>"
                      "MQTT Username: <input type=\"text\" name=\"mqtt_username\" placeholder=\"" + String(mqtt_username) + "\"><br>"
                      "MQTT Password: <input type=\"text\" name=\"mqtt_password\" placeholder=\"" + String(mqtt_password) + "\"><br>"
                      "<input type=\"submit\" value=\"Save\">"
                      "</form>"
                      "<button onclick=\"location.href='/thermal'\">View Thermal Image</button>";
        server.send(200, "text/html", html);
    });

    server.on("/save", HTTP_POST, []() {
      new_ssid = server.arg("ssid");
      new_password = server.arg("password");
      new_mqtt_server = server.arg("mqtt_server");
      new_mqtt_username = server.arg("mqtt_username");
      new_mqtt_password = server.arg("mqtt_password");
  
      saveSettings(new_ssid, new_password, new_mqtt_server, new_mqtt_username, new_mqtt_password);
  
      server.send(200, "text/html", "Settings saved. Restarting...");
      delay(2000);
      ESP.restart();
    });

    server.on("/thermal", HTTP_GET, []() {
        String html = "<style>"
                      "body { font-family: Arial, sans-serif; text-align: center; padding: 50px; }"
                      "canvas { border: 1px solid black; margin-top: 20px; max-width: 90%; height: auto; display: block; margin-left: auto; margin-right: auto; }"
                      "button { font-size: 1.5em; padding: 10px; margin: 20px auto; background-color: #008CBA; color: white; border: none; cursor: pointer; display: block; }"
                      "button:hover { background-color: #005f73; }"
                      "</style>"
                      "<h1>Thermal Image</h1>"
                      "<canvas id=\"thermalCanvas\"></canvas>"
                      "<button onclick=\"location.href='/'\">Back to Main Page</button>"
                      "<script>"
                      "var canvas = document.getElementById('thermalCanvas');"
                      "var ctx = canvas.getContext('2d');"
                      "var width = 8, height = 8;"
                      "function resizeCanvas() {"
                      "    var scale = Math.min(window.innerWidth / 12, window.innerHeight / 12);"
                      "    canvas.width = width * scale;"
                      "    canvas.height = height * scale;"
                      "}"
                      "window.addEventListener('resize', resizeCanvas);"
                      "resizeCanvas();"
                      "function tempToColor(temp) {"
                      "    var minTemp = 16, maxTemp = 25;"
                      "    var ratio = (temp - minTemp) / (maxTemp - minTemp);"
                      "    var r = Math.round(255 * ratio);"
                      "    var g = Math.round(255 * (1 - ratio));"
                      "    var b = 100;"
                      "    return `rgb(${r},${g},${b})`;"
                      "}"
                      "function updateThermalImage() {"
                      "    fetch('/thermal_data').then(response => response.json()).then(data => {"
                      "        var temperatureMatrix = data.temperatureMatrix;"
                      "        var blobs = data.blobs || [];"
                      "        var centroids = data.centroids || [];"
                      "        ctx.clearRect(0, 0, canvas.width, canvas.height);"
                      "        var scaleX = canvas.width / width;"
                      "        var scaleY = canvas.height / height;"
                      "        for (var y = 0; y < height; y++) {"
                      "            for (var x = 0; x < width; x++) {"
                      "                var temp = temperatureMatrix[y * width + x];"
                      "                ctx.fillStyle = tempToColor(temp);"
                      "                ctx.fillRect(x * scaleX, y * scaleY, scaleX, scaleY);"
                      "            }"
                      "        }"
                      "        blobs.forEach(blob => {"
                      "            ctx.beginPath();"
                      "            ctx.arc(blob.x * scaleX, blob.y * scaleY, blob.radius * scaleX, 0, 2 * Math.PI);"
                      "            ctx.strokeStyle = 'white';"
                      "            ctx.lineWidth = 2;"
                      "            ctx.stroke();"
                      "        });"
                      "        centroids.forEach(centroid => {"
                      "            ctx.beginPath();"
                      "            ctx.arc(centroid.x * scaleX, centroid.y * scaleY, 5, 0, 2 * Math.PI);"
                      "            ctx.fillStyle = 'black';"
                      "            ctx.fill();"
                      "        });"
                      "    });"
                      "}"
                      "setInterval(updateThermalImage, 100);"
                      "</script>";
        server.send(200, "text/html", html);
    });

    server.on("/thermal_data", HTTP_GET, []() {
        StaticJsonDocument<2048> doc;
        JsonArray tempArray = doc.createNestedArray("temperatureMatrix");
        for (int i = 0; i < 64; i++) {
            tempArray.add(filteredPixels[i]);
        }
        JsonArray blobArray = doc.createNestedArray("blobs");
        for (const auto& blob : blobs) {
            JsonObject blobObj = blobArray.createNestedObject();
            blobObj["x"] = blob.x;
            blobObj["y"] = blob.y;
            blobObj["radius"] = blob.radius;
        }
        JsonArray centroidArray = doc.createNestedArray("centroids");
        for (const auto& [id, centroid] : activeCentroids) {
            JsonObject centroidObj = centroidArray.createNestedObject();
            centroidObj["x"] = centroid.x;
            centroidObj["y"] = centroid.y;
        }
        String response;
        serializeJson(doc, response);
        server.send(200, "application/json", response);
    });

    server.onNotFound([]() {
      server.send(404, "text/plain", "Not found");
    });
  
    server.begin();
}