// Public code not up2 date yet
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <dhtnew.h>

#include "Config.h"
#include "SerialCom.h"
#include "Types.h"

particleSensorState_t state;

uint8_t mqttRetryCounter = 0;

WiFiManager wifiManager;
WiFiClient wifiClient;
PubSubClient mqttClient;

WiFiManagerParameter custom_mqtt_server("server", "mqtt server", Config::mqtt_server, sizeof(Config::mqtt_server));
WiFiManagerParameter custom_mqtt_user("user", "MQTT username", Config::username, sizeof(Config::username));
WiFiManagerParameter custom_mqtt_pass("pass", "MQTT password", Config::password, sizeof(Config::password));

uint32_t lastMqttConnectionAttempt = 0;
const uint16_t mqttConnectionInterval = 60000; // 1 minute = 60 seconds = 60000 milliseconds

uint32_t statusPublishPreviousMillis = 0;
const uint16_t statusPublishInterval = 30000; // 30 seconds = 30000 milliseconds

char identifier[24];
#define FIRMWARE_PREFIX "esp8266-vindriktning-particle-sensor"
#define AVAILABILITY_ONLINE "online"
#define AVAILABILITY_OFFLINE "offline"
char MQTT_TOPIC_AVAILABILITY[128];
char MQTT_TOPIC_STATE[128];
char MQTT_TOPIC_COMMAND[128];

char MQTT_TOPIC_AUTOCONF_WIFI_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_PM25_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_AM2302_TEMP_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_AM2302_HUM_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_PIR_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_LIGHT_SENSOR[128];


bool shouldSaveConfig = false;

// DHT setup
static const uint8_t PIN_DHT = 12; //GPIO12 , D6
DHTNEW DHTSensor(PIN_DHT);   //
// DHT setup END

// PIR setup
static const uint8_t PIN_PIR = 20; //GPIO20 , D1
// PIR setup END

// Light sensor
static const int PIN_LUX = A0;  // GPIO2 ESP8266 Analog Pin ADC0 = A0 . Resistor value depends on the an R2R on the board.
int sensorValue = 0;            // value read from sensor
int mVolt = 0;
// Light sensor END

void saveConfigCallback() {
  shouldSaveConfig = true;
}



void setup() {
  Serial.begin(115200);
  SerialCom::setup();

  Serial.println("\n");
  Serial.println("Hello from esp8266-vindriktning-particle-sensor");
  Serial.printf("Core Version: %s\n", ESP.getCoreVersion().c_str());
  Serial.printf("Boot Version: %u\n", ESP.getBootVersion());
  Serial.printf("Boot Mode: %u\n", ESP.getBootMode());
  Serial.printf("CPU Frequency: %u MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("Reset reason: %s\n", ESP.getResetReason().c_str());

  delay(3000);

  snprintf(identifier, sizeof(identifier), "VINDRIKTNING-%X", ESP.getChipId());
  snprintf(MQTT_TOPIC_AVAILABILITY, 127, "%s/%s/status", FIRMWARE_PREFIX, identifier);
  snprintf(MQTT_TOPIC_STATE, 127, "%s/%s/state", FIRMWARE_PREFIX, identifier);
  snprintf(MQTT_TOPIC_COMMAND, 127, "%s/%s/command", FIRMWARE_PREFIX, identifier);

  snprintf(MQTT_TOPIC_AUTOCONF_PM25_SENSOR, 127, "homeassistant/sensor/%s/%s_pm25/config", FIRMWARE_PREFIX, identifier);
  snprintf(MQTT_TOPIC_AUTOCONF_WIFI_SENSOR, 127, "homeassistant/sensor/%s/%s_wifi/config", FIRMWARE_PREFIX, identifier);
  snprintf(MQTT_TOPIC_AUTOCONF_AM2302_TEMP_SENSOR, 127, "homeassistant/sensor/%s/%s_temp/config", FIRMWARE_PREFIX, identifier);
  snprintf(MQTT_TOPIC_AUTOCONF_AM2302_HUM_SENSOR, 127, "homeassistant/sensor/%s/%s_hum/config", FIRMWARE_PREFIX, identifier);
  snprintf(MQTT_TOPIC_AUTOCONF_LIGHT_SENSOR, 127, "homeassistant/sensor/%s/%s_light/config", FIRMWARE_PREFIX, identifier);
  snprintf(MQTT_TOPIC_AUTOCONF_PIR_SENSOR, 127, "homeassistant/sensor/%s/%s_pir/config", FIRMWARE_PREFIX, identifier);

  WiFi.hostname(identifier);


  //Config::load();
  Serial.printf("Config load done");

  setupWifi();
  Serial.printf("Wifi setup done");
  setupOTA();
  Serial.printf("OTA Done ");
  mqttClient.setServer(Config::mqtt_server, 1883);
  mqttClient.setKeepAlive(10);
  mqttClient.setBufferSize(2048);
  mqttClient.setCallback(mqttCallback);
  Serial.printf("MQTT Client done ");

  Serial.printf("Hostname: %s\n", identifier);
  Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());

  Serial.println("-- Current GPIO Configuration --");
  Serial.printf("PIN_UART_RX: %d\n", SerialCom::PIN_UART_RX);
  Serial.printf("PIN_DHT: %d\n", PIN_DHT);
  Serial.printf("PIN_Light: %d\n", PIN_DHT);
  Serial.printf("PIN_PIR: %d\n", PIN_PIR);

  // DHT setup and serial output
  // This would be useful if you know the skew for your sensor
  Serial.println("DHT reading raw, before Offset change");
  DHTSensor.read();
  Serial.print(DHTSensor.getHumidity(), 1);
  Serial.print("\t");
  Serial.println(DHTSensor.getTemperature(), 1);
  //DHTSensor.setHumOffset(10);
  //DHTSensor.setTempOffset(-3.5);
  //DHT setup END

  // Setup interrupt based PIR detection
  //XXX attachInterrupt(digitalPinToInterrupt(PIN_PIR), ISR, mode);

  mqttReconnect();

}

void setupOTA() {
  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  ArduinoOTA.setHostname(identifier);

  // This is less of a security measure and more against accidental flash prevention
  ArduinoOTA.setPassword(identifier);
  ArduinoOTA.begin();
}
//XXX THIS was WRONG
int readLight() {
  sensorValue = analogRead(PIN_LUX);
  return sensorValue;
}

void loop() {
  //DHT
  if (millis() - DHTSensor.lastRead() > 2000)
  {
    DHTSensor.read();
    Serial.print(DHTSensor.getHumidity(), 1);
    Serial.print("\t");
    Serial.println(DHTSensor.getTemperature(), 1);
  }
  // DHT end

  ArduinoOTA.handle();
  SerialCom::handleUart(state);
  mqttClient.loop();

  const uint32_t currentMillis = millis();
  if (currentMillis - statusPublishPreviousMillis >= statusPublishInterval) {
    statusPublishPreviousMillis = currentMillis;

    if (state.valid) {
      printf("Publish state\n");
      publishState();
    }
  }

  if (!mqttClient.connected() && currentMillis - lastMqttConnectionAttempt >= mqttConnectionInterval) {
    lastMqttConnectionAttempt = currentMillis;
    printf("Reconnect mqtt\n");
    mqttReconnect();
  }

}

void setupWifi() {
wifiManager.setDebugOutput(false);
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);

  WiFi.hostname(identifier);
  wifiManager.autoConnect(identifier);
  mqttClient.setClient(wifiClient);

  strcpy(Config::mqtt_server, custom_mqtt_server.getValue());
  strcpy(Config::username, custom_mqtt_user.getValue());
  strcpy(Config::password, custom_mqtt_pass.getValue());

  if (shouldSaveConfig) {
    Config::save();
  } else {
    // For some reason, the read values get overwritten in this function
    // To combat this, we just reload the config
    // This is most likely a logic error which could be fixed otherwise
    Config::load();
  }
}

void resetWifiSettingsAndReboot() {
  wifiManager.resetSettings();
  delay(3000);
  ESP.restart();
}

void mqttReconnect() {
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    if (mqttClient.connect(identifier, Config::username, Config::password, MQTT_TOPIC_AVAILABILITY, 1, true, AVAILABILITY_OFFLINE)) {
      mqttClient.publish(MQTT_TOPIC_AVAILABILITY, AVAILABILITY_ONLINE, true);
      publishAutoConfig();

      // Make sure to subscribe after polling the status so that we never execute commands with the default data
      mqttClient.subscribe(MQTT_TOPIC_COMMAND);
      break;
    }
    delay(5000);
  }
}

bool isMqttConnected() {
  return mqttClient.connected();
}

// XXX Pull my sensor value here and serialize
// XXX error checking ? what is that?
void publishState() {

  DHTSensor.read();

  DynamicJsonDocument wifiJson(192);
  DynamicJsonDocument stateJson(604);
  DynamicJsonDocument dht22Json(604);
  DynamicJsonDocument LightJson(604); // XXX resize properly
  DynamicJsonDocument PIRJson(604); // XXX resize properly

  sensorValue = readLight();
  mVolt = map(sensorValue, 0, 1023, 0, 3300);
  float volt = (double)mVolt / 1000;
  Serial.print(" sensor value = ");
  Serial.print(sensorValue);
  Serial.print(" | mVolt = ");
  Serial.print(mVolt);
  Serial.print(" | Volt = ");
  Serial.print(volt);
  Serial.print("\n");


  char payload[256];

  wifiJson["ssid"] = WiFi.SSID();
  wifiJson["ip"] = WiFi.localIP().toString();
  wifiJson["rssi"] = WiFi.RSSI();

  dht22Json["temp"] = DHTSensor.getTemperature();
  dht22Json["hum"] = DHTSensor.getHumidity();

  //XXX Need to read the global state
  //    PIRJson["Triggered"]=

  stateJson["pm25"] = state.avgPM25;

  stateJson["wifi"] = wifiJson.as<JsonObject>();
  stateJson["dht22"] = dht22Json.as<JsonObject>();

  serializeJson(stateJson, payload);
  mqttClient.publish(&MQTT_TOPIC_STATE[0], &payload[0], true);
}

void mqttCallback(char* topic, uint8_t* payload, unsigned int length) { }

void publishAutoConfig() {
  char mqttPayload[2048];
  DynamicJsonDocument device(256);
  DynamicJsonDocument autoconfPayload(1024);
  StaticJsonDocument<64> identifiersDoc;
  JsonArray identifiers = identifiersDoc.to<JsonArray>();

  identifiers.add(identifier);

  device["identifiers"] = identifiers;
  device["manufacturer"] = "Ikea";
  device["model"] = "VINDRIKTNING";
  device["name"] = identifier;
  device["sw_version"] = "2022.07.0";

  autoconfPayload["device"] = device.as<JsonObject>();
  autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
  autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
  autoconfPayload["name"] = identifier + String(" WiFi");
  autoconfPayload["value_template"] = "{{value_json.wifi.rssi}}";
  autoconfPayload["unique_id"] = identifier + String("_wifi");
  autoconfPayload["unit_of_measurement"] = "dBm";
  autoconfPayload["json_attributes_topic"] = MQTT_TOPIC_STATE;
  autoconfPayload["json_attributes_template"] = "{\"ssid\": \"{{value_json.wifi.ssid}}\", \"ip\": \"{{value_json.wifi.ip}}\"}";
  autoconfPayload["icon"] = "mdi:wifi";

  serializeJson(autoconfPayload, mqttPayload);
  mqttClient.publish(&MQTT_TOPIC_AUTOCONF_WIFI_SENSOR[0], &mqttPayload[0], true);

  autoconfPayload.clear();

  autoconfPayload["device"] = device.as<JsonObject>();
  autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
  autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
  autoconfPayload["name"] = identifier + String(" PM 2.5");
  autoconfPayload["unit_of_measurement"] = "μg/m³";
  autoconfPayload["value_template"] = "{{value_json.pm25}}";
  autoconfPayload["unique_id"] = identifier + String("_pm25");
  autoconfPayload["icon"] = "mdi:air-filter";

  serializeJson(autoconfPayload, mqttPayload);
  mqttClient.publish(&MQTT_TOPIC_AUTOCONF_PM25_SENSOR[0], &mqttPayload[0], true);

  autoconfPayload.clear();
  // DHT code starts here
  // Added 2 "sensor" temp and humidity that are now properly advertised.

  autoconfPayload["device"] = device.as<JsonObject>();
  autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
  autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
  autoconfPayload["name"] = identifier + String(" Temperature");
  autoconfPayload["unit_of_measurement"] = "°C";
  autoconfPayload["value_template"] = "{{value_json.temp}}";
  autoconfPayload["unique_id"] = identifier + String("_temp");
  autoconfPayload["icon"] = "mdi:dht22-temp"; // It is unlikely this icon exist in HA

  serializeJson(autoconfPayload, mqttPayload);
  mqttClient.publish(&MQTT_TOPIC_AUTOCONF_AM2302_TEMP_SENSOR[0], &mqttPayload[0], true);

  autoconfPayload.clear();

  autoconfPayload["device"] = device.as<JsonObject>();
  autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
  autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
  autoconfPayload["name"] = identifier + String(" Humidity");
  autoconfPayload["unit_of_measurement"] = "%";
  autoconfPayload["value_template"] = "{{value_json.hum}}";
  autoconfPayload["unique_id"] = identifier + String("_hum");
  autoconfPayload["icon"] = "mdi:dht22-humidity"; // it is unlikely this icon exist in HA

  serializeJson(autoconfPayload, mqttPayload);
  mqttClient.publish(&MQTT_TOPIC_AUTOCONF_AM2302_HUM_SENSOR[0], &mqttPayload[0], true);

  autoconfPayload.clear();

}
