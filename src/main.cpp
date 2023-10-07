//******************************************************************************
//!
//! @file       main.cpp
//! @author     Felix Betz
//!
//! @brief      main.cpp
//!
//******************************************************************************

/*----------------------------------------------------------------------------*/
/* Includes                                                                   */
/*----------------------------------------------------------------------------*/
#include <Arduino.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include "secrets.h"

/*----------------------------------------------------------------------------*/
/* Defines                                                                    */
/*----------------------------------------------------------------------------*/
#define SENSOR_NAME "motion_sensor_floor_01"
#define SENSOR_NAME_FRIEDLY "Motion Sensor Floor";

#define MOTION_SENSOR_0_PIN 16 // D0 / GPIO16
#define MOTION_SENSOR_1_PIN 5  // D1 / GPIO5

#define MOTION_SENSOR_READ_MS 10
#define MOTION_SENSOR_CYCLIC_MQTT_MS 10000

#define HOSTNAME "ESP_MOTION_FLOOR_01"

/*----------------------------------------------------------------------------*/
/* Type definitions                                                           */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Variables                                                                  */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Functions                                                                  */
/*----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Local definitions------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
char g_ssid[] = WIFI_SSID;
const char g_password[] = WIFI_PASSWORD;
const char *g_mqtt_server = MQTT_SERVER;
const char *g_mqttUser = MQTT_USER;
const char *g_mqttPsw = MQTT_PASSWORD;
int g_mqttPort = 1883;
const char *device_name = "your_device_name";
const char *discovery_topic = "homeassistant/sensor/your_sensor/config";

unsigned long g_readSensorTime = 0;
unsigned long g_cylicMqttTime = 0;

bool g_isMotionDetectedOld = false;

WiFiClient espClient;
PubSubClient client(espClient);

//------------------------------------------------------------------------------
//   setTimeout
//------------------------------------------------------------------------------
unsigned long setTimeout(void)
{
  // build 2nd complement of the current system time
  unsigned long loc_timestamp = ((unsigned long)(~millis())) + 1;
  return loc_timestamp;
}

//------------------------------------------------------------------------------
//   isTimeoutExpired
//------------------------------------------------------------------------------
bool isTimeoutExpired(unsigned long arg_timestamp, unsigned long arg_timeMs)
{
  // check if timeout expired
  if ((arg_timestamp + millis()) >= arg_timeMs)
  {
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------
//   sendDiscoveryMessage
//------------------------------------------------------------------------------
void sendDiscoveryMessage()
{
  DynamicJsonDocument doc(256); // Adjust the size based on your JSON payload

  doc["name"] = SENSOR_NAME_FRIEDLY;
  doc["unique_id"] = SENSOR_NAME;
  doc["device_class"] = "motion";
  doc["state_topic"] = "home/" SENSOR_NAME "/state";
  doc["expire_after"] = "60"; // expire after 60s

  String payload;
  serializeJson(doc, payload);

  const char *discovery_topic = "homeassistant/binary_sensor/" SENSOR_NAME "/config";
  client.publish(discovery_topic, payload.c_str());
}

//------------------------------------------------------------------------------
//   sendMotion
//------------------------------------------------------------------------------
void mqttSendMotion(bool arg_isMotionDetected)
{
  const char *motion_topic = "home/" SENSOR_NAME "/state";
  client.publish(motion_topic, arg_isMotionDetected ? "ON" : "OFF");

  Serial.print("Motion Detected: ");
  Serial.println(arg_isMotionDetected ? "ON" : "OFF");
}
//------------------------------------------------------------------------------
//   setup_wifi
//------------------------------------------------------------------------------
void setup_wifi()
{
  int counter = 0;
  byte mac[6];
  delay(10);
  // We start by connecting to a WiFi network
  Serial.print("Connecting to ");
  Serial.println(g_ssid);

  WiFi.hostname(HOSTNAME);
  WiFi.begin(g_ssid, g_password);

  WiFi.macAddress(mac);
  String g_UniqueId = String(mac[0], HEX) + String(mac[1], HEX) + String(mac[2], HEX) + String(mac[3], HEX) + String(mac[4], HEX) + String(mac[5], HEX);

  Serial.print("Unique ID: ");
  Serial.println(g_UniqueId);

  while (WiFi.status() != WL_CONNECTED && counter++ < 8)
  {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("WiFi NOT connected!!!");
  }
}

//------------------------------------------------------------------------------
//   MqttReconnect
//------------------------------------------------------------------------------
void mqttReconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(device_name, g_mqttUser, g_mqttPsw))
    {
      Serial.println("connected");
      sendDiscoveryMessage();
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

//------------------------------------------------------------------------------
//   MqttReceiverCallback
//------------------------------------------------------------------------------
void mqttReceiverCallback(char *topic, byte *payload, unsigned int length)
{
  // Handle incoming MQTT messages if needed
}

//------------------------------------------------------------------------------
//   setup
//------------------------------------------------------------------------------
void setupMotionSensor()
{
  pinMode(MOTION_SENSOR_0_PIN, INPUT);
  pinMode(MOTION_SENSOR_1_PIN, INPUT);
  g_isMotionDetectedOld = false;
  g_readSensorTime = setTimeout();
  g_cylicMqttTime = setTimeout();
}

//------------------------------------------------------------------------------
//   setup
//------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);

  setup_wifi();
  ArduinoOTA.begin();
  client.setServer(g_mqtt_server, g_mqttPort);
  client.setCallback(mqttReceiverCallback);

  setupMotionSensor();
}

//------------------------------------------------------------------------------
//   loop
//------------------------------------------------------------------------------
void loop()
{
  ArduinoOTA.handle();
  if (!client.connected())
  {
    mqttReconnect();
  }
  else
  {
    if (isTimeoutExpired(g_readSensorTime, MOTION_SENSOR_READ_MS))
    {
      g_readSensorTime = setTimeout();

      bool loc_isMotionDetected = digitalRead(MOTION_SENSOR_0_PIN) || digitalRead(MOTION_SENSOR_1_PIN);

      if ((loc_isMotionDetected != g_isMotionDetectedOld) || isTimeoutExpired(g_cylicMqttTime, MOTION_SENSOR_CYCLIC_MQTT_MS))
      {
        g_cylicMqttTime = setTimeout();
        mqttSendMotion(loc_isMotionDetected);
        g_isMotionDetectedOld = loc_isMotionDetected;
      }
    }
  }
  client.loop();
}
