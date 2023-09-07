#ifndef MQTT_FUNCTIONS_H
#define MQTT_FUNCTIONS_H
#include <Arduino.h>

#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <Adafruit_MCP23017.h>
#include <LiquidCrystal_I2C.h>
//prototyps
extern  void onMqttMessage(char* , char* , AsyncMqttClientMessageProperties , size_t , size_t , size_t );
extern  void connectToWifi();
extern  void connectToMqtt();
extern  void WiFiEvent(WiFiEvent_t );
extern  void onMqttConnect(bool );
extern  void onMqttDisconnect(AsyncMqttClientDisconnectReason );
extern  void onMqttPublish(uint16_t );
extern  void SETUPMQTT();
extern  void Pub (String);

#endif
