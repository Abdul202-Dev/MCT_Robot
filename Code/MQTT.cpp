#include "MQTT.h"
#include "Motoren.h"
#include "LCDdisplay.h"

volatile  char* payload1;
volatile boolean bMQTTGet=0;
#include <Adafruit_MCP23017.h>
volatile int Speed10=400;
volatile int Speed01=-Speed10;

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

// WiFi connection
#define WIFI_SSID "ABC"
#define WIFI_PASSWORD "geheim123456"

// MQTT connection
#define MQTT_HOST IPAddress(192, 168, 4, 1)
#define MQTT_PORT 1883
#define MQTT_USER "benutzer"
#define MQTT_PASSWORD "AMJ1234"



AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
Adafruit_MCP23017 io2;
//Adafruit_MCP23X17 io2;


void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);

//connect to Mqtt if is connected to the Wifi
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      //Serial.println("WiFi connected");
     // Serial.println("IP address: ");
      //Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
     // Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); 
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  //subscribe to atopic
  mqttClient.subscribe("Nodered/#", 0);
  
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
   bMQTTGet=true;

  payload[len] = 0; // Terminate payload string
  String SmS=String(payload);
  Serial.print("  payload: ");
//decide the next movement, depends on the received payload
  if (SmS == "for") {
    Printsecond("MQTT vorwaer");
    forward();
  } else if (SmS == "right") {
    Printsecond("MQTT rechts");
    setMotorSpeed(Speed10, Speed01);
  } else if (SmS == "left") {
    Printsecond("MQTT Links");
    setMotorSpeed(Speed01, Speed10);
  } else if (SmS == "back") {
    Printsecond("MQTT ruckwaer");
    backward();
  } else if (SmS=="Stop") {
    Printsecond("MQTT stop");
    setMotorSpeed(0, 0);
  }
  bMQTTGet=false;

}
 void Pub(String Payload){
  if(!bMQTTGet){
          mqttClient.publish("Rob/send/", 0, true,  String(Payload).c_str());
            bMQTTGet=true;

  }
}


void SETUPMQTT() {
  // Setup Serial
  Serial.println();





  // Setup Wire
  Wire.setClock(400000); // change i2c bus clock to 400 kHz

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USER, MQTT_PASSWORD);

  connectToWifi();
}
void MQTTActive(){
  
}
//IN Kern 1 Ende
