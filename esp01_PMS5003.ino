#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include "WiFiClient.h"
#include <PMserial.h> // Arduino library for PM sensors with serial interface
#include "KickSort.h"

#define WIFI_SSID "section9_iot"
#define WIFI_PASSWORD "laughingman"



#define MQTT_HOST IPAddress(192, 168, 2, 247)
#define MQTT_PORT 1883
#define MQTT_PUB_PMS5003_01PM "topic/PM1.0"
#define MQTT_PUB_PMS5003_25PM "topic/PM2.1"
#define MQTT_PUB_PMS5003_TENPM "topic/PM10"
//#define MQTT_PUB_                                                                                       PMSA003_XTRA "home/PMSa003/xtra"
#define NUM_READINGS 11
#define NUM_TOPICS_TO_PUB 3 //update this as needed. This will be used as a sort of tick-tock to see if we published data and if we might need to perform a network recovery because the linksys AP is total shit


#define FIVE_SECONDS_IN_MILLS 5000
#define TEN_SECONDS_IN_MILLS 10000
#define THIRTY_SECONDS_IN_MILLS 30000
#define SIXTY_SECONDS_IN_MILLS 60000
AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

SerialPM pms(PMSx003, Serial); // PMSx003, UART

//unsigned long WifiStartMillis;
unsigned long MqttStartMillis;

int TopicsPublished = 0;

unsigned int cycleWaitTime = SIXTY_SECONDS_IN_MILLS;


bool connectToWifiOrTimeout() {
  unsigned long WifiStartMillis = millis();
  unsigned long currMillis;
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  do {
    Serial1.print("`");
    currMillis = millis();
    // five second timeout. Decided on purely by measuing the time a connection takes for me at home.
    if (currMillis - WifiStartMillis > TEN_SECONDS_IN_MILLS) {
      Serial1.printf("\nWiFi connect 10 second timeout (%d)\n\r",currMillis - WifiStartMillis);
      WiFi.disconnect( true );
      return false;
    }
    delay (100);
  } while (!WiFi.isConnected());
  Serial1.printf("Connected to wifi in %d millis\n\r",currMillis - WifiStartMillis);
  return true;
}

void connectToWifi() {
  Serial1.println("connectToWifi()");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

bool connectToMqttOrTimeout() {
  unsigned long MqttStartMillis = millis();
  unsigned long currMillis;
  
  mqttClient.connect();
  do {
    currMillis = millis();
    if (currMillis - MqttStartMillis > TEN_SECONDS_IN_MILLS) {
      Serial1.println("MQTT connect 10 second timeout");
      WiFi.disconnect( true );
      return false;
    }
    delay (100);
  } while (!mqttClient.connected());
  Serial1.printf("Connected to MQTT in %d millis\n\r",currMillis - MqttStartMillis);
  return true;
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial1.print("onWifiDisconnect, reason: "); 
  PrintDisconnectReason(event.reason); 
  Serial1.println();
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
  //consider ESP.restart()
}

void connectToMqtt() {
  //Serial.println("Connecting to MQTT...");
  MqttStartMillis = millis();
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  //Serial.println("Connected to MQTT.");
  //MqttConnected = true;
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial1.print("onMqttDisconnect, reason: ");
  printMqttDisconnectReason(reason);
  Serial1.println();
  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttPublish(uint16_t packetId) {
  TopicsPublished++;
}

void setup() {

  //Serial1.begin(74880);
  Serial.begin(9600);
  //Serial.swap(); //This must be run for hte NodeMCU. It appears the extra circuitry for the USB UART interfears with the sensor.
  //delay (100);
  //pms.passiveMode();    // Switch to passive mode
 
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
  
  Serial1.println("registering MQTT callbacks");
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  
  connectToWifi();
  Serial1.println("setup: Starting wifi");
  if (!connectToWifiOrTimeout()){
    Serial1.println("setup: Wifi connect timeout,wait 10 sec and reboot");
    delay(TEN_SECONDS_IN_MILLS);
    ESP.restart();
  }
  
  Serial1.println("setup: Starting MQTT");
  if (!connectToMqttOrTimeout()){
    Serial1.println("setup: MQTT connect timeout,wait 10 sec and reboot");
    delay(TEN_SECONDS_IN_MILLS);
    ESP.restart();
  }
}

void loop() {
  uint16_t pm25_Array [NUM_READINGS];
  uint16_t pm01_Array [NUM_READINGS];
  uint16_t pm10_Array [NUM_READINGS];
  int i;
  Serial1.println("Waking up, wait 30 seconds for stable readings...");
  pms.wake();
  delay(30000);
  
  for (i=0;i < NUM_READINGS; i++){
    pms.read();
    if (pms) {
      pm01_Array[i] = pms.pm01;
      pm25_Array[i] = pms.pm25;
      pm10_Array[i] = pms.pm10;
      //Serial1.printf("%d_%2d ", i, pms.pm25);

    } else {
      printPmsStatus();
    }
    delay(150); //just a random figure
  }
  pms.sleep();

////////////////////////////////////begin averaging/normalizing////////////////////// 
//The one PMSa003 I have for testing shows weird spikes in the data.
//To try and address that problem, this tries to take a number of readings, throw out potetnal outliers,
//then takes the average of the remaining readings.
//Note that this did not address the spikey data, perhaps my unit is just bad. 
//But I'm leaving this in as an example of using kicksort for myself.

  Serial1.println();
  //Serial1.println("sorted:");
  KickSort<uint16_t>::bubbleSort(pm25_Array, NUM_READINGS);
  KickSort<uint16_t>::bubbleSort(pm01_Array, NUM_READINGS);
  KickSort<uint16_t>::bubbleSort(pm10_Array, NUM_READINGS);

//we will throw out the highest and lowest values assuming they are outliers. 
//What if there are multiple readings that are outliers?
//That's a bigger issue. I could shrink the array or even just pick the midpoint rather than take an average. 
//but for now just try this to see what happens.

//clear these values to use for storage.
  pm25_Array[0] = 0;
  pm01_Array[0] = 0;
  pm10_Array[0] = 0;
    
//print the array in a format I can just plug it into a calculator app
Serial1.printf("(");
for (i=1;i < NUM_READINGS - 1; i++){
    pm25_Array[0] += pm25_Array[i];
    Serial1.printf("%d+",pm25_Array[i]);
    pm01_Array[0] += pm01_Array[i];
    pm10_Array[0] += pm10_Array[i];
}
Serial1.printf("0) / %d\n\r",NUM_READINGS - 2);
Serial1.printf ("total %d, #readings %d\r\n\r\n",pm25_Array[0], i);

pm25_Array[0] = round ((pm25_Array[0] / (NUM_READINGS - 2.0)));
pm01_Array[0] = round ((pm01_Array[0] / (NUM_READINGS - 2.0)));
pm10_Array[0] = round ((pm10_Array[0] / (NUM_READINGS - 2.0)));



Serial1.printf("pm25_avg %d\n\r",pm25_Array[0]);
Serial1.printf("pm01_avg %d\n\r",pm01_Array[0]);
Serial1.printf("pm10_avg %d\n\r",pm10_Array[0]);

////////////////////////////////////end averaging/normalizing//////////////////////


    //char resDataStr[13]; //6 bytes, that's 12 chars plus one for the terminating null. I'll log al the data in one go and try to pull trends from it when it's in a spreadsheet.
    //snprintf (resDataStr, 13, "%02x%02x%02x%02x%02x%02x",data.reserved_fields[0],data.reserved_fields[1],data.reserved_fields[2],data.reserved_fields[3],data.reserved_fields[4],data.reserved_fields[5]);
    
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_PMS5003_01PM, 1, true, String(pm01_Array[0]).c_str());
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_PMS5003_25PM, 1, true, String(pm25_Array[0]).c_str());
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_PMS5003_TENPM, 1, true, String(pm10_Array[0]).c_str());

  Serial1.println("Putting sensor to sleep");

  //succeed or fail, wait 1 sec
  delay(10000); // wait 10 seconds. We'll check on the status of out topics and take corrective action as needed. Then we wait another 30 seconds.
  //the topics should have been published by now
  if (TopicsPublished != NUM_TOPICS_TO_PUB) {
    Serial1.printf("failure to publish topics, got %d out\n\r", TopicsPublished);
    // I am pretty certain from another dodgy sensor that the connection is dead but the ESP doesn't know it.
    InfraRecovery();
  }
  delay (50000); //If no issues, this is the rest of the 60 second sample time. Maybe I sould make this a var that each action borrows against and the rest it used here
}


void InfraRecovery() {
  //unreliable wifi causes a loss of connection between wifi and MQTT.
  //The only viable recovery action appears to be to disconnect everything, reconnect everything, and retry.
  mqttClient.disconnect(true); //force a disconnect
  delay (5);
  Serial1.printf("Final disconnect: %d published\r\n",TopicsPublished);
  WiFi.disconnect( true );
  delay(5);
  // Two possible actions to take, reconnect or reboot.
  // Try a reconnect for now, and reboot if either fails.
    Serial1.println("setup: Starting wifi");
  if (!connectToWifiOrTimeout()){
    Serial1.println("setup: Wifi connect timeout,wait 10 sec and reboot");
    delay(TEN_SECONDS_IN_MILLS);
    ESP.restart();
  }
  
  Serial1.println("setup: Starting MQTT");
  if (!connectToMqttOrTimeout()){
    Serial1.println("setup: MQTT connect timeout,wait 10 sec and reboot");
    delay(TEN_SECONDS_IN_MILLS);
    ESP.restart();
  }
}

void printPmsStatus() {
    switch (pms.status)
    {
    case pms.OK: // should never come here
      break;     // included to compile without warnings
    case pms.ERROR_TIMEOUT:
      Serial.println(F(PMS_ERROR_TIMEOUT));
      break;
    case pms.ERROR_MSG_UNKNOWN:
      Serial.println(F(PMS_ERROR_MSG_UNKNOWN));
      break;
    case pms.ERROR_MSG_HEADER:
      Serial.println(F(PMS_ERROR_MSG_HEADER));
      break;
    case pms.ERROR_MSG_BODY:
      Serial.println(F(PMS_ERROR_MSG_BODY));
      break;
    case pms.ERROR_MSG_START:
      Serial.println(F(PMS_ERROR_MSG_START));
      break;
    case pms.ERROR_MSG_LENGTH:
      Serial.println(F(PMS_ERROR_MSG_LENGTH));
      break;
    case pms.ERROR_MSG_CKSUM:
      Serial.println(F(PMS_ERROR_MSG_CKSUM));
      break;
    case pms.ERROR_PMS_TYPE:
      Serial.println(F(PMS_ERROR_PMS_TYPE));
      break;
    }
}

void PrintDisconnectReason(WiFiDisconnectReason reason)
{
#if MY_DEBUG
  switch(reason) {
    case WIFI_DISCONNECT_REASON_UNSPECIFIED:
      Serial1.print("WIFI_DISCONNECT_REASON_UNSPECIFIED");
      break;
    case WIFI_DISCONNECT_REASON_AUTH_EXPIRE:
      Serial1.print("WIFI_DISCONNECT_REASON_AUTH_EXPIRE");
      break;
    case WIFI_DISCONNECT_REASON_AUTH_LEAVE:
      Serial1.print("WIFI_DISCONNECT_REASON_AUTH_LEAVE");
      break;
    case WIFI_DISCONNECT_REASON_ASSOC_EXPIRE:
      Serial1.print("WIFI_DISCONNECT_REASON_ASSOC_EXPIRE");
      break;
    case WIFI_DISCONNECT_REASON_ASSOC_TOOMANY:
      Serial1.print("WIFI_DISCONNECT_REASON_ASSOC_TOOMANY");
      break;
    case WIFI_DISCONNECT_REASON_NOT_AUTHED:
      Serial1.print("WIFI_DISCONNECT_REASON_NOT_AUTHED");
      break;
    case WIFI_DISCONNECT_REASON_NOT_ASSOCED:
      Serial1.print("WIFI_DISCONNECT_REASON_NOT_ASSOCED");
      break;
    case WIFI_DISCONNECT_REASON_ASSOC_LEAVE:
      Serial1.print("WIFI_DISCONNECT_REASON_ASSOC_LEAVE");
      break;
    case WIFI_DISCONNECT_REASON_ASSOC_NOT_AUTHED:
      Serial1.print("WIFI_DISCONNECT_REASON_ASSOC_NOT_AUTHED");
      break;
    case WIFI_DISCONNECT_REASON_DISASSOC_PWRCAP_BAD:
      Serial1.print("WIFI_DISCONNECT_REASON_DISASSOC_PWRCAP_BAD");
      break;
    case WIFI_DISCONNECT_REASON_DISASSOC_SUPCHAN_BAD:
      Serial1.print("WIFI_DISCONNECT_REASON_DISASSOC_SUPCHAN_BAD");
      break;
    case WIFI_DISCONNECT_REASON_IE_INVALID:
      Serial1.print("WIFI_DISCONNECT_REASON_IE_INVALID");
      break;
    case WIFI_DISCONNECT_REASON_MIC_FAILURE:
      Serial1.print("WIFI_DISCONNECT_REASON_MIC_FAILURE");
      break;
    case WIFI_DISCONNECT_REASON_4WAY_HANDSHAKE_TIMEOUT:
      Serial1.print("WIFI_DISCONNECT_REASON_4WAY_HANDSHAKE_TIMEOUT");
      break;
    case WIFI_DISCONNECT_REASON_GROUP_KEY_UPDATE_TIMEOUT:
      Serial1.print("WIFI_DISCONNECT_REASON_GROUP_KEY_UPDATE_TIMEOUT");
      break;
    case WIFI_DISCONNECT_REASON_IE_IN_4WAY_DIFFERS:
      Serial1.print("WIFI_DISCONNECT_REASON_IE_IN_4WAY_DIFFERS");
      break;
    case WIFI_DISCONNECT_REASON_GROUP_CIPHER_INVALID:
      Serial1.print("WIFI_DISCONNECT_REASON_GROUP_CIPHER_INVALID");
      break;
    case WIFI_DISCONNECT_REASON_PAIRWISE_CIPHER_INVALID:
      Serial1.print("WIFI_DISCONNECT_REASON_PAIRWISE_CIPHER_INVALID");
      break;
    case WIFI_DISCONNECT_REASON_AKMP_INVALID:
      Serial1.print("WIFI_DISCONNECT_REASON_AKMP_INVALID");
      break;
    case WIFI_DISCONNECT_REASON_UNSUPP_RSN_IE_VERSION:
      Serial1.print("WIFI_DISCONNECT_REASON_UNSUPP_RSN_IE_VERSION");
      break;
    case WIFI_DISCONNECT_REASON_INVALID_RSN_IE_CAP:
      Serial1.print("WIFI_DISCONNECT_REASON_INVALID_RSN_IE_CAP");
      break;
    case WIFI_DISCONNECT_REASON_802_1X_AUTH_FAILED:
      Serial1.print("WIFI_DISCONNECT_REASON_802_1X_AUTH_FAILED");
      break;
    case WIFI_DISCONNECT_REASON_CIPHER_SUITE_REJECTED:
      Serial1.print("WIFI_DISCONNECT_REASON_CIPHER_SUITE_REJECTED");
      break;
    case WIFI_DISCONNECT_REASON_BEACON_TIMEOUT:
      Serial1.print("WIFI_DISCONNECT_REASON_BEACON_TIMEOUT");
      break;
    case WIFI_DISCONNECT_REASON_NO_AP_FOUND:
      Serial1.print("WIFI_DISCONNECT_REASON_NO_AP_FOUND");
      break;
    case WIFI_DISCONNECT_REASON_AUTH_FAIL:
      Serial1.print("WIFI_DISCONNECT_REASON_AUTH_FAIL");
      break;
    case WIFI_DISCONNECT_REASON_ASSOC_FAIL:
      Serial1.print("WIFI_DISCONNECT_REASON_ASSOC_FAIL");
      break;
    case WIFI_DISCONNECT_REASON_HANDSHAKE_TIMEOUT:
      Serial1.print("WIFI_DISCONNECT_REASON_HANDSHAKE_TIMEOUT");
      break;
    default:
      DEBUG_PRINTF("reason value %d\n\r", reason);
      break;
  };
  #endif
}

void printWifiStatus (uint8_t status){
#if MY_DEBUG
  switch (status) {
  case WL_NO_SHIELD:
    Serial1.println("WL_NO_SHIELD");
    break;
  case WL_IDLE_STATUS:
    Serial1.println("WL_IDLE_STATUS");
    break;
  case WL_NO_SSID_AVAIL:
    Serial1.println("WL_NO_SSID_AVAIL");
    break;
  case WL_SCAN_COMPLETED:
    Serial1.println("WL_SCAN_COMPLETED");
    break;
  case WL_CONNECTED:
    Serial1.println("WL_CONNECTED");
    break;
  case WL_CONNECT_FAILED:
    Serial1.println("WL_CONNECT_FAILED");
    break;
  case WL_CONNECTION_LOST:
    Serial1.println("WL_CONNECTION_LOST");
    break;
  case WL_DISCONNECTED:
    Serial1.println("WL_DISCONNECTED");
    break;
  default:
    Serial1.printf("wifi error %d\n\r",status);
    break;
  }
#endif
}

void printMqttDisconnectReason (AsyncMqttClientDisconnectReason reason){
  switch (reason) {
  case AsyncMqttClientDisconnectReason::TCP_DISCONNECTED:
    Serial1.println("TCP_DISCONNECTED");
    break;
  case AsyncMqttClientDisconnectReason::MQTT_UNACCEPTABLE_PROTOCOL_VERSION:
    Serial1.println("MQTT_UNACCEPTABLE_PROTOCOL_VERSION");
    break;
  case AsyncMqttClientDisconnectReason::MQTT_IDENTIFIER_REJECTED:
    Serial1.println("MQTT_IDENTIFIER_REJECTED");
    break;
  case AsyncMqttClientDisconnectReason::MQTT_SERVER_UNAVAILABLE:
    Serial1.println("MQTT_SERVER_UNAVAILABLE");
    break;
  case AsyncMqttClientDisconnectReason::MQTT_MALFORMED_CREDENTIALS:
    Serial1.println("MQTT_MALFORMED_CREDENTIALS");
    break;
  case AsyncMqttClientDisconnectReason::MQTT_NOT_AUTHORIZED:
    Serial1.println("MQTT_NOT_AUTHORIZED");
    break;
  case AsyncMqttClientDisconnectReason::ESP8266_NOT_ENOUGH_SPACE:
    Serial1.println("ESP8266_NOT_ENOUGH_SPACE");
    break;
  case AsyncMqttClientDisconnectReason::TLS_BAD_FINGERPRINT:
    Serial1.println("TLS_BAD_FINGERPRINT");
    break;
  default:
    Serial1.printf("MQTT error %d\n\r",reason);
    break;
  }
}
