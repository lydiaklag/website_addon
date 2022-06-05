#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "esp_system.h"
// #include "esp_heap_alloc_caps.h
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
// #include "freertos/heap_regions.h"
#include <Wire.h>
#include <WiFi.h>
#include <SPI.h>
#include <ADXL362.h>
#include "max30105.h"
#include "IIRFilter.h"
#include <Time.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

#define WIFI_NETWORK "secret passage_plus" // the name of the wifi network //at lab it is ESP_Test
#define WIFI_PASSWORD "Afra!17p89#"        // at lab it is esp8266_test
#define WIFI_TIMEOUT_MS 20000
#define WIFI_SSID "secret passage_plus"


#define API_KEY "AIzaSyAIE_5ozQRoAcZaprySgTDVu_YB7QJycik" //we call this FIREBASE_Authorization_Key
#define DATABASE_URL "https://accel-42dfb-default-rtdb.europe-west1.firebasedatabase.app/" //we call that FIREBASE_HOST in this code

// FirebaseData firebaseData;
// FirebaseJson json;
ADXL362 xl;
int16_t XValue, YValue, ZValue, Temperature;

// void setup() {
//   Serial.begin(115200);
//   xl.begin(5);
//   xl.beginMeasure();
//   WiFi.begin (WIFI_SSID, WIFI_PASSWORD);
//   Serial.print("Connecting...");
//   while (WiFi.status() != WL_CONNECTED)
//   {
//     Serial.print(".");
//     delay(300);
//   }
//   Serial.println();
//   Serial.print("IP Address: ");
//   Serial.println(WiFi.localIP());
//   Serial.println();
//   Firebase.begin(DATABASE_URL,API_KEY);
  
// }

// void loop() {
//   // float hum = dht.readHumidity();
//   // float temp = dht.readTemperature();  
//   XValue = xl.readXData();
//   YValue = xl.readYData();
//   ZValue = xl.readZData();
//   // if (isnan(hum) || isnan(temp)  ){
//   //   Serial.println(F("Failed to read from DHT sensor!"));
//   //   return;
//   // }

//   Serial.print("X: ");
//   Serial.print(XValue);
//   Serial.print("m/s2");
//   Serial.print(" Y: ");
//   Serial.print(YValue);
//   Serial.print("m/s2");
//   Serial.println();

//   Firebase.setFloat(firebaseData, "/ESP32_APP/TEMPERATURE", XValue);
//   Firebase.setFloat(firebaseData, "/ESP32_APP/HUMIDITY", YValue);
//   delay(200);
// } //this whole code above is for an app https://microcontrollerslab.com/esp32-send-sensor-data-google-firebase-display-android-app/
//now I will quickly implement the website (simpler)
//i will only add some lines in the setup function
#ifdef ESP32
  #include <WiFi.h>
  #include <ESPAsyncWebServer.h>
  #include <SPIFFS.h>
#else
  #include <Arduino.h>
  // #include <ESP8266WiFi.h>
  // #include <Hash.h>
  // #include <ESPAsyncTCP.h>
  #include <ESPAsyncWebServer.h>
  #include <FS.h>
#endif
#include <Wire.h>
#include <Adafruit_Sensor.h>

AsyncWebServer server(80);

String read_Z() {
  // float temp = xl.readZData();
  // if (isnan(temp)) {    
  //   Serial.println("Failed to read from BME280 sensor!");
  //   return "";
  // }
  // else {
    // int16_t XValue, YValue, ZValue, Temperature;
  xl.readXYZTData(XValue, YValue, ZValue, Temperature);
  // float(ZValue);
  Serial.print("\nZ= ");
  Serial.println(ZValue);
  return String(ZValue);
  // }
}

String read_Y() {
  float hum = xl.readYData();
  if (isnan(hum)) {
    Serial.println("Failed to read from BME280 sensor!");
    return "";
  }
  else {
    Serial.println(hum);
    return String(hum);
  }
}

String read_X() {
  float pres = xl.readXData();
  if (isnan(pres)) {
    Serial.println("Failed to read from BME280 sensor!");
    return "";
  }
  else {
    Serial.println(pres);
    return String(pres);
  }
}

void setup(){
  Serial.begin(9600);
  
  // bool status; 
  // status = xl.begin(5);
  xl.begin(5);  
  xl.beginMeasure();
  // if (!(xl.begin(5))) {
  //   Serial.println("BME280 not connected properly. Check circuit!");
  //   while (1);
  // }

  if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  SPIFFS.begin();
  // WiFi.mode(WIFI_STA); //sets it to station mode
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("IP Address: ");
  Serial.println(WiFi.localIP());

  // for (int i = 0; i<20;i++){
  //   // read_Z();
  //   // read_X();
  //   // read_Y();
  //   Serial.println(xl.readXData());
  //   Serial.println(xl.readYData());
  //   Serial.println(xl.readZData());
  //   delay(1000);
  // }
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/Index.html");
  });
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", read_Z().c_str());
  });
  server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", read_Y().c_str());
  });
  server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", read_X().c_str());
  });

  server.begin();
 ////// the following lines of code are in order to check if a file from folder data has been uploaded to ESP32 via SPIFFS
 // it works
 // steps: put file in folder <data> 
 // 2 : go to platform IO, click on it
  // 3: click on PLATFORM
  //4: click on Build Filesystem
  //5: click on Upload Fileststem
  // that's it yayyyy
  //yayy 

  // if(!SPIFFS.begin(true)){
  //   Serial.println("An Error has occurred while mounting SPIFFS");
  //   return;
  // }
  
  // File file = SPIFFS.open("/Index.html");
  // if(!file){
  //   Serial.println("Failed to open file for reading");
  //   return;
  // }
  
  // Serial.println("File Content:");
  // while(file.available()){
  //   Serial.write(file.read());
  // }
  // file.close();
  //end of section

}

void loop(){}