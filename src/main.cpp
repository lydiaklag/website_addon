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

// #define WIFI_NETWORK "secret passage_plus" // the name of the wifi network //at lab it is ESP_Test
// #define WIFI_PASSWORD "Afra!17p89#"        // at lab it is esp8266_test
// #define WIFI_TIMEOUT_MS 20000
// #define WIFI_SSID "secret passage_plus"

// #define WIFI_NETWORK "secret passage" // the name of the wifi network //at lab it is ESP_Test
// #define WIFI_PASSWORD "Afra!17p89#"        // at lab it is esp8266_test
// #define WIFI_TIMEOUT_MS 20000
// #define WIFI_SSID "secret passage"

#define WIFI_NETWORK "YellowCafe" // the name of the wifi network //at lab it is ESP_Test
#define WIFI_PASSWORD "Colombia1"        // at lab it is esp8266_test
#define WIFI_TIMEOUT_MS 20000
#define WIFI_SSID "YellowCafe"

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
//declaration of functions
String read_Z();
String read_Y();
String read_X();
String read_HR_data();
String read_SPo2_data();
int readSamples();
void ComputeHeartRate();
void convert_signal_to_positive(int p1, int p2, int *x, int point);
int find_min_negative(int p1, int p2, int *x);
int Find_Mean(int p1, int p2, int *x);
int Find_Peak(int p1, int p2, int *x);
void iir_ma_filter_for_hr();
void loopHR();
void ComputeSpO2(void * pvParameter);
void setup();
void loop();

AsyncWebServer server(80);
MAX30105 Sensor;
int samp_freq = 25;          // for each led
const int Num_Samples = 100; // it stores 4 sec
uint32_t gr_buffer[Num_Samples];
int filtered_gr_buffer[Num_Samples];
int ma_gr_buffer[Num_Samples];
const int points_pr = 4;
float PR[points_pr];
float Pulse_Rate_next = 0, Pulse_Rate_previous = 70;
int HR;
//***Butterworth band-pass (0.5Hz-5Hz) 2nd order
const double b[] = {0.175087643672101, 0, -0.350175287344202, 0, 0.175087643672101};
const double a[] = {1, -2.299055356038497, 1.967497759984451, -0.874805556449481, 0.219653983913695};
IIRFilter f(b, a);
double filtered_gr = 0;
int Moving_Average_Num = 2;
int Num_Points = 2 * Moving_Average_Num + 1; //***5-point moving average filter
int Sum_Points;
void *ax;
int Sampling_Time = 2400; // 2400ms = 4s
int flag_unplugged = 1;   // 0 means unplugged
//next section is for SpO2
const int points_spo2 = 4; //I need to increase that point, then the SpO2 task will last longer
double SpO2_dc_ir[points_spo2]; //no need to initialise the arrays, that is done inside the SpO2 func
double SpO2_ac_ir[points_spo2];
double SpO2_dc_red[points_spo2];
double SpO2_ac_red[points_spo2];
double Oxy[points_spo2]; 
int ma_ir2_buffer[Num_Samples];
int ma_red_buffer[Num_Samples];
double SpO2_next; //maybe double
double SpO2_previous = 99;
double SpO2;
void *aaa;


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

String read_HR_data() {
  Serial.println(HR);
  return String(HR);
}

String read_SPo2_data() {
  Serial.println(SpO2);
  return String(SpO2);
}

void loopHR(){
  double t_HR = millis();
  Serial.print("t_HR: ");    Serial.print(t_HR);    Serial.println();
  int flag = readSamples();
    if (flag)
    { //***if sensor reads real data
      iir_ma_filter_for_hr();
      int neg = find_min_negative(Moving_Average_Num, Num_Samples - Moving_Average_Num, ma_gr_buffer);
      convert_signal_to_positive(Moving_Average_Num, Num_Samples - Moving_Average_Num, ma_gr_buffer, neg);
      ComputeHeartRate();
      Serial.print("NEW DATA--> ");
      Serial.print("HR: ");
      Serial.print(HR);
      // flag_unplugged = 1;
    }
    else
    { //***else if sensor is unplugged
      // flag_unplugged = 0;
      HR = 0;
      Serial.print("HR : finger unplugged");
    }
    Serial.print("\nthe HR task took to execute: ms : "); Serial.print(millis() - t_HR); Serial.println();
    Serial.println();
    Serial.println("----------------------------------------------------------------------------------------------------");
    // return String(HR);
}

void iir_ma_filter_for_hr()
{

  for (int i = 0; i < Num_Samples; i++)
  {

    filtered_gr = f.filter(double(gr_buffer[i]));
    filtered_gr_buffer[i] = round(filtered_gr);
  }

  for (int i = Moving_Average_Num; i < Num_Samples - Moving_Average_Num; i++)
  {
    Sum_Points = 0;
    for (int k = 0; k < Num_Points; k++)
    {
      Sum_Points = Sum_Points + filtered_gr_buffer[i - Moving_Average_Num + k];
    }
    ma_gr_buffer[i] = Sum_Points / Num_Points;
  }
}

int Find_Peak(int p1, int p2, int *x)
{
  int Peak_Magnitude = 0;
  for (int m = p1; m < p2; m++)
  {
    if (Peak_Magnitude < x[m])
    {
      Peak_Magnitude = x[m];
    }
  }
  return Peak_Magnitude;
}

int Find_Mean(int p1, int p2, int *x)
{
  int Mean = 0;
  int c = 0;
  for (int m = p1; m < p2; m++)
  {
    if (x[m] > 0)
    {
      Mean = Mean + x[m];
      c++;
    }
  }
  if (c > 0)
  {
    return Mean / c;
  }
  else
  {
    return 1;
  }
}

int find_min_negative(int p1, int p2, int *x)
{
  int min_magnitude = 0;
  for (int m = p1; m < p2; m++)
  {
    if (min_magnitude > x[m])
    {
      min_magnitude = x[m];
    }
  }
  return min_magnitude;
}

void convert_signal_to_positive(int p1, int p2, int *x, int point)
{
  for (int m = p1; m < p2; m++)
  {
    x[m] = x[m] - point;
  }
}

void ComputeHeartRate()
{

  int Mean_Magnitude = Find_Mean(Moving_Average_Num, Num_Samples - Moving_Average_Num, ma_gr_buffer);

  //***detect successive peaks and compute PR
  for (int i = 0; i < points_pr; i++)
  {
    PR[i] = 0;
  }
  int Peak = 0;
  int Index = 0;
  int p = 0;

  for (int j = Moving_Average_Num; j < Num_Samples - Moving_Average_Num; j++)
  {
    //***Find first peak

    if (ma_gr_buffer[j] > ma_gr_buffer[j - 1] && ma_gr_buffer[j] > ma_gr_buffer[j + 1] && ma_gr_buffer[j] > Mean_Magnitude && Peak == 0)
    {
      Peak = ma_gr_buffer[j];
      Index = j*40;
    }

    //***Search for next peak

    if (Peak > 0)
    {
      if (ma_gr_buffer[j] > ma_gr_buffer[j - 1] && ma_gr_buffer[j] > ma_gr_buffer[j + 1] && ma_gr_buffer[j] > Mean_Magnitude)
      {
        float d = (j*40) - Index;
        float pulse=(float)60000/d; // bpm for each PEAK interval
        PR[p] = pulse;
        p++;
        p %= points_pr; // Wrap variable
        Peak = ma_gr_buffer[j];
        Index = j*40;
      }
    }
  }

  float sum = 0;
  int c = 0;
  for (int i = 0; i < points_pr; i++)
  {
    if (PR[i] != 0 && PR[i] != INFINITY)
    {
      sum = sum + PR[i];
      c = c + 1;
    }
  }

  if (c != 0)
  {
    Pulse_Rate_next = sum / c;
    if (Pulse_Rate_next > 40 && Pulse_Rate_next < 200)
    {
      if (Pulse_Rate_next - Pulse_Rate_previous >= 5)
      {
        Pulse_Rate_next = Pulse_Rate_previous + 1;
      }
      if (Pulse_Rate_next - Pulse_Rate_previous <= -5)
      {
        Pulse_Rate_next = Pulse_Rate_previous - 1;
      }
      Pulse_Rate_previous = Pulse_Rate_next;
    }
    else
    {
      Pulse_Rate_next = Pulse_Rate_previous;
    }
  }
  else
  {
    Pulse_Rate_next = Pulse_Rate_previous;
  }

  HR = int(round(Pulse_Rate_next));
}

int readSamples()
{
  int flag = 1;
  for (uint32_t i = 0; i < Num_Samples; i++)
  {
    // read max30105
    // gr_buffer[i] = Sensor.getIR();
    // gr_buffer[i] = Sensor.getRed();
    // if (flag_movement){
      // gr_buffer[i] = Sensor.getGreen(); //if there is movement, then take samples using the green light only
    // }
    gr_buffer[i] = Sensor.getIR();
    // else {
    //   gr_buffer[i] = Sensor.getRed();
    //   // should I also include measurements from IR and then take the mean of both to get the final HR
    //   gr_buffer[i] = Sensor.getIR();
    // }
    // Sensor.nextSample();
    if (gr_buffer[i] < 10000)
    {
      flag = 0; //if the flag = 0, this means that the finger is unplugged 
    }
    delay(40);
  }
  return flag;
}

void setup(){
  Serial.begin(9600);
  
  // bool status; 
  // status = xl.begin(5);
  xl.begin(5);  //for accel
  xl.beginMeasure();  //for accel
  // if (!(xl.begin(5))) {
  //   Serial.println("BME280 not connected properly. Check circuit!");
  //   while (1);
  // }
  Sensor.begin();  //for max30105
  byte ledBrightness = 0xDF;                                                             // Options: 0=Off to 255=50mA  --> DF=~44mA
  byte sampleAverage = 8;                                                                // Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3;                                                                      // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 200;                                                                  // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;                                                                  // Options: 69, 118, 215, 411
  int adcRange = 16384;                                                                   // Options: 2048, 4096, 8192, 16384
  Sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Configure sensor with these settings
  Sensor.setPulseAmplitudeRed(0xDF);
  Sensor.setPulseAmplitudeIR(0xDF); //if the value was 0, here we basically turn off the red LED, so green and IR LEDs are active
  Sensor.setPulseAmplitudeGreen(0);


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
  server.on("/Zz", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", read_Z().c_str());
  });
  server.on("/Yy", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", read_Y().c_str());
  });
  server.on("/Xx", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", read_X().c_str());
  });
  server.on("/hr", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", read_HR_data().c_str());
  });
  server.on("/SPo2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", read_SPo2_data().c_str());
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

void ComputeSpO2(void * pvParameter){
   //need to initialise all the variables for SpO2, make them global
  // for(;;){ //we need this huge endless loop, for the FreeRTOS task, requirement
    // xSemaphoreTake( baton, portMAX_DELAY);
    // xSemaphoreGive( baton ); //this is the second line because it is the shortest task (compared to HR)
    double t_SP = millis();
    // Serial.println(); Serial.print("t_SP: "); Serial.print(t_SP); Serial.println();
    if (flag_unplugged = 1) { //calculate the SpO2 only if the finger is plugged, name of the flag is counterintuitive
      int mean_ir = Find_Mean(0, Num_Samples, ma_ir2_buffer);
      int mean_red = Find_Mean(0, Num_Samples, ma_red_buffer);    
      //***detect successive peaks and mins
      for (int i = 0; i < points_spo2; i++){
        SpO2_dc_ir[i] = 0;
        SpO2_ac_ir[i]=0;
        SpO2_dc_red[i] = 0;
        SpO2_ac_red[i]=0;
        Oxy[i]=0;
      }
      int p_ir=0;
      int p_red=0;
      int peak_ir=0;
      int peak_red=0;
      int m_ir=0;
      int m_red=0;
      for (int j = 101; j < 350; j++){
        //***Find peaks and means for IR
        if(ma_ir2_buffer[j] > ma_ir2_buffer[j-1] && ma_ir2_buffer[j] > ma_ir2_buffer[j+1] && ma_ir2_buffer[j] > mean_ir && peak_ir==0){
          // Serial.print("peak  ir: ");
          // Serial.print(ma_ir2_buffer[j]);
          // Serial.println();
          SpO2_dc_ir[p_ir]=ma_ir2_buffer[j]; 
          p_ir++;
          p_ir %= points_spo2; //Wrap variable
          peak_ir=1;
        }
        
        if(ma_ir2_buffer[j] < ma_ir2_buffer[j-1] && ma_ir2_buffer[j] < ma_ir2_buffer[j+1] && ma_ir2_buffer[j] < mean_ir && peak_ir==1){
          // Serial.print("min  ir: ");
          // Serial.print(ma_ir2_buffer[j]);
          // Serial.println();
          SpO2_ac_ir[m_ir]=SpO2_dc_ir[p_ir-1]-ma_ir2_buffer[j]; 
          m_ir++;
          m_ir %= points_spo2; //Wrap variable
          peak_ir=0;
        }

        //***Find peaks and means for RED
        if(ma_red_buffer[j] > ma_red_buffer[j-1] && ma_red_buffer[j] > ma_red_buffer[j+1] && ma_red_buffer[j] > mean_red && peak_red==0){
        
          SpO2_dc_red[p_red]=ma_red_buffer[j]; 
          p_red++;
          p_red %= points_spo2; //Wrap variable
          peak_red=1;
        }

        if(ma_red_buffer[j] < ma_red_buffer[j-1] && ma_red_buffer[j] < ma_red_buffer[j+1] && ma_red_buffer[j] < mean_red && peak_red==1){
        
          SpO2_ac_red[m_red]=SpO2_dc_red[p_red-1]-ma_red_buffer[j]; 
          m_red++;
          m_red %= points_spo2; //Wrap variable
          peak_red=0;
        }
      }
      


      for (int i=1; i< points_spo2; i++){

        if (SpO2_ac_ir[i]!=0 && SpO2_ac_red[i]!=0){
          float R =(float(SpO2_ac_red[i])/float(SpO2_dc_red[i]))/(float(SpO2_ac_ir[i])/float(SpO2_dc_ir[i]));
          Oxy[i]=-45.060*R*R + 30.354*R + 94.845;
          // Serial.print("oxy: ");
          // Serial.print(Oxy[i]);
          // Serial.println();
        }   
            
      }

      float sumSP=0;
      int cSP=0;
      
      for (int i=1; i< points_spo2; i++){

        if (Oxy[i]>84 && Oxy[i]<=100){
          sumSP=sumSP+Oxy[i];
          cSP=cSP+1;
        }   
      }
    
      if(cSP>0){
        SpO2_next=sumSP/cSP;
      }
      
      // Serial.print("SpO2 predicted: ");
      // Serial.print(SpO2_next);
      // Serial.println();

      if(SpO2_next > 84 && SpO2_next <= 100){  
        if (SpO2_next-SpO2_previous<-4){
          SpO2_next=SpO2_previous;
        }
        if (SpO2_next-SpO2_previous<-3){
          SpO2_next=SpO2_previous-1;
        }
        SpO2_previous=SpO2_next;
      }else{
        SpO2_next=SpO2_previous;
      }
      
      SpO2=int(round(SpO2_next));
      if (flag_unplugged == 0 || HR == 0){ //if unplugged show this 
        //SpO2 = 0; //this is probably not needed
        Serial.println();
        Serial.println("SpO2 : unplugged");
        SpO2 = 0;
      }else {
        Serial.println();
        Serial.print("SpO2 : ");
        Serial.print(SpO2);
        Serial.print(" %");
        Serial.println();
      }
      // Serial.println(); Serial.print("time to run the SpO2 task: "); Serial.print(millis() - t_SP); Serial.println(); // I want to measure how long does it take for SpO2 task to run
      // vTaskDelay(Sampling_Time / portTICK_PERIOD_MS);
    }
    // xSemaphoreGive( baton ); //this line not here, as this is the shortest task, we immediately give the semaphore to the other task
    // delay(50);
  // }
}

void loop(){
  loopHR();
  ComputeSpO2(&aaa);
}