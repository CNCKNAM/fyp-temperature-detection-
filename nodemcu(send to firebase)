#include<SoftwareSerial.h>
#include <ArduinoJson.h>
#include "FirebaseESP8266.h"  // Install Firebase ESP8266 library
#include <ESP8266WiFi.h>  // Install DHT11 Library and Adafruit Unified Sensor Librar
#include <WiFiUdp.h>
#include <NTPClient.h>

SoftwareSerial mySUART(4, 5);  //D2, D1 = SRX, STX

#define FIREBASE_HOST "login2-aa49e-default-rtdb.firebaseio.com/" //Without http:// or https:// schemes
#define FIREBASE_AUTH "ni7c5jPwgcwgH9GUj2wH4LynW3JRvyzUWkGoIbtk"
#define WIFI_SSID "TP-Link_673B"
#define WIFI_PASSWORD "05083114"

//Define FirebaseESP8266 data object
FirebaseData firebaseData;

FirebaseJson json;
  FirebaseJson pixelarray;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

void setup() {
    timeClient.begin();
     timeClient.setTimeOffset(28800);
  // put your setup code here, to run once:
  Serial.begin(115200);
  mySUART.begin(115200);

   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

}

void loop() //mysuarttext = 
{
 float maxtemp = -99;
 timeClient.update();
 unsigned long epochTime = timeClient.getEpochTime();
 struct tm *ptm = gmtime ((time_t *)&epochTime);
 int monthDay = ptm->tm_mday;
 int currentMonth = ptm->tm_mon+1;
 int currentYear = ptm->tm_year+1900;
 String currentDate = String(monthDay) + "/" + String(currentMonth) + "/" + String(currentYear);
 String formattedTime = timeClient.getFormattedTime();
 
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, mySUART);
  if (error) {
  Serial.print(F("deserializeJson() failed: "));
  Serial.println(error.c_str());
  return;
  }
  Serial.println("JSON received and parsed");
  serializeJsonPretty(doc,Serial);

  for (int i = 0; i < 64;i++){
     String pixel = doc[i];
     pixelarray.set("pixel/"+String(i),doc[i].as<String>());
     if (maxtemp < doc[i])
     {
      maxtemp = doc[i];
      }
  }
  pixelarray.set("time",formattedTime);
  pixelarray.set("date", currentDate );
  
json.set("date", currentDate );
json.set("temp", String(maxtemp));
json.set("time", formattedTime);


if (Firebase.pushJSON(firebaseData, "/Visitors", json)) {

  Serial.println(firebaseData.dataPath());

  Serial.println(firebaseData.pushName());

  Serial.println(firebaseData.dataPath() + "/"+ firebaseData.pushName());

} else {
  Serial.println(firebaseData.errorReason());
}

//set pixel
if (Firebase.setJSON(firebaseData, "/realtime", pixelarray))
{

  Serial.println(firebaseData.dataPath());

  Serial.println(firebaseData.pushName());

  Serial.println(firebaseData.dataPath() + "/"+ firebaseData.pushName());

} else {
  Serial.println(firebaseData.errorReason());
}



}
