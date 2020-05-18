/*
    This project is for create a simple WebSocket Gateway between UART and Wifi
    
    Heavely inspire by : https://github.com/tttapa/ESP8266/tree/master/Examples/14.%20WebSocket/A-WebSocket_LED_control

    Copyright (C) 2020 Alban Ponche
    https://github.com/warrenberberd/UART2WiFi

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <Arduino.h>

#include <ArduinoOTA.h>

#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#elif defined(ESP32)
#include <WiFi.h>
#include <WebServer.h>
#else
#error Invalid platform
#endif

#include <FS.h>
#include <WebSocketsServer.h>

#include "WiFiCred.h"

ESP8266WiFiMulti wifiMulti;       // Create an instance of the ESP8266WiFiMulti class, called 'wifiMulti'

ESP8266WebServer server(80);       // Create a webserver object that listens for HTTP request on port 80
WebSocketsServer webSocket(81);    // create a websocket server on port 81

File fsUploadFile;                 // a File variable to temporarily store the received file

#define ESP_NAME "GPSDO_ESP8266"

const char* mdnsName = ESP_NAME; // Domain name for the mDNS responder

const char *ssid = ESP_NAME; // The name of the Wi-Fi network that will be created
const char *password = WIFI_PASS;   // The password required to connect to it, leave blank for an open network

const char *OTAName = ESP_NAME;           // A name and a password for the OTA service
const char *OTAPassword = WIFI_PASS;



/*  For MQTT Publish   */
#define ENABLE_MQTT true
#ifdef ENABLE_MQTT
  bool MQTT_ENABLED=true;
  #include "MQTTCred.h"
  WiFiClient client;
  #include "Adafruit_MQTT.h"
  #include "Adafruit_MQTT_Client.h"
  #define MQTT_FEED "devices/GPSDO_ESP8266/UART/RX"
  Adafruit_MQTT_Client mqtt(&client,MQTT_SERVER,MQTT_PORT,MQTT_USER,MQTT_PASS);
  Adafruit_MQTT_Publish feed = Adafruit_MQTT_Publish(&mqtt, MQTT_FEED);

  uint lastMQTTKeepAlive=0;
#else
  bool MQTT_ENABLED=false;
#endif

#define DISCONNECTED  -1
#define LED_PIN   LED_BUILTIN
//#define LED_PIN   DISCONNECTED

/*__________________________________________________________HELPER_FUNCTIONS__________________________________________________________*/

String formatBytes(size_t bytes) { // convert sizes in bytes to KB and MB
  if (bytes < 1024) {
    return String(bytes) + "B";
  } else if (bytes < (1024 * 1024)) {
    return String(bytes / 1024.0) + "KB";
  } else if (bytes < (1024 * 1024 * 1024)) {
    return String(bytes / 1024.0 / 1024.0) + "MB";
  }
}

String getContentType(String filename) { // determine the filetype of a given filename, based on the extension
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

// Set time via NTP, as required for x.509 validation
time_t setClock(){
  configTime(3 * 3600, 0, "0.fr.pool.ntp.org", "1.fr.pool.ntp.org", "2.fr.pool.ntp.org");

  #ifdef DEBUG
    Serial.print("Waiting for NTP time sync: ");
  #endif
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    #ifdef DEBUG
      Serial.print(".");
    #endif
    now = time(nullptr);
  }
  #ifdef DEBUG
    Serial.println("");
  #endif
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  #ifdef DEBUG
    Serial.print("Current time: ");
    Serial.print(asctime(&timeinfo));
  #endif
  return now;
}

String getStrTimestamp(){
  // Getting time from NTP
  time_t now=setClock();

  // Formatting timestamp
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);

  String strTimestamp=String(asctime(&timeinfo));strTimestamp.trim();

  return strTimestamp;
}

/*__________________________________________________________SERVER_HANDLERS__________________________________________________________*/
bool handleFileRead(String path) { // send the right file to the client (if it exists)
  #ifdef DEBUG
    Serial.println("handleFileRead: " + path);
  #endif

  if (path.endsWith("/")) path += "index.html";          // If a folder is requested, send the index file
  String contentType = getContentType(path);             // Get the MIME type
  String pathWithGz = path + ".gz";

  if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)) { // If the file exists, either as a compressed archive, or normal
    if (SPIFFS.exists(pathWithGz))                         // If there's a compressed version available
      path += ".gz";                                         // Use the compressed verion
    File file = SPIFFS.open(path, "r");                    // Open the file
    size_t sent = server.streamFile(file, contentType);    // Send it to the client
    file.close();                                          // Close the file again

    #ifdef DEBUG
      Serial.println(String("\tSent file: ") + path);
    #endif

    return true;
  }
  #ifdef DEBUG
    Serial.println(String("\tFile Not Found: ") + path);   // If the file doesn't exist, return false
  #endif
  return false;
}

void handleNotFound(){ // if the requested file or page doesn't exist, return a 404 not found error
  if(!handleFileRead(server.uri())){          // check if the file exists in the flash memory (SPIFFS), if so, send it
    server.send(404, "text/plain", "404: File Not Found");
  }
}

void handleFileUpload(){ // upload a new file to the SPIFFS
  HTTPUpload& upload = server.upload();
  String path;
  if(upload.status == UPLOAD_FILE_START){
    path = upload.filename;
    if(!path.startsWith("/")) path = "/"+path;
    if(!path.endsWith(".gz")) {                          // The file server always prefers a compressed version of a file 
      String pathWithGz = path+".gz";                    // So if an uploaded file is not compressed, the existing compressed
      if(SPIFFS.exists(pathWithGz))                      // version of that file must be deleted (if it exists)
         SPIFFS.remove(pathWithGz);
    }
    #ifdef DEBUG
    Serial.print("handleFileUpload Name: "); Serial.println(path);
    #endif
    fsUploadFile = SPIFFS.open(path, "w");            // Open the file for writing in SPIFFS (create if it doesn't exist)
    path = String();
  } else if(upload.status == UPLOAD_FILE_WRITE){
    if(fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize); // Write the received bytes to the file
  } else if(upload.status == UPLOAD_FILE_END){
    if(fsUploadFile) {                                    // If the file was successfully created
      fsUploadFile.close();                               // Close the file again
      #ifdef DEBUG
      Serial.print("handleFileUpload Size: "); Serial.println(upload.totalSize);
      #endif
      server.sendHeader("Location","/success.html");      // Redirect the client to the success page
      server.send(303);
    } else {
      server.send(500, "text/plain", "500: couldn't create file");
    }
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) { // When a WebSocket message is received
  switch (type) {
    case WStype_DISCONNECTED:             // if the websocket is disconnected
      #ifdef DEBUG
        Serial.printf("[%u] Disconnected!\n", num);
      #endif
      break;
    case WStype_CONNECTED: {              // if a new websocket connection is established
        IPAddress ip = webSocket.remoteIP(num);
        #ifdef DEBUG
          Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
        #endif

      }
      break;
    case WStype_TEXT:                     // if new text data is received
      #ifdef DEBUG
        Serial.printf("[%u] get Text: %s\n", num, payload);
      #endif

      break;
    case WStype_BIN:
      #ifdef DEBUG
        hexdump(payload, lenght);
      #endif
      // echo data back to browser
      webSocket.sendBIN(num, payload, lenght);
      break;
  }
}


/*__________________________________________________________SETUP_FUNCTIONS__________________________________________________________*/
void startWiFi() { // Start a Wi-Fi access point, and try to connect to some given access points. Then wait for either an AP or STA connection
  WiFi.softAP(ssid, password);             // Start the access point
  #ifdef DEBUG
  Serial.print("Access Point \"");
  Serial.print(ssid);
  Serial.println("\" started\r\n");
  #endif

  wifiMulti.addAP(WIFI_SSID, WIFI_PASS);   // add Wi-Fi networks you want to connect to
  //wifiMulti.addAP("ssid_from_AP_2", "your_password_for_AP_2");
  //wifiMulti.addAP("ssid_from_AP_3", "your_password_for_AP_3");
  #ifdef DEBUG
    Serial.println("Connecting");
  #endif
  while (wifiMulti.run() != WL_CONNECTED && WiFi.softAPgetStationNum() < 1) {  // Wait for the Wi-Fi to connect
    delay(250);
    #ifdef DEBUG
    Serial.print('.');
    #endif
  }
  #ifdef DEBUG
    Serial.println("\r\n");
  #endif

  if(WiFi.softAPgetStationNum() == 0) {      // If the ESP is connected to an AP
    #ifdef DEBUG
      Serial.print("Connected to ");
      Serial.println(WiFi.SSID());             // Tell us what network we're connected to
      Serial.print("IP address:\t");
      Serial.print(WiFi.localIP());            // Send the IP address of the ESP8266 to the computer
    #endif

    if(LED_PIN>DISCONNECTED) digitalWrite(LED_PIN,LOW); // LED ON
  } else {                                   // If a station is connected to the ESP SoftAP
    #ifdef DEBUG
      Serial.print("Station connected to ESP8266 AP");
    #endif
  }
  Serial.println("\r\n");
}

void startOTA() { // Start the OTA service
  ArduinoOTA.setHostname(OTAName);
  ArduinoOTA.setPassword(OTAPassword);

  ArduinoOTA.onStart([]() {
    #ifdef DEBUG
      Serial.println("Start");
    #endif
    //digitalWrite(LED_RED, 0);    // turn off the LEDs
    //digitalWrite(LED_GREEN, 0);
    //digitalWrite(LED_BLUE, 0);
  });

  ArduinoOTA.onEnd([]() {
    #ifdef DEBUG
      Serial.println("\r\nEnd");
    #endif
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    #ifdef DEBUG
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    #endif
  });

  ArduinoOTA.onError([](ota_error_t error) {
    #ifdef DEBUG
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    #endif
  });

  ArduinoOTA.begin();
  #ifdef DEBUG
    Serial.println("OTA ready\r\n");
  #endif
}

void startSPIFFS() { // Start the SPIFFS and list all contents
  SPIFFS.begin();                             // Start the SPI Flash File System (SPIFFS)
  #ifdef DEBUG
    Serial.println("SPIFFS started. Contents:");
  #endif
  {
    Dir dir = SPIFFS.openDir("/");
    while (dir.next()) {                      // List the file system contents
      String fileName = dir.fileName();
      size_t fileSize = dir.fileSize();
      #ifdef DEBUG
        Serial.printf("\tFS File: %s, size: %s\r\n", fileName.c_str(), formatBytes(fileSize).c_str());
      #endif
    }
    #ifdef DEBUG
      Serial.printf("\n");
    #endif
  }
}

void startWebSocket() { // Start a WebSocket server
  webSocket.begin();                          // start the websocket server
  webSocket.onEvent(webSocketEvent);          // if there's an incomming websocket message, go to function 'webSocketEvent'
  #ifdef DEBUG
    Serial.println("WebSocket server started on port : 81.");
  #endif
}

void startMDNS() { // Start the mDNS responder
  MDNS.begin(mdnsName);                        // start the multicast domain name server
  #ifdef DEBUG
    Serial.print("mDNS responder started: http://");
    Serial.print(mdnsName);
    Serial.println(".local");
  #endif
}

void startServer() { // Start a HTTP server with a file read handler and an upload handler
  server.on("/edit.html",  HTTP_POST, []() {  // If a POST request is sent to the /edit.html address,
    server.send(200, "text/plain", ""); 
  }, handleFileUpload);                       // go to 'handleFileUpload'

  server.onNotFound(handleNotFound);          // if someone requests any other file or page, go to function 'handleNotFound'
                                              // and check if the file exists

  server.begin();                             // start the HTTP server
  #ifdef DEBUG
    Serial.println("HTTP server started.");
  #endif
}

// To send data from ESP to CLIENTS
bool sendDataToWebSocketsClients(String payload){
  if(payload.isEmpty()) return false;
  if(webSocket.connectedClients()<1){
    #ifdef DEBUG
      Serial.println("No client connected to transmit data");
    #endif
    return false;
  }

  #ifdef DEBUG
    Serial.println("Sending to WebSocket : " + payload);
  #endif
  // We send data to ALL connected clients
  for(uint8_t clientNum=0;clientNum<webSocket.connectedClients();clientNum++){
    if(!webSocket.sendTXT(clientNum,payload)){
      #ifdef DEBUG
        Serial.printf("Sending to client %i failed !\n",clientNum);
      #endif
      continue;
    }

    #ifdef DEBUG
      Serial.println("Data transmitted to client " + clientNum);
    #endif
  }

  return true;
}

// Read input data from RX UART
String readDataFromUART(){
  if(Serial.available()<1) return "";

  #ifdef DEBUG
    Serial.println("Data available on UART RX !");
  #endif

  uint start=millis();
  String out;
  while(Serial.available() && millis()<start+1000){
    /* char c=(char)Serial.read(); // Read char by char
    out+=c;
    // If char is newline, add timestamp
    if(c=='\n') out+=getStrTimestamp() + "  "; */
    out+=getStrTimestamp() + "  " + Serial.readStringUntil('\n');
  }

  #ifdef DEBUG
    Serial.println(out);
  #endif
  
  return out;
}

#ifdef ENABLE_MQTT
  // Function to connect and reconnect as necessary to the MQTT server.
  // Should be called in the loop function and it will take care if connecting.
  bool MQTT_connect() {
    if(!MQTT_ENABLED) return false;

    int8_t ret;

    // Stop if already connected.
    if (mqtt.connected()) return true;

    #ifdef DEBUG
      Serial.print(F("Connecting to MQTT... "));
    #endif

    uint8_t retries = 3;
    while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
      #ifdef DEBUG
        Serial.println(mqtt.connectErrorString(ret));
        Serial.println(F("Retrying MQTT connection in 5 seconds..."));
      #endif

      mqtt.disconnect();
      delay(1000);  // wait 1 seconds
      retries--;

      if (retries == 0) return false;
    }

    #ifdef DEBUG
      Serial.println(F("MQTT Connected!"));
    #endif

    return true;
  }

  bool MQTT_send(String data){
    if(!MQTT_ENABLED) return false;

    // KeepAlive each 3 seconds
    if(lastMQTTKeepAlive<millis()-3000){
      lastMQTTKeepAlive=millis();
      if(!mqtt.ping()) mqtt.disconnect();  // Keep Alive
    }

    if(data.isEmpty()) return false;

    if(!MQTT_connect()) return false;

    if (!feed.publish(data.c_str())){
      #ifdef DEBUG
        Serial.println(F("Publish Failed."));
      #endif

      return false;
    }

    #ifdef DEBUG
      Serial.println(F("Publish Success!"));
    #endif

    return true;
  }
#endif

/*__________________________________________________________SETUP__________________________________________________________*/
void setup() {
  delay(10);
  
  // use LED as indicator
  if(LED_PIN>DISCONNECTED) pinMode(LED_PIN,OUTPUT);
  if(LED_PIN>DISCONNECTED) digitalWrite(LED_PIN,LOW); // LED ON


  Serial.begin(9600);

  #ifdef DEBUG
    while (!Serial) ; // wait for Arduino Serial Monitor to open
    Serial.setDebugOutput(false);

    Serial.println("");
  #endif

  delay(1000);

  if(LED_PIN>DISCONNECTED) digitalWrite(LED_PIN,HIGH); // LED OFF
  

  startWiFi();                 // Start a Wi-Fi access point, and try to connect to some given access points. Then wait for either an AP or STA connection
  
  startOTA();                  // Start the OTA service
  
  startSPIFFS();               // Start the SPIFFS and list all contents

  startWebSocket();            // Start a WebSocket server
  
  startMDNS();                 // Start the mDNS responder

  startServer();               // Start a HTTP server with a file read handler and an upload handler

  setClock();

  #ifdef ENABLE_MQTT
    MQTT_connect();
  #endif
}

/*__________________________________________________________LOOP__________________________________________________________*/
void loop() {
  webSocket.loop();                           // constantly check for websocket events
  server.handleClient();                      // run the server
  ArduinoOTA.handle();                        // listen for OTA events
  
  String intputString=readDataFromUART();

  #ifdef ENABLE_MQTT
    MQTT_send(intputString);
  #endif

  sendDataToWebSocketsClients(intputString);

  optimistic_yield(1000);
}