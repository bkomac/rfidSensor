#include <Arduino.h>

#include <FS.h>

//#include <Adafruit_INA219.h>
#include <Wire.h>

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>

// needed for library
//#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <Hash.h>
#include <WebSocketsServer.h>

#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

#include <ArduinoJson.h>
#include <DHT.h>

#include <MFRC522.h>
#include <SPI.h>


void fadeIn();
void fadeOut();
void blink();
void blink(int);
void blink(int, int);
void blink(int, int, int);
void readFS();
bool testWifi();
void setupAP();
void sendRequest();
void saveConfig(JsonObject &json);
void mqPublish(String);
void createWebServer();

ESP8266WebServer server(80);

// websocket
WebSocketsServer webSocket = WebSocketsServer(81);

WiFiClient client;
PubSubClient mqClient(client);

// Vcc measurement
ADC_MODE(ADC_VCC);
int lightTreshold = 50; // 0 - dark, >100 - light

// APP
String FIRM_VER = "1.0.0";
String SENSOR = "RFID,DHT22"; // BMP180, HTU21, DHT11

String app_id = "";
float adc;
long startTime;
String espIp;
String apSsid;
String apPass;
int rssi;
String ssid;

// DHT
float humd = NULL;
float temp = NULL;

// RGB
int redPin = 16;   // 13;
int greenPin = 16; // 12;
int bluePin = 16;

int red = 1024;
int green = 1024;
int blue = 500;

String sensorData = "";
String statusText = "";

// CONF
char deviceName[100] = "RFIDsensor";

char essid[40] = "iottest";
char epwd[40] = "esptest123";
String securityToken = "";
String defaultMODE;
String MODE = "AUTO";
int timeOut = 5000;

// mqtt config
char mqttAddress[200] = "";
int mqttPort = 1883;
char mqttUser[20] = "";
char mqttPassword[20] = "";
char mqttPublishTopic[200] = "esp/rfidSensor";
char mqttSuscribeTopic[200] = "esp/rfidSensor";

// REST API CONFIG
char rest_server[40] = "";

boolean rest_ssl = false;
char rest_path[200] = "";
int rest_port = 80;
char api_token[100] = "";
char api_payload[400] = "";

boolean buttonPressed = false;
boolean requestSent = false;
int lastTime = millis();

int BUILTINLED = 2;
int RELEY = 500;
int GPIO_IN = 400;
int BUTTON = 100;

// DHT
#define DHTPIN 3
#define DHTTYPE DHT22 // DHT11
DHT dht(DHTPIN, DHTTYPE);

// RFID RC552
#define RST_PIN 12
#define SS_PIN 14
MFRC522 mfrc522(SS_PIN, RST_PIN);

// MDNS
String mdns = "";

void setup() { //------------------------------------------------
  Serial.begin(115200);
  Serial.println("Setup ...");
  delay(300);

  startTime = millis();
  if (BUTTON < 100)
    pinMode(BUTTON, INPUT);
  pinMode(BUILTINLED, OUTPUT);

  digitalWrite(RELEY, LOW);
  digitalWrite(BUILTINLED, LOW);

  // RGB
  if (redPin < 100 && greenPin < 100 && bluePin < 100) {
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);

    analogWrite(redPin, 1024);
    analogWrite(greenPin, 1024);
    analogWrite(bluePin, 1024);
  }

  blink(1, 500);
  dht.begin();

  app_id = "ESP" + getMac();
  Serial.print(F("**App ID: "));
  Serial.println(app_id);
  app_id.toCharArray(deviceName, 200, 0);

  // auto connect
  WiFi.setAutoConnect(true);

  // clean FS, for testing
  // SPIFFS.format();

  // read config
  readFS();

  Serial.print(F("**Security token: "));
  Serial.println(securityToken);

  apSsid = "Config_" + app_id;
  apPass = "esp12345";

  pinMode(RELEY, OUTPUT);
  pinMode(BUILTINLED, OUTPUT);
  if (GPIO_IN < 100)
    pinMode(GPIO_IN, INPUT);

  if (String(essid) != "" && String(essid) != "nan") {
    Serial.print("SID found. Trying to connect to: ");
    Serial.print(essid);
    Serial.println("");
    WiFi.begin(essid, epwd);
    delay(100);
  }

  if (!testWifi())
    setupAP();
  else {
    if (!MDNS.begin(app_id.c_str())) {
      Serial.println("Error setting up MDNS responder!");
      mdns = "error";
    } else {
      Serial.println("mDNS responder started");
      // Add service to MDNS-SD
      MDNS.addService("http", "tcp", 80);
      mdns = app_id.c_str();
    }
  }

  ssid = WiFi.SSID();
  Serial.print(F("\nconnected to "));
  Serial.print(ssid);
  Serial.print(F(" "));
  Serial.println(rssi);

  Serial.println(" ");
  IPAddress ip = WiFi.localIP();
  espIp = getIP(ip);
  Serial.print(F("local ip: "));
  Serial.println(espIp);
  yield();
  createWebServer();

  // MQTT
  if (String(mqttAddress) != "") {
    Serial.print(F("Seting mqtt server and callback... "));
    mqClient.setServer(mqttAddress, mqttPort);
    mqClient.setCallback(mqCallback);
    mqReconnect();
  }

  // RFID
  SPI.begin();
  mfrc522.PCD_Init();
  mfrc522.PCD_DumpVersionToSerial();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
} //--

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload,
                    size_t length) {

  switch (type) {
  case WStype_DISCONNECTED:
    Serial.printf("[%u] Disconnected!\n", num);
    break;
  case WStype_CONNECTED: {
    IPAddress ip = webSocket.remoteIP(num);
    Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0],
                  ip[1], ip[2], ip[3], payload);

    // send message to client
    webSocket.sendTXT(num, "Connected");
  } break;
  case WStype_TEXT:
    Serial.printf("[%u] get Text: %s\n", num, payload);

    // send message to client
    // webSocket.sendTXT(num, "message here");

    // send data to all connected clients
    // webSocket.broadcastTXT("message here");
    break;
  case WStype_BIN:
    Serial.printf("[%u] get binary length: %u\n", num, length);
    hexdump(payload, length);

    // send message to client
    // webSocket.sendBIN(num, payload, length);
    break;
  }
}

// -----------------------------------------------------------------------------
// loop ------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void loop() {
  delay(10);
  rssi = WiFi.RSSI();

  server.handleClient();

  int inputState = LOW;
  if (GPIO_IN < 100)
    inputState = digitalRead(GPIO_IN);

  int buttonState = HIGH;
  if (BUTTON < 100)
    buttonState = digitalRead(BUTTON);

  adc = ESP.getVcc() / 1000.00;
  // adc = analogRead(A0);

  float humd1 = NULL;
  float temp1 = NULL;

  // DHT
  delay(100);
  humd1 = dht.readHumidity();
  delay(100);
  temp1 = dht.readTemperature();

  if (String(humd1) != "nan" && String(temp1) != "nan" && temp1 != NULL) {
    humd = humd1;
    temp = temp1;

    Serial.print(F("\nTemperature: "));
    Serial.print(temp);
    Serial.print(F("\nHumidity: "));
    Serial.println(humd);

    Serial.print("Sending ws msg...");
    webSocket.broadcastTXT("{\"temp\":" + String(temp, 1) +
                           ", \"hum\":" + String(humd, 1) + "}");
  }

  yield();
  sensorData = "";

  // DHT
  if (humd != NULL && temp != NULL)
    sensorData = "\"temp\":" + String(temp) + ", \"hum\":" + String(humd);

  // RFID
  readRFID(sensorData);

  if (MODE == "AUTO") {
    if (inputState == HIGH) {
      Serial.println(F("Sensor high..."));
      // Serial.println(adc);
      if (adc <= lightTreshold) {
        Serial.print(F("\nLight treshold = "));
        Serial.println(lightTreshold);
        Serial.print(F("\nADC = "));
        Serial.println(adc);
        digitalWrite(RELEY, HIGH);
      }
      digitalWrite(BUILTINLED, LOW);

      buttonPressed = false;

      if (!requestSent) {
        fadeIn();
        blink();
        sendRequest(sensorData);
        requestSent = true;
      }
      lastTime = millis();
    }

    if (millis() > lastTime + timeOut && !buttonPressed && inputState != HIGH) {
      digitalWrite(RELEY, LOW);
      digitalWrite(BUILTINLED, HIGH);
      if (requestSent)
        fadeOut();
      requestSent = false;

      if (WiFi.status() == WL_CONNECTED && digitalRead(RELEY) == HIGH){
        blink(1, 5);
        sendRequest(sensorData);
      }
      lastTime = millis();
    }
  }else{ //MODE=MANUAL

    if (millis() > lastTime + timeOut && !buttonPressed) {
      blink(1, 5);
      sendRequest(sensorData);

      lastTime = millis();
    }
  }

  // button pressed
  if (buttonState == LOW) {
    Serial.println(F("Button pressed..."));
    buttonPressed = true;
    if (digitalRead(RELEY) == HIGH) {
      digitalWrite(RELEY, LOW);
      digitalWrite(BUILTINLED, HIGH);
    } else {
      digitalWrite(RELEY, HIGH);
      digitalWrite(BUILTINLED, LOW);
    }
    delay(300);
  }

  // MQTT client
  if (String(mqttSuscribeTopic) != "")
    mqClient.loop();

  delay(100);
  webSocket.loop();

} //---------------------------------------------------------------

// web server
void createWebServer() {
  Serial.println(F("Starting server..."));
  yield();
  Serial.println(F("REST handlers init..."));

  server.on("/", []() {
    blink();
    String content;

    content = "<!DOCTYPE HTML>\r\n<html>";
    content += "<h1><u>ESP web server</u></h1>";
    content += "<p>IP: " + espIp + "</p>";
    content += "<p>MAC/AppId: " + app_id + "</p>";
    content += "<p>Version: " + FIRM_VER + "</p>";
    content += "<p>ADC: " + String(adc) + "</p>";
    content += "<br><p><b>REST API: </b>";
    content += "<br>GET: <a href='http://" + espIp + "/switch/auto'>http://" +
               espIp + "/switch/auto </a>";
    content += "<br>GET: <a href='http://" + espIp + "/switch/on'>http://" +
               espIp + "/switch/on </a>";
    content += "<br>GET: <a href='http://" + espIp + "/switch/off'>http://" +
               espIp + "/switch/off </a>";
    content += "<br>GET: <a href='http://" + espIp + "/status'>http://" +
               espIp + "/status </a>";
    content += "<br>GET: <a href='http://" + espIp + "/update'>http://" +
               espIp + "/update </a>";
    content += "<br>POST: <a href='http://" + espIp + "/update'>http://" +
               espIp + "/update </a>";
    content += "<br>GET: <a href='http://" + espIp + "/config'>http://" +
               espIp + "/config </a>";
    content += "<br>POST: <a href='http://" + espIp + "/config'>http://" +
               espIp + "/config </a>";
    content += "<br>GET: <a href='http://" + espIp + "/ssid'>http://" + espIp +
               "/ssid </a>";
    content += "<br>POST: <a href='http://" + espIp + "/ssid'>http://" + espIp +
               "/ssid </a>";
    content += "<br>GET: <a href='http://" + espIp + "/reset'>http://" + espIp +
               "/reset </a>";
    content += "<br>DELETE: <a href='http://" + espIp + "/reset'>http://" +
               espIp + "/reset </a>";

    content += "</html>";
    server.send(200, "text/html", content);
  });

  server.on("/switch/auto", []() {
    blink();
    DynamicJsonBuffer jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();
    root["rc"] = 200;
    if (digitalRead(RELEY) == HIGH) {
      root["status"] = "on";
      digitalWrite(BUILTINLED, LOW);
    } else {
      root["status"] = "off";
      digitalWrite(BUILTINLED, HIGH);
    }
    MODE = "AUTO";
    String content;
    root.printTo(content);
    server.send(200, "application/json", content);
  });

  server.on("/switch/on", []() {
    blink();
    DynamicJsonBuffer jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();
    root["rc"] = 200;
    root["status"] = "on";
    MODE = "MANUAL";
    String content;
    root.printTo(content);
    digitalWrite(RELEY, HIGH);
    digitalWrite(BUILTINLED, LOW);
    server.send(200, "application/json", content);
  });

  server.on("/switch/off", []() {
    blink();
    DynamicJsonBuffer jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();
    root["rc"] = 200;
    root["status"] = "off";
    MODE = "MANUAL";
    String content;
    root.printTo(content);
    digitalWrite(RELEY, LOW);
    digitalWrite(BUILTINLED, HIGH);
    server.send(200, "application/json", content);
  });

  server.on("/status", HTTP_GET, []() {
    blink();
    DynamicJsonBuffer jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();
    root["rc"] = 200;
    root["mode"] = MODE;
    if (digitalRead(RELEY) == HIGH) {
      root["status"] = "on";
      digitalWrite(BUILTINLED, LOW);
    } else {
      root["status"] = "off";
      digitalWrite(BUILTINLED, HIGH);
    }

    root["data"] = sensorData;

    JsonObject &meta = root.createNestedObject("meta");
    meta["version"] = FIRM_VER;
    meta["sensor"] = SENSOR;
    meta["id"] = app_id;
    meta["deviceName"] = deviceName;
    meta["adc_vcc"] = adc;

    meta["ssid"] = essid;
    meta["rssi"] = rssi;
    meta["mdns"] = mdns;
    meta["freeHeap"] = ESP.getFreeHeap();
    meta["upTimeSec"] = (millis() - startTime) / 1000;

    String content;
    root.printTo(content);
    server.send(200, "application/json", content);

    Serial.print("\n\n/switch/status: ");
    root.printTo(Serial);

  });

  server.on("/update", HTTP_GET, []() {
    blink();
    DynamicJsonBuffer jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();

    root["url"] = "Enter url to bin file here and POST json object to ESP.";

    String content;
    root.printTo(content);
    server.send(200, "application/json", content);
  });

  server.on("/update", HTTP_POST, []() {
    blink();
    DynamicJsonBuffer jsonBuffer;
    JsonObject &root = jsonBuffer.parseObject(server.arg("plain"));
    String content;

    String secToken = root["securityToken"].asString();
    if(secToken == securityToken){
    Serial.println("Updating firmware...");
    String message = "";
    //    for (uint8_t i = 0; i < server.args(); i++) {
    //      message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    //    }

    String url = root["url"];
    Serial.println("");
    Serial.print("Update url: ");
    Serial.println(url);

    blink(10, 80);

    t_httpUpdate_return ret = ESPhttpUpdate.update(url);

    switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s",
                    ESPhttpUpdate.getLastError(),
                    ESPhttpUpdate.getLastErrorString().c_str());
      root["rc"] = ESPhttpUpdate.getLastError();
      message += "HTTP_UPDATE_FAILD Error (";
      message += ESPhttpUpdate.getLastError();
      message += "): ";
      message += ESPhttpUpdate.getLastErrorString().c_str();
      root["msg"] = message;
      root.printTo(content);
      server.send(400, "application/json", content);
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      root["rc"] = 400;
      root["msg"] = "HTTP_UPDATE_NO_UPDATES";
      root.printTo(content);
      server.send(400, "application/json", content);
      break;

    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      root["rc"] = 200;
      root["msg"] = "HTTP_UPDATE_OK";
      root.printTo(content);
      server.send(200, "application/json", content);
      break;
    }
  }else{
    root["rc"] = 401;
    root["msg"] = "Security token is not valid!";
    root.printTo(content);
    server.send(401, "application/json", content);
  }

  });

  server.on("/config", HTTP_GET, []() {
    blink();
    DynamicJsonBuffer jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();

    root["securityToken"] = "";
    root["timeOut"] = timeOut;
    root["relleyPin"] = RELEY;
    root["sensorInPin"] = GPIO_IN;
    root["buttonPin"] = BUTTON;
    root["statusLed"] = BUILTINLED;
    root["lightTreshold"] = lightTreshold;
    root["defaultMode"] = defaultMODE;

    root["mqttAddress"] = mqttAddress;
    root["mqttPort"] = mqttPort;
    root["mqttUser"] = mqttUser;
    root["mqttPassword"] = "******";

    root["mqttPublishTopic"] = mqttPublishTopic;
    root["mqttSuscribeTopic"] = mqttSuscribeTopic;

    root["restApiServer"] = rest_server;
    root["restApiSSL"] = rest_ssl;
    root["restApiPath"] = rest_path;
    root["restApiPort"] = rest_port;
    root["restApiToken"] = api_token;
    root["restApiPayload"] = api_payload;

    root["deviceName"] = deviceName;

    String content;
    root.printTo(content);
    server.send(200, "application/json", content);
  });

  server.on("/config", HTTP_POST, []() {
    blink();
    DynamicJsonBuffer jsonBuffer;
    JsonObject &root = jsonBuffer.parseObject(server.arg("plain"));
    Serial.println(F("\nSaving config..."));
    String content;

    String secToken = root["securityToken"].asString();
    if(secToken == securityToken){

      securityToken = root["newSecurityToken"].asString();
      root["securityToken"] = root["newSecurityToken"];

    String timeOut1 = root["timeOut"];
    timeOut = timeOut1.toInt();
    String relley1 = root["relleyPin"];
    RELEY = relley1.toInt();
    String gpioIn1 = root["sensorInPin"];
    GPIO_IN = gpioIn1.toInt();
    String button1 = root["buttonPin"];
    BUTTON = button1.toInt();
    String builtInLed1 = root["statusLed"];
    BUILTINLED = builtInLed1.toInt();
    String lightTreshold1 = root["lightTreshold"];
    lightTreshold = lightTreshold1.toInt();

    defaultMODE = root["defaultMode"].asString();

    String mqttAddress1 = root["mqttAddress"].asString();
    mqttAddress1.toCharArray(mqttAddress, 200, 0);
    String mqttPort1 = root["mqttPort"];
    mqttPort = mqttPort1.toInt();
    String mqttUser1 = root["mqttUser"].asString();
    mqttUser1.toCharArray(mqttUser, 20, 0);
    String mqttPassword1 = root["mqttPassword"].asString();
    mqttPassword1.toCharArray(mqttPassword, 20, 0);

    String mqttPubTopic1 = root["mqttPublishTopic"].asString();
    mqttPubTopic1.toCharArray(mqttPublishTopic, 200, 0);
    String mqttSusTopic1 = root["mqttSuscribeTopic"].asString();
    mqttSusTopic1.toCharArray(mqttSuscribeTopic, 200, 0);

    String rest_server1 = root["restApiServer"].asString();
    rest_server1.toCharArray(rest_server, 40, 0);

    String rest_ssl1 = root["restApiSSL"];
    if (rest_ssl1 == "true")
      rest_ssl = true;
    else
      rest_ssl = false;

    String rest_path1 = root["restApiPath"].asString();
    rest_path1.toCharArray(rest_path, 200, 0);
    String rest_port1 = root["restApiPort"];
    rest_port = rest_port1.toInt();
    String api_token1 = root["restApiToken"].asString();
    api_token1.toCharArray(api_token, 200, 0);
    String api_payload1 = root["restApiPayload"].asString();
    api_payload1.toCharArray(api_payload, 400, 0);

    String deviceName1 = root["deviceName"].asString();
    if (deviceName1 != "")
      deviceName1.toCharArray(deviceName, 200, 0);

    root.printTo(Serial);

    saveConfig(root);
    pinMode(RELEY, OUTPUT);
    pinMode(GPIO_IN, INPUT);

    Serial.println(F("\nConfiguration is saved."));
    root["rc"] = 200;
    root["msg"] = "Configuration is saved.";
    root.printTo(content);
    server.sendHeader("Access-Control-Allow-Methods", "POST,GET,OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers",
                      "Origin, X-Requested-With, Content-Type, Accept");
    server.send(200, "application/json", content);
  }else{
    Serial.println(F("\nSecurity tokent not valid!"));
    root["rc"] = 401;
    root["msg"] = "Security tokent not valid!";
    root.printTo(content);
    server.sendHeader("Access-Control-Allow-Methods", "POST,GET,OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers",
                      "Origin, X-Requested-With, Content-Type, Accept");
    server.send(401, "application/json", content);
  }




    if (mqttAddress != "") {
      mqClient.setServer(mqttAddress, mqttPort);
    }
  });

  server.on("/config", HTTP_OPTIONS, []() {
    blink();

    // server.sendHeader("Access-Control-Max-Age", "10000");
    server.sendHeader("Access-Control-Allow-Methods", "POST,GET,OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers",
                      "Origin, X-Requested-With, Content-Type, Accept");
    server.send(200, "text/plain", "");

  });

  server.on("/ssid", HTTP_GET, []() {
    blink();
    DynamicJsonBuffer jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();

    root["ssid"] = essid;
    root["password"] = epwd;

    root["connectedTo"] = WiFi.SSID();
    root["localIP"] = WiFi.localIP().toString();

    root["softAPIP"] = WiFi.softAPIP().toString();

    String content;
    root.printTo(content);
    server.send(200, "application/json", content);

  });

  server.on("/ssid", HTTP_POST, []() {
    blink(5);
    DynamicJsonBuffer jsonBuffer;
    JsonObject &root = jsonBuffer.parseObject(server.arg("plain"));
    Serial.print(F("\nSetting ssid to "));

    String secToken = root["securityToken"].asString();
    if(secToken == securityToken){
    String ssid1 = root["ssid"].asString();
    ssid1.toCharArray(essid, 40, 0);
    String pwd1 = root["password"].asString();
    pwd1.toCharArray(epwd, 40, 0);

    Serial.println(essid);

    Serial.println(F("\nNew ssid is set. ESP will reconect..."));
    root["rc"] = 200;
    root["msg"] = "New ssid is set. ESP will reconect.";

    String content;
    root.printTo(content);
    saveSsid(root);
    server.send(200, "application/json", content);

    delay(500);
    ESP.eraseConfig();
    delay(1000);
    WiFi.disconnect();
    delay(1000);
    WiFi.mode(WIFI_STA);
    delay(1000);
    WiFi.begin(essid, epwd);
    delay(1000);
    testWifi();
  }else{
    Serial.println(F("\nSecurity token not valid!"));
    root["rc"] = 401;
    root["msg"] = "Security token not valid!";

    String content;
    root.printTo(content);
    server.send(401, "application/json", content);

  }

  });

  server.on("/reset", HTTP_DELETE, []() {
    blink();
    DynamicJsonBuffer jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();

    String secToken = root["securityToken"].asString();
    if(secToken == securityToken){
    root["rc"] = 200;
    root["msg"] = "Reseting ESP config. Configuration will be erased ...";
    Serial.println(F("\nReseting ESP config. Configuration will be erased..."));

    String content;
    root.printTo(content);
    server.send(200, "application/json", content);
    delay(3000);

    // clean FS, for testing
    SPIFFS.format();
    delay(1000);
    // reset settings - for testing
    WiFi.disconnect(true);
    delay(1000);

    ESP.eraseConfig();

    ESP.reset();
    delay(5000);
  }else{
    root["rc"] = 401;
    root["msg"] = "Security token not valid!";
    Serial.println(F("\nSecurity token not valid!"));

    String content;
    root.printTo(content);
    server.send(401, "application/json", content);
  }
  });

  server.on("/reset", HTTP_GET, []() {
    blink();
    DynamicJsonBuffer jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();

    root["rc"] = 200;
    root["msg"] = "Reseting ESP...";
    Serial.println(F("\nReseting ESP..."));

    String content;
    root.printTo(content);
    server.send(200, "application/json", content);

    ESP.reset();
    delay(5000);
  });

  server.on("/rgb", HTTP_POST, []() {
    blink();
    DynamicJsonBuffer jsonBuffer;
    JsonObject &root = jsonBuffer.parseObject(server.arg("plain"));
    Serial.print(F("\nSetting RGB led... "));

    String red1 = root["red"];
    red = red1.toInt();
    String green1 = root["green"];
    green = green1.toInt();
    String blue1 = root["blue"];
    blue = blue1.toInt();

    red = red + 450;
    if (red > 1024)
      red = 1024;

    analogWrite(redPin, red);
    analogWrite(greenPin, green);
    analogWrite(bluePin, blue);

    Serial.println(F("\nRGB is set..."));
    root["rc"] = 200;
    root["msg"] = "New RGB value are set.";

    String content;
    root.printTo(content);
    server.send(200, "application/json", content);
  });

  server.on("/rgb", HTTP_GET, []() {
    blink();
    DynamicJsonBuffer jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();
    Serial.println(F("\nRGB..."));

    root["red"] = red;
    root["green"] = green;
    root["blue"] = blue;

    root["rc"] = 200;

    String content;
    root.printTo(content);
    server.send(200, "application/json", content);
  });

  server.on("/fadeIn", HTTP_GET, []() {
    blink();
    DynamicJsonBuffer jsonBuffer;
    JsonObject &root = jsonBuffer.parseObject(server.arg("plain"));
    Serial.println(F("\nFading in..."));

    fadeIn();

    root["rc"] = 200;
    root["msg"] = "Fading in...";

    String content;
    root.printTo(content);
    server.send(200, "application/json", content);
  });

  server.on("/fadeOut", HTTP_GET, []() {
    blink();
    DynamicJsonBuffer jsonBuffer;
    JsonObject &root = jsonBuffer.parseObject(server.arg("plain"));

    Serial.println(F("\nFading out..."));
    fadeOut();

    root["rc"] = 200;
    root["msg"] = "Fading out...";

    String content;
    root.printTo(content);
    server.send(200, "application/json", content);
  });

  server.begin();
  Serial.println("Server started");

} //--

// send request ---------------------------------------------------------
void sendRequest(String sensorData) {
  String api_payload_s = String(api_payload);
  if (api_payload_s != "")
    api_payload_s = api_payload_s + ", ";

  String data = "{" + api_payload_s + "\"sensor\":{\"sensor_type\":\"" +
                SENSOR + "\", \"data\":{" + sensorData + "}" + ", \"ver\":\"" +
                FIRM_VER + "\"" + ", \"ip\":\"" + espIp + "\"" + ", \"id\":\"" +
                app_id + "\"" + ", \"name\":\"" +
                String(deviceName) + "\"" + ", \"adc\":" + adc + "}}";

  // REST request
  if (String(rest_server) != "" && getIP(WiFi.localIP()) != "") {
    WiFiClientSecure client;
    // WiFiClient client;
    int i = 0;
    String url = rest_path;
    Serial.println("");
    Serial.print(F("Connecting for request... "));
    Serial.print(String(rest_server));
    Serial.print(":");
    Serial.println(String(rest_port));

    while (!client.connect(rest_server, rest_port)) {
      Serial.print("Try ");
      Serial.print(i);
      Serial.print(" ... ");

      if (i == 4) {
        blink(5, 20);
        Serial.println("");
        return;
      }
      i++;
    }

    Serial.print("POST data to URL: ");
    Serial.println(url);
    delay(10);
    String req =
        String("POST ") + url + " HTTP/1.1\r\n" +
        "Host: " + String(rest_server) + "\r\n" + "User-Agent: ESP/1.0\r\n" +
        "Content-Type: application/json\r\n" + "Cache-Control: no-cache\r\n" +
        "Sensor-Id: " + String(app_id) + "\r\n" +
        "Token: " + String(api_token) + "\r\n" +
        "Content-Type: application/x-www-form-urlencoded;\r\n" +
        "Content-Length: " + data.length() + "\r\n" + "\r\n" + data;
    Serial.print(F("Request: "));
    Serial.println(req);
    client.print(req);

    delay(100);

    Serial.println(F("Response: "));
    while (client.available()) {
      String line = client.readStringUntil('\r');
      Serial.print(line);
    }

    blink(2, 40);

    Serial.println(F("\nConnection closed"));
    req = "";
  }

  // MQTT publish
  if (String(mqttAddress) != "" && String(mqttPublishTopic) != "" && getIP(WiFi.localIP()) != "") {
    Serial.println();
    Serial.println(F("MQTT publish..."));

    mqPublish(data);
  }

  api_payload_s = "";
  data = "";

} //--

void saveConfig(JsonObject &json) {

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println(F("failed to open config file for writing"));
  }

  json.printTo(Serial);
  json.printTo(configFile);
  configFile.close();
}

void saveSsid(JsonObject &json) {

  File configFile = SPIFFS.open("/ssid.json", "w");
  if (!configFile) {
    Serial.println(F("failed to open ssid.json file for writing"));
  }

  json.printTo(Serial);
  json.printTo(configFile);
  configFile.close();
}

// MQTT
void mqCallback(char *topic, byte *payload, unsigned int length) {
  Serial.print(F("Message arrived ["));
  Serial.print(topic);
  Serial.print(F("] "));
  String msg = "";
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    msg = msg + (char)payload[i];
  }
  Serial.println();
  blink(3, 50);

  DynamicJsonBuffer jsonBuffer;
  JsonObject &rootMqtt = jsonBuffer.parseObject(msg);

  String temp = rootMqtt["sensor"]["data"]["temp"].asString();
  String hum = rootMqtt["sensor"]["data"]["hum"].asString();
}

bool mqReconnect() {
  yield();
  if (!mqClient.connected()) {
    Serial.print(F("\nAttempting MQTT connection... "));
    Serial.print(mqttAddress);
    Serial.print(F(":"));
    Serial.print(mqttPort);
    Serial.print(F("@"));
    Serial.print(mqttUser);
    Serial.print(F(" Pub: "));
    Serial.print(mqttPublishTopic);
    Serial.print(F(", Sub: "));
    Serial.print(mqttSuscribeTopic);

    yield();
    // Attempt to connect
    if (mqClient.connect(app_id.c_str(), mqttUser, mqttPassword)) {
      yield();
      Serial.print(F("\nconnected with cid: "));
      Serial.println(app_id);

      // suscribe
      if (String(mqttSuscribeTopic) != ""){
        mqClient.subscribe(String(mqttSuscribeTopic).c_str());
        Serial.print(F("\nSuscribed to toppic: "));
        Serial.println(mqttSuscribeTopic);
      }

    } else {
      Serial.print(F("\nfailed to connect! Client state: "));
      Serial.println(mqClient.state());

    }
  } else {
    Serial.print(F("\nMQTT is already connected."));
  }
  return mqClient.connected();
}

void mqPublish(String msg) {

  if (mqReconnect()) {

    // mqClient.loop();

    Serial.print(F("\nPublish message to topic '"));
    Serial.print(mqttPublishTopic);
    Serial.print(F("':"));
    Serial.println(msg);
    mqClient.publish(String(mqttPublishTopic).c_str(), msg.c_str());

  } else {
    Serial.print(F("\nPublish failed!"));
  }
}

// tesing for wifi connection
bool testWifi() {
  int c = 0;
  Serial.println("Waiting for Wifi to connect...");
  while (c < 20) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print(F("WiFi connected to "));
      Serial.println(WiFi.SSID());

      blink(2, 30, 1000);
      return true;
    }
    blink(1, 200);
    delay(500);
    Serial.print(F("Retrying to connect to WiFi ssid: "));
    Serial.print(essid);
    Serial.print(F(" status="));
    Serial.println(WiFi.status());
    c++;
  }
  Serial.println(F(""));
  Serial.println(F("Connect timed out."));

  blink(20, 30);
  delay(1000);
  // reset esp
  // ESP.reset();
  // delay(3000);
  return false;
}

void setupAP(void) {
  Serial.print(F("Setting up access point. WiFi.mode= "));
  Serial.println(WIFI_STA);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  delay(100);
  int n = WiFi.scanNetworks();
  Serial.println(F("Scanning network done."));
  if (n == 0)
    Serial.println(F("No networks found."));
  else {
    Serial.print(n);
    Serial.println(F(" networks found"));
    Serial.println(F("---------------------------------"));
    for (int i = 0; i < n; ++i) {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(F(": "));
      Serial.print(WiFi.SSID(i));
      Serial.print(F(" ("));
      Serial.print(WiFi.RSSI(i));
      Serial.print(F(")"));
      Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*");
      delay(10);
    }
  }
  Serial.println("");
  delay(100);

  WiFi.softAP(apSsid.c_str(), apPass.c_str());
  Serial.print(F("SoftAP ready. AP SID: "));
  Serial.print(apSsid);

  IPAddress apip = WiFi.softAPIP();
  Serial.print(F(", AP IP address: "));

  String softAp = getIP(apip);

  Serial.println(apip);
  espIp = String(apip);
  blink(5, 100);
}

void readFS() {
  // read configuration from FS json
  Serial.println(F("mounting FS..."));

  if (SPIFFS.begin()) {
    Serial.println(F("mounted file system"));
    if (SPIFFS.exists("/config.json")) {
      // file exists, reading and loading
      Serial.println(F("reading config.json file"));
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println(F("opened config.json file"));
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject &jsonConfig = jsonBuffer.parseObject(buf.get());
        jsonConfig.printTo(Serial);

        if (jsonConfig.success()) {
          Serial.println(F("\nparsed config.json"));

          // config parameters
          securityToken = jsonConfig["securityToken"].asString();

          String deviceName1 = jsonConfig["deviceName"].asString();
          if (deviceName1 != "")
            deviceName1.toCharArray(deviceName, 200, 0);

          String timeOut1 = jsonConfig["timeOut"];
          timeOut = timeOut1.toInt();
          String relley1 = jsonConfig["relleyPin"];
          RELEY = relley1.toInt();
          String gpioIn1 = jsonConfig["sensorInPin"];
          GPIO_IN = gpioIn1.toInt();
          String button1 = jsonConfig["buttonPin"];
          BUTTON = button1.toInt();
          String builtInLed1 = jsonConfig["statusLed"];
          BUILTINLED = builtInLed1.toInt();

          defaultMODE = jsonConfig["defaultMode"].asString();
          MODE = defaultMODE;

          String mqttAddress1 = jsonConfig["mqttAddress"].asString();
          mqttAddress1.toCharArray(mqttAddress, 200, 0);
          String mqttPort1 = jsonConfig["mqttPort"];
          mqttPort = mqttPort1.toInt();

          String mqttUser1 = jsonConfig["mqttUser"].asString();
          mqttUser1.toCharArray(mqttUser, 20, 0);
          String mqttPassword1 = jsonConfig["mqttPassword"].asString();
          mqttPassword1.toCharArray(mqttPassword, 20, 0);

          String mqttPubTopic1 = jsonConfig["mqttPublishTopic"].asString();
          mqttPubTopic1.toCharArray(mqttPublishTopic, 200, 0);
          String mqttSusTopic1 = jsonConfig["mqttSuscribeTopic"].asString();
          mqttSusTopic1.toCharArray(mqttSuscribeTopic, 200, 0);

          String rest_server1 = jsonConfig["restApiServer"].asString();
          rest_server1.toCharArray(rest_server, 40, 0);

          String rest_ssl1 = jsonConfig["restApiSSL"];
          if (rest_ssl1 == "true")
            rest_ssl = true;
          else
            rest_ssl = false;

          String rest_path1 = jsonConfig["restApiPath"].asString();
          rest_path1.toCharArray(rest_path, 200, 0);
          String rest_port1 = jsonConfig["restApiPort"];
          rest_port = rest_port1.toInt();
          String api_token1 = jsonConfig["restApiToken"].asString();
          api_token1.toCharArray(api_token, 200, 0);
          String api_payload1 = jsonConfig["restApiPayload"].asString();
          api_payload1.toCharArray(api_payload, 400, 0);

          String lightTreshold1 = jsonConfig["lightTreshold"];
          lightTreshold = lightTreshold1.toInt();

        } else {
          Serial.println(F("failed to load json config"));
        }
      }
    } else {
      Serial.println(F("Config.json file doesn't exist yet!"));
    }

    // ssid.json
    if (SPIFFS.exists("/ssid.json")) {
      // file exists, reading and loading
      Serial.println(F("reading ssid.json file"));
      File ssidFile = SPIFFS.open("/ssid.json", "r");
      if (ssidFile) {
        Serial.println(F("opened config file"));
        size_t size = ssidFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        ssidFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject &jsonConfig = jsonBuffer.parseObject(buf.get());
        jsonConfig.printTo(Serial);
        if (jsonConfig.success()) {
          Serial.println(F("\nparsed ssid.json"));

          // ssid parameters
          String ssid1 = jsonConfig["ssid"].asString();
          ssid1.toCharArray(essid, 40, 0);
          String pwd1 = jsonConfig["password"].asString();
          pwd1.toCharArray(epwd, 40, 0);

        } else {
          Serial.println(F("failed to load ssid.json"));
        }
      }
    } else {
      Serial.println(F("ssid.json file doesn't exist yet!"));
    }
  } else {
    Serial.println(F("failed to mount FS"));
    blink(10, 50, 20);
  }
  // end read
}

// blink
void blink(void) { blink(1, 30, 30); }

void blink(int times) { blink(times, 30, 30); }

void blink(int times, int milisec) { blink(times, milisec, milisec); }

void blink(int times, int himilisec, int lowmilisec) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUILTINLED, LOW);
    delay(lowmilisec);
    digitalWrite(BUILTINLED, HIGH);
    delay(himilisec);
  }
}

String getMac() {
  unsigned char mac[6];
  WiFi.macAddress(mac);
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
  }
  return result;
}

// RGB
int fadeSpeed = 10;
int fadeStep = 2;

// RFID
void readRFID(String &sensorData) {
  if (sensorData == NULL)
    sensorData = "";
  else
    sensorData += ", ";
  // Look for new cards
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return;
  }

  // Select one of the cards
  if (!mfrc522.PICC_ReadCardSerial()) {
    return;
  }

  // Dump debug info about the card; PICC_HaltA() is automatically called
  // mfrc522.PICC_DumpToSerial(&(mfrc522.uid));
  Serial.print(F("Card UID:"));
  String id = dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
  Serial.println();

  sensorData += "\"rfidId\":\"" + id + "\"";
  sendRequest(sensorData);
}

String dump_byte_array(byte *buffer, byte bufferSize) {

  /*for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }*/

  String val1 = String(mfrc522.uid.uidByte[0], HEX);
  String val2 = String(mfrc522.uid.uidByte[1], HEX);
  String val3 = String(mfrc522.uid.uidByte[2], HEX);
  String val4 = String(mfrc522.uid.uidByte[3], HEX);
  val1.toUpperCase();
  val2.toUpperCase();
  val3.toUpperCase();
  val4.toUpperCase();
  String uID = val1 + val2 + val3 + val4;

  Serial.println();
  Serial.print("U:");
  Serial.println(uID);
  return uID;
}

void fadeIn() {
  int redx, greenx, bluex;
  redx = greenx = bluex = 1024;

  for (int i = 1024; i >= 0; i--) {

    if (redx >= red) {
      redx = redx - fadeStep;
    }
    if (redx <= 0)
      redx = 0;
    if (redx > 1024)
      redx = 1024;
    if (greenx >= green) {
      greenx = greenx - fadeStep;
    }
    if (greenx <= 0)
      greenx = 0;
    if (greenx > 1024)
      greenx = 1024;
    if (bluex >= blue) {
      bluex = bluex - fadeStep;
    }
    if (bluex <= 0)
      bluex = 0;
    if (bluex > 1024)
      bluex = 1024;

    int redcorect = redx + 500;
    if (redcorect > 1024)
      redcorect = 1024;

    analogWrite(redPin, redcorect);
    analogWrite(greenPin, greenx);
    analogWrite(bluePin, bluex);

    delay(fadeSpeed);
    if (greenx <= green && redx <= red && bluex <= blue)
      break;
  }
}

void fadeOut() {
  if (red == 1024 && green == 1024 && blue == 1024)
    return;

  int redx = red;
  int greenx = green;
  int bluex = blue;

  for (int i = 0; i <= 1024; i++) {
    redx = redx + fadeStep;
    if (redx >= 1024)
      redx = 1024;

    greenx = greenx + fadeStep;
    if (greenx >= 1024)
      greenx = 1024;

    bluex = bluex + fadeStep;
    if (bluex >= 1024)
      bluex = 1024;

    int redcorect = redx + 500;
    if (redcorect > 1024)
      redcorect = 1024;

    analogWrite(redPin, redx);
    analogWrite(greenPin, greenx);
    analogWrite(bluePin, bluex);
    delay(fadeSpeed);
    if (greenx >= 1024 && redx >= 1024 && bluex >= 1024)
      break;
  }
}

String getIP(IPAddress ip){
  String ipo = String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' +
          String(ip[3]);
  if(ipo=="0.0.0.0")
  ipo = "";
  return ipo;
}
