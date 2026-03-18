#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>   // used to upload .html with ESP32 Sketch Data Upload
#include <esp_wifi.h>   // Needed for WPA2 Enterprise + power save control
#include "esp_wpa2.h"
#include <esp_now.h>    // Station to Rover and vice versa
//#include <NimBLEDevice.h>   // Rover scans, Station advertises (disabled - was causing mutex assert)
#include <BLEDevice.h>       // Classic BLE (more stable with WiFi and ESP-NOW)
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>


// ========================
// WIFI SETTINGS
// ========================

// Regular Networks
//const char* homeSSID = "MySpectrumWiFid8-2G"; // Russell's House
//const char* homePassword = "loudguppy846";

//const char* homeSSID = "GLAPTOP_PANDA 4216"; // Russell's Hotspot FOR ON CAMPUS
//const char* homePassword = "N)9n7448";

const char* friendSSID = "Soto-Home";  // Alexis's House
const char* friendPassword = "Jmakg1979";

//  const char* friendSSID = "Alexis iphone"; //Alexis's HotSpot
//  const char* friendPassword = "RoverPlease1";

//const char* friendSSID = "SKYNET13"; // Tommy's Apartment
//const char* friendPassword = "greenjungle938";

// School Wi-Fi (WPA2-Enterprise) (kept for reference)
//const char* schoolSSID = "UCR-SECURE";
//const char* identity = "rmala007";
//const char* username = "rmala007";
//const char* schoolPassword = "Panda@ucr500";


// ----- NETWORK SSID & PASSWORD -----
//const char* ssid = homeSSID;
//const char* password = homePassword;
const char* ssid = friendSSID;
const char* password = friendPassword;
//const char* ssid = schoolSSID;
//const char* password = schoolPassword;

// ========================
// Control Variables (kept for website sync)
// ========================
bool manualMode = false;

// Web button tracking (multi-key support)
bool wPressed = false, aPressed = false, sPressed = false, dPressed = false;

// ========================
// Undock sequencing
// ========================
static bool undockPending = false;
static unsigned long undockStartMs = 0;
static const unsigned long UNDOCK_RELEASE_DELAY_MS = 600;

// ========================
// Web server
// ========================
AsyncWebServer server(80);

// ========================
// BLE Dock Beacon Init (Old NimBLE version that is kept for reference)
// ========================

// static NimBLEUUID DOCK_SERVICE_UUID("12345678-1234-1234-1234-1234567890ab");

// void bleBeaconInit() {
//   NimBLEDevice::init("ROVER_DOCK");
//   NimBLEServer* pServer = NimBLEDevice::createServer();
//   (void)pServer;

//   NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
//   adv->addServiceUUID(DOCK_SERVICE_UUID);
//   adv->start();

//   Serial.println("BLE Dock Beacon Advertising");
// }

// ========================
// BLE Dock Beacon Init (Classic BLE)
// ========================
static BLEUUID DOCK_SERVICE_UUID("12345678-1234-1234-1234-1234567890ab");

void bleBeaconInit() {
  BLEDevice::init("ROVER_DOCK");

  BLEServer* tempServer = BLEDevice::createServer();
  (void)tempServer;

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(DOCK_SERVICE_UUID);

  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);

  BLEDevice::startAdvertising();
  Serial.println("BLE Dock Beacon Advertising (Classic BLE)");
}

// ========================
// ESP-NOW Rover Forwarding
// ========================

// Rover ESP32 MAC address
uint8_t roverMac[6] = {0x44,0x1D,0x64,0xF4,0xE6,0xE8};

// StationFunction (3rd ESP32) MAC address
uint8_t stationFuncMac[6] = {0x44,0x1D,0x64,0xF6,0x05,0xE8};

typedef struct __attribute__((packed)) {
  char key;        
  uint8_t state;   
  uint8_t manual;  
} RoverCmd;

// MUST MATCH roverMain.ino
typedef struct __attribute__((packed)) {
  uint8_t manual;          
  uint8_t s1, s2, s3, s4, s5;
  int16_t distanceMm;      

  int8_t dockRssi;         
  uint16_t dockAgeMs;      

  uint16_t batt_mV;        
  int16_t  batt_mA;        
  uint16_t batt_cPct;      
  uint8_t  docked;         
  uint8_t  returning;      
} RoverTelemetry;

// StationFunction command + data structs
typedef struct __attribute__((packed)) {
  char key;
  uint8_t state;   
  uint8_t manual;  
} StationFuncCmd;

// StationFunction data (MUST MATCH StationFunction)
typedef struct __attribute__((packed)) {
  uint16_t stBatt_mV;
  int16_t  stBatt_mA;

  uint16_t chgIn_mV;
  int16_t  chgIn_mA;

  uint8_t  dockDown;
  uint8_t  docked;
  uint8_t  mode;

  uint8_t  usbINA_ok;
  uint8_t  stINA_ok;
} StationFuncTelemetry;

bool espNowReady = false;

// Latest data from rover
volatile bool hasTelemetry = false;
volatile unsigned long lastTelemetryRxMs = 0;
RoverTelemetry lastTel = {};

// Latest data from StationFunction
volatile bool hasFuncTelemetry = false;
volatile unsigned long lastFuncTelemetryRxMs = 0;
StationFuncTelemetry lastFuncTel = {};

// ========================
// Bridge safety stop when StationFunction dock switch is pressed
// ========================
static bool dockStopSent = false;

static bool funcDockCandidate = false;
static bool funcDockStable = false;
static unsigned long funcDockChangeMs = 0;

static const unsigned long DOCK_CONFIRM_MS = 120;
static const bool AUTO_START_ROVER_CHARGE_ON_DOCK = false;

void onEspNowSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  (void)tx_info;
  (void)status;
}

void onEspNowRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  (void)recv_info;

  if (len == (int)sizeof(RoverTelemetry)) {
    memcpy((void*)&lastTel, data, sizeof(RoverTelemetry));
    hasTelemetry = true;
    lastTelemetryRxMs = millis();
    return;
  }

  if (len == (int)sizeof(StationFuncTelemetry)) {
    memcpy((void*)&lastFuncTel, data, sizeof(StationFuncTelemetry));
    hasFuncTelemetry = true;
    lastFuncTelemetryRxMs = millis();
    return;
  }
}

void espNowInit() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    espNowReady = false;
    return;
  }

  esp_now_register_send_cb(onEspNowSent);
  esp_now_register_recv_cb(onEspNowRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, roverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    espNowReady = true;
    Serial.println("ESP-NOW peer added (rover)");
  } else {
    espNowReady = false;
    Serial.println("ESP-NOW peer NOT added (set roverMac first)");
  }

  esp_now_peer_info_t funcPeer = {};
  memcpy(funcPeer.peer_addr, stationFuncMac, 6);
  funcPeer.channel = 0;
  funcPeer.encrypt = false;

  if (esp_now_add_peer(&funcPeer) == ESP_OK) {
    Serial.println("ESP-NOW peer added (stationFunction)");
  } else {
    Serial.println("ESP-NOW peer NOT added (stationFunction)");
  }
}

void sendToRover(char key, uint8_t state) {
  if (!espNowReady) return;

  RoverCmd cmd;
  cmd.key = key;
  cmd.state = state;
  cmd.manual = manualMode ? 1 : 0;

  for (int i = 0; i < 3; i++) {
    esp_now_send(roverMac, (uint8_t*)&cmd, sizeof(cmd));
  }
}

void sendToStationFunc(char key, uint8_t state) {
  if (!espNowReady) return;

  StationFuncCmd cmd;
  cmd.key = key;
  cmd.state = state;
  cmd.manual = 0;

  for (int i = 0; i < 3; i++) {
    esp_now_send(stationFuncMac, (uint8_t*)&cmd, sizeof(cmd));
  }
}

// ========================
// Setup
// ========================
void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
  } else {
    Serial.println("SPIFFS Mounted OK");
  }

  // ---------------- Regular WiFi (And for Wifi Hotspot) ----------------
  
  Serial.println("Connecting to WiFi...");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nWiFi connected: http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed.");
  }
 

  // ---------------- WPA2-Enterprise (REFERENCE) ----------------
  /*
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);

  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)identity, strlen(identity));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)username, strlen(username));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)schoolPassword, strlen(schoolPassword));
  esp_wifi_sta_wpa2_ent_enable();

  WiFi.begin(schoolSSID);

  Serial.println("Connecting to WPA2-Enterprise WiFi...");

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 20000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected: http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connection failed.");
  }
  */

  // ---------------- ESP-NOW init ----------------
  espNowInit();

  // ---------------- BLE Beacon ----------------
  bleBeaconInit();

  // ---------------- Web server ----------------

  // NEW: ultra-simple sanity endpoints (do not depend on SPIFFS)
  server.on("/ping", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "pong");
  });
  server.on("/ip", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", WiFi.softAPIP().toString());
  });

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("stationWebsite.html");

  server.on("/press", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("key") && request->hasParam("state")) {
      String key = request->getParam("key")->value();
      int state = request->getParam("state")->value().toInt();

      if (key == "w") wPressed = state;
      else if (key == "a") aPressed = state;

      if (false && key == "s") wPressed = wPressed;
      else if (key == "s") sPressed = state;
      else if (key == "d") dPressed = state;

      if (key == "m" && state == 1) manualMode = !manualMode;

      if ((key == "w" || key == "a" || key == "s" || key == "d") && state == 1) {
        manualMode = true;
      }

      if (key.length() == 1) {
        char k = key.charAt(0);

        if (k=='w' || k=='a' || k=='s' || k=='d' || k=='m' || k=='h' || k=='c' || k=='p') {
          if (k == 'h' && state == 1) dockStopSent = false;
          sendToRover(k, (uint8_t)state);
        }

        if (k=='O' || k=='S' || k=='R' || k=='U' || k=='D' || k=='E') {
          sendToStationFunc(k, (uint8_t)state);
        }

        // ADDED:
        // 'u' = undock sequence
        // 1) tell StationFunction to release hardware using 'U'
        // 2) after a short delay, tell rover to perform undock using 'u'
        if (k=='u' && state == 1) {
          dockStopSent = false;
          funcDockStable = false;
          funcDockCandidate = false;

          sendToStationFunc('U', 1);

          undockPending = true;
          undockStartMs = millis();

          manualMode = false;
        }
      }
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/mode", HTTP_GET, [](AsyncWebServerRequest *request) {
    char json[32];
    snprintf(json, sizeof(json), "{\"manual\":%d}", manualMode ? 1 : 0);
    request->send(200, "application/json", json);
  });

  server.on("/linesensors", HTTP_GET, [](AsyncWebServerRequest *request) {
    uint8_t s1=0,s2=0,s3=0,s4=0,s5=0;
    if (hasTelemetry) {
      s1 = lastTel.s1; s2 = lastTel.s2; s3 = lastTel.s3; s4 = lastTel.s4; s5 = lastTel.s5;
    }
    char json[96];
    snprintf(json, sizeof(json),
             "{\"s1\":%u,\"s2\":%u,\"s3\":%u,\"s4\":%u,\"s5\":%u}",
             s1,s2,s3,s4,s5);
    request->send(200, "application/json", json);
  });

  server.on("/distance", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!hasTelemetry || lastTel.distanceMm < 0) {
      request->send(200, "text/plain", "nan");
      return;
    }
    float cm = lastTel.distanceMm / 10.0f;
    char buf[32];
    snprintf(buf, sizeof(buf), "%.2f", cm);
    request->send(200, "text/plain", buf);
  });

  server.on("/telemetry", HTTP_GET, [](AsyncWebServerRequest *request) {
    bool linkOk = false;
    unsigned long age = 0;
    if (hasTelemetry) {
      age = millis() - (unsigned long)lastTelemetryRxMs;
      linkOk = (age < 1500);
    }

    uint8_t manualReport = hasTelemetry ? lastTel.manual : (manualMode ? 1 : 0);

    unsigned long funcAge = 65535UL;
    if (hasFuncTelemetry) {
      funcAge = millis() - (unsigned long)lastFuncTelemetryRxMs;
      if (funcAge > 65535UL) funcAge = 65535UL;
    }

    char json[900];
    snprintf(json, sizeof(json),
      "{"
        "\"manual\":%u,"
        "\"s1\":%u,\"s2\":%u,\"s3\":%u,\"s4\":%u,\"s5\":%u,"
        "\"distanceMm\":%d,"
//        "\"dockRssi\":%d,"
//        "\"dockAgeMs\":%u,"
        "\"batt_mV\":%u,"
        "\"batt_mA\":%d,"
        "\"batt_cPct\":%u,"
        "\"docked\":%u,"
        "\"returning\":%u,"
        "\"linkOk\":%s,"

        "\"stBatt_mV\":%u,"
        "\"stBatt_mA\":%d,"
        "\"chgIn_mV\":%u,"
        "\"chgIn_mA\":%d,"
        "\"dockDown\":%u,"
        "\"usbINA_ok\":%u,"
        "\"stINA_ok\":%u,"
        "\"funcDocked\":%u,"
        "\"funcMode\":%u,"
        "\"funcAgeMs\":%u"
      "}",
      manualReport,
      hasTelemetry ? lastTel.s1 : 0, hasTelemetry ? lastTel.s2 : 0, hasTelemetry ? lastTel.s3 : 0,
      hasTelemetry ? lastTel.s4 : 0, hasTelemetry ? lastTel.s5 : 0,
      hasTelemetry ? (int)lastTel.distanceMm : -1,
//      hasTelemetry ? (int)lastTel.dockRssi : -127,
//      hasTelemetry ? (unsigned int)lastTel.dockAgeMs : 65535u,
      hasTelemetry ? (unsigned int)lastTel.batt_mV : 0u,
      hasTelemetry ? (int)lastTel.batt_mA : 0,
      hasTelemetry ? (unsigned int)lastTel.batt_cPct : 0u,
      hasTelemetry ? (unsigned int)lastTel.docked : 0u,
      hasTelemetry ? (unsigned int)lastTel.returning : 0u,
      linkOk ? "true" : "false",

      hasFuncTelemetry ? (unsigned int)lastFuncTel.stBatt_mV : 0u,
      hasFuncTelemetry ? (int)lastFuncTel.stBatt_mA : 0,
      hasFuncTelemetry ? (unsigned int)lastFuncTel.chgIn_mV : 0u,
      hasFuncTelemetry ? (int)lastFuncTel.chgIn_mA : 0,
      hasFuncTelemetry ? (unsigned int)lastFuncTel.dockDown : 0u,
      hasFuncTelemetry ? (unsigned int)lastFuncTel.usbINA_ok : 0u,
      hasFuncTelemetry ? (unsigned int)lastFuncTel.stINA_ok : 0u,
      hasFuncTelemetry ? (unsigned int)lastFuncTel.docked : 0u,
      hasFuncTelemetry ? (unsigned int)lastFuncTel.mode : 0u,
      (unsigned int)funcAge
    );

    request->send(200, "application/json", json);
  });

  server.onNotFound([](AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "404: Not Found");
  });

  server.begin();
  Serial.println("ESP32 Charging Station Ready (SoftAP)");
}

// ========================
// Loop
// ========================
void loop() {
  if (undockPending && (millis() - undockStartMs >= UNDOCK_RELEASE_DELAY_MS)) {
    sendToRover('u', 1);
    undockPending = false;
  }

  if (hasFuncTelemetry) {
    bool rawDock = (lastFuncTel.docked == 1);
    unsigned long now = millis();

    if (rawDock != funcDockCandidate) {
      funcDockCandidate = rawDock;
      funcDockChangeMs = now;
    }

    if ((now - funcDockChangeMs) >= DOCK_CONFIRM_MS) {
      funcDockStable = funcDockCandidate;
    }

    if (funcDockStable && !dockStopSent) {
      sendToRover('p', 2);
      dockStopSent = true;

      if (AUTO_START_ROVER_CHARGE_ON_DOCK) {
        sendToStationFunc('R', 1);
      }
    }

    if (!funcDockStable) {
      dockStopSent = false;
    }
  }

  delay(0);
}
