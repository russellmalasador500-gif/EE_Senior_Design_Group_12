#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_INA219.h>

struct InaReadout {
  uint16_t mV_load;
  int16_t  mA;
};

InaReadout readINA(Adafruit_INA219 &ina, bool ok);

// ========================
// WIFI SETTINGS
// ======================

// Regular Networks
//const char* homeSSID = "MySpectrumWiFid8-2G"; // Russell's House
//const char* homePassword = "loudguppy846";

//const char* homeSSID = "GLAPTOP_PANDA 4216"; // Russell's Hotspot
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


// ----- NETWORK SSID & PASSWORD ----- (kept for reference)
//const char* ssid = homeSSID;
//const char* password = homePassword;
const char* ssid = friendSSID;
const char* password = friendPassword;
//const char* ssid = schoolSSID;
//const char* password = schoolPassword;

// =====================================================
// STATIONFUNCTION BOARD
// - Reads 2 INA219 sensors (two separate I2C buses)
// - Controls servo for dock/release
// - Uses bump switch on GPIO25 to confirm rover is docked
// - Receives website commands via ESP-NOW
// - Sends telemetry to StationWebsite via ESP-NOW
// - Safe logic: default OFF + servo UP
// =====================================================

// =================
// MAC addresses
// ====================
uint8_t stationWebsiteMac[6] = {0x00,0x4B,0x12,0x9B,0x14,0x10};

// =================
// Pins
// ==============
const int PIN_STATION_EN  = 18;
const int PIN_ROVER_EN    = 19;
const int PIN_SERVO       = 23;
const int PIN_DOCK_SWITCH = 25; // bump switch pressed -> GND

// ✅ calibrated angles
const int SERVO_UP_ANGLE   = 110;
const int SERVO_DOWN_ANGLE = 190;

// Enable logic
const int ENABLE_LEVEL  = HIGH;
const int DISABLE_LEVEL = LOW;

// ======================
// INA219 wiring
// ===================
// USB input charger INA219 bus
const int SDA_USB = 21;
const int SCL_USB = 22;

// Station battery INA219 bus
const int SDA_STB = 26;
const int SCL_STB = 27;

// Two separate I2C instances
TwoWire I2C_USB = TwoWire(0);
TwoWire I2C_STB = TwoWire(1);

// INA devices (both can be 0x40 because separate buses)
Adafruit_INA219 inaUsbIn(0x40);
Adafruit_INA219 inaStationBatt(0x40);

bool usbINA_ok = false;
bool stINA_ok  = false;

// =================
// Servo
// ===========
Servo dockServo;
bool dockDown = false; // false=UP, true=DOWN

void servoUp() {
  dockServo.write(SERVO_UP_ANGLE);
  dockDown = false;
}

void servoDown() {
  dockServo.write(SERVO_DOWN_ANGLE);
  dockDown = true;
}

// ====================
// Dock switch debounce
// ======================
bool docked = false;
bool lastRawDock = false;
uint32_t lastDockChangeMs = 0;
const uint32_t DOCK_DEBOUNCE_MS = 30;

bool dockRaw() {
  return digitalRead(PIN_DOCK_SWITCH) == LOW;
}

void updateDockDebounced() {
  bool raw = dockRaw();
  uint32_t now = millis();

  if (raw != lastRawDock) {
    lastRawDock = raw;
    lastDockChangeMs = now;
  }

  if ((now - lastDockChangeMs) >= DOCK_DEBOUNCE_MS) {
    docked = raw;
  }
}

// ======================
// Operating modes + sequencing delays
// ====================
enum Mode : uint8_t {
  MODE_OFF = 0,
  MODE_STATION = 1,
  MODE_ROVER = 2
};

Mode mode = MODE_OFF;

const uint32_t DOCK_SERVO_SETTLE_MS = 400;
const uint32_t UNDOCK_PAUSE_MS      = 150;

// ===============
// INA helpers
// =================
static uint16_t clampU16(int v) {
  if (v < 0) return 0;
  if (v > 65535) return 65535;
  return (uint16_t)v;
}

static int16_t clampI16(int v) {
  if (v < -32768) return -32768;
  if (v > 32767) return 32767;
  return (int16_t)v;
}

//struct InaReadout {
//  uint16_t mV_load; // load voltage in mV
//  int16_t  mA;      // current in mA (signed)
//};

InaReadout readINA(Adafruit_INA219 &ina, bool ok) {
  InaReadout out{};

  if (!ok) {
    out.mV_load = 0;
    out.mA = 0;
    return out;
  }

  float busV = ina.getBusVoltage_V();
  float shunt_mV = ina.getShuntVoltage_mV();
  float loadV = busV + (shunt_mV / 1000.0f);
  float current_mA = ina.getCurrent_mA();

  out.mV_load = clampU16((int)(loadV * 1000.0f + 0.5f));
  out.mA = clampI16((int)current_mA);
  return out;
}

// ====================
// Safe control actions
// ======================
void allOffSafe() {
  digitalWrite(PIN_STATION_EN, DISABLE_LEVEL);
  digitalWrite(PIN_ROVER_EN, DISABLE_LEVEL);
  servoUp();
  mode = MODE_OFF;
}

void stationChargeOn() {
  digitalWrite(PIN_ROVER_EN, DISABLE_LEVEL);
  delay(10);

  servoUp();
  digitalWrite(PIN_STATION_EN, ENABLE_LEVEL);
  mode = MODE_STATION;
}

void roverChargeOnSequence() {
  if (!docked) {
    allOffSafe();
    return;
  }

  digitalWrite(PIN_STATION_EN, DISABLE_LEVEL);
  delay(10);

  servoDown();
  delay(DOCK_SERVO_SETTLE_MS);

  digitalWrite(PIN_ROVER_EN, ENABLE_LEVEL);
  mode = MODE_ROVER;
}

void roverUndockToOff() {
  digitalWrite(PIN_ROVER_EN, DISABLE_LEVEL);
  delay(UNDOCK_PAUSE_MS);

  servoUp();
  mode = MODE_OFF;
}

// ======================
// ESP-NOW structs
// ===================
// Incoming commands from StationWebsite:
//  'O' = all off
//  'S' = station charge
//  'R' = rover charge (requires docked)
//  'U' = toggle dock (manual servo only)
//  'D' = force dock (manual servo only; only if docked)
//  'E' = force release (manual servo only)
typedef struct __attribute__((packed)) {
  char key;
  uint8_t state;   // act only on 1
  uint8_t manual;  // unused; kept compatible
} StationFuncCmd;

// Telemetry to StationWebsite
typedef struct __attribute__((packed)) {
  // Station battery INA219 (26/27)
  uint16_t stBatt_mV;
  int16_t  stBatt_mA;

  // USB input INA219 (21/22)
  uint16_t chgIn_mV;
  int16_t  chgIn_mA;

  // Dock/mode
  uint8_t  dockDown;
  uint8_t  docked;
  uint8_t  mode;

  // Status flags
  uint8_t  usbINA_ok;
  uint8_t  stINA_ok;
} StationFuncTelemetry;

bool espNowReady = false;

// ====================
// ESP-NOW receive
// ======================
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 4)
void onEspNowRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  (void)recv_info;
  if (len != (int)sizeof(StationFuncCmd)) return;

  StationFuncCmd cmd;
  memcpy(&cmd, data, sizeof(cmd));
  if (cmd.state != 1) return;

  switch (cmd.key) {
    case 'O': allOffSafe(); break;
    case 'S': stationChargeOn(); break;
    case 'R': roverChargeOnSequence(); break;

    case 'U':
      if (dockDown) servoUp();
      else { if (docked) servoDown(); else servoUp(); }
      break;
    case 'D':
      if (docked) servoDown();
      else servoUp();
      break;
    case 'E':
      servoUp();
      break;

    default:
      break;
  }
}
#else
void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;
  if (len != (int)sizeof(StationFuncCmd)) return;

  StationFuncCmd cmd;
  memcpy(&cmd, data, sizeof(cmd));
  if (cmd.state != 1) return;

  switch (cmd.key) {
    case 'O': allOffSafe(); break;
    case 'S': stationChargeOn(); break;
    case 'R': roverChargeOnSequence(); break;

    case 'U':
      if (dockDown) servoUp();
      else { if (docked) servoDown(); else servoUp(); }
      break;
    case 'D':
      if (docked) servoDown();
      else servoUp();
      break;
    case 'E':
      servoUp();
      break;

    default:
      break;
  }
}
#endif

void espNowInit() {
  WiFi.mode(WIFI_STA);
  //WiFi.disconnect(true);

  esp_wifi_set_ps(WIFI_PS_NONE);

  if (esp_now_init() != ESP_OK) {
    espNowReady = false;
    return;
  }

  esp_now_register_recv_cb(onEspNowRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, stationWebsiteMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  espNowReady = (esp_now_add_peer(&peerInfo) == ESP_OK);
}

void sendTelemetry() {
  if (!espNowReady) return;

  InaReadout stRead = readINA(inaStationBatt, stINA_ok);
  InaReadout usbRead = readINA(inaUsbIn, usbINA_ok);

  StationFuncTelemetry t{};
  t.stBatt_mV = stRead.mV_load;
  t.stBatt_mA = stRead.mA;

  t.chgIn_mV = usbRead.mV_load;
  t.chgIn_mA = usbRead.mA;

  t.dockDown = dockDown ? 1 : 0;
  t.docked = docked ? 1 : 0;
  t.mode = (uint8_t)mode;

  t.usbINA_ok = usbINA_ok ? 1 : 0;
  t.stINA_ok = stINA_ok ? 1 : 0;

  esp_now_send(stationWebsiteMac, (uint8_t*)&t, sizeof(t));
}

// ======================
// Setup
// ===============
void setup() {
  Serial.begin(115200);
  delay(200);

  // ---------------- Regular WiFi ----------------
  Serial.println("Connecting to WiFi...");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);

  // Improvement: disable WiFi power save to reduce latency/jitter for web + ESP-NOW coexistence
  esp_wifi_set_ps(WIFI_PS_NONE);

  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nWiFi connected.");
  } else {
    Serial.println("\nWiFi connection failed.");
  }

  // ---------------- WiFi WPA2-Enterprise -------------------
  // Use if on school campus

  //  WiFi.disconnect(true);
  //  WiFi.mode(WIFI_STA);

  //  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)identity, strlen(identity));
  //  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)username, strlen(username));
  //  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)schoolPassword, strlen(schoolPassword));

  //  esp_wifi_sta_wpa2_ent_enable();

  //  WiFi.begin(schoolSSID);

  //  Serial.println("Connecting to WPA2-Enterprise WiFi...");

  //  unsigned long startAttemptTime = millis();
  //  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 20000) {
  //  delay(500);
  //  Serial.print(".");
  //  }

  //  if (WiFi.status() == WL_CONNECTED) {
  //    Serial.print("WiFi connected: http://");
  //    Serial.println(WiFi.localIP());
  //  } else {
  //    Serial.println("WiFi connection failed.");
  //  }

  pinMode(PIN_STATION_EN, OUTPUT);
  pinMode(PIN_ROVER_EN, OUTPUT);
  digitalWrite(PIN_STATION_EN, DISABLE_LEVEL);
  digitalWrite(PIN_ROVER_EN, DISABLE_LEVEL);

  pinMode(PIN_DOCK_SWITCH, INPUT_PULLUP);

  dockServo.setPeriodHertz(50);
  dockServo.attach(PIN_SERVO, 500, 2400);

  allOffSafe();

  I2C_USB.begin(SDA_USB, SCL_USB);
  I2C_STB.begin(SDA_STB, SCL_STB);

  usbINA_ok = inaUsbIn.begin(&I2C_USB);
  stINA_ok = inaStationBatt.begin(&I2C_STB);

  espNowInit();

  Serial.println("StationFunction boot:");
  Serial.print("  MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("  ESP-NOW: "); Serial.println(espNowReady ? "OK" : "FAIL");
  Serial.print("  INA USB (21/22): "); Serial.println(usbINA_ok ? "OK" : "NOT FOUND");
  Serial.print("  INA StationBatt (26/27): "); Serial.println(stINA_ok ? "OK" : "NOT FOUND");
  Serial.print("  Dock raw: "); Serial.println(dockRaw() ? "PRESSED" : "released");
  Serial.println("  Default: ALL OFF + SERVO UP");
}

// =======
//  loop
// ==========


void loop() {
  updateDockDebounced();

  if (mode == MODE_ROVER && !docked) {
    roverUndockToOff();
  }

  static uint32_t last = 0;
  if (millis() - last >= 500) {
    last = millis();
    sendTelemetry();
  }

  delay(0);
}
