#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"   // Needed for WPA2 Enterprise
#include "esp_wpa2.h"   // Constant deprecated warning, just ignore
#include <esp_now.h>    // Station to Rover and vice versa
#include <NimBLEDevice.h>   // Rover scans, Station advertises

// ========================
// INA219 (Battery Voltage/Current)
// ========================
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

// ========================
// WIFI SETTINGS
// ========================

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

// ========================
// Ultrasonic Sensor Pins (ROVER)
// ========================
// NOTE: Pins changed to free up SDA=22 / SCL=23 for INA219 I2C
const int trigPin = 13;
const int echoPin = 34;

#define SOUND_SPEED 0.034

long duration;
int16_t distanceMm = -1;

void readDistanceMm() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) {
    distanceMm = -1;
    return;
  }

  float distanceCm = (duration * SOUND_SPEED) / 2.0f;
  distanceMm = (int16_t)(distanceCm * 10.0f);
}

// ====================
// INA219 helpers
// =====================

uint16_t batt_mV = 0;
int16_t  batt_mA = 0;
uint16_t batt_cPct = 0;

// Battery mapping (2S Li-ion)
// Full=8.40V, Empty=6.40V (you can tweak later)
const float FULL_V  = 8.40f;
const float EMPTY_V = 6.40f;

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

static uint16_t clampPctCenti(float pct) {
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;

  int centi = (int)(pct * 100.0f + 0.5f);
  return (uint16_t)centi;
}

void readBatteryINA219() {
  float busV = ina219.getBusVoltage_V();
  float shunt_mV = ina219.getShuntVoltage_mV();
  float loadV = busV + (shunt_mV / 1000.0f);

  float current_mA = ina219.getCurrent_mA();

  int mv = (int)(loadV * 1000.0f + 0.5f);
  batt_mV = clampU16(mv);

  batt_mA = clampI16((int)current_mA);

  float pct = ((loadV - EMPTY_V) / (FULL_V - EMPTY_V)) * 100.0f;
  batt_cPct = clampPctCenti(pct);
}

// ================
// Motor pins
// ==================
const int enable1Pin = 32;  // ENA (PWM)
const int motor1Pin1 = 25;
const int motor1Pin2 = 33;

const int enable2Pin = 14;  // ENB (PWM)
const int motor2Pin1 = 26;
const int motor2Pin2 = 27;

// ========================
// IR Line Sensors (ACTIVE = LOW)
// ========================
const int segpin1 = 21;   // far left
const int segpin2 = 19;   // mid left
const int segpin3 = 18;   // center
const int segpin4 = 4;    // mid right
const int segpin5 = 15;   // far right

// ========================
// PWM Settings
// ====================
const int pwmFreq = 1000;
const int pwmResolution = 8;

// ======================
// Control Variables
// ====================
int baseSpeed = 135;
int turnSpeed = 135;
bool manualMode = false;

// Command tracking (multi-key support)
bool wPressed = false, aPressed = false, sPressed = false, dPressed = false;

// =====================
// Return-to-Home / Docking control
// ========================
bool returnHomeActive = false;
bool docked = false;

// Auto-trigger at about 10%
const uint16_t AUTO_HOME_CPCT = 1000;

// Dock maneuver tuning
const unsigned long DOCK_TURN_MS = 1000;
const unsigned long DOCK_REVERSE_MS = 1000; // unused now (we reverse until 'p')
const unsigned long DOCK_ALIGN_MS = 600;    // reverse-line-follow align time

const unsigned long DOCK_RETRY_REVERSE_MS = 2500;
const unsigned long DOCK_RETRY_FORWARD_MS = 400;
const unsigned long DOCK_RETRY_PAUSE_MS   = 250;
const unsigned long DOCK_BLIND_REVERSE_MS = 800;

unsigned long dockStepStart = 0;
// 0=not docking, 1=turn (unused), 2=reverse-straight, 3=stopped, 4=reverse-line-follow-align
int dockStep = 0;

// ========================
// Line-following recovery
// ========================
int lastLineDirection = 0; // -1 = left, 0 = center, 1 = right
unsigned long recoveryStart = 0;
bool recovering = false;
int recoveryPhase = 0;

const unsigned long creepTime  = 100;
//const unsigned long exesivetime  = 500;
const unsigned long reverseTime = 350;
const unsigned long pivotTime   = 220;

// ========================
// Forward declarations
// =====================
void autoLineFollowBackward();

// ===================
// PWM helpers
// ========================
void setSpeed(int left, int right) {
  left = constrain(left, 0, 255);
  right = constrain(right, 0, 255);
  ledcWrite(enable1Pin, left);
  ledcWrite(enable2Pin, right);
}

// ===================
// Motor control
// ==================
void motor1Forward()  { digitalWrite(motor1Pin1, HIGH); digitalWrite(motor1Pin2, LOW); }
void motor1Backward() { digitalWrite(motor1Pin1, LOW);  digitalWrite(motor1Pin2, HIGH); }
void motor2Forward()  { digitalWrite(motor2Pin1, HIGH); digitalWrite(motor2Pin2, LOW); }
void motor2Backward() { digitalWrite(motor2Pin1, LOW);  digitalWrite(motor2Pin2, HIGH); }

void bothStop() {
  digitalWrite(motor1Pin1, LOW); digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW); digitalWrite(motor2Pin2, LOW);
  setSpeed(0, 0);
}

void forward() { motor1Forward(); motor2Forward(); setSpeed(baseSpeed, baseSpeed); }
void moveBackward() { motor1Backward(); motor2Backward(); setSpeed(baseSpeed, baseSpeed); }
void slightRight() { motor1Forward(); motor2Forward(); setSpeed(turnSpeed, baseSpeed); }
void slightLeft() { motor1Forward(); motor2Forward(); setSpeed(baseSpeed, turnSpeed); }
void hardRight() { motor1Backward(); motor2Forward(); setSpeed(baseSpeed, baseSpeed); }
void hardLeft() { motor1Forward(); motor2Backward(); setSpeed(baseSpeed, baseSpeed); }
void softLeft()  { motor1Forward(); motor2Forward(); setSpeed(baseSpeed - 25, baseSpeed + 25); }
void softRight() { motor1Forward(); motor2Forward(); setSpeed(baseSpeed + 25, baseSpeed - 25); }

//void revRight()  { motor1Forward(); motor2Backward(); setSpeed(baseSpeed , baseSpeed ); }
//void revLeft() { motor1Backward(); motor2Forward(); setSpeed(baseSpeed , baseSpeed ); }
//void slrevRight()  { motor1Backward(); motor2Backward(); setSpeed(baseSpeed -35, baseSpeed ); }
//void slrevLeft() { motor1Backward(); motor2Backward(); setSpeed(baseSpeed , baseSpeed - 35); }
// ========================
// Reverse docking helpers
// Front-wheel drive rover:
// both motors are at the front, so while reversing we want
// gentle speed differences, NOT pivot spins.
// ========================
//const int dockBaseSpeed = 128;
//const int dockSoftOffset = 8;
//const int dockHardOffset = 16;
//
//void revStraight() {
//  motor1Backward();
//  motor2Backward();
//  setSpeed(dockBaseSpeed, dockBaseSpeed);
//}
//
//// These names describe the correction direction you WANT,
//// but you should test once physically to confirm they match.
//void revBiasRightSoft() {
//  motor1Backward();
//  motor2Backward();
//  setSpeed(dockBaseSpeed - dockSoftOffset, dockBaseSpeed);
//}
//
//void revBiasLeftSoft() {
//  motor1Backward();
//  motor2Backward();
//  setSpeed(dockBaseSpeed, dockBaseSpeed - dockSoftOffset);
//}
//
//void revBiasRightHard() {
//  motor1Backward();
//  motor2Backward();
//  setSpeed(dockBaseSpeed - dockHardOffset, dockBaseSpeed);
//}
//
//void revBiasLeftHard() {
//  motor1Backward();
//  motor2Backward();
//  setSpeed(dockBaseSpeed, dockBaseSpeed - dockHardOffset);
//}

// ========================
// Manual buttons handling (ESP-NOW)
// ========================
void handleManualControls() {
  if (wPressed) forward();
  else if (sPressed) moveBackward();
  else if (aPressed) hardLeft();
  else if (dPressed) hardRight();
  else bothStop();
}

// ==================================================
// Auto line-follow with recovery & hard left or right turn detection
// =====================================
void autoLineFollow() {
  bool s1 = !digitalRead(segpin1);
  bool s2 = !digitalRead(segpin2);
  bool s3 = !digitalRead(segpin3);
  bool s4 = !digitalRead(segpin4);
  bool s5 = !digitalRead(segpin5);

  // ultasonic 

  if (s1 || s2 || s3 || s4 || s5) {
    recovering = false;
    recoveryPhase = 0;

    if (s1 && s2 && s3 && s4 && s5) {
      forward();
      lastLineDirection = 0;
      return;
    }

    if ((s1 || s2) && !s3 && !s4 && !s5) {
      hardLeft();
      lastLineDirection = -1;
      return;
    }

    if ((s4 || s5) && !s3 && !s1 && !s2) {
      hardRight();
      lastLineDirection = 1;
      return;
    }

    if (s3 && !(s2 || s4 || s1 || s5)) {
      forward();
      lastLineDirection = 0;
      return;
    } else if (s2 || s4) {
      if (s2 && !s4) {
        softLeft();
        lastLineDirection = -1;
        return;
      }
      if (s4 && !s2) {
        softRight();
        lastLineDirection = 1;
        return;
      }

      forward();
      lastLineDirection = 0;
      return;
    } else if (s1 || s5) {
      if (s1 && !s5) {
        hardLeft();
        lastLineDirection = -1;
        return;
      }
      if (s5 && !s1) {
        hardRight();
        lastLineDirection = 1;
        return;
      }

      forward();
      lastLineDirection = 0;
      return;
    }

    forward();
    lastLineDirection = 0;
    return;
  }

  if (!recovering) {
    recovering = true;
    recoveryStart = millis();
    recoveryPhase = 0;
  }

  unsigned long now = millis();

  if (recoveryPhase == 0) {
    motor1Forward();
    motor2Forward();
    setSpeed(baseSpeed - 25, baseSpeed - 25);

    if (now - recoveryStart >= creepTime) {
      recoveryPhase = 1;
      recoveryStart = now;
    }
  }
  else if (recoveryPhase == 1) {
    motor1Backward();
    motor2Backward();
    setSpeed(baseSpeed - 25, baseSpeed - 25);

    if (now - recoveryStart >= reverseTime) {
      recoveryPhase = 2;
      recoveryStart = now;
    }
  }
  else if (recoveryPhase == 2) {
    if (lastLineDirection == -1) hardLeft();
    else if (lastLineDirection == 1) hardRight();
    else if (lastLineDirection == 2) moveBackward();
    else forward();

    if (now - recoveryStart >= pivotTime) {
      bothStop();
      recoveryPhase = 3;
    }
  }
}

// ======================
// Strong reverse steering helpers
// ========================
const int dockBaseSpeed = 140;
const int dockTurnBoost = 35;

// Straight reverse
void revStraight() {
  motor1Backward();
  motor2Backward();
  setSpeed(dockBaseSpeed, dockBaseSpeed);
}

// Backward arc right
void revArcRight() {
  motor1Backward();
  motor2Backward();
  setSpeed(dockBaseSpeed - dockTurnBoost, dockBaseSpeed);
}

// Backward arc left
void revArcLeft() {
  motor1Backward();
  motor2Backward();
  setSpeed(dockBaseSpeed, dockBaseSpeed - dockTurnBoost);
}

// Hard reverse pivot right
void revPivotRight() {
  motor1Forward();
  motor2Backward();
  setSpeed(dockBaseSpeed, dockBaseSpeed);
}

// Hard reverse pivot left
void revPivotLeft() {
  motor1Backward();
  motor2Forward();
  setSpeed(dockBaseSpeed, dockBaseSpeed);
}

// ========================
// Reverse line-follow (used for docking alignment)
// ======================
void autoLineFollowBackward() {
  bool s1 = !digitalRead(segpin1);
  bool s2 = !digitalRead(segpin2);
  bool s3 = !digitalRead(segpin3);
  bool s4 = !digitalRead(segpin4);
  bool s5 = !digitalRead(segpin5);

  if (s1 || s2 || s3 || s4 || s5) {
    if (s1 && s2 && s3 && s4 && s5) {
      revStraight();
      lastLineDirection = 0;
      return;
    }

    if ((s1 || s2) && !s3 && !s4 && !s5) {
      revPivotRight();
      lastLineDirection = -1;
      return;
    }

    if ((s4 || s5) && !s3 && !s1 && !s2) {
      revPivotLeft();
      lastLineDirection = 1;
      return;
    }

    if (s3 && !(s2 || s4 || s1 || s5)) {
      revStraight();
      lastLineDirection = 0;
      return;
    }

    if (s2 || s4) {
      if (s2 && !s4) {
        revArcRight();
        lastLineDirection = -1;
        return;
      }
      if (s4 && !s2) {
        revArcLeft();
        lastLineDirection = 1;
        return;
      }

      revStraight();
      lastLineDirection = 0;
      return;
    }

    if (s1 || s5) {
      if (s1 && !s5) {
        revPivotRight();
        lastLineDirection = -1;
        return;
      }
      if (s5 && !s1) {
        revPivotLeft();
        lastLineDirection = 1;
        return;
      }

      revStraight();
      lastLineDirection = 0;
      return;
    }

    revStraight();
    lastLineDirection = 0;
    return;
  }

  if (lastLineDirection == -1) {
    revPivotRight();
  } else if (lastLineDirection == 1) {
    revPivotLeft();
  } else {
    revStraight();
  }
}

// ======================
// Return Home behavior
// ========================
void startReturnHome() {
  returnHomeActive = true;
  docked = false;
  dockStep = 0;
  dockStepStart = 0;
  manualMode = false;
  bothStop();
}

void cancelReturnHome() {
  returnHomeActive = false;
  dockStep = 0;
  dockStepStart = 0;
}

void setDockedStop() {
  docked = true;
  returnHomeActive = false;
  dockStep = 3;
  bothStop();
}

void runReturnHome() {
  if (docked) {
    bothStop();
    return;
  }

  bool s1 = !digitalRead(segpin1);
  bool s2 = !digitalRead(segpin2);
  bool s3 = !digitalRead(segpin3);
  bool s4 = !digitalRead(segpin4);
  bool s5 = !digitalRead(segpin5);

  if (dockStep == 1) {
    moveBackward();

    if (millis() - dockStepStart >= DOCK_BLIND_REVERSE_MS) {
      dockStep = 2;
      dockStepStart = millis();
    }
    return;
  }

  if (dockStep == 2) {
    autoLineFollowBackward();

    if (millis() - dockStepStart >= DOCK_RETRY_REVERSE_MS) {
      dockStep = 5;
      dockStepStart = millis();
    }
    return;
  }

  if (dockStep == 5) {
    autoLineFollow();

    if (millis() - dockStepStart >= DOCK_RETRY_FORWARD_MS) {
      dockStep = 6;
      dockStepStart = millis();
      bothStop();
    }
    return;
  }

  if (dockStep == 6) {
    bothStop();

    if (millis() - dockStepStart >= DOCK_RETRY_PAUSE_MS) {
      dockStep = 1;
      dockStepStart = millis();
    }
    return;
  }

  if (dockStep == 3) {
    bothStop();
    return;
  }

  if (s1 && s2 && s3 && s4 && s5) {
    dockStep = 1;
    dockStepStart = millis();
    return;
  }

  autoLineFollow();
}

// =====================================================
// ESP-NOW data structs (must match the charging station)
// =====================================================
uint8_t stationMac[6] = {0x00,0x4B,0x12,0x9B,0x14,0x10};  // charging station MAC address

typedef struct __attribute__((packed)) {
  char key;
  uint8_t state;
  uint8_t manual;
} RoverCmd;

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

bool espNowReady = false;

// ESP32 core 3.x / IDF5 callback signatures:
void onEspNowSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  (void)tx_info;
  (void)status;
}

void onEspNowRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  (void)recv_info;
  if (len != (int)sizeof(RoverCmd)) return;

  RoverCmd cmd;
  memcpy(&cmd, data, sizeof(cmd));

  char k = cmd.key;
  uint8_t state = cmd.state;

  if (k == 'w') wPressed = state;
  else if (k == 'a') aPressed = state;
  else if (k == 's') sPressed = state;
  else if (k == 'd') dPressed = state;

  if (docked && (k == 'w' || k == 'a' || k == 's' || k == 'd') && state == 1) {
    docked = false;
    dockStep = 0;
    dockStepStart = 0;
    returnHomeActive = false;
  }

  if (k == 'm' && state == 1) {
    manualMode = !manualMode;
    cancelReturnHome();
    bothStop();
  }

  if ((k == 'w' || k == 'a' || k == 's' || k == 'd') && state == 1) {
    manualMode = true;
    cancelReturnHome();
  }

  if (k == 'h' && state == 1) startReturnHome();
  if (k == 'c' && state == 1) cancelReturnHome();

  if (k == 'p' && state != 0) setDockedStop();

  if (k == 'u' && state == 1) {
    docked = false;
    returnHomeActive = false;
    dockStep = 0;
    dockStepStart = 0;

    // Choose what you want after undock:
    // Option A: go back into AUTO line follow
    manualMode = true;

    wPressed = false;
    aPressed = false;
    sPressed = false;
    dPressed = false;

    bothStop();
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
  memcpy(peerInfo.peer_addr, stationMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    espNowReady = true;
    Serial.println("ESP-NOW peer added (station)");
  } else {
    espNowReady = false;
    Serial.println("ESP-NOW peer NOT added (set stationMac first)");
  }
}

//// ====================================================================================================
//// BLE Dock RSSI Monitoring (Rover scans) (FURTHER UPDATE: NO LONGER USING THIS DUE TO NEW METHOD OF RETURNING TO DOCK)
//// =======================================================================================
//
//// Must match the station's advertised service UUID
//static NimBLEUUID DOCK_SERVICE_UUID("12345678-1234-1234-1234-1234567890ab");
//
//// Latest dock RSSI (dBm). -127 means "not seen yet"
//volatile int8_t dockRssi = -127;
//volatile unsigned long dockLastSeenMs = 0;
//
//class DockScanCallbacks : public NimBLEScanCallbacks {
//  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
//    bool match = false;
//
//    if (advertisedDevice->haveServiceUUID()) {
//      NimBLEUUID uuid = advertisedDevice->getServiceUUID();
//      if (uuid.equals(DOCK_SERVICE_UUID)) {
//        match = true;
//      }
//    }
//
//    if (match) {
//      int rssi = advertisedDevice->getRSSI();
//      if (rssi < -127) rssi = -127;
//      if (rssi > 0) rssi = 0;
//
//      dockRssi = (int8_t)rssi;
//      dockLastSeenMs = millis();
//    }
//  }
//};
//
//NimBLEScan* pScan = nullptr;
//
//void bleDockScanInit() {
//  NimBLEDevice::init("");
//  pScan = NimBLEDevice::getScan();
//  pScan->setScanCallbacks(new DockScanCallbacks(), false);
//
//  pScan->setActiveScan(false);
//  pScan->setInterval(45);
//  pScan->setWindow(15);
//
//  pScan->start(0, true, true);
//  Serial.println("BLE dock scan started");
//}

void sendTelemetry() {
  if (!espNowReady) return;

  RoverTelemetry t;
  t.manual = manualMode ? 1 : 0;

  t.s1 = !digitalRead(segpin1);
  t.s2 = !digitalRead(segpin2);
  t.s3 = !digitalRead(segpin3);
  t.s4 = !digitalRead(segpin4);
  t.s5 = !digitalRead(segpin5);

  readDistanceMm();
  t.distanceMm = distanceMm;

  t.dockRssi = dockRssi;

  unsigned long now = millis();
  unsigned long age = (dockLastSeenMs == 0) ? 65535UL : (now - dockLastSeenMs);
  if (age > 65535UL) age = 65535UL;
  t.dockAgeMs = (uint16_t)age;

  readBatteryINA219();
  t.batt_mV = batt_mV;
  t.batt_mA = batt_mA;
  t.batt_cPct = batt_cPct;

  t.docked = docked ? 1 : 0;
  t.returning = returnHomeActive ? 1 : 0;

  esp_now_send(stationMac, (uint8_t*)&t, sizeof(t));
}

// =================
// Setup
// ========================
void setup() {
  Serial.begin(115200);
  delay(800);

  // INA219 I2C
  Wire.begin(22, 23);
  if (!ina219.begin()) {
    Serial.println("INA219 not found (check wiring/address)");
  } else {
    Serial.println("INA219 OK");
  }

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connection failed.");
  }

  // Ultrasonic pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Motor pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // PWM attach (ESP32 core 3.x style)
  ledcAttach(enable1Pin, pwmFreq, pwmResolution);
  ledcAttach(enable2Pin, pwmFreq, pwmResolution);
  bothStop();

  // Line sensors
  pinMode(segpin1, INPUT_PULLUP);
  pinMode(segpin2, INPUT_PULLUP);
  pinMode(segpin3, INPUT_PULLUP);
  pinMode(segpin4, INPUT_PULLUP);
  pinMode(segpin5, INPUT_PULLUP);

  // ESP-NOW
  espNowInit();

  // BLE scan for dock RSSI
  bleDockScanInit();

  Serial.println("ESP32 Rover Ready");
}

// ========================
// Loop
// ========================
void loop() {
  if (docked) {
    bothStop();
  }
  else if (returnHomeActive) {
    runReturnHome();
  }
  else if (manualMode) {
    handleManualControls();
  }
  else {
    autoLineFollow();
  }

  // Telemetry every 250 ms (4 Hz)
  static unsigned long lastTel = 0;
  if (millis() - lastTel >= 250) {
    lastTel = millis();
    sendTelemetry();

    if (!docked && !returnHomeActive && batt_cPct <= AUTO_HOME_CPCT) {
      startReturnHome();
    }
  }

  delay(10);
}
