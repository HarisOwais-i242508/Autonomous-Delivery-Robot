/*
  ESP32 car motion controller for RL grid project.

  Features:
  - MPU-6050 / MPU-6500 IMU init (accepts WHO_AM_I = 0x68 or 0x70)
  - Relative heading reference (straight = 0 deg at startup)
  - Approx 90-degree turn primitives with IMU closed-loop stop window
  - One-cell forward burst with heading hold
  - MQTT JSON command/ack protocol:
      command topic: car/command
      status topic:  car/status
      telemetry:     car/telemetry
*/

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_AHRS.h>

// -------------------- User Wi-Fi / MQTT --------------------
const char *WIFI_SSID = "flash";
const char *WIFI_PASS = "p@kist@n2@22";

const char *MQTT_HOST = "192.168.100.18";
const uint16_t MQTT_PORT = 1883;
const char *MQTT_USER = "";
const char *MQTT_PASS = "";

const char *TOPIC_COMMAND = "car/command";
const char *TOPIC_STATUS = "car/status";
const char *TOPIC_TELEMETRY = "car/telemetry";

// -------------------- Pin map --------------------
const int IN1 = 25;
const int IN2 = 26;
const int IN3 = 27;
const int IN4 = 14;
const int E1 = 33;
const int E2 = 32;

const int SCL_PIN = 22;
const int SDA_PIN = 21;

// -------------------- IMU --------------------
Adafruit_Madgwick filter;
uint8_t imuAddr = 0x68;

static const float G = 9.80665f;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_CONFIG = 0x1A;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t REG_WHO_AM_I = 0x75;

float accelLsbPerG = 4096.0f;    // +-8g
float gyroLsbPerDegS = 65.5f;    // +-500 dps

float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f; // rad/s
float accBiasX = 0.0f, accBiasY = 0.0f, accBiasZ = 0.0f;    // m/s^2

float yawDeg = 0.0f;
float referenceYawDeg = 0.0f;
float relativeYawDeg = 0.0f;
unsigned long lastImuUs = 0;

// If right-turn yaw goes negative on your car, set to -1.
const int YAW_RIGHT_SIGN = 1;

// -------------------- Motion tuning --------------------
const int PWM_FREQ = 2000;
const int PWM_RES_BITS = 8;
const int PWM_CH_LEFT = 0;
const int PWM_CH_RIGHT = 1;

int desiredHeadingIndex = 0; // 0=N,1=E,2=S,3=W in RELATIVE grid frame

const int BASE_FWD_PWM = 170;
const float FWD_HEADING_KP = 2.6f;
const unsigned long FORWARD_CELL_MS = 760;    // tune on floor for 12 inch

const int BASE_TURN_PWM = 170;
const float TURN_KP = 1.9f;
const float TURN_TOL_DEG = 7.5f;              // near-90 tolerance
const unsigned long TURN_TIMEOUT_MS = 3200;
const uint8_t TURN_STABLE_TICKS = 8;

// -------------------- MQTT command state --------------------
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

volatile bool pendingCommand = false;
String pendingAction = "";
int pendingActionId = -1;

unsigned long lastTelemetryMs = 0;
bool busyExecuting = false;

// -------------------- Helpers --------------------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
void pwmInit() {
  // New ESP32 core (3.x): channel is managed internally per pin.
  ledcAttach(E1, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(E2, PWM_FREQ, PWM_RES_BITS);
}

void pwmWriteLeft(int duty) {
  ledcWrite(E1, duty);
}

void pwmWriteRight(int duty) {
  ledcWrite(E2, duty);
}
#else
void pwmInit() {
  // Legacy ESP32 core (2.x): explicit channel setup + pin attach.
  ledcSetup(PWM_CH_LEFT, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(E1, PWM_CH_LEFT);
  ledcAttachPin(E2, PWM_CH_RIGHT);
}

void pwmWriteLeft(int duty) {
  ledcWrite(PWM_CH_LEFT, duty);
}

void pwmWriteRight(int duty) {
  ledcWrite(PWM_CH_RIGHT, duty);
}
#endif

float wrapAngle180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

float headingIndexToRelDeg(int idx) {
  float d = (idx % 4) * 90.0f * (float)YAW_RIGHT_SIGN;
  return wrapAngle180(d);
}

bool imuWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool imuReadRegs(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t got = Wire.requestFrom((int)imuAddr, (int)len);
  if (got != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

uint8_t readWhoAmI(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(REG_WHO_AM_I);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom((int)addr, 1) != 1) return 0xFF;
  return Wire.read();
}

void readImu(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint8_t b[14];
  if (!imuReadRegs(REG_ACCEL_XOUT_H, b, 14)) {
    ax = ay = az = gx = gy = gz = 0.0f;
    return;
  }

  int16_t rawAx = (int16_t)((b[0] << 8) | b[1]);
  int16_t rawAy = (int16_t)((b[2] << 8) | b[3]);
  int16_t rawAz = (int16_t)((b[4] << 8) | b[5]);
  int16_t rawGx = (int16_t)((b[8] << 8) | b[9]);
  int16_t rawGy = (int16_t)((b[10] << 8) | b[11]);
  int16_t rawGz = (int16_t)((b[12] << 8) | b[13]);

  ax = (rawAx / accelLsbPerG) * G;
  ay = (rawAy / accelLsbPerG) * G;
  az = (rawAz / accelLsbPerG) * G;

  gx = (rawGx / gyroLsbPerDegS) * DEG_TO_RAD; // rad/s
  gy = (rawGy / gyroLsbPerDegS) * DEG_TO_RAD;
  gz = (rawGz / gyroLsbPerDegS) * DEG_TO_RAD;
}

void stopMotors() {
  pwmWriteLeft(0);
  pwmWriteRight(0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void setMotorRaw(int leftPwmSigned, int rightPwmSigned) {
  int l = constrain(abs(leftPwmSigned), 0, 255);
  int r = constrain(abs(rightPwmSigned), 0, 255);

  if (leftPwmSigned >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  if (rightPwmSigned >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  pwmWriteLeft(l);
  pwmWriteRight(r);
}

bool initImu() {
  uint8_t who68 = readWhoAmI(0x68);
  uint8_t who69 = readWhoAmI(0x69);
  Serial.print("WHO_AM_I @0x68 = 0x");
  Serial.println(who68, HEX);
  Serial.print("WHO_AM_I @0x69 = 0x");
  Serial.println(who69, HEX);

  if (who68 == 0x68 || who68 == 0x70) {
    imuAddr = 0x68;
  } else if (who69 == 0x68 || who69 == 0x70) {
    imuAddr = 0x69;
  } else {
    Serial.println("No MPU6050/6500 found.");
    return false;
  }

  if (!imuWriteReg(REG_PWR_MGMT_1, 0x00)) return false; // wake
  delay(80);
  if (!imuWriteReg(REG_CONFIG, 0x04)) return false;      // DLPF
  if (!imuWriteReg(REG_GYRO_CONFIG, 0x08)) return false; // +-500 dps
  if (!imuWriteReg(REG_ACCEL_CONFIG, 0x10)) return false;// +-8g

  filter.begin(100.0f); // fallback rate; we pass dt explicitly.
  Serial.print("IMU ready at 0x");
  Serial.println(imuAddr, HEX);
  return true;
}

void calibrateImu(uint16_t samples = 700) {
  Serial.println("Keep car still... calibrating IMU");
  delay(900);
  float sx = 0, sy = 0, sz = 0;
  float sax = 0, say = 0, saz = 0;
  float ax, ay, az, gx, gy, gz;
  for (uint16_t i = 0; i < samples; i++) {
    readImu(ax, ay, az, gx, gy, gz);
    sx += gx; sy += gy; sz += gz;
    sax += ax; say += ay; saz += az;
    delay(3);
  }
  gyroBiasX = sx / samples;
  gyroBiasY = sy / samples;
  gyroBiasZ = sz / samples;
  accBiasX = sax / samples;
  accBiasY = say / samples;
  accBiasZ = (saz / samples) - G;
  Serial.println("Calibration done.");
}

void updateFusion() {
  float ax, ay, az, gx, gy, gz;
  readImu(ax, ay, az, gx, gy, gz);
  gx -= gyroBiasX;
  gy -= gyroBiasY;
  gz -= gyroBiasZ;
  ax -= accBiasX;
  ay -= accBiasY;
  az -= accBiasZ;

  unsigned long nowUs = micros();
  float dt = (nowUs - lastImuUs) * 1.0e-6f;
  lastImuUs = nowUs;
  if (dt <= 0.00005f || dt > 0.2f) dt = 0.01f;

  // Adafruit updateIMU expects gyro in deg/s.
  filter.updateIMU(gx / DEG_TO_RAD, gy / DEG_TO_RAD, gz / DEG_TO_RAD, ax, ay, az, dt);
  yawDeg = filter.getYaw();
  relativeYawDeg = wrapAngle180(yawDeg - referenceYawDeg);
}

void captureStraightReference() {
  float ax, ay, az, gx, gy, gz;
  unsigned long prevUs = micros();
  for (uint16_t i = 0; i < 140; i++) {
    readImu(ax, ay, az, gx, gy, gz);
    gx -= gyroBiasX; gy -= gyroBiasY; gz -= gyroBiasZ;
    ax -= accBiasX; ay -= accBiasY; az -= accBiasZ;
    unsigned long nowUs = micros();
    float dt = (nowUs - prevUs) * 1.0e-6f;
    prevUs = nowUs;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.005f;
    filter.updateIMU(gx / DEG_TO_RAD, gy / DEG_TO_RAD, gz / DEG_TO_RAD, ax, ay, az, dt);
    delay(4);
  }
  referenceYawDeg = filter.getYaw();
  desiredHeadingIndex = 0;
  relativeYawDeg = 0.0f;
  Serial.print("Straight reference yaw: ");
  Serial.println(referenceYawDeg, 2);
}

void publishStatus(int actionId, const char *status, bool ok, const char *detail) {
  StaticJsonDocument<256> doc;
  doc["v"] = 1;
  doc["action_id"] = actionId;
  doc["status"] = status;
  doc["ok"] = ok;
  doc["ts_ms"] = millis();
  if (detail && detail[0] != '\0') doc["detail"] = detail;
  char out[256];
  size_t n = serializeJson(doc, out, sizeof(out));
  mqttClient.publish(TOPIC_STATUS, out, n);
}

void publishTelemetry() {
  StaticJsonDocument<192> doc;
  doc["v"] = 1;
  doc["yaw_rel_deg"] = relativeYawDeg;
  doc["heading_idx"] = desiredHeadingIndex;
  doc["busy"] = busyExecuting;
  doc["ts_ms"] = millis();
  char out[192];
  size_t n = serializeJson(doc, out, sizeof(out));
  mqttClient.publish(TOPIC_TELEMETRY, out, n);
}

bool turnToTargetRel(float targetRelDeg) {
  unsigned long start = millis();
  uint8_t stableTicks = 0;

  while (millis() - start < TURN_TIMEOUT_MS) {
    updateFusion();
    float err = wrapAngle180(targetRelDeg - relativeYawDeg);
    float absErr = fabs(err);

    if (absErr <= TURN_TOL_DEG) {
      stableTicks++;
      stopMotors();
      if (stableTicks >= TURN_STABLE_TICKS) {
        return true;
      }
      delay(8);
      continue;
    }
    stableTicks = 0;

    int pwm = constrain((int)(BASE_TURN_PWM + TURN_KP * absErr), 120, 230);
    if (err > 0) {
      // Need to increase relative yaw => spin right if YAW_RIGHT_SIGN==1.
      if (YAW_RIGHT_SIGN == 1) {
        setMotorRaw(+pwm, -pwm);
      } else {
        setMotorRaw(-pwm, +pwm);
      }
    } else {
      if (YAW_RIGHT_SIGN == 1) {
        setMotorRaw(-pwm, +pwm);
      } else {
        setMotorRaw(+pwm, -pwm);
      }
    }
    delay(6);
  }

  stopMotors();
  return false;
}

bool doTurnRight() {
  desiredHeadingIndex = (desiredHeadingIndex + 1) % 4;
  float targetRel = headingIndexToRelDeg(desiredHeadingIndex);
  bool ok = turnToTargetRel(targetRel);
  stopMotors();
  delay(80);
  return ok;
}

bool doTurnLeft() {
  desiredHeadingIndex = (desiredHeadingIndex + 3) % 4;
  float targetRel = headingIndexToRelDeg(desiredHeadingIndex);
  bool ok = turnToTargetRel(targetRel);
  stopMotors();
  delay(80);
  return ok;
}

bool doForwardOneCell() {
  unsigned long start = millis();
  float targetRel = headingIndexToRelDeg(desiredHeadingIndex);

  while (millis() - start < FORWARD_CELL_MS) {
    updateFusion();
    float err = wrapAngle180(targetRel - relativeYawDeg);
    int corr = (int)(FWD_HEADING_KP * err);
    int left = constrain(BASE_FWD_PWM + corr, 120, 230);
    int right = constrain(BASE_FWD_PWM - corr, 120, 230);
    setMotorRaw(left, right);
    delay(6);
  }

  stopMotors();
  delay(80);
  return true;
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  if (strcmp(topic, TOPIC_COMMAND) != 0 || length == 0) return;

  String data;
  data.reserve(length);
  for (unsigned int i = 0; i < length; i++) data += (char)payload[i];

  String action = "";
  int actionId = -1;

  if (data[0] == '{') {
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, data);
    if (!err) {
      if (doc["action"].is<const char *>()) action = String((const char *)doc["action"]);
      if (doc["action_id"].is<int>()) actionId = doc["action_id"].as<int>();
    }
  } else {
    action = data;
  }

  if (action.length() == 0) return;
  pendingAction = action;
  pendingActionId = actionId;
  pendingCommand = true;
}

void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());
}

void connectMqtt() {
  if (mqttClient.connected()) return;
  while (!mqttClient.connected()) {
    Serial.print("MQTT connecting...");
    bool ok;
    if (strlen(MQTT_USER) > 0) {
      ok = mqttClient.connect("car-esp32-controller", MQTT_USER, MQTT_PASS);
    } else {
      ok = mqttClient.connect("car-esp32-controller");
    }
    if (ok) {
      Serial.println("ok");
      mqttClient.subscribe(TOPIC_COMMAND);
      publishStatus(-1, "IDLE", true, "controller_online");
    } else {
      Serial.print("fail rc=");
      Serial.println(mqttClient.state());
      delay(1000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(250);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);

  pwmInit();
  stopMotors();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(120);

  if (!initImu()) {
    while (true) {
      delay(1000);
    }
  }
  calibrateImu();
  lastImuUs = micros();
  captureStraightReference();

  connectWifi();
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  connectMqtt();
}

void executePending() {
  if (!pendingCommand) return;
  pendingCommand = false;

  String action = pendingAction;
  int actionId = pendingActionId;
  action.trim();
  action.toUpperCase();

  busyExecuting = true;
  bool ok = false;
  const char *status = "ERROR";
  const char *detail = "";

  if (action == "FORWARD") {
    ok = doForwardOneCell();
    status = ok ? "DONE_FORWARD" : "ERROR";
  } else if (action == "TURN_LEFT") {
    ok = doTurnLeft();
    status = ok ? "DONE_LEFT" : "ERROR";
  } else if (action == "TURN_RIGHT") {
    ok = doTurnRight();
    status = ok ? "DONE_RIGHT" : "ERROR";
  } else if (action == "STOP") {
    stopMotors();
    ok = true;
    status = "IDLE";
  } else {
    ok = false;
    status = "ERROR";
    detail = "unknown_action";
  }

  busyExecuting = false;
  publishStatus(actionId, status, ok, detail);
}

void loop() {
  connectWifi();
  connectMqtt();
  mqttClient.loop();

  updateFusion();
  executePending();

  if (millis() - lastTelemetryMs >= 500) {
    lastTelemetryMs = millis();
    publishTelemetry();
    Serial.print("RelYaw=");
    Serial.print(relativeYawDeg, 1);
    Serial.print(" headIdx=");
    Serial.print(desiredHeadingIndex);
    Serial.print(" busy=");
    Serial.println(busyExecuting ? "1" : "0");
  }
}
