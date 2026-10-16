#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

constexpr uint8_t SCREEN_WIDTH = 128;
constexpr uint8_t SCREEN_HEIGHT = 64;
constexpr uint8_t OLED_ADDRESS = 0x3C;

constexpr uint8_t BUTTON1_PIN = 21;
constexpr uint8_t BUTTON2_PIN = 20;
constexpr uint8_t BUTTON3_PIN = 10;

constexpr uint8_t POT_PIN = 2;
constexpr uint8_t JOY_X_PIN = 0;
constexpr uint8_t JOY_Y_PIN = 3;

constexpr uint8_t SDA_PIN = 8;
constexpr uint8_t SCL_PIN = 9;

constexpr uint16_t SEND_INTERVAL_MS = 50;
constexpr uint16_t SCREEN_INTERVAL_MS = 100;
constexpr uint16_t TELEMETRY_TIMEOUT_MS = 1200;
constexpr uint16_t ADC_MAX = 4095;

// Put your measured Hall joystick calibration here.
constexpr int JOY_X_MIN = 150;
constexpr int JOY_X_CENTER = 2048;
constexpr int JOY_X_MAX = 3950;
constexpr int JOY_Y_MIN = 150;
constexpr int JOY_Y_CENTER = 2048;
constexpr int JOY_Y_MAX = 3950;

constexpr uint8_t PAGE_FLIGHT = 0;
constexpr uint8_t PAGE_PID = 1;
constexpr uint8_t PAGE_MOTORS = 2;
constexpr uint8_t PAGE_DEBUG = 3;
constexpr uint8_t PAGE_BATTERY = 4;
constexpr uint8_t PAGE_COUNT = 5;

constexpr uint8_t CMD_NONE = 0;
constexpr uint8_t CMD_PID_TUNE = 1;
constexpr uint8_t CMD_MOTOR_TEST = 2;

constexpr uint8_t PID_AXIS_PITCH_ID = 0;
constexpr uint8_t PID_AXIS_ROLL_ID = 1;
constexpr uint8_t PID_AXIS_YAW_ID = 2;
constexpr uint8_t PID_AXIS_ALTITUDE_ID = 3;

constexpr uint8_t PID_TERM_P_ID = 0;
constexpr uint8_t PID_TERM_I_ID = 1;
constexpr uint8_t PID_TERM_D_ID = 2;

uint8_t DRONE_MAC[] = {0x28, 0x84, 0x85, 0x57, 0xCA, 0x54};

struct ControllerMessage {
  int b1;
  int b2;
  int b3;
  float pot;
  float x;
  float y;
  uint8_t page;
  uint8_t command;
  uint8_t pidAxis;
  uint8_t pidTerm;
  uint8_t motorTestIndex;
  float commandValue;
};

struct TelemetryMessage {
  float alt;
  float temp;
  float m1;
  float m2;
  float m3;
  float m4;
  float yaw;
  float roll;
  float pitch;
  float batteryVoltage;
  bool armed;
  bool hold;
  bool motorTestActive;
};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

ControllerMessage control = {};
TelemetryMessage telemetry = {};

float pidValues[4][3] = {
  {4.0f, 0.0f, 1.2f},
  {4.0f, 0.0f, 1.2f},
  {2.0f, 0.0f, 0.4f},
  {20.0f, 0.0f, 4.0f},
};

bool espNowReady = false;
bool sendFailed = false;
uint8_t page = PAGE_FLIGHT;
uint8_t selectedPidAxis = PID_AXIS_PITCH_ID;
uint8_t selectedPidTerm = PID_TERM_P_ID;
uint8_t selectedMotor = 0;
int lastB1 = 0;
int lastB2 = 0;
int lastB3 = 0;
unsigned long lastSendMs = 0;
unsigned long lastScreenMs = 0;
unsigned long lastTelemetryMs = 0;

float analogUnit(float rawValue) {
  return constrain(rawValue / ADC_MAX, 0.0f, 1.0f);
}

float calibratedAxis(int raw, int minValue, int centerValue, int maxValue) {
  raw = constrain(raw, minValue, maxValue);
  if (raw < centerValue) {
    return map(raw, minValue, centerValue, 0, 2048);
  }
  return map(raw, centerValue, maxValue, 2048, 4095);
}

float pidMaxFor(uint8_t axis, uint8_t term) {
  if (term == PID_TERM_I_ID) {
    return 5.0f;
  }
  if (term == PID_TERM_D_ID) {
    return axis == PID_AXIS_ALTITUDE_ID ? 20.0f : 10.0f;
  }
  return axis == PID_AXIS_ALTITUDE_ID ? 60.0f : 20.0f;
}

bool telemetryOnline() {
  return lastTelemetryMs != 0 && millis() - lastTelemetryMs <= TELEMETRY_TIMEOUT_MS;
}

const __FlashStringHelper *pageName() {
  if (page == PAGE_PID) {
    return F("PID");
  }
  if (page == PAGE_MOTORS) {
    return F("MOTOR");
  }
  if (page == PAGE_DEBUG) {
    return F("DEBUG");
  }
  if (page == PAGE_BATTERY) {
    return F("BATT");
  }
  return F("FLIGHT");
}

const char *axisName(uint8_t axis) {
  static const char *names[] = {"PITCH", "ROLL", "YAW", "ALT"};
  return names[constrain(axis, 0, 3)];
}

const char *termName(uint8_t term) {
  static const char *names[] = {"P", "I", "D"};
  return names[constrain(term, 0, 2)];
}

void showBootMessage(const __FlashStringHelper *line1, const __FlashStringHelper *line2 = nullptr) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(line1);
  if (line2 != nullptr) {
    display.println();
    display.println(line2);
  }
  display.display();
}

void onReceiveData(const uint8_t *data, int length) {
  if (length != sizeof(TelemetryMessage)) {
    return;
  }

  memcpy(&telemetry, data, sizeof(telemetry));
  lastTelemetryMs = millis();
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int length) {
  (void)info;
  onReceiveData(data, length);
}

void onSend(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  sendFailed = status != ESP_NOW_SEND_SUCCESS;
}
#else
void onReceive(const uint8_t *mac, const uint8_t *data, int length) {
  (void)mac;
  onReceiveData(data, length);
}

void onSend(const uint8_t *mac, esp_now_send_status_t status) {
  (void)mac;
  sendFailed = status != ESP_NOW_SEND_SUCCESS;
}
#endif

void setupPins() {
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);
  analogReadResolution(12);
}

void setupScreen() {
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("Display init failed"));
    while (true) {
      delay(1000);
    }
  }

  showBootMessage(F("Controller Online"), F("Connecting to drone..."));
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.enableLongRange(true);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);

  Serial.print(F("Controller MAC: "));
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("ESP-NOW init failed"));
    showBootMessage(F("ESP-NOW failed"), F("Restart controller"));
    return;
  }

  esp_now_register_recv_cb(onReceive);
  esp_now_register_send_cb(onSend);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, DRONE_MAC, sizeof(DRONE_MAC));
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println(F("Failed to add drone peer"));
    showBootMessage(F("Peer setup failed"), F("Check drone MAC"));
    return;
  }

  espNowReady = true;
  Serial.println(F("ESP-NOW ready"));
}

void drawValueBar(int16_t x, int16_t y, int16_t w, int16_t h, float value) {
  value = constrain(value, 0.0f, 1.0f);
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  display.fillRect(x + 1, y + 1, static_cast<int16_t>((w - 2) * value), h - 2, SSD1306_WHITE);
}

void drawCenteredAxis(int16_t x, int16_t y, int16_t w, int16_t h, float rawValue) {
  float value = analogUnit(rawValue);
  int16_t center = x + w / 2;
  int16_t pos = x + static_cast<int16_t>(value * (w - 1));

  display.drawRect(x, y, w, h, SSD1306_WHITE);
  display.drawFastVLine(center, y, h, SSD1306_WHITE);
  display.fillRect(pos - 1, y + 1, 3, h - 2, SSD1306_WHITE);
}

void configurePageCommand(float potUnit) {
  control.page = page;
  control.command = CMD_NONE;
  control.pidAxis = selectedPidAxis;
  control.pidTerm = selectedPidTerm;
  control.motorTestIndex = selectedMotor;
  control.commandValue = 0.0f;

  if (page == PAGE_PID) {
    float value = potUnit * pidMaxFor(selectedPidAxis, selectedPidTerm);
    pidValues[selectedPidAxis][selectedPidTerm] = value;
    control.command = CMD_PID_TUNE;
    control.commandValue = value;
  } else if (page == PAGE_MOTORS) {
    control.command = CMD_MOTOR_TEST;
    control.commandValue = potUnit;
  }
}

bool readControls() {
  int b1 = digitalRead(BUTTON1_PIN) == LOW;
  int b2 = digitalRead(BUTTON2_PIN) == LOW;
  int b3 = digitalRead(BUTTON3_PIN) == LOW;
  bool buttonChanged = b1 != lastB1 || b2 != lastB2 || b3 != lastB3;

  if (b3 && !lastB3) {
    page = (page + 1) % PAGE_COUNT;
  }

  if (page == PAGE_PID) {
    if (b1 && !lastB1) {
      selectedPidAxis = (selectedPidAxis + 1) % 4;
    }
    if (b2 && !lastB2) {
      selectedPidTerm = (selectedPidTerm + 1) % 3;
    }
  } else if (page == PAGE_MOTORS) {
    if (b1 && !lastB1) {
      selectedMotor = (selectedMotor + 3) % 4;
    }
    if (b2 && !lastB2) {
      selectedMotor = (selectedMotor + 1) % 4;
    }
  }

  lastB1 = b1;
  lastB2 = b2;
  lastB3 = b3;

  int potRaw = analogRead(POT_PIN);
  float potUnit = analogUnit(potRaw);

  control.b1 = page == PAGE_FLIGHT ? b1 : 0;
  control.b2 = page == PAGE_FLIGHT ? b2 : 0;
  control.b3 = 0;
  control.pot = potRaw;
  control.x = calibratedAxis(analogRead(JOY_X_PIN), JOY_X_MIN, JOY_X_CENTER, JOY_X_MAX);
  control.y = calibratedAxis(analogRead(JOY_Y_PIN), JOY_Y_MIN, JOY_Y_CENTER, JOY_Y_MAX);
  configurePageCommand(potUnit);

  return buttonChanged;
}

void sendControls() {
  if (!espNowReady) {
    return;
  }

  esp_err_t result = esp_now_send(DRONE_MAC, reinterpret_cast<uint8_t *>(&control), sizeof(control));
  if (result != ESP_OK) {
    sendFailed = true;
    Serial.println(F("ESP-NOW send error"));
  }
}

void drawHeader() {
  display.setCursor(0, 0);
  display.print(pageName());
  display.print(' ');
  display.print(telemetry.armed ? F("ARM") : F("SAFE"));
  display.setCursor(84, 0);
  if (!espNowReady) {
    display.print(F("SETUP"));
  } else if (sendFailed) {
    display.print(F("SEND"));
  } else if (!telemetryOnline()) {
    display.print(F("NO RX"));
  } else {
    display.print(F("LINK"));
  }
  display.drawFastHLine(0, 10, SCREEN_WIDTH, SSD1306_WHITE);
}

void drawFlightPage() {
  display.setCursor(0, 14);
  display.print(F("THR"));
  drawValueBar(24, 13, 42, 8, analogUnit(control.pot));
  display.setCursor(74, 14);
  display.print(telemetry.hold ? F("HOLD") : F("MAN "));

  display.setCursor(0, 26);
  display.print(F("X"));
  drawCenteredAxis(10, 25, 52, 8, control.x);
  display.setCursor(68, 26);
  display.print(F("Y"));
  drawCenteredAxis(78, 25, 50, 8, control.y);

  display.setCursor(0, 38);
  display.print(F("ALT "));
  display.print(telemetry.alt, 1);
  display.print(F("m "));
  display.print(telemetry.temp, 0);
  display.print(F("C"));

  display.setCursor(0, 49);
  display.print(F("Y"));
  display.print(telemetry.yaw, 0);
  display.print(F(" R"));
  display.print(telemetry.roll, 0);
  display.print(F(" P"));
  display.print(telemetry.pitch, 0);

  display.setCursor(0, 56);
  display.print(F("M "));
  display.print(telemetry.m1, 0);
  display.print(' ');
  display.print(telemetry.m2, 0);
  display.print(' ');
  display.print(telemetry.m3, 0);
  display.print(' ');
  display.print(telemetry.m4, 0);
}

void drawPidPage() {
  float value = pidValues[selectedPidAxis][selectedPidTerm];
  display.setCursor(0, 15);
  display.print(F("AX "));
  display.print(axisName(selectedPidAxis));
  display.print(F("  TERM "));
  display.print(termName(selectedPidTerm));

  display.setCursor(0, 28);
  display.print(F("VALUE "));
  display.print(value, 2);
  drawValueBar(0, 40, 128, 9, value / pidMaxFor(selectedPidAxis, selectedPidTerm));

  display.setCursor(0, 54);
  display.print(F("B1 axis B2 term Pot set"));
}

void drawMotorPage() {
  float amount = analogUnit(control.pot);
  display.setCursor(0, 15);
  display.print(F("MOTOR M"));
  display.print(selectedMotor + 1);
  display.print(telemetry.motorTestActive ? F(" ACTIVE") : F(" READY"));

  display.setCursor(0, 30);
  display.print(F("TEST "));
  display.print(static_cast<int>(1000 + amount * 300));
  display.print(F(" us"));
  drawValueBar(0, 42, 128, 9, amount);

  display.setCursor(0, 55);
  display.print(F("B1/B2 select Pot power"));
}

void drawDebugPage() {
  display.setCursor(0, 14);
  display.print(F("Raw/cal X "));
  display.print(static_cast<int>(control.x));
  display.setCursor(0, 24);
  display.print(F("Raw/cal Y "));
  display.print(static_cast<int>(control.y));
  display.setCursor(0, 36);
  display.print(F("Btn "));
  display.print(lastB1);
  display.print(lastB2);
  display.print(lastB3);
  display.setCursor(0, 48);
  display.print(F("Packet "));
  display.print(sizeof(ControllerMessage));
  display.print(F("/"));
  display.print(sizeof(TelemetryMessage));
}

void drawBatteryPage() {
  display.setCursor(0, 16);
  display.print(F("Battery "));
  if (isnan(telemetry.batteryVoltage)) {
    display.print(F("--.- V"));
  } else {
    display.print(telemetry.batteryVoltage, 2);
    display.print(F(" V"));
  }

  display.setCursor(0, 31);
  display.print(F("No auto-disarm"));
  display.setCursor(0, 45);
  display.print(F("Wire ADC later"));
}

void writeScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  drawHeader();

  if (page == PAGE_PID) {
    drawPidPage();
  } else if (page == PAGE_MOTORS) {
    drawMotorPage();
  } else if (page == PAGE_DEBUG) {
    drawDebugPage();
  } else if (page == PAGE_BATTERY) {
    drawBatteryPage();
  } else {
    drawFlightPage();
  }

  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  setupPins();
  setupScreen();
  setupEspNow();
}

void loop() {
  const unsigned long now = millis();

  bool buttonChanged = readControls();

  if (buttonChanged || now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    sendControls();
  }

  if (now - lastScreenMs >= SCREEN_INTERVAL_MS) {
    lastScreenMs = now;
    writeScreen();
  }
}
