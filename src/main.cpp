#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LPS.h>
#include <Helpers.h>
#include <InputManager.h>
#include <PID.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

/*
USB C PORT FACES UP
M2___      ___M3
     \   /
      \ /
       X
      / \
   __/   \___
M1   USBC    M4

USBC port is forward.
M1 rear-left, M2 front-left, M3 front-right, M4 rear-right.
*/

constexpr float PITCH_KP = 4.0f;
constexpr float PITCH_KI = 0.0f;
constexpr float PITCH_KD = 1.2f;

constexpr float ROLL_KP = 4.0f;
constexpr float ROLL_KI = 0.0f;
constexpr float ROLL_KD = 1.2f;

constexpr float YAW_KP = 2.0f;
constexpr float YAW_KI = 0.0f;
constexpr float YAW_KD = 0.4f;

constexpr float ALT_KP = 20.0f;
constexpr float ALT_KI = 0.0f;
constexpr float ALT_KD = 4.0f;

InputManager *IM = nullptr;
PID pitchPID(PITCH_KP, PITCH_KI, PITCH_KD);
PID yawPID(YAW_KP, YAW_KI, YAW_KD);
PID rollPID(ROLL_KP, ROLL_KI, ROLL_KD);
PID altitudePID(ALT_KP, ALT_KI, ALT_KD);

QueueHandle_t coreQueue = nullptr;

uint8_t CONTROLLER_MAC[] = {0xA4, 0xCB, 0x8F, 0x20, 0x55, 0xD8};

esp_now_peer_info_t peerInfo = {};

Controller_Message incomingData = {};
Telemetry_Message outgoingData = {};
Motors currentMotors = {MOTOR_IDLE_US, MOTOR_IDLE_US, MOTOR_IDLE_US, MOTOR_IDLE_US};
Control currentControl = {};

volatile bool newData = false;
volatile unsigned long lastReceiveMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastDebugPrintMs = 0;

bool armed = false;
bool hold = false;
bool lastArmButton = false;
bool lastHoldButton = false;
bool armHoldConsumed = false;
bool holdAltitudeValid = false;
float holdAltitude = 0.0f;
float yawTarget = 0.0f;
unsigned long armPressStartMs = 0;
unsigned long lastMotorTestMs = 0;
bool motorTestActive = false;
uint8_t motorTestIndex = 0;

void IOCore(void *parameter);
void PIDCore(void *parameter);

float clampUnit(float value)
{
    return constrain(value, 0.0f, 1.0f);
}

float centeredAxis(float raw)
{
    float value = (raw - 2048.0f) / 2048.0f;

    if (fabs(value) < 0.05f)
    {
        return 0.0f;
    }

    return constrain(value, -1.0f, 1.0f);
}

bool controllerLinkHealthy()
{
    return lastReceiveMs != 0 && millis() - lastReceiveMs <= CONTROLLER_TIMEOUT_MS;
}

float readBatteryVoltage()
{
#if BATTERY_SENSE_ENABLED
    return analogRead(BATTERYADCINPUT) * (BATTERY_ADC_REF_V / BATTERY_ADC_MAX) * BATTERY_DIVIDER_RATIO;
#else
    return NAN;
#endif
}

void setIdleMotors()
{
    currentMotors = {MOTOR_IDLE_US, MOTOR_IDLE_US, MOTOR_IDLE_US, MOTOR_IDLE_US};
    IM->setMotors(currentMotors);
}

void updateToggle(bool pressed, bool &lastPressed, bool &state)
{
    if (pressed && !lastPressed)
    {
        state = !state;
    }

    lastPressed = pressed;
}

PID *pidForAxis(uint8_t axis)
{
    if (axis == PID_AXIS_PITCH)
    {
        return &pitchPID;
    }
    if (axis == PID_AXIS_ROLL)
    {
        return &rollPID;
    }
    if (axis == PID_AXIS_YAW)
    {
        return &yawPID;
    }
    if (axis == PID_AXIS_ALTITUDE)
    {
        return &altitudePID;
    }

    return nullptr;
}

void applyPidTune(uint8_t axis, uint8_t term, float value)
{
    PID *pid = pidForAxis(axis);
    if (pid == nullptr || term > PID_TERM_D || !isfinite(value))
    {
        return;
    }

    value = constrain(value, 0.0f, 100.0f);
    if (fabs(pid->getTuningTerm(term) - value) > 0.001f)
    {
        pid->setTuningTerm(term, value);
    }
}

Motors motorTestMotors(uint8_t index, float amount)
{
    float speed = MOTOR_IDLE_US + clampUnit(amount) * (MOTOR_TEST_MAX_US - MOTOR_IDLE_US);
    Motors motors = {MOTOR_IDLE_US, MOTOR_IDLE_US, MOTOR_IDLE_US, MOTOR_IDLE_US};

    if (index == 0)
    {
        motors.m1 = speed;
    }
    else if (index == 1)
    {
        motors.m2 = speed;
    }
    else if (index == 2)
    {
        motors.m3 = speed;
    }
    else if (index == 3)
    {
        motors.m4 = speed;
    }

    return motors;
}

void onSend(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    (void)mac_addr;

    if (status != ESP_NOW_SEND_SUCCESS)
    {
        Serial.println("Telemetry delivery failed");
    }
}

void onReceive(const uint8_t *mac, const uint8_t *data, int len)
{
    (void)mac;

    if (len != sizeof(Controller_Message))
    {
        return;
    }

    memcpy(&incomingData, data, sizeof(incomingData));
    lastReceiveMs = millis();
    newData = true;
    setStatusLed(true);
}

void setupESPNOW()
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.enableLongRange(true);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);

    Serial.print("Drone MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_send_cb(onSend);
    esp_now_register_recv_cb(onReceive);

    memcpy(peerInfo.peer_addr, CONTROLLER_MAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add controller peer");
        return;
    }

    Serial.println("ESP-NOW ready in long-range mode");
}

void setup()
{
    Serial.begin(115200);
    delay(300);

    Serial.println("Astra Revision " FIRMWAREVER " starting...");
    Serial.println("Starting I2C.");
    blinkStartupCode(1);

    Wire.begin(SERIALDATAPIN, SERIALCLOCKPIN);
    Wire.setClock(400000);
    Serial.println("I2C started at 400kHz.");

    Serial.println("Creating InputManager...");
    IM = new InputManager();
    setIdleMotors();
    Serial.println("Sensors and motors initialized.");
    blinkStartupCode(2);

    Serial.println("Creating FreeRTOS queue...");
    coreQueue = xQueueCreate(1, sizeof(CoreMessage));
    if (coreQueue == nullptr)
    {
        Serial.println("Failed to create core queue");
        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("Starting ESP-NOW...");
    setupESPNOW();

    Serial.println("Starting tasks...");
    xTaskCreatePinnedToCore(IOCore, "IO Core", 4096, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(PIDCore, "PID Core", 4096, nullptr, 3, nullptr, 1);
    blinkStartupCode(3);

    Serial.println("=== Astra startup complete ===");
}

void loop()
{
    vTaskDelay(portMAX_DELAY);
}

void setTelemetryMessage(Telemetry_Message &t, const Orientation &o)
{
    t.alt = o.altitude;
    t.pitch = o.pitch;
    t.yaw = o.yaw;
    t.roll = o.roll;
    t.m1 = currentMotors.m1;
    t.m2 = currentMotors.m2;
    t.m3 = currentMotors.m3;
    t.m4 = currentMotors.m4;
    t.batteryVoltage = readBatteryVoltage();
    t.armed = armed;
    t.hold = hold;
    t.motorTestActive = motorTestActive;
    t.temp = IM->getAverageTemperature();
}

void setupControlData(Control &c)
{
    bool armPressed = incomingData.b1 == 1;
    bool holdPressed = incomingData.b2 == 1;
    float throttle = clampUnit(incomingData.pot / 4095.0f);

    if (incomingData.command == CONTROLLER_CMD_PID_TUNE)
    {
        applyPidTune(incomingData.pidAxis, incomingData.pidTerm, incomingData.commandValue);
    }

    if (incomingData.command == CONTROLLER_CMD_MOTOR_TEST && !armed)
    {
        motorTestActive = true;
        motorTestIndex = constrain(incomingData.motorTestIndex, 0, 3);
        currentMotors = motorTestMotors(motorTestIndex, incomingData.commandValue);
        lastMotorTestMs = millis();
    }
    else if (incomingData.command != CONTROLLER_CMD_MOTOR_TEST)
    {
        motorTestActive = false;
    }

    if (armPressed && !lastArmButton)
    {
        armPressStartMs = millis();
        armHoldConsumed = false;
    }

    if (!armPressed)
    {
        armHoldConsumed = false;
    }

    if (armPressed && !armHoldConsumed && millis() - armPressStartMs >= ARM_HOLD_MS)
    {
        if (armed || throttle <= ARM_THROTTLE_MAX)
        {
            armed = !armed;
            hold = false;
            holdAltitudeValid = false;
            motorTestActive = false;
            armHoldConsumed = true;
        }
    }

    updateToggle(holdPressed, lastHoldButton, hold);
    lastArmButton = armPressed;

    c.a = armed;
    c.b = hold;
    c.c = incomingData.b3;
    c.x = centeredAxis(incomingData.x);
    c.y = centeredAxis(incomingData.y);
    c.t = throttle;

    currentControl = c;
    newData = false;
}

Orientation calculateGoalOrientation(const Control &c, const Orientation &actual)
{
    Orientation o = actual;
    o.pitch = c.y * MAX_TILT_DEG;
    o.roll = ROLL_DOWN;
    yawTarget += c.x * MAX_YAW_RATE_DEG * 0.01f;
    o.yaw = yawTarget;

    if (hold)
    {
        if (!holdAltitudeValid)
        {
            holdAltitude = actual.altitude;
            holdAltitudeValid = true;
        }

        o.altitude = holdAltitude;
    }
    else
    {
        o.altitude = actual.altitude;
        holdAltitudeValid = false;
    }

    return o;
}

void sendTelemetry(const Orientation &o)
{
    if (millis() - lastTelemetryMs < 500)
    {
        return;
    }

    lastTelemetryMs = millis();
    setTelemetryMessage(outgoingData, o);
    esp_now_send(CONTROLLER_MAC, reinterpret_cast<uint8_t *>(&outgoingData), sizeof(outgoingData));
}

void IOCore(void *parameter)
{
    (void)parameter;

    while (true)
    {
        IM->updateICM();
        IM->getAltitude();

        if (newData)
        {
            Control parsedControl = currentControl;
            setupControlData(parsedControl);
        }

        bool linkHealthy = controllerLinkHealthy();
        if (!linkHealthy)
        {
            armed = false;
            hold = false;
            holdAltitudeValid = false;
            motorTestActive = false;
            currentControl = {};
            setStatusLed(false);
        }
        else if (motorTestActive && millis() - lastMotorTestMs > 200)
        {
            motorTestActive = false;
        }

        Orientation actual = IM->getOrientation();
        if (!armed)
        {
            yawTarget = actual.yaw;
        }

        sendTelemetry(actual);

        CoreMessage message = {};
        message.c = currentControl;
        message.actual = actual;
        message.goal = calculateGoalOrientation(currentControl, actual);
        message.linkHealthy = linkHealthy;
        xQueueOverwrite(coreQueue, &message);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

Motors mixMotors(float throttle, float pitchCorrection, float rollCorrection, float yawCorrection, float altitudeCorrection)
{
    float base = MOTOR_ARMED_IDLE_US + throttle * MOTOR_THROTTLE_RANGE_US + altitudeCorrection;

    pitchCorrection = constrain(pitchCorrection, -MOTOR_CORRECTION_LIMIT_US, MOTOR_CORRECTION_LIMIT_US);
    rollCorrection = constrain(rollCorrection, -MOTOR_CORRECTION_LIMIT_US, MOTOR_CORRECTION_LIMIT_US);
    yawCorrection = constrain(yawCorrection, -MOTOR_CORRECTION_LIMIT_US, MOTOR_CORRECTION_LIMIT_US);

    Motors motors;
    motors.m1 = base + pitchCorrection + rollCorrection - yawCorrection;
    motors.m2 = base - pitchCorrection + rollCorrection + yawCorrection;
    motors.m3 = base - pitchCorrection - rollCorrection - yawCorrection;
    motors.m4 = base + pitchCorrection - rollCorrection + yawCorrection;

    motors.m1 = constrain(motors.m1, MOTOR_ARMED_IDLE_US, MOTOR_MAX_US);
    motors.m2 = constrain(motors.m2, MOTOR_ARMED_IDLE_US, MOTOR_MAX_US);
    motors.m3 = constrain(motors.m3, MOTOR_ARMED_IDLE_US, MOTOR_MAX_US);
    motors.m4 = constrain(motors.m4, MOTOR_ARMED_IDLE_US, MOTOR_MAX_US);

    return motors;
}

void printDebugLine(const CoreMessage &message, float pitchCorrection, float rollCorrection, float yawCorrection, float altitudeCorrection)
{
    if (millis() - lastDebugPrintMs < 1000)
    {
        return;
    }

    lastDebugPrintMs = millis();
    Serial.printf(
        "DBG armed=%d hold=%d link=%d thr=%.2f actual[p=%.1f r=%.1f y=%.1f alt=%.1f] goal[p=%.1f r=%.1f y=%.1f alt=%.1f] corr[p=%.1f r=%.1f y=%.1f alt=%.1f] motors[%.0f %.0f %.0f %.0f]\n",
        armed,
        hold,
        message.linkHealthy,
        message.c.t,
        message.actual.pitch,
        message.actual.roll,
        message.actual.yaw,
        message.actual.altitude,
        message.goal.pitch,
        message.goal.roll,
        message.goal.yaw,
        message.goal.altitude,
        pitchCorrection,
        rollCorrection,
        yawCorrection,
        altitudeCorrection,
        currentMotors.m1,
        currentMotors.m2,
        currentMotors.m3,
        currentMotors.m4);
}

void PIDCore(void *parameter)
{
    (void)parameter;

    CoreMessage message = {};

    while (true)
    {
        if (xQueueReceive(coreQueue, &message, portMAX_DELAY))
        {
            if (message.linkHealthy && motorTestActive && !armed)
            {
                IM->setMotors(currentMotors);
                continue;
            }

            if (!message.linkHealthy || !armed)
            {
                setIdleMotors();
                continue;
            }

            pitchPID.setPoints(message.goal.pitch, message.actual.pitch);
            rollPID.setPoints(message.goal.roll, message.actual.roll);
            yawPID.setPoints(message.goal.yaw, message.actual.yaw);
            altitudePID.setPoints(message.goal.altitude, message.actual.altitude);

            float pitchCorrection = pitchPID.updatePID();
            float rollCorrection = rollPID.updatePID();
            float yawCorrection = yawPID.updatePID();
            float altitudeCorrection = hold ? altitudePID.updatePID() : 0.0f;

            currentMotors = mixMotors(
                message.c.t,
                pitchCorrection,
                rollCorrection,
                yawCorrection,
                altitudeCorrection);

            IM->setMotors(currentMotors);
            printDebugLine(message, pitchCorrection, rollCorrection, yawCorrection, altitudeCorrection);
        }
    }
}
