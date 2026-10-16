#pragma once

#include <Arduino.h>

struct ICM
{
    float gx, gy, gz;
    float ay, ax, az;
    float mx, my, mz;
    float temp;
};

struct Orientation
{
    float yaw, roll, pitch;
    float altitude;
};

struct Control
{               // buttons from the face of the controller
    bool a, b;  // arm,hold
    int c;      // hold, change monitor(not useful here)
    float x, y; // yaw,pitch
    float t;    // throttle, 0.0 -> 1.0
};

struct CoreMessage
{
    Control c;
    Orientation actual;
    Orientation goal;
    bool linkHealthy;
};

struct PIDData
{
    Orientation goal, actual;
};

// IO Pins
#define SERIALCLOCKPIN 9
#define SERIALDATAPIN 8

#define MOTOR1PIN 11
#define MOTOR2PIN 12
#define MOTOR3PIN 13
#define MOTOR4PIN 14
#define BATTERYADCINPUT 10
#define BAR_ADDR 0x5D
#define MPU_ADDR 0x68
#define MPU_ADDR_ALT 0x69
#define LED_18 18
#define LED_ACTIVE_LEVEL HIGH

#define STATUS_PRINT_INTERVAL_MS 5000
#define CLEAR_BT_KEYS_ON_BOOT 0

#define LPS_ERROR 1
#define IMU_ERROR 2
#define I2C_ERROR 3
#define FIRMWAREVER "1.0.0"
#define MAX_TILT_DEG 20.0f
#define MAX_YAW_RATE_DEG 90.0f
#define FILTERRATE 100
#define ROLL_DOWN 0
#define MOTOR_IDLE_US 1000.0f
#define MOTOR_ARMED_IDLE_US 1050.0f
#define MOTOR_MAX_US 2000.0f
#define MOTOR_THROTTLE_RANGE_US 650.0f
#define MOTOR_CORRECTION_LIMIT_US 250.0f
#define CONTROLLER_TIMEOUT_MS 500
inline void setStatusLed(bool on)
{
    pinMode(LED_18, OUTPUT);
    digitalWrite(LED_18, on ? LED_ACTIVE_LEVEL : !LED_ACTIVE_LEVEL);
}

inline void blinkStatusLed(unsigned int onMs, unsigned int offMs)
{
    setStatusLed(true);
    Serial.print(".");
    delay(onMs);
    Serial.print(".");
    setStatusLed(false);
    delay(offMs);
}

inline void blinkStartupCode(int code)
{
    for (int repeat = 0; repeat < 2; repeat++)
    {
        for (int i = 0; i < code; i++)
        {
            blinkStatusLed(90, 120);
        }
        blinkStatusLed(420, 650);
    }
}

inline void callError(int code)
{
    Serial.print("Fatal startup error code: ");
    Serial.println(code);

    while (1)
    {
        blinkStartupCode(code);
        delay(1200);
    }
}

struct Motors
{
    float m1, m2, m3, m4;
};

struct Controller_Message
{
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

struct Telemetry_Message
{
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

#define CONTROLLER_PAGE_FLIGHT 0
#define CONTROLLER_PAGE_PID 1
#define CONTROLLER_PAGE_MOTORS 2
#define CONTROLLER_PAGE_DEBUG 3
#define CONTROLLER_PAGE_BATTERY 4

#define CONTROLLER_CMD_NONE 0
#define CONTROLLER_CMD_PID_TUNE 1
#define CONTROLLER_CMD_MOTOR_TEST 2

#define PID_AXIS_PITCH 0
#define PID_AXIS_ROLL 1
#define PID_AXIS_YAW 2
#define PID_AXIS_ALTITUDE 3

#define PID_TERM_P 0
#define PID_TERM_I 1
#define PID_TERM_D 2

#define BATTERY_SENSE_ENABLED 0
#define BATTERY_ADC_MAX 4095.0f
#define BATTERY_ADC_REF_V 3.3f
#define BATTERY_DIVIDER_RATIO 2.0f
#define ARM_HOLD_MS 1000
#define ARM_THROTTLE_MAX 0.05f
#define MOTOR_TEST_MAX_US 1300.0f
