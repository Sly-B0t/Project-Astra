#pragma once

#include <LPS.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Helpers.h>
#include <ESP32Servo.h>
#include <MadgwickAHRS.h>

class InputManager
{
private:
    static LPS ps;
    static Adafruit_ICM20948 icm;
    ICM icm_struct = {};
    Orientation orientation = {};
    unsigned long previousFusionTime = 0;
    float gyroXOffset = 0.0f;
    float gyroYOffset = 0.0f;
    float gyroZOffset = 0.0f;
    Madgwick filter;
    Servo m1;
    Servo m2;
    Servo m3;
    Servo m4;

public:
    InputManager();
    ~InputManager();
    float getAltitude();
    float getAverageTemperature();
    float getTemperatureBar();
    float getTemperatureICM();
    Orientation getOrientation();
    Orientation sensorFusion();
    void setMotors(Motors motors);
    void updateICM();
};
