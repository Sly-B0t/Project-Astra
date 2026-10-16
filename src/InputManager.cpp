
#include <LPS.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <InputManager.h>
#include <Helpers.h>
#include <ESP32Servo.h>
#include <MadgwickAHRS.h>

LPS InputManager::ps;
Adafruit_ICM20948 InputManager::icm;

InputManager::InputManager()
{
    Serial.println("InputManager: begin.");

    Serial.println("InputManager: initializing LPS barometer...");
    if (!ps.init())
    {
        Serial.println("InputManager: LPS barometer init failed.");
        callError(LPS_ERROR);
    }
    ps.enableDefault();
    Serial.println("InputManager: LPS barometer ready.");

    Serial.println("InputManager: initializing ICM-20948...");
    bool imuReady = false;
    const uint8_t imuAddresses[] = {MPU_ADDR, MPU_ADDR_ALT};
    for (int attempt = 0; attempt < 5 && !imuReady; attempt++)
    {
        for (uint8_t address : imuAddresses)
        {
            Serial.printf("InputManager: ICM-20948 init attempt %d at 0x%02X...\n", attempt + 1, address);
            if (icm.begin_I2C(address, &Wire))
            {
                Serial.printf("InputManager: ICM-20948 ready at 0x%02X.\n", address);
                imuReady = true;
                break;
            }
            delay(100);
        }
        delay(250);
    }

    if (!imuReady)
    {
        Serial.println("InputManager: ICM-20948 init failed.");
        callError(IMU_ERROR);
    }

    Serial.println("InputManager: attaching ESC outputs...");
    m1.setPeriodHertz(50);
    m2.setPeriodHertz(50);
    m3.setPeriodHertz(50);
    m4.setPeriodHertz(50);

    m1.attach(MOTOR1PIN, 1000, 2000);
    m2.attach(MOTOR2PIN, 1000, 2000);
    m3.attach(MOTOR3PIN, 1000, 2000);
    m4.attach(MOTOR4PIN, 1000, 2000);

    setMotors({1000, 1000, 1000, 1000});
    Serial.println("InputManager: motors idle at 1000us.");

    delay(3000);
    Serial.println("InputManager: complete.");
    filter.begin(FILTERRATE);
}

InputManager::~InputManager()
{
}

float InputManager::getAltitude()
{
    float pressure = ps.readPressureMillibars();
    orientation.altitude = ps.pressureToAltitudeMeters(pressure);
    return orientation.altitude;
}
float InputManager::getAverageTemperature()
{
    return (getTemperatureBar() + getTemperatureICM()) / 2;
}

float InputManager::getTemperatureBar()
{
    return ps.readTemperatureC();
}

float InputManager::getTemperatureICM()
{
    updateICM();
    return icm_struct.temp;
}

Orientation InputManager::getOrientation()
{
    return orientation;
}

Orientation InputManager::sensorFusion()
{
    filter.update(icm_struct.gx, icm_struct.gy, icm_struct.gz, icm_struct.ax, icm_struct.ay, icm_struct.az, icm_struct.mx, icm_struct.my, icm_struct.mz);
    orientation.pitch = filter.getPitch();
    orientation.roll = filter.getRoll();
    orientation.yaw = filter.getYaw();
    return orientation;
}

void InputManager::setMotors(Motors motors)
{

    m1.writeMicroseconds(constrain(motors.m1, MOTOR_IDLE_US, MOTOR_MAX_US));
    m2.writeMicroseconds(constrain(motors.m2, MOTOR_IDLE_US, MOTOR_MAX_US));
    m3.writeMicroseconds(constrain(motors.m3, MOTOR_IDLE_US, MOTOR_MAX_US));
    m4.writeMicroseconds(constrain(motors.m4, MOTOR_IDLE_US, MOTOR_MAX_US));
}

void InputManager::updateICM()
{
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;

    icm.getEvent(&accel, &gyro, &temp, &mag);

    ICM newicm = {static_cast<float>(gyro.gyro.x * RAD_TO_DEG), static_cast<float>(gyro.gyro.y * RAD_TO_DEG),
                  static_cast<float>(gyro.gyro.z * RAD_TO_DEG), accel.acceleration.x, accel.acceleration.y,
                  accel.acceleration.z, mag.magnetic.x, mag.magnetic.y,
                  mag.magnetic.z, temp.temperature};

    icm_struct = newicm;
    sensorFusion();
}

