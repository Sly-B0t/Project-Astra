#include <PID.h>
#include <Arduino.h>

PID::PID(float p, float i, float d)
{
    kp = p;
    kd = d;
    ki = i;
    errorSum = 0;
    lastTruth = 0;
    lastTime = millis();
}

PID::~PID()
{
}

void PID::setPoints(float g, float t)
{
    goal = g;
    truth = t;
}

void PID::setTunings(float p, float i, float d)
{
    kp = p;
    ki = i;
    kd = d;
}

void PID::setTuningTerm(uint8_t term, float value)
{
    if (term == PID_TERM_P)
    {
        kp = value;
    }
    else if (term == PID_TERM_I)
    {
        ki = value;
    }
    else if (term == PID_TERM_D)
    {
        kd = value;
    }

    reset();
}

float PID::getTuningTerm(uint8_t term) const
{
    if (term == PID_TERM_P)
    {
        return kp;
    }
    if (term == PID_TERM_I)
    {
        return ki;
    }
    if (term == PID_TERM_D)
    {
        return kd;
    }

    return 0.0f;
}

void PID::reset()
{
    errorSum = 0.0f;
    lastTruth = truth;
    output = 0.0f;
    lastTime = millis();
}

float PID::updatePID()
{
    // change in seconds since last call
    float now = millis();
    float dt = (now - lastTime) / 1000.0f;
    if (dt <= 0)
        return output;
    float error = goal - truth;
    float P = kp * error;
    errorSum += error * dt;

    float D = kd * (truth - lastTruth) / dt;
    float I = ki * errorSum;
    output = P + I - D;
    lastTruth = truth;
    lastTime = now;
    return output;
}
