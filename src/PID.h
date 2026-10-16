#pragma once

#include <Helpers.h>

class PID
{
private:
    float goal;
    float truth;
    float errorSum;
    float lastTime;
    float lastTruth;
    float output;

    // coefficents
    float kp;
    float ki;
    float kd;

public:
    PID(float p, float i, float d);
    ~PID();
    void setPoints(float g, float t);
    void setTunings(float p, float i, float d);
    void setTuningTerm(uint8_t term, float value);
    float getTuningTerm(uint8_t term) const;
    void reset();
    float updatePID();
};
