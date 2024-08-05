#ifndef PIDCONTROLLER

#define PIDCONTROLLER
#include <Arduino.h>


class PIDcontroller {
  public:
    // Default constructor with initial values
    PIDcontroller(float kp=1.0f, float ki=0.0f, float kd=0.0f, uint32_t sampletime_ms=100, uint32_t derivativetimeconstant_ms=100, int32_t setpoint=0);

    // Setter & getter for set point
    void setSetPoint(int32_t sp);
    int32_t getSetPoint();

    // Setter for gains
    void setGains(float kp, float ki, float kd);

    // Getters for gains
    float getKp();
    float getKi();
    float getKd();

    // Update the PID controller with a new measurement
    int32_t update(int32_t measurement);

  private:
    // PID state variables
    int32_t setPoint;
    float proportional;
    float differential;
    float integrator;

    // Internal error variables
    int32_t previousMeasurement;
    int32_t currentError;
    int32_t previousError;

    // Tuning parameters
    int32_t sampleTimeUs;
    uint32_t derTimeConstantS;

    // Gains (can be floats for finer control)
    float Kp;
    float Ki;
    float Kd;
};

#endif
