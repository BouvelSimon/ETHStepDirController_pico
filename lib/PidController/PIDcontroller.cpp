#include <Arduino.h>
#include "PIDcontroller.h"

PIDcontroller::PIDcontroller(float kp, float ki, float kd, uint32_t sampletime_us, uint32_t derivativetimeconstant_ms, int32_t setpoint){
  integrator=0;
  differential=0;

  previousError=0;
  previousMeasurement=0;

  Kp=kp;
  Ki=ki;
  Kd=kd;

  sampleTimeUs=sampletime_us;
  derTimeConstantS=derivativetimeconstant_ms;
  setPoint=setpoint;

}
	
void PIDcontroller::setSetPoint(int32_t sp){
  setPoint=sp;
}
int32_t PIDcontroller::getSetPoint(){
  return setPoint;
}


void PIDcontroller::setGains(float kp, float ki, float kd){
  Kp=kp;
  Ki=ki;
  Kd=kd;
}

float PIDcontroller::getKp(){
  return Kp;
}
float PIDcontroller::getKi(){
  return Ki;
}
float PIDcontroller::getKd(){
  return Kd;
}





int32_t PIDcontroller::update(int32_t measurement){
  
  currentError=setPoint-measurement;
  proportional=currentError*Kp;
  integrator=integrator+Ki*sampleTimeUs*(currentError+previousError)*0.000005f;
  differential=0; // todo : implement differential gain
  
  previousError=currentError;
  previousMeasurement=measurement;
  
  return proportional+integrator+differential;
}
	