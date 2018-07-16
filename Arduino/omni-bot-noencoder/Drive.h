#ifndef OMNI_DRIVE_H
#define OMNI_DRIVE_H

#include "Motor.h"

class Drive {
private:
  float MotorA, MotorB;
  float Throttle, Steer;
  float steering_point;
  float turn_factor;
  float Speed;
  //float constrain(float in_val, float min_val, float max_val);
public:
  Drive(float max_speed);
  void update(int joyX, int joyY);
  void drive_motors(Motor* motorA, Motor* motorB);
  float getMotorA() const;
  float getMotorB() const;
};


#endif
