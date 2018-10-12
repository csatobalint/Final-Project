#ifndef MOTOR_H
#define MOTOR_H

#include "mbed.h"

class Motor {
private:
  DigitalOut in1;
  DigitalOut in2;
  PwmOut  en;
  int p_min;
  int p_max;
public:
  Motor(PinName pin_in1, PinName pin_in2, PinName pin_en, int p_min=0, int p_max=255);
  void set_direction(bool forward);
  void stop();
  void set_speed(float speed);
  void set_signed_speed(float speed);
};

#endif
