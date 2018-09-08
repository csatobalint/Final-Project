#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
private:
  int in1;
  int in2;
  int en;
  int p_min;
  int p_max;
public:
  Motor(int pin_in1, int pin_in2, int pin_en, int p_min=0, int p_max=255);
  void set_direction(bool forward);
  void stop();
  void set_speed(float speed);
  void set_signed_speed(float speed);
};

#endif
