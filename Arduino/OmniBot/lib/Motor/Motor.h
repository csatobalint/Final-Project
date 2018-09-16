#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
private:
  int in1;    //direciton control pins, logical values (LOW,HIGH)
  int in2;
  int en;     //pwm enable pins 0..255
  int p_min;  //minimal pwm value where the wheel start to rotate
  int p_max;  //maximal pwm value corresponding to the max speed
public:
  Motor(int pin_in1, int pin_in2, int pin_en, int p_min=0, int p_max=255);
  void set_direction(bool forward);
  void stop();
  void set_speed(float speed);
  void set_signed_speed(float speed);
};

#endif
