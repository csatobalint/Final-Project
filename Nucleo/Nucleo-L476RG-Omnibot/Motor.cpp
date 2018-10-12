#include "Motor.h"

Motor::Motor(PinName pin_in1, PinName pin_in2, PinName pin_en, int p_min, int p_max) :
  in1(pin_in1), in2(pin_in2), en(pin_en), p_min(p_min), p_max(p_max) {
  set_direction(true);
  set_speed(0);
}

void Motor::set_direction(bool forward) {
  if (forward) {
    in1 = 1;
    in2 = 0;
  } else {
    in1 = 0;
    in2 = 1;
  }
}

void Motor::stop() {
  in1 = 0;
  in2 = 0;
}

void Motor::set_speed(float s) {
  float spd = p_min + s*(p_max-p_min);
  if (spd >= 255.0) spd = 255.0;
  en = spd/255.0; // write percent 0...1
}

void Motor::set_signed_speed(float s) {
  if (s==0) {
    stop();
  } else if (s>0) {
    set_direction(true);
  } else {
    set_direction(false);
  }
  set_speed(abs(s));
}
