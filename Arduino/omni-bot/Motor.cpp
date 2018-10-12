#include "Motor.h"

Motor::Motor(int pin_in1, int pin_in2, int pin_en, int p_min, int p_max) :
  in1(pin_in1), in2(pin_in2), en(pin_en), p_min(p_min), p_max(p_max) {
  pinMode(pin_in1, OUTPUT);
  pinMode(pin_in2, OUTPUT);
  set_direction(true);
  set_speed(0);
}

void Motor::set_direction(bool forward) {
  if (forward) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
}

void Motor::stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void Motor::set_speed(float s) {
  int spd = p_min + s*(p_max-p_min);
  if (spd >= 255) spd = 255;
  analogWrite(en, spd);
}

void Motor::set_signed_speed(float s) {
  if (abs(s)<0.05) {
    stop();
  } else if (s>0) {
    set_direction(true);
  } else {
    set_direction(false);
  }
  set_speed(abs(s));
}
