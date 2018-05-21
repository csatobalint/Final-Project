#include "Drive.h"

/*float Drive::constrain(float in_val, float min_val, float max_val) {
  if (in_val <= min_val) return min_val;
  else if (in_val >= max_val) return max_val;
  else return in_val;
}*/

Drive::Drive(float max_speed) {
  MotorA = 0.0f, MotorB = 0.0f;
  steering_point = 1.00f;
  turn_factor = 0.50f;
  Throttle = Steer = 0.0f;
  Speed = max_speed;
}

void Drive::update(int joyX, int joyY) {
  float x = (float)joyX/100.0;
  float y = (float)joyY/100.0;
  Throttle = steering_point - abs(y / 1.0f); //  inverse magnitude
  float steer_modifier = Throttle * turn_factor;
  Steer = x / 1.0f * turn_factor;
  // Turn With Throttle
  float twt_MotorA = y * (steering_point + Steer);
  float twt_MotorB = y * (steering_point - Steer);
  // No Throttle Steering
  float nt_MotorA = x * steer_modifier;
  float nt_MotorB = -x * steer_modifier;
  // Mixing
  MotorA = twt_MotorA + nt_MotorA;
  MotorB = twt_MotorB + nt_MotorB;
  MotorA = constrain(MotorA, -1.0f, 1.0f);
  MotorB = constrain(MotorB, -1.0f, 1.0f);
}

void Drive::drive_motors(Motor* motorA, Motor* motorB) {
  motorA->set_direction((bool)(MotorA > 0.0));
  motorA->set_speed(abs(MotorA)*Speed);
  motorB->set_direction((bool)(MotorB > 0.0));
  motorB->set_speed(abs(MotorB)*Speed);
}

float Drive::getMotorA() const { return MotorA; }
float Drive::getMotorB() const { return MotorB; }
