#include "ControlledMotor.h"

ControlledMotor::ControlledMotor(Motor* motor_pointer,
								 Encoder* encoder_pointer, 
								 int encoder_pulses, 
								 double wheel_radius_m) : m(motor_pointer), e(encoder_pointer), wheel_radius(wheel_radius_m) {
	P=0.005;
	D=0.0;
  I=0.002;
	stepToRad=2.0*3.141592653589793238462/encoder_pulses;
  prev_pos = e->get_position();
  prev_time_micros = e->get_time_micros();
  prev_av = 0.0;
  int_error = 0.0;
}

void ControlledMotor::setP(double p) {
	P=p;
}

void ControlledMotor::setD(double d) {
	D=d;
}

void ControlledMotor::set_target_velocity(double rad_per_sec) {
	av_target = rad_per_sec;
  int_error = 0.0;
}

double ControlledMotor::get_current_velocity() {
  return av_current;
}

double ControlledMotor::get_target_velocity() {
  return av_target;
}

void ControlledMotor::update() {
  // Get current values
  int pos = e->get_position();
  long int time_micros = e->get_time_micros();
  // Calculate angular velocity
  // Check for pos overflow/underflow
  double diff = double(pos)-double(prev_pos);
  if (abs(diff)>60000) {
    if (diff<0) {
      // This means, that the encoder counter rolled over 32768 and became negative: -32767 + something, diff should be positive
      diff += 65536.0;
    }
    else { 
      // This means, that the encoder rolled under -32767 and became positive. (Diff should be negative)
      diff -= 65536.0;
    }
  }
  if (abs(diff)<1) {
    av_current = 0;
    
  } else {
    av_current = diff*100000.0/(time_micros-prev_time_micros);
  }
  // Calculate angular acceleration
  double aa_current = (av_current-prev_av)*100000.0/(time_micros-prev_time_micros);
  if (abs(av_current-prev_av)<0.01) {
    aa_current = 0;
  }
  // PD control
  int_error+=(av_target-av_current);
  double spd = av_target/700.0+(av_target-av_current)*P+int_error*I;
  //Serial.print(av_current); Serial.print(","); Serial.print(av_target); Serial.print(","); Serial.println(spd);
  m->set_signed_speed(spd);
  // Update previous values
  prev_pos = pos;
  prev_time_micros = time_micros;
  prev_av = av_current;
}

