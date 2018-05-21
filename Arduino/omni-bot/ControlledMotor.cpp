#include "ControlledMotor.h"

ControlledMotor::ControlledMotor(Motor* motor_pointer,
								 Encoder* encoder_pointer, 
								 int encoder_pulses, 
								 double wheel_radius_m) : m(motor_pointer), e(encoder_pointer), wheel_radius(wheel_radius_m) {
	P=0;
	D=0;
	stepToRad=2.0*3.141592653589793238462/encoder_pulses;
}

void ControlledMotor::setP(double p) {
	P=p;
}

void ControlledMotor::setD(double d) {
	D=d;
}

void ControlledMotor::set_angular_velocity(double rad_per_sec) {
	av_target = rad_per_sec;
}

void ControlledMotor::set_velocity(double meter_per_sec) {
	av_target = meter_per_sec/wheel_radius;
}
