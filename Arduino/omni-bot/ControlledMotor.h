#ifndef CONTROLLED_MOTOR_H
#define CONTROLLED_MOTOR_H

#include "Motor.h"
#include "Encoder.h"

class ControlledMotor {
private:
	Motor* m;
	Encoder* e;
	double P, D;		/// Control parameters
	double stepToRad;	/// Conversion ratio from encoder steps to radians
	double wheel_radius;/// Wheel radius in meters
	double av_current;  /// Current angular velocity
	double av_target;	/// Target angular velocity
public:
	ControlledMotor(Motor* motor_pointer, Encoder* encoder_pointer, int encoder_pulses, double wheel_radius_m);
	void setP(double p);
	void setD(double d);
	void set_angular_velocity(double rad_per_sec);
	void set_velocity(double meter_per_sec);
	/// TODO: Implement a PD-control on Motor using angular displacement readings from Encoder
};

#endif