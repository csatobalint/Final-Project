#ifndef CONTROLLED_MOTOR_H
#define CONTROLLED_MOTOR_H

#include "Motor.h"
#include "Encoder.h"

class ControlledMotor {
private:
	Motor* m;
	Encoder* e;
	double P, D, I;		/// Control parameters
	double stepToRad;	/// Conversion ratio from encoder steps to radians
	double wheel_radius;/// Wheel radius in meters
	double av_current;  /// Current angular velocity
	double av_target;	/// Target angular velocity
  double av_prev; /// Previous angular velocity
  double int_error;
	double spd; /// Calculated velocity of controlled motor
  int pos_prev;
  long int time_micros_prev;
public:
	ControlledMotor(Motor* motor_pointer, Encoder* encoder_pointer, int encoder_pulses, double wheel_radius_m);
	void setP(double p);
	void setD(double d);
	void set_target_velocity(double rad_per_sec);
  double get_current_velocity();
  double get_target_velocity();
	double get_spd();
  void update();
	/// TODO: Implement a PD-control on Motor using angular displacement readings from Encoder
};

#endif
