#ifndef ENCODER_H
#define ENCODER_H

class PollingEncoder {
private:
  int position;
  long int position_time;
  bool ch1;
  bool ch2;
  bool prev_ch1;
  bool prev_ch2;
  int ch1_pin;
  int ch2_pin;
public:
  PollingEncoder(int pin1, int pin2, bool p_ch1, bool p_ch2);
  void encoder_update_position();
  void encoder_setup();
  int get_position();
  int get_time_micros();
};

#endif
