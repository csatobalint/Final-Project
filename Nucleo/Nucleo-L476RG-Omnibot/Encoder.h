#ifndef ENCODER_H
#define ENCODER_H

#include "mbed.h"

class Encoder {
private:
  InterruptIn ch1, ch2;  
  Timer* t_ptr;  
  uint32_t encoder_pos;
  uint32_t  time_micros;
  void ch1rise();
  void ch1fall();
  void ch2rise();
  void ch2fall();
public:
  Encoder(PinName pin_ch1, PinName pin_ch2, Timer* timer);
  uint32_t get_position();
  uint32_t  get_time_micros();
};

#endif