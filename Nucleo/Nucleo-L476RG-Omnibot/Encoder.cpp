#include "Encoder.h"

Encoder::Encoder(PinName pin_ch1, PinName pin_ch2, Timer* timer) : ch1(pin_ch1), ch2(pin_ch2) {
  t_ptr = timer;
  ch1.rise(callback(this, &Encoder::ch1rise));
  ch1.fall(callback(this, &Encoder::ch1fall));
  ch2.rise(callback(this, &Encoder::ch2rise));
  ch2.fall(callback(this, &Encoder::ch2fall));
}

void Encoder::ch1rise() {
  encoder_pos += (ch1.read() != ch2.read()) ? +1 : -1;
  time_micros = t_ptr->read_us();
}

void Encoder::ch1fall() {
  encoder_pos += (ch1.read() != ch2.read()) ? +1 : -1;
  time_micros = t_ptr->read_us();
}

void Encoder::ch2rise() {
  encoder_pos += (ch1.read() == ch2.read()) ? +1 : -1;
  time_micros = t_ptr->read_us();
}

void Encoder::ch2fall() {
  encoder_pos += (ch1.read() == ch2.read()) ? +1 : -1;
  time_micros = t_ptr->read_us();
}

uint32_t Encoder::get_position() {
  return encoder_pos;
}

uint32_t Encoder::get_time_micros() {
  return time_micros;
}
