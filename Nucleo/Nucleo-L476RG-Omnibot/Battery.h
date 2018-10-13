#ifndef BATTERY_H
#define BATTERY_H

#include <mbed.h>

class Battery {
private:
  float min_adc;
  float max_adc;
  float range;
  AnalogIn pin;
  float curr_adc;
  float percent;
public:
  Battery(float min_voltage, float max_voltage, PinName analog_in_pin) : pin(analog_in_pin) {
    min_adc = min_voltage/5.0f;
    max_adc = max_voltage/5.0f;
    range = max_adc - min_adc;
  }
  void update() {
    curr_adc = pin.read();
    /*if (curr_adc < min_adc) {
      percent = 0.0f;
    } else if (curr_adc > max_adc) {
      percent = 100.0f;
    } else {
      percent = (float)(curr_adc-min_adc)/range*100.0f;
    }*/
    percent = curr_adc*100.0f;
  }
  float get_percent() const {
    return percent;
  }
};

#endif // BATTERY_H
