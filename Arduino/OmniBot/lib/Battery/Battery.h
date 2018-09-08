#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>

class Battery {
private:
  int min_adc;
  int max_adc;
  float range;
  int pin;
  int curr_adc;
  float percent;
public:
  Battery(float min_voltage, float max_voltage, int analog_in_pin) {
    min_adc = 1024.0f*min_voltage/5.0f;
    max_adc = 1024.0f*max_voltage/5.0f;
    range = max_adc - min_adc;
    pin = analog_in_pin;
  }
  void update() {
    curr_adc = analogRead(pin);
    if (curr_adc < min_adc) {
      percent = 0.0f;
    } else if (curr_adc > max_adc) {
      percent = 100.0f;
    } else {
      percent = (float)(curr_adc-min_adc)/range*100.0f;
    }
  }
  float get_percent() const {
    return percent;
  }
};

#endif // BATTERY_H
