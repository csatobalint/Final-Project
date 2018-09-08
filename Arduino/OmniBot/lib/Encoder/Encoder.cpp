#include "Encoder.h"
#include <Arduino.h>
#include "pin_config.h"

// This library supports up to 3 encoders
// On Arduino Mega/2560, Interrupt pins are 2,3,18,19,20,21

volatile int encoder1_pos = 0;
volatile int encoder2_pos = 0;
volatile int encoder3_pos = 0;
volatile long int time1_micros;
volatile long int time2_micros;
volatile long int time3_micros;
bool encoder1_ch1 = false;
bool encoder1_ch2 = false;
bool encoder2_ch1 = false;
bool encoder2_ch2 = false;
bool encoder3_ch1 = false;
bool encoder3_ch2 = false;

/* Encoder 1 */

void encoder1_channel1() {
  encoder1_ch1 = digitalRead(ENCODER1_CH1) == HIGH;
  encoder1_pos += (encoder1_ch1 != encoder1_ch2) ? +1 : -1;
  time1_micros = micros();
}

void encoder1_channel2() {
  encoder1_ch2 = digitalRead(ENCODER1_CH2) == HIGH;
  encoder1_pos += (encoder1_ch1 == encoder1_ch2) ? +1 : -1;
  time1_micros = micros();
}

void encoder1_setup() {
  attachInterrupt(digitalPinToInterrupt(ENCODER1_CH1), encoder1_channel1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_CH2), encoder1_channel2, CHANGE);
}

Encoder1::Encoder1() {
  encoder1_setup();
}

int Encoder1::get_position() {
  return encoder1_pos;
}

long int Encoder1::get_time_micros() {
  return time1_micros;
}

/* Encoder 2 */

void encoder2_channel1() {
  encoder2_ch1 = digitalRead(ENCODER2_CH1) == HIGH;
  encoder2_pos += (encoder2_ch1 != encoder2_ch2) ? +1 : -1;
  time2_micros = micros();
}

void encoder2_channel2() {
  encoder2_ch2 = digitalRead(ENCODER2_CH2) == HIGH;
  encoder2_pos += (encoder2_ch1 == encoder2_ch2) ? +1 : -1;
  time2_micros = micros();
}

void encoder2_setup() {
  attachInterrupt(digitalPinToInterrupt(ENCODER2_CH1), encoder2_channel1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_CH2), encoder2_channel2, CHANGE);
}

Encoder2::Encoder2() {
  encoder2_setup();
}

int Encoder2::get_position() {
  return encoder2_pos;
}

long int Encoder2::get_time_micros() {
  return time2_micros;
}

/* Encoder 3 */

void encoder3_channel1() {
  encoder3_ch1 = digitalRead(ENCODER3_CH1) == HIGH;
  encoder3_pos += (encoder3_ch1 != encoder3_ch2) ? +1 : -1;
  time3_micros = micros();
}

void encoder3_channel2() {
  encoder3_ch2 = digitalRead(ENCODER3_CH2) == HIGH;
  encoder3_pos += (encoder3_ch1 == encoder3_ch2) ? +1 : -1;
  time3_micros = micros();
}

void encoder3_setup() {
  attachInterrupt(digitalPinToInterrupt(ENCODER3_CH1), encoder3_channel1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_CH2), encoder3_channel2, CHANGE);
}

Encoder3::Encoder3() {
  encoder3_setup();
}

int Encoder3::get_position() {
  return encoder3_pos;
}

long int Encoder3::get_time_micros() {
  return time3_micros;
}


