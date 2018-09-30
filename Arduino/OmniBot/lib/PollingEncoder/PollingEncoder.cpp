#include "PollingEncoder.h"
#include <Arduino.h>
#include "pin_config.h"

PollingEncoder::PollingEncoder(int pin1, int pin2, bool p_ch1, bool p_ch2) {
  ch1_pin = pin1;
  ch2_pin = pin2;
  prev_ch2 = p_ch1;
  prev_ch1 = p_ch2;
}

void PollingEncoder::encoder_setup() {
  pinMode (ch1_pin,INPUT);
  pinMode (ch2_pin,INPUT);
  prev_ch1 = digitalRead(ch1_pin);
}

void PollingEncoder::encoder_update_position() {
  ch1 = digitalRead(ch1_pin);
  ch2 = digitalRead(ch2_pin); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   /*if (ch1 != prev_ch1){
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(ch2_pin) != ch1) {
       position ++;
       position_time = micros();
     } else {
       position --;
       position_time = micros();
     }
     //Serial.print("Position: ");Serial.println(position);
   }*/

   if (ch2 != prev_ch2) {
     position += (ch2-prev_ch2) * (ch1 ? +1 : -1);
     position_time = micros();
   }
   else if (ch1 != prev_ch1)  {
     position += (ch1-prev_ch1) * (ch2 ? -1 : +1);
     position_time = micros();
   }
   else return; //nothing changed: exit

   prev_ch1 = ch1;
   prev_ch2 = ch2;

}

int PollingEncoder::get_position() {
  return position;
}

int PollingEncoder::get_time_micros() {
  return position_time;
}
