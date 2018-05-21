#include "pin_config.h"

#include "Drive.h"
#include "Battery.h"
#include "BluetoothJoystick.h"
#include "Encoder.h"
/*#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>*/

Motor motorA(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_EN, 20, 255);
Motor motorB(MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_EN, 20, 255);
Motor motorC(MOTOR_C_IN1, MOTOR_C_IN2, MOTOR_C_EN, 0, 150);
Encoder1 encoderA;
Encoder2 encoderB;
Encoder3 encoderC;

Battery battery(3.2f, 4.2f, BATTERY_ANALOG_IN);
BluetoothJoystickCommander bjc(&RN42_SERIAL_PORT);

void setup() {
  Serial.begin(57600);
  RN42_SERIAL_PORT.begin(57600);
  motorA.set_signed_speed(0.0);
  motorB.set_signed_speed(0.0);
  motorC.set_signed_speed(0.0);
}

void drive() {
  float x = bjc.getX()/100.0;
  float y = bjc.getY()/100.0;
  float eAx = +0.5; float eAy = +0.866025;
  float eBx = +0.5; float eBy = -0.866025;
  float eCx = -1.0; float eCy = 0.0;
  float vA = eAx*x + eAy*y;
  float vB = eBx*x + eBy*y;
  float vC = eCx*x + eCy*y;
  Serial.print(vA); Serial.print(",");
  Serial.print(vB); Serial.print(",");
  Serial.print(vC); Serial.println("");
  motorA.set_signed_speed(vA);
  motorB.set_signed_speed(vB);
  motorC.set_signed_speed(vC);
}

void loop() {
  battery.update();
  int p = bjc.process();
  int pA = encoderA.get_position();
  int pB = encoderB.get_position();
  int pC = encoderC.get_position();
  Serial.print("Encoders: "); Serial.print(pA); Serial.print(", ");
  Serial.print(pB); Serial.print(", ");
  Serial.print(pC); Serial.println(".");
  bjc.setData3(String(battery.get_percent())+"%");
  if (p==2) {
    //Serial.print("Buttons 1,2: ");
    //Serial.print(bjc.getB0()); Serial.print(", ");
    //Serial.println(bjc.getB1());
  } else if (p==7) {
    //Serial.print("Joystick X,Y: ");
    //Serial.print(bjc.getX()); Serial.print(", ");
    //Serial.println(bjc.getY());
    drive();
  }
}

