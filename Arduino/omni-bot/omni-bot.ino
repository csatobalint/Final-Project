#include "pin_config.h"

#include "Drive.h"
#include "Battery.h"
#include "BluetoothJoystick.h"
#include "Encoder.h"
#include "ControlledMotor.h"
#include "Timer.h"

/*#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>*/

Motor motorA(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_EN, 34, 243);
Motor motorB(MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_EN, 30, 255);
Motor motorC(MOTOR_C_IN1, MOTOR_C_IN2, MOTOR_C_EN, 11, 140);
Encoder1 encoderA;
Encoder2 encoderB;
Encoder3 encoderC;
ControlledMotor cMotorA(&motorA, &encoderA, 4096, 0.035);
ControlledMotor cMotorB(&motorB, &encoderB, 4096, 0.035);
ControlledMotor cMotorC(&motorC, &encoderC, 4096, 0.035);

Battery battery(3.2f, 4.2f, BATTERY_ANALOG_IN);
BluetoothJoystickCommander bjc(&RN42_SERIAL_PORT);

Timer t;

void timed_loop();

bool omni_dir_drive = true;
bool led_red_on = true;
bool led_yel_on = true;

void setup() {
  Serial.begin(57600);
  RN42_SERIAL_PORT.begin(57600);
  motorA.set_signed_speed(0.0);
  motorB.set_signed_speed(0.0);
  motorC.set_signed_speed(0.0);
  cMotorA.set_target_velocity(0.0);
  cMotorB.set_target_velocity(0.0);
  cMotorC.set_target_velocity(0.0);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YEL, OUTPUT);
  t.every(25, timed_loop); // Every 25 ms
}

void drive() {
  float x = bjc.getX()/100.0;
  float y = bjc.getY()/100.0;
  float vA = 0.0;
  float vB = 0.0;
  float vC = 0.0;
  if (omni_dir_drive) {
    float eAx = +0.5; float eAy = +0.866025;
    float eBx = +0.5; float eBy = -0.866025;
    float eCx = -1.0; float eCy = 0.0;
    vA = eAx*x + eAy*y;
    vB = eBx*x + eBy*y;
    vC = eCx*x + eCy*y;
    led_red_on = !led_red_on;
    led_yel_on = !led_yel_on;
    digitalWrite(LED_RED, led_red_on);
    digitalWrite(LED_YEL, led_yel_on);
  } else {
    // Forward velocity + angular velocity
    vA = y;
    vB = -y;
    vC = 0.0;
    float omega = 0.5;
    vA += omega*x;
    vB += omega*x;
    vC += omega*x;
    if (x > 0.1) {
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_YEL, LOW);
    } else if (x < -0.1) {
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_YEL, HIGH);
    } else {
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_YEL, LOW);
    }
  }
  //Serial.print(vA); Serial.print(",");
  //Serial.print(vB); Serial.print(",");
  //Serial.print(vC); Serial.println("");
  //motorA.set_signed_speed(vA);
  //motorB.set_signed_speed(vB);
  //motorC.set_signed_speed(vC);
  cMotorA.set_target_velocity(vA*700.0);
  cMotorB.set_target_velocity(vB*700.0);
  cMotorC.set_target_velocity(vC*700.0);
}

void loop() {
  // Update timer which will trigger timed callbacks
  t.update();
  
  // Update battery percentage and outcoming data
  battery.update();
  bjc.setData3(String(battery.get_percent())+"%");
  bjc.setData1(omni_dir_drive ? "Omni" : "Car-like");
  
  /*int pA = encoderA.get_position();
  int pB = encoderB.get_position();
  int pC = encoderC.get_position();
  Serial.print("Encoders: "); Serial.print(pA); Serial.print(", ");
  Serial.print(pB); Serial.print(", ");
  Serial.print(pC); Serial.println(".");*/

  // Process incoming data from BluetoothJoystickController
  int p = bjc.process();
  if (p==2) {
    //Serial.print("Buttons 1,2: ");
    //Serial.print(bjc.getB0()); Serial.print(", ");
    //Serial.println(bjc.getB1());
    omni_dir_drive = !bjc.getB0(); // B0 not pressed = omni_drive, B0 pressed = car-like drive
  } else if (p==7) {
    //Serial.print("Joystick X,Y: ");
    //Serial.print(bjc.getX()); Serial.print(", ");
    //Serial.println(bjc.getY());
    drive();
  }
}

void timed_loop() {
  cMotorA.update();
  cMotorB.update();
  cMotorC.update();
  //Serial.print("Wheel velocities: "); Serial.print(cMotorA.get_current_velocity()); Serial.print(", ");
  //Serial.print(cMotorB.get_current_velocity()); Serial.print(", ");
  //Serial.print(cMotorC.get_current_velocity()); Serial.println(".");
}

