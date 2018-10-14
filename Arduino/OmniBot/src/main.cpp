#include "pin_config.h"

#include "Drive.h"
#include "Battery.h"
#include "BluetoothJoystick.h"

#include "Timer.h"

/*#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>*/


Motor motorA(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_EN, 0.00, 255);
Motor motorB(MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_EN, 0.00, 255);
Motor motorC(MOTOR_C_IN1, MOTOR_C_IN2, MOTOR_C_EN, 0.00, 255);

#define maxWheelSpeed 100
#include "Encoder.h"
#include "ControlledMotor.h"
Encoder1 encoderA;
Encoder2 encoderB;
Encoder3 encoderC;
ControlledMotor cMotorA(&motorA, &encoderA, 3960, 0.035);
ControlledMotor cMotorB(&motorB, &encoderB, 3960, 0.035);
ControlledMotor cMotorC(&motorC, &encoderC, 3960, 0.035);

/*
#include "PollingEncoder.h"
#include "PollingControlledMotor.h"
#define maxWheelSpeed 100
PollingEncoder encoderA(ENCODER1_CH1,ENCODER1_CH2,digitalRead(ENCODER1_CH1),digitalRead(ENCODER1_CH2));
PollingEncoder encoderB(ENCODER2_CH1,ENCODER2_CH2,digitalRead(ENCODER2_CH1),digitalRead(ENCODER2_CH2));
PollingEncoder encoderC(ENCODER3_CH1,ENCODER3_CH2,digitalRead(ENCODER3_CH1),digitalRead(ENCODER3_CH2));
PollingControlledMotor cMotorA(&motorA, &encoderA, 4096, 0.035);
PollingControlledMotor cMotorB(&motorB, &encoderB, 4096, 0.035);
PollingControlledMotor cMotorC(&motorC, &encoderC, 4096, 0.035);*/

//Battery battery(3.2f, 4.2f, BATTERY_ANALOG_IN);
BluetoothJoystickCommander bjc(&RN42_SERIAL_PORT);

Timer t;

void control_loop();
void inceremental_loop();

bool omni_dir_drive = true;
bool incremental_header = true;

bool led_red_on = true;
//bool led_yel_on = true;

#define measurementDelay 1000

/*////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// SETUP ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////*/
void setup() {

  Serial.begin(57600);
  RN42_SERIAL_PORT.begin(57600);

  Serial.println(); Serial.println("Motor Speed Measurement");
  delay(measurementDelay);
  Serial.print(measurementDelay); Serial.println(",0,0,0");

  /*float pwm = 0.5;
  motorA.set_signed_speed(pwm); //-1...0...1
  motorB.set_signed_speed(pwm);
  motorC.set_signed_speed(pwm);*/

  float t_velocity = maxWheelSpeed*1;
  cMotorA.set_target_velocity(t_velocity); //-700...0...700
  cMotorB.set_target_velocity(t_velocity);
  cMotorC.set_target_velocity(t_velocity);

  pinMode(LED_RED, OUTPUT);
  //pinMode(LED_YEL, OUTPUT);

  t.every(10, control_loop); // Every 25 ms run the timed_loop, it works unitl the main loop is faster
  //t.every(200, inceremental_loop);

  digitalWrite(LED_RED, led_red_on);
  randomSeed(analogRead(1));

  /*encoderA.encoder_setup();
  encoderB.encoder_setup();
  encoderC.encoder_setup();*/
}

void print_encoder_positions() {
    Serial.print("Encoders: "); Serial.print(encoderA.get_position()); Serial.print(", ");
    Serial.print(encoderB.get_position()); Serial.print(", ");
    Serial.print(encoderC.get_position()); Serial.println(".");
}

void print_wheel_velocities() {
  Serial.print(millis()); Serial.print(", ");
  Serial.print(cMotorA.get_current_velocity()); Serial.print(", ");
  Serial.print(cMotorB.get_current_velocity()); Serial.print(", ");
  Serial.print(cMotorC.get_current_velocity()); Serial.print(", ");
  Serial.print(cMotorA.get_current_acceleration()); Serial.print(", ");
  Serial.print(cMotorB.get_current_acceleration()); Serial.print(", ");
  Serial.print(cMotorC.get_current_acceleration()); Serial.println("");
  RN42_SERIAL_PORT.print(millis()); RN42_SERIAL_PORT.print(", ");
  RN42_SERIAL_PORT.print(cMotorA.get_current_velocity()); RN42_SERIAL_PORT.print(", ");
  RN42_SERIAL_PORT.print(cMotorB.get_current_velocity()); RN42_SERIAL_PORT.print(", ");
  RN42_SERIAL_PORT.print(cMotorC.get_current_velocity()); RN42_SERIAL_PORT.print(", ");
  RN42_SERIAL_PORT.print(cMotorA.get_current_acceleration()); RN42_SERIAL_PORT.print(", ");
  RN42_SERIAL_PORT.print(cMotorB.get_current_acceleration()); RN42_SERIAL_PORT.print(", ");
  RN42_SERIAL_PORT.print(cMotorC.get_current_acceleration()); RN42_SERIAL_PORT.println("");
}

void print_bluetooth_joystick_data(){
    Serial.print("Joystick X,Y: ");
    Serial.print(bjc.getX()); Serial.print(", ");
    Serial.println(bjc.getY());
}

void print_calculated_velocites(float vA, float vB, float vC){
  Serial.print(vA); Serial.print(",");
  Serial.print(vB); Serial.print(",");
  Serial.print(vC); Serial.println("");
}

void ledController(bool omni_dir_drive, int x){
  if(omni_dir_drive){
    /*led_red_on = !led_red_on;
    led_yel_on = !led_yel_on;
    digitalWrite(LED_RED, led_red_on);
    digitalWrite(LED_YEL, led_yel_on);*/
  }
  else{
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
}

float currentVelocity = 0;

void control_loop() {
  cMotorA.update();
  cMotorB.update();
  cMotorC.update();
  print_wheel_velocities();
}

float ramp_up_value = 0;
float ramp_up_value_increment = 0.01;

void inceremental_loop() {

  if(incremental_header){
    Serial.println("Time, vA, vB, vC, targetVelocity, pwmA, pwmB, pwmC");
    incremental_header = false;
  }

  cMotorA.set_target_velocity(ramp_up_value*maxWheelSpeed); //-700...0...700
  cMotorB.set_target_velocity(ramp_up_value*maxWheelSpeed);
  cMotorC.set_target_velocity(ramp_up_value*maxWheelSpeed);

  Serial.print(millis());                       Serial.print(", ");
  Serial.print(cMotorA.get_current_velocity()); Serial.print(", ");
  Serial.print(cMotorB.get_current_velocity()); Serial.print(", ");
  Serial.print(cMotorC.get_current_velocity()); Serial.print(",");
  Serial.print(ramp_up_value*maxWheelSpeed); Serial.println("");

  ramp_up_value = ramp_up_value + ramp_up_value_increment;
  if(ramp_up_value>1) ramp_up_value_increment *= -1;
  if(ramp_up_value<0) ramp_up_value_increment *= -1;
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
    //vC = 0.0;
  } else {
    // Forward velocity + angular velocity
    vA = y;
    vB = -y;
    vC = 0.0;
    float omega = 0.5;
    vA += omega*x;
    vB += omega*x;
    vC += omega*x;
  }
  /*print_calculated_velocites(vA,vB,vC);*/
  /*ledController(omni_dir_drive,x);*/

  /*motorA.set_signed_speed(vA);
  motorB.set_signed_speed(vB);
  motorC.set_signed_speed(vC);*/
  cMotorA.set_target_velocity(vA*maxWheelSpeed);
  cMotorB.set_target_velocity(vB*maxWheelSpeed);
  cMotorC.set_target_velocity(vC*maxWheelSpeed);
}

/****************************************************************************
*********************************** LOOP ************************************
*****************************************************************************/
void loop() {
  //long int startmillis = millis(); long int startmicros = micros();

  /*encoderA.encoder_update_position();
  encoderB.encoder_update_position();
  encoderC.encoder_update_position();*/

  t.update();

  // Update battery percentage and outcoming data
  /*battery.update(); bjc.setData3(String(battery.get_percent())+"%");*/

  // Set dirve method
  bjc.setData1(omni_dir_drive ? "Omni" : "Car-like");

  // Process incoming data from BluetoothJoystickController
  int p = bjc.process();
  if (p==2) {
    //Serial.print("Buttons 1,2: ");
    //Serial.print(bjc.getB0()); Serial.print(", ");
    //Serial.println(bjc.getB1());
    omni_dir_drive = !bjc.getB0(); // B0 not pressed = omni_drive, B0 pressed = car-like drive
  } else if (p==7) {
    /*print_bluetooth_joystick_data()*/
    drive();
  }

  //print_encoder_positions();

  //Serial.print("Loop time: "); Serial.print(millis()-startmillis); Serial.print(" [ms] "); Serial.print(micros()-startmicros); Serial.println(" [us] ");

  /*val = analogRead(potPin);
  float valFloat = (float)val/1023;
  motorA.set_signed_speed(valFloat);
  motorB.set_signed_speed(valFloat);
  motorC.set_signed_speed(valFloat);*/
  /*Serial.print(valFloat); Serial.println(",");*/
  //RN42_SERIAL_PORT.println(valFloat);
}
