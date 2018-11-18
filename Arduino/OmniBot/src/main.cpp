#include "pin_config.h"

#include "Drive.h"
#include "Battery.h"
#include "BluetoothJoystick.h"

#include "Timer.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055();


Motor motorA(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_EN, 0.00, 255);
Motor motorB(MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_EN, 0.00, 255);
Motor motorC(MOTOR_C_IN1, MOTOR_C_IN2, MOTOR_C_EN, 0.00, 255);

#define robotCaseRadius 0.12   //[m]
#define robotWheelRadius 0.03   //[m]
#define maxWheelSpeed 100     //[rpm]
#include "Encoder.h"
#include "ControlledMotor.h"
Encoder1 encoderA;
Encoder2 encoderB;
Encoder3 encoderC;
ControlledMotor cMotorA(&motorA, &encoderA, 3960, robotWheelRadius);
ControlledMotor cMotorB(&motorB, &encoderB, 3960, robotWheelRadius);
ControlledMotor cMotorC(&motorC, &encoderC, 3960, robotWheelRadius);

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
void bno_read_loop();
void inceremental_loop();
void parameterEstimationMeasurement();
void maxPowerSpeedAndAccelerationSpinningMeasurment();
void maxPowerSpeedAndAccelerationForwardMeasurment();
void parameterEstimationValidation();
void accelerateAndStop();
void accelerateAndReverse();

bool omni_dir_drive = true;
bool incremental_header = true;
bool measurement_on = false;
float measurement_on_start_time = 0.0;
bool led_red_on = true;
//bool led_yel_on = true;

#define measurementDelay 1000

/*////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// SETUP ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////*/
void setup() {

  Serial.begin(57600);
  RN42_SERIAL_PORT.begin(57600);

  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(100);

  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");
  bno.setExtCrystalUse(false);
  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  //RN42_SERIAL_PORT.println("BT serial");

  /*Serial.println(); Serial.println("Motor Speed Measurement");
  delay(measurementDelay);
  Serial.print(measurementDelay);
  Serial.println("0,0,0,0");*/

  float pwm = 0.0;
  motorA.set_signed_speed(pwm); //-1...0...1
  motorB.set_signed_speed(pwm);
  motorC.set_signed_speed(pwm);

/*  float t_velocity = maxWheelSpeed*0.05;
  cMotorA.set_target_velocity(t_velocity); //-700...0...700
  cMotorB.set_target_velocity(t_velocity);
  cMotorC.set_target_velocity(t_velocity);*/

  pinMode(LED_RED, OUTPUT);
  //pinMode(LED_YEL, OUTPUT);

  t.every(10, control_loop); // Every 25 ms run the timed_loop, it works unitl the main loop is faster
  //t.every(200, inceremental_loop);
  t.every(1000, bno_read_loop);

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

  imu::Vector<3> linAcceleration = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> angVelocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  if(measurement_on){
    Serial.print(millis()-measurement_on_start_time); Serial.print(", ");
    Serial.print(cMotorA.get_current_velocity()); Serial.print(", ");
    Serial.print(cMotorB.get_current_velocity()); Serial.print(", ");
    Serial.print(cMotorC.get_current_velocity()); Serial.print(", ");
    Serial.print(cMotorA.get_current_acceleration()); Serial.print(", ");
    Serial.print(cMotorB.get_current_acceleration()); Serial.print(", ");
    Serial.print(cMotorC.get_current_acceleration()); Serial.print(", ");
    Serial.print(linAcceleration.x());Serial.print(", ");
    Serial.print(linAcceleration.y());Serial.print(", ");
    Serial.print(linAcceleration.z());Serial.println("");
  }
  if(measurement_on){
    RN42_SERIAL_PORT.print(millis()-measurement_on_start_time); RN42_SERIAL_PORT.print(", ");
    RN42_SERIAL_PORT.print(cMotorA.get_current_velocity()); RN42_SERIAL_PORT.print(", ");
    RN42_SERIAL_PORT.print(cMotorB.get_current_velocity()); RN42_SERIAL_PORT.print(", ");
    RN42_SERIAL_PORT.print(cMotorC.get_current_velocity()); RN42_SERIAL_PORT.print(", ");
    RN42_SERIAL_PORT.print(cMotorA.get_current_acceleration()); RN42_SERIAL_PORT.print(", ");
    RN42_SERIAL_PORT.print(cMotorB.get_current_acceleration()); RN42_SERIAL_PORT.print(", ");
    RN42_SERIAL_PORT.print(cMotorC.get_current_acceleration()); RN42_SERIAL_PORT.print(", ");
    RN42_SERIAL_PORT.print(linAcceleration.x()); RN42_SERIAL_PORT.print(", ");
    RN42_SERIAL_PORT.print(linAcceleration.y()); RN42_SERIAL_PORT.print(", ");
    /*RN42_SERIAL_PORT.print(linAcceleration.z()); RN42_SERIAL_PORT.print(", ");
    RN42_SERIAL_PORT.print(angVelocity.x());RN42_SERIAL_PORT.print(", ");
    RN42_SERIAL_PORT.print(angVelocity.y());RN42_SERIAL_PORT.print(", ");*/
    RN42_SERIAL_PORT.print(angVelocity.z());RN42_SERIAL_PORT.println("");
  }
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

void bno_read_loop() {
  imu::Vector<3> linAcceleration = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> angVelocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  Serial.print("X: ");
  Serial.print(linAcceleration.x());
  Serial.print(" Y: ");
  Serial.print(linAcceleration.y());
  Serial.print(" Z: ");
  Serial.print(linAcceleration.z());
  Serial.print("\t\t");

  Serial.print("X: ");
  Serial.print(angVelocity.x());
  Serial.print(" Y: ");
  Serial.print(angVelocity.y());
  Serial.print(" Z: ");
  Serial.print(angVelocity.z());
  Serial.print("\t\t");

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);
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

void control_loop(){
  cMotorA.update();
  cMotorB.update();
  cMotorC.update();
  print_wheel_velocities();
}

void drive() {
  float x = bjc.getX()/100.0;
  float y = bjc.getY()/100.0;
  float vA = 0.0;
  float vB = 0.0;
  float vC = 0.0;
  if (omni_dir_drive) {
    float eAx = -0.5; float eAy = +0.866025;
    float eBx = -0.5; float eBy = -0.866025;
    float eCx = +1.0; float eCy = 0.0;
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
void omniDriveVelocity(float u, float v, float w){
  float vA = 0.0;  float vB = 0.0;  float vC = 0.0; float scalingFactor = 1;
  float eAx = -0.5; float eAy = +0.866025;
  float eBx = -0.5; float eBy = -0.866025;
  float eCx = +1.0; float eCy =  0.0;
  vA = eAx*u + eAy*v;
  vB = eBx*u + eBy*v;
  vC = eCx*u + eCy*v;
  vA += w*robotCaseRadius;
  vB += w*robotCaseRadius;
  vC += w*robotCaseRadius;
  if(abs(vA)>1)       scalingFactor = abs(vA);
  if(abs(vB)>abs(vA) && abs(vB)>1) scalingFactor = abs(vB);
  if(abs(vC)>abs(vB) && abs(vC)>1) scalingFactor = abs(vC);
  vA /= scalingFactor;
  vB /= scalingFactor;
  vC /= scalingFactor;

  cMotorA.set_target_velocity(vA*maxWheelSpeed);
  cMotorB.set_target_velocity(vB*maxWheelSpeed);
  cMotorC.set_target_velocity(vC*maxWheelSpeed);
}
int BluetoothSerialCommand = -1;
float globalOmega = 0;

/****************************************************************************
*********************************** LOOP ************************************
*****************************************************************************/
void loop() {
  //long int startmillis = millis(); long int startmicros = micros();

  /*encoderA.encoder_update_position();
  encoderB.encoder_update_position();
  encoderC.encoder_update_position();*/

  // Update battery percentage and outcoming data
  /*battery.update(); bjc.setData3(String(battery.get_percent())+"%");*/


  // Set dirve method
  bjc.setData1(omni_dir_drive ? "Omni" : "Car-like");

  // Process incoming data from BluetoothJoystickController
  /*int p = bjc.process();
  if (p==2) { //Button packet
    //Serial.print("Buttons 1,2: ");
    //Serial.print(bjc.getB0()); Serial.print(", ");
    //Serial.println(bjc.getB1());
    omni_dir_drive = !bjc.getB0(); // B0 not pressed = omni_drive, B0 pressed = car-like drive
    globalOmega = bjc.getB1() ? 1 : 0;
  } else if (p==7) { //Coordinates
    //print_bluetooth_joystick_data()
    //drive();
    float x = bjc.getX()/100.0;
    float y = bjc.getY()/100.0;
    omniDriveVelocity(x, y, globalOmega);
  }
  */

  //ControlVia BT serial
  int command = RN42_SERIAL_PORT.read();
  if (command == 'g') {
    RN42_SERIAL_PORT.println("Measurement Started!");
    RN42_SERIAL_PORT.println("0,0,0,0,0,0,0");
    measurement_on = true;
    measurement_on_start_time = millis();
  }

  maxPowerSpeedAndAccelerationSpinningMeasurment();

  t.update();

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
void serialBluetoothCommander(int command){
  switch (command) {
    case 'g':
      //omni_rotate();
      break;
    }
}
void parameterEstimationMeasurement(){
  if(measurement_on){
    float delta = millis() - measurement_on_start_time;
  	if(delta< 2001){
  		float pwm = 1.0;
  		motorA.set_signed_speed(pwm); //-1...0...1
  		motorB.set_signed_speed(pwm);
  		motorC.set_signed_speed(pwm);
  	}
  	else if(delta>=2001 && delta<4001){
  		float pwm = -1.0;
  		motorA.set_signed_speed(pwm);
  		motorB.set_signed_speed(pwm);
  		motorC.set_signed_speed(pwm);
  	}
  	else if(delta>=4001 && delta<6001){
  		float pwm = 1.0;
  		motorA.set_signed_speed(pwm);
  		motorB.set_signed_speed(pwm);
  		motorC.set_signed_speed(pwm);
  	}
  	else if(delta>=6001 && delta<8001){
  		float pwm = -1.0;
  		motorA.set_signed_speed(pwm);
  		motorB.set_signed_speed(pwm);
  		motorC.set_signed_speed(pwm);
  	}
  	else if(delta >= 8001){
  		measurement_on = false;
  		float pwm = 0.0;
  		motorA.set_signed_speed(pwm); //-1...0...1
  		motorB.set_signed_speed(pwm);
  		motorC.set_signed_speed(pwm);
  	}
  }
}
void maxPowerSpeedAndAccelerationSpinningMeasurment(){
  if(measurement_on){
    float delta = millis() - measurement_on_start_time;
    if(delta< 5001){
      float pwm = 1.0;
      motorA.set_signed_speed(pwm); //-1...0...1
      motorB.set_signed_speed(pwm);
      motorC.set_signed_speed(pwm);
    }
    else if(delta >= 5001){
      measurement_on = false;
      float pwm = 0.0;
      motorA.set_signed_speed(pwm); //-1...0...1
      motorB.set_signed_speed(pwm);
      motorC.set_signed_speed(pwm);
    }
  }
  else{
    float pwm = 0.0;
    motorA.set_signed_speed(pwm); //-1...0...1
    motorB.set_signed_speed(pwm);
    motorC.set_signed_speed(pwm);
  }
}
void maxPowerSpeedAndAccelerationForwardMeasurment(){
  float delta = millis() - measurement_on_start_time;
  if(delta< 2001){
    float pwm = 1.0;
    motorA.set_signed_speed(pwm); //-1...0...1
    motorB.set_signed_speed(-pwm);
    motorC.set_signed_speed(0);
  }
  else if(delta >= 2001){
    measurement_on = false;
    float pwm = 0.0;
    motorA.set_signed_speed(pwm); //-1...0...1
    motorB.set_signed_speed(pwm);
    motorC.set_signed_speed(pwm);
  }
}
void parameterEstimationValidation(){
  if(measurement_on){
    float delta = millis() - measurement_on_start_time;
  	if(delta< 5001){
  		float pwm = 0.50;
  		motorA.set_signed_speed(pwm); //-1...0...1
  		motorB.set_signed_speed(pwm);
  		motorC.set_signed_speed(pwm);
  	}
  	else if(delta>=5001 && delta<10001){
  		float pwm = 0.75;
  		motorA.set_signed_speed(pwm);
  		motorB.set_signed_speed(pwm);
  		motorC.set_signed_speed(pwm);
  	}
    else if(delta>=10001 && delta<15001){
  		float pwm = 1.00;
  		motorA.set_signed_speed(pwm);
  		motorB.set_signed_speed(pwm);
  		motorC.set_signed_speed(pwm);
  	}
    else if(delta>=15001 && delta<20001){
  		float pwm = 0.00;
  		motorA.set_signed_speed(pwm);
  		motorB.set_signed_speed(pwm);
  		motorC.set_signed_speed(pwm);
  	}
  	else if(delta >= 20001){
  		measurement_on = false;
  	}
  }
}

int ramped_velocity = 10;

void accelerateAndStop(){
  if(measurement_on){
    float delta = millis() - measurement_on_start_time;
    if(delta< 2001){
      float pwm = 1.0;
      cMotorA.set_target_velocity(pwm*maxWheelSpeed); //-1...0...1
      cMotorB.set_target_velocity(-pwm*maxWheelSpeed);
      cMotorC.set_target_velocity(0*maxWheelSpeed);
    }
    else if(delta >= 2001 && delta<3001){
      //measurement_on = false;
      float pwm = 0.0;
      cMotorA.set_target_velocity(pwm); //-1...0...1
      cMotorB.set_target_velocity(pwm);
      cMotorC.set_target_velocity(pwm);
    }
    else if(delta >= 3001){
      measurement_on = false;
    }
  }
  else{
    float pwm = 0.0;
    cMotorA.set_target_velocity(pwm); //-1...0...1
    cMotorB.set_target_velocity(pwm);
    cMotorC.set_target_velocity(pwm);
  }
}
void accelerateAndReverse(){
  if(measurement_on){
    float delta = millis() - measurement_on_start_time;
    if(delta< 2001){
      float pwm = 1.0;
      motorA.set_signed_speed(pwm); //-1...0...1
      motorB.set_signed_speed(-pwm);
      motorC.set_signed_speed(0);
    }
    else if(delta >= 2001 && delta<3000){
      float pwm = 1.0;
      motorA.set_signed_speed(-pwm); //-1...0...1
      motorB.set_signed_speed(pwm);
      motorC.set_signed_speed(0);
    }
    else if(delta >= 3001){
      measurement_on = false;
      float pwm = 0.0;
      motorA.set_signed_speed(pwm); //-1...0...1
      motorB.set_signed_speed(pwm);
      motorC.set_signed_speed(pwm);
    }
  }
  else{
    float pwm = 0.0;
    motorA.set_signed_speed(pwm); //-1...0...1
    motorB.set_signed_speed(pwm);
    motorC.set_signed_speed(pwm);
  }
}
