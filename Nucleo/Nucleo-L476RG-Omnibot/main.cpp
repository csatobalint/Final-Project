#include "mbed.h"

#include "pin_config.h"
#include "Motor.h"
#include "Drive.h"
#include "Battery.h"
#include "BluetoothJoystick.h"
#include "BNO055.h"
#include "Encoder.h"
//#include "ControlledMotor.h"


Motor motorA(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_EN, 0, 255);
Motor motorB(MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_EN, 0, 255);
Motor motorC(MOTOR_C_IN1, MOTOR_C_IN2, MOTOR_C_EN, 0, 255);
Timer t;
Encoder encoderA(ENCODER1_CH1, ENCODER1_CH2, &t);
Encoder encoderB(ENCODER2_CH1, ENCODER3_CH2, &t);
Encoder encoderC(ENCODER3_CH1, ENCODER3_CH2, &t);
/*ControlledMotor cMotorA(&motorA, &encoderA, 3960, 0.035);
ControlledMotor cMotorB(&motorB, &encoderB, 3960, 0.035);
ControlledMotor cMotorC(&motorC, &encoderC, 3960, 0.035);*/

Battery battery(3.2f, 4.2f, BATTERY_ANALOG_IN);
Serial BT(RN42_TX, RN42_RX, 57600);
BluetoothJoystickCommander bjc(&BT);
I2C i2c(BNO_SDA, BNO_SCL);
BNO055 bno(i2c, BNO_RESET, BNO055_G_CHIP_ADDR, MODE_NDOF);
uint8_t calib;
BNO055_EULER_TypeDef euler;

void timed_loop();
bool omni_dir_drive = true;
bool led_red_on = true;
bool led_yel_on = true;

//DigitalOut Red(LED_RED);
//DigitalOut Yel(LED_YEL);

Serial PC(SERIAL_TX, SERIAL_RX, 57600);


void setup() {
  motorA.set_signed_speed(0.0);
  motorB.set_signed_speed(0.0);
  motorC.set_signed_speed(0.0);
  //t.every(25, timed_loop); // Every 25 ms
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
    //led_red_on = !led_red_on;
    //led_yel_on = !led_yel_on;
    //Red = led_red_on;
    //Yel = led_yel_on;
  } else {
    // Forward velocity + angular velocity
    vA = y;
    vB = -y;
    vC = 0.0;
    float omega = 0.5;
    vA += omega*x;
    vB += omega*x;
    vC += omega*x;
    /*if (x > 0.1) {
      Red = 1; Yel = 0;
    } else if (x < -0.1) {
      Red = 0; Yel = 1;
    } else {
      Red = 0; Yel = 0;
    }*/
  }
  motorA.set_signed_speed(vA);
  motorB.set_signed_speed(vB);
  motorC.set_signed_speed(vC);
}

char message[50];

uint8_t csys,cmag,cacc,cgyr;

void loop() {
  // Update timer which will trigger timed callbacks
  //t.update();
  
  // BNO
  calib = bno.read_calib_status();
  bno.get_Euler_Angles(&euler);
  PC.printf("Calib state SGAM %d %d %d %d\n", 
    (calib & 0b11000000) >> 6,
    (calib & 0b00110000) >> 4, 
    (calib & 0b00001100) >> 2, 
    (calib & 0b00000011));
  PC.printf("Heading, roll, pitch: %4.2f,%4.2f,%4.2f\n", 
    euler.h, euler.r, euler.p);
  
  // Update battery percentage and outcoming data
  battery.update();
  sprintf(message, "%4.2f %%", battery.get_percent());
  bjc.setData3(string(message));
  sprintf(message, "%4.2f,%4.2f,%4.2f", euler.h, euler.r, euler.p);
  bjc.setData2(string(message));
  bjc.setData1(omni_dir_drive ? "Omni" : "Car-like");

  // Process incoming data from BluetoothJoystickController
  int p = bjc.process();
  if (p==2) {
    omni_dir_drive = !bjc.getB0(); // B0 not pressed = omni_drive, B0 pressed = car-like drive
  } else if (p==7) {
    drive();
  }
  
}

void timed_loop() {
  //cMotorA.update();
  //cMotorB.update();
  //cMotorC.update();
}

int main() {
    setup();
    while(1) {
        loop();
    }
}

