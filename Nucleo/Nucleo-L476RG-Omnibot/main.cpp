#include "mbed.h"

#include "pin_config.h"
#include "Motor.h"
#include "Drive.h"
#include "Battery.h"
#include "BluetoothJoystick.h"
//#include "Encoder.h"
//#include "ControlledMotor.h"
//#include "Timer.h"

Motor motorA(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_EN, 0, 255);
Motor motorB(MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_EN, 0, 255);
Motor motorC(MOTOR_C_IN1, MOTOR_C_IN2, MOTOR_C_EN, 0, 255);
/*Encoder1 encoderA;
Encoder2 encoderB;
Encoder3 encoderC;
ControlledMotor cMotorA(&motorA, &encoderA, 4096, 0.035);
ControlledMotor cMotorB(&motorB, &encoderB, 4096, 0.035);
ControlledMotor cMotorC(&motorC, &encoderC, 4096, 0.035);*/

Battery battery(3.2f, 4.2f, BATTERY_ANALOG_IN);
Serial BT(RN42_TX, RN42_RX, 57600);
BluetoothJoystickCommander bjc(&BT);

void timed_loop();
bool omni_dir_drive = true;
bool led_red_on = true;
bool led_yel_on = true;

//DigitalOut Red(LED_RED);
//DigitalOut Yel(LED_YEL);

//Serial PC(USBTX, USBRX, 57600);


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

void loop() {
  // Update timer which will trigger timed callbacks
  //t.update();
  
  // Update battery percentage and outcoming data
  battery.update();
  sprintf(message, "%4.2f %%", battery.get_percent());
  bjc.setData3(string(message));
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

