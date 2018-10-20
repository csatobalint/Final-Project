/**
 * Define PIN numbers here
 */

// Interrupt pins on Mega: 2, 3, 18, 19, 20, 21

// Motor C
#define MOTOR_C_IN1 6
#define MOTOR_C_IN2 5
#define MOTOR_C_EN  4
#define ENCODER3_CH1  3
#define ENCODER3_CH2  2

// Motor B
#define MOTOR_B_IN1 22 //7
#define MOTOR_B_IN2 23 //8
#define MOTOR_B_EN  9
#define ENCODER2_CH1 19
#define ENCODER2_CH2 18

// Motor A
#define MOTOR_A_IN1  24 //10
#define MOTOR_A_IN2  25 //11
#define MOTOR_A_EN  7 //12
#define ENCODER1_CH1  20
#define ENCODER1_CH2  21

// Serial connection of RN42 BT (TX2,RX2) Serial2 on Mega
//#define RN42_TX 16
//#define RN42_RX 17
#define RN42_SERIAL_PORT Serial2

// Battery input (80k-160k voltage divider)
#define BATTERY_ANALOG_IN 7

#define LED_RED 31
#define LED_YEL 32

//PotPin
#define potPin 0
