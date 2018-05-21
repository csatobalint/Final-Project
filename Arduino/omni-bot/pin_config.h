/**
 * Define PIN numbers here
 */

// Interrupt pins on Mega: 2, 3, 18, 19, 20, 21
 
// Motor A
#define MOTOR_A_IN1 5
#define MOTOR_A_IN2 6
#define MOTOR_A_EN  4
#define ENCODER1_CH1  2
#define ENCODER1_CH2  3

// Motor B
#define MOTOR_B_IN1 7
#define MOTOR_B_IN2 8
#define MOTOR_B_EN  9
#define ENCODER2_CH1 18
#define ENCODER2_CH2 19

// Motor C
#define MOTOR_C_IN1 10
#define MOTOR_C_IN2 11
#define MOTOR_C_EN  12
#define ENCODER3_CH1  21
#define ENCODER3_CH2  20

// Serial connection of RN42 BT (TX2,RX2) Serial2 on Mega
//#define RN42_TX 16
//#define RN42_RX 17
#define RN42_SERIAL_PORT Serial2

// Battery input (80k-160k voltage divider)
#define BATTERY_ANALOG_IN 7
