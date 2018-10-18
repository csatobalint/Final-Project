/**
 * Define PIN numbers here
 */
 
// Motor A
#define MOTOR_A_IN1 D4
#define MOTOR_A_IN2 D3
#define MOTOR_A_EN  D5
#define ENCODER1_CH1  PB_13
#define ENCODER1_CH2  PB_14

// Motor B
#define MOTOR_B_IN1 D6
#define MOTOR_B_IN2 D7
#define MOTOR_B_EN  D9
#define ENCODER2_CH1 PB_11
#define ENCODER2_CH2 PB_12

// Motor C
#define MOTOR_C_IN1 D11
#define MOTOR_C_IN2 D12
#define MOTOR_C_EN  D10
#define ENCODER3_CH1  PA_11
#define ENCODER3_CH2  PA_12

// Serial connection of RN42 BT (TX2,RX2) Serial2 on L476RG
#define RN42_TX D8
#define RN42_RX D2

// I2C Bus (BNO055)
#define BNO_SDA D14
#define BNO_SCL D15
#define BNO_RESET PC_5

// Battery input (80k-160k voltage divider)
#define BATTERY_ANALOG_IN D13

#define LED_RED PC_6
#define LED_YEL PC_8