#ifndef BLUETOOTH_JOYSTICK_COMM_H
#define BLUETOOTH_JOYSTICK_COMM_H

#include <Arduino.h>

#define    STX          0x02
#define    ETX          0x03
#define    SLOW         750                  // Datafields refresh rate (ms)
#define    FAST         250                  // Datafields refresh rate (ms)

class BluetoothJoystickCommander {
private:
  HardwareSerial* serial;
  int mJoystickX, mJoystickY;
  uint8_t mBufferPtr;
  char cmd[8];                 // bytes received
  uint8_t buttonStatus;           // first Byte sent to Android device
  long previousMillis;         // will store last time Buttons status was updated
  long sendInterval;           // interval between Buttons status transmission (milliseconds)
  void sendReply();
  void getJoystickState();
  void getButtonState(int bStatus);
  String getButtonStatusString();
  String data1,data2,data3;
public:
  BluetoothJoystickCommander(HardwareSerial* s_ptr);
  uint8_t process();
  int getX() const;
  int getY() const;
  bool getB0() const;
  bool getB1() const;
  char getCmd(uint8_t i) { return cmd[i]; }
  void setData1(String d);
  void setData2(String d);
  void setData3(String d);
};


#endif