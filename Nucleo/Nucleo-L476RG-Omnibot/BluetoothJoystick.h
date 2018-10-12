#ifndef BLUETOOTH_JOYSTICK_COMM_H
#define BLUETOOTH_JOYSTICK_COMM_H

#include "mbed.h"
#include <string> 

#define    STX          0x02
#define    ETX          0x03
#define    SLOW         750                  // Datafields refresh rate (ms)
#define    FAST         250                  // Datafields refresh rate (ms)

class BluetoothJoystickCommander {
private:
  Serial* serial;
  int mJoystickX, mJoystickY;
  uint8_t mBufferPtr;
  char cmd[8];                 // bytes received
  uint8_t buttonStatus;           // first Byte sent to Android device
  long previousMillis;         // will store last time Buttons status was updated
  long sendInterval;           // interval between Buttons status transmission (milliseconds)
  void sendReply();
  void getJoystickState();
  void getButtonState(int bStatus);
  string getButtonStatusString();
  string data1,data2,data3;
public:
  BluetoothJoystickCommander(Serial* s_ptr);
  uint8_t process();
  int getX() const;
  int getY() const;
  bool getB0() const;
  bool getB1() const;
  char getCmd(uint8_t i) { return cmd[i]; }
  void setData1(string d);
  void setData2(string d);
  void setData3(string d);
};

#endif