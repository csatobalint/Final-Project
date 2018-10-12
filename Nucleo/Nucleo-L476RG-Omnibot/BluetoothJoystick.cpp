#include "BluetoothJoystick.h"

string BluetoothJoystickCommander::getButtonStatusString()  {
  string bStatus = "";
  for (int i=0; i<6; i++)  {
    if (buttonStatus & (0x20 >> i))    bStatus += "1";
    else                               bStatus += "0";
  }
  return bStatus;
}

void BluetoothJoystickCommander::sendReply() {
  serial->putc((char)STX);                                           // Start of Transmission
  serial->printf("%s",getButtonStatusString().c_str());  serial->putc((char)0x1);  // buttons status feedback
  serial->printf("%s",data1.c_str());  serial->putc((char)0x4);                    // datafield #1
  serial->printf("%s",data2.c_str());  serial->putc((char)0x5);                    // datafield #2
  serial->printf("%s",data3.c_str());                                              // datafield #3
  serial->putc((char)ETX);                                           // End of Transmission
}

void BluetoothJoystickCommander::getJoystickState() {
  int joyX = (cmd[1]-48)*100 + (cmd[2]-48)*10 + (cmd[3]-48);       // obtain the Int from the ASCII representation
  int joyY = (cmd[4]-48)*100 + (cmd[5]-48)*10 + (cmd[6]-48);
  joyX = joyX - 200;                                                  // Offset to avoid
  joyY = joyY - 200;                                                  // transmitting negative numbers
  mJoystickX = joyX;
  mJoystickY = joyY;
  if (joyX<-100 || joyX>100 || joyY<-100 || joyY>100) return;         // commmunication error
  mJoystickX = joyX;
  mJoystickY = joyY;
}

void BluetoothJoystickCommander::getButtonState(int bStatus)  {
  switch (bStatus) {
    case 'A':
      buttonStatus |= 0x01;        // ON
      break;
    case 'B':
      buttonStatus &= 0x3E;        // OFF
      break;
    case 'C':
      buttonStatus |= 0x02;        // ON
      break;
    case 'D':
      buttonStatus &= 0x3D;        // OFF
      break;
    case 'E':
      buttonStatus |= 0x04;        // ON
      break;
    case 'F':
      buttonStatus &= 0x3B;        // OFF
      break;
    case 'G':
      buttonStatus |= 0x08;        // ON
      break;
    case 'H':
      buttonStatus &= 0x37;        // OFF
      break;
    case 'I':
      buttonStatus |= 0x10;        // ON
      break;
    case 'J':
      buttonStatus &= 0x2F;        // OFF
      break;
    case 'K':
      buttonStatus |= 0x20;        // ON
      break;
    case 'L':
      buttonStatus &= 0x1F;        // OFF
      break;
  }
}

BluetoothJoystickCommander::BluetoothJoystickCommander(Serial* s_ptr) {
  for (uint8_t i=0; i<8; i++) cmd[i]=0;
  buttonStatus = 0;                                  // first Byte sent to Android device
  previousMillis = 0;                                // will store last time Buttons status was updated
  sendInterval = SLOW;                               // interval between Buttons status transmission (milliseconds)
  serial = s_ptr;
  mJoystickX = mJoystickY = 0;
  mBufferPtr = 0;
  data1 = "";
  data2 = "";
  data3 = "Connected!";
}

uint8_t BluetoothJoystickCommander::process() {
  bool packet_received = false;
  if(serial->readable())  {
    char c = serial->getc();
    if ((mBufferPtr == 0) && (c==STX)) {
      cmd[mBufferPtr] = c;
      mBufferPtr++;
    } else if ((mBufferPtr > 0) && (mBufferPtr < 8)) {
      cmd[mBufferPtr] = c;
      mBufferPtr++;
      if (c == ETX) packet_received = true;
    } else {
      // Packet overflow, restart
      mBufferPtr = 0;
    }
    if (packet_received) {
      // Check packet size
      if (mBufferPtr == 3) {
        // Button packet: 3 Bytes  ex: < STX "C" ETX >
        getButtonState(cmd[1]);    //
        sendReply();
      } else if (mBufferPtr == 8) {
        // Joystick packet: 8 Bytes  ex: < STX "200" "180" ETX >
        getJoystickState();
        sendReply();
      }
      return mBufferPtr-1;
    }
  }
  return 0;
}

int BluetoothJoystickCommander::getX() const { return mJoystickX; }

int BluetoothJoystickCommander::getY() const { return mJoystickY; }

bool BluetoothJoystickCommander::getB0() const {
  return (buttonStatus & 0x01);
}

bool BluetoothJoystickCommander::getB1() const {
  return (buttonStatus & 0x02);
}

void BluetoothJoystickCommander::setData1(string d) {
  data1 = d;
}

void BluetoothJoystickCommander::setData2(string d) {
  data2 = d;
}

void BluetoothJoystickCommander::setData3(string d) {
  data3 = d;
}
