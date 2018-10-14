#include <Arduino.h>
#include <Wire.h> // I2C protocoll based commmunication
#include <Adafruit_Sensor.h> // Bosch orientation sensor
#include <Adafruit_BNO055.h> // Bosch orientation sensor
#include <utility/imumaths.h>
#include <timer.h>

Adafruit_BNO055 bno = Adafruit_BNO055();  //Orientation sensor
bool imu_installed = false;
uint8_t sys, gyro, accel, mag = 0;        // BNO calibration flags
Timer orientation_timer;
float heading;

// Providing the actual heading degree by means of the orientation sensor
void readOrientationSensor(){
  if (orientation_timer.poll(25)) {
    // This code runs once every 100ms (set-es részt átnézni)
    if (imu_installed) {
      sensors_event_t event;
      bno.getEvent(&event);
      heading = (float)event.orientation.x; //orientation of robot on the plane
      //ha a sensor nem tökéletesen függőleges akkor nem pontosan fogja számolni a szöget!
      bno.getCalibration(&sys, &gyro, &accel, &mag); //gives values from 0..3 to express the state of calibration
    }
      //heading1 = heading*0.5 + headingOld*0.5; //Pidbe
      //headingold=heading1;
      }
    }


void setup() {
    // put your setup code here, to run once:
    if(!bno.begin()) { imu_installed = false; }
    else {
      imu_installed = true;
      bno.setExtCrystalUse(true);
    }
    Serial.begin(56700);
    while (!Serial);
    if (imu_installed) {
      Serial.println(F("IMU found!"));
    } else {
      Serial.println(F("IMU not found!"));
    }

}

void loop() {
    // put your main code here, to run repeatedly:
    //Serial.println(heading);
}
