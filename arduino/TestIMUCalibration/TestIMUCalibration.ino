#include "Wire.h"
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#define TCAADDR 0x70
#define TCAFOREARM 2
#define TCAUPPERARM 3
#define TCAWRIST 1
Adafruit_BNO055 bnoForearm = Adafruit_BNO055(55, BNO055_ADDRESS_A); 
Adafruit_BNO055 bnoUpperarm = Adafruit_BNO055(56, BNO055_ADDRESS_A); 
Adafruit_BNO055 bnoWrist = Adafruit_BNO055(57, BNO055_ADDRESS_A);

sensors_event_t event;

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


void setup() {
  Serial.begin(115200);
  tcaselect(TCAFOREARM);
  bnoForearm.begin();
  tcaselect(TCAWRIST);
  bnoWrist.begin();
  tcaselect(TCAUPPERARM);
  bnoUpperarm.begin();

  tcaselect(TCAFOREARM); 
    if(!bnoForearm.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
    else{ 
    Serial.println("#2 is all ok!"); 
  }
  delay(1000);
  bnoForearm.setExtCrystalUse(true);  
  
  tcaselect(TCAWRIST); 
    if(!bnoWrist.begin())
  {
    /* There was a problem detecting the BNO455 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }  
  else{ 
    Serial.println("#1 is all ok!"); 
  }
  delay(1000);
  bnoWrist.setExtCrystalUse(true);

  tcaselect(TCAUPPERARM); 
    if(!bnoUpperarm.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
    else{ 
    Serial.println("#3 is all ok!"); 
  }
  delay(1000);
  bnoForearm.setExtCrystalUse(true);

}

void loop() {
  uint8_t system, gyro, accel, mag;

  long start_time = millis();
   
  Serial.println("BEFORE CAL");
  while (millis() - start_time < 10000){
    imu::Quaternion quat = bnoUpperarm.getQuat();
    imu::Vector<3> eul = quat.toEuler();

    Serial.print(eul.x());
    Serial.print("    ");
    Serial.print(eul.y());
    Serial.print("    ");
    Serial.println(eul.z()); 
  }

  system = gyro = accel = mag = 0;

// remove accel calibration because it doesn't work
//  while (!system or !gyro or !accel or !mag){
  while (!system or !gyro or !mag){
    bnoUpperarm.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("IMU UPPERARM");
    Serial.print("\t");
    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);

    Serial.println();
  }

  start_time = millis();
  Serial.println("AFTER CAL");
  
  while (millis() - start_time < 10000000){
    imu::Quaternion quat = bnoUpperarm.getQuat();
    imu::Vector<3> eul = quat.toEuler();


    Serial.print(eul.x());
    Serial.print("    ");
    Serial.print(eul.y());
    Serial.print("    ");
    Serial.println(eul.z()); 
  }

  

}
