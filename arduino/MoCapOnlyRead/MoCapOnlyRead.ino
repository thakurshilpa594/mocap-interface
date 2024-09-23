#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Empty.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define pi 3.14159265

#define TCAADDR 0x70
#define TCAFOREARM 2
#define TCAUPPERARM 3
#define TCAWRIST 1

Adafruit_BNO055 bnoForearm = Adafruit_BNO055(55, BNO055_ADDRESS_A);
Adafruit_BNO055 bnoUpperarm = Adafruit_BNO055(55, BNO055_ADDRESS_A);
Adafruit_BNO055 bnoWrist = Adafruit_BNO055(55, BNO055_ADDRESS_A);

/*
 * rosrun rosserial_python serial_node.py /dev/ttyACM0
 * x axis: orthogonal from chip plane
 * y axis: in ADR->SCL direction
 * z axis: in RST->Vin direction
 */

ros::NodeHandle nh;

geometry_msgs::Vector3 imuForearm;
geometry_msgs::Vector3 imuUpperarm;
geometry_msgs::Vector3 imuWrist;

geometry_msgs::Quaternion imuForearmQ;
geometry_msgs::Quaternion imuUpperarmQ;
geometry_msgs::Quaternion imuWristQ;

//---------IMUS---------------//

ros::Publisher imuForearm_pub("imuLowerArm", &imuForearm);
ros::Publisher imuUpperarm_pub("imuUpperArm", &imuUpperarm);
ros::Publisher imuWrist_pub("imuWrist", &imuWrist);

ros::Publisher imuForearmQ_pub("imuLowerArmQ", &imuForearmQ);
ros::Publisher imuUpperarmQ_pub("imuUpperArmQ", &imuUpperarmQ);
ros::Publisher imuWristQ_pub("imuWristQ", &imuWristQ);

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

unsigned long start;

void setup() {

  nh.initNode();

  nh.advertise(imuForearm_pub);
  nh.advertise(imuUpperarm_pub);
  nh.advertise(imuWrist_pub);

  nh.advertise(imuWristQ_pub);
  nh.advertise(imuForearmQ_pub);
  nh.advertise(imuUpperarmQ_pub);

  tcaselect(TCAUPPERARM);
  bnoUpperarm.begin();
  tcaselect(TCAFOREARM);
  bnoForearm.begin();
  tcaselect(TCAWRIST);
  bnoWrist.begin();
  
  tcaselect(TCAUPPERARM); 
  if(!bnoUpperarm.begin()) {
    // There was a problem detecting the BNO055 ... check your connections
    nh.loginfo("No bnoUpperarm detected");
    while(1);
  }
  else { 
    nh.loginfo("bnoUpperarm detected"); 
  }
  delay(1000);
  bnoForearm.setExtCrystalUse(true);

  tcaselect(TCAFOREARM);
  if(!bnoForearm.begin()) {
    // There was a problem detecting the BNO055 ... check your connections
    nh.loginfo("No bnoForeArm detected");
    while(1);
  }
  else { 
    nh.loginfo("bnoForearm detected"); 
  }
  delay(1000);
  bnoForearm.setExtCrystalUse(true);  

  tcaselect(TCAWRIST); 
  if(!bnoWrist.begin()) {
    // There was a problem detecting the BNO455 ... check your connections
    nh.loginfo("No bnoWrist detected");
    while(1);
  }  
  else {
    nh.loginfo("bnoWrist detected"); 
  }
  delay(1000);
  bnoWrist.setExtCrystalUse(true);
}

void loop() {
  
  sensors_event_t event;
  sensors_event_t event2;
  sensors_event_t event3;

  tcaselect(TCAUPPERARM);
  imu::Quaternion Upperquat = bnoUpperarm.getQuat();
  imu::Vector<3> UpperEuler = Upperquat.toEuler();
  imuUpperarm.x = UpperEuler.x()*180.0/pi;
  imuUpperarm.y = UpperEuler.y()*180.0/pi;
  imuUpperarm.z = UpperEuler.z()*180.0/pi;
  
  imuUpperarmQ.x = Upperquat.x();
  imuUpperarmQ.y = Upperquat.y();
  imuUpperarmQ.z = Upperquat.z();
  imuUpperarmQ.w = Upperquat.w();
  
  imuUpperarm_pub.publish( &imuUpperarm );
  imuUpperarmQ_pub.publish( &imuUpperarmQ );
  
  tcaselect(TCAFOREARM);
  imu::Quaternion Forearmquat = bnoForearm.getQuat();
  imu::Vector<3> ForearmEuler = Forearmquat.toEuler();
  imuForearm.x = ForearmEuler.x()*180.0/pi;
  imuForearm.y = ForearmEuler.y()*180.0/pi;
  imuForearm.z = ForearmEuler.z()*180.0/pi;
  
  imuForearmQ.x = Forearmquat.x();
  imuForearmQ.y = Forearmquat.y();
  imuForearmQ.z = Forearmquat.z();
  imuForearmQ.w = Forearmquat.w();
  
  imuForearm_pub.publish( &imuForearm );
  imuForearmQ_pub.publish( &imuForearmQ );

  
  tcaselect(TCAWRIST);
  imu::Quaternion Wristquat = bnoWrist.getQuat();
  imu::Vector<3> WristEuler = Wristquat.toEuler();
  imuWrist.x = WristEuler.x()*180.0/pi;
  imuWrist.y = WristEuler.y()*180.0/pi;
  imuWrist.z = WristEuler.z()*180.0/pi;
  
  imuWristQ.x = Wristquat.x();
  imuWristQ.y = Wristquat.y();
  imuWristQ.z = Wristquat.z();
  imuWristQ.w = Wristquat.w();
  
  imuWrist_pub.publish( &imuWrist );
  imuWristQ_pub.publish( &imuWristQ );
  
  nh.spinOnce();
}
