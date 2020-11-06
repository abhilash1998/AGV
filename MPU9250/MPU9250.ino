/*
 * Library: https://github.com/bolderflight/MPU9250
Basic_I2C.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/*
 * Updated by Ahmad Shamshiri on July 09, 2018 for Robojax.com
 * in Ajax, Ontario, Canada
 * watch instrucion video for this code: 
For this sketch you need to connect:
VCC to 5V and GND to GND of Arduino
SDA to A4 and SCL to A5

S20A is 3.3V voltage regulator MIC5205-3.3BM5
*/
#include<ros.h>
#include "MPU9250.h"
//#include<std_msgs/string.h>
#include<geometry_msgs/Vector3.h>

ros::NodeHandle nh_imu;

geometry_msgs::Vector3 acc;
geometry_msgs::Vector3 gyro;
geometry_msgs::Vector3 mag;



// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;


ros::Publisher imu_acc_data("imu/acc_data",&acc);
ros::Publisher imu_gyro_data("imu/gyro_data",&gyro);
ros::Publisher imu_mag_data("imu/mag_data",&mag);


void setup() {
  // serial to display data
  status = IMU.begin();
  nh_imu.initNode();
  nh_imu.advertise(imu_acc_data);
  nh_imu.advertise(imu_gyro_data);
  nh_imu.advertise(imu_mag_data);
//   Serial.begin(115200);

//   while(!Serial) {}

//   // start communication with IMU 
  
//   if (status < 0) {
//     Serial.println("IMU initialization unsuccessful");
//     Serial.println("Check IMU wiring or try cycling power");
//     Serial.print("Status: ");
//     Serial.println(status);
//     while(1) {}
//   }
delay(100);
}

void loop() {
  // read the sensor
  IMU.readSensor();
  // display the data
//  Serial.print("AccelX: ");
//  Serial.print(IMU.getAccelX_mss(),6);
  acc.x=IMU.getAccelX_mss();
  acc.y=IMU.getAccelY_mss();
  acc.z=IMU.getAccelZ_mss();
  gyro.x=IMU.getGyroX_rads();
  gyro.y=IMU.getGyroY_rads();
  gyro.z=IMU.getGyroZ_rads();
  mag.x=IMU.getMagX_uT();
  mag.y=IMU.getMagY_uT();
  mag.z=IMU.getMagZ_uT();
  imu_acc_data.publish(&acc);
  imu_gyro_data.publish(&gyro);
  imu_mag_data.publish(&mag);


//   Serial.print("  ");
//   Serial.print("AccelY: ");  
//   Serial.print(IMU.getAccelY_mss(),6);
//   Serial.print("  ");
//   Serial.print("AccelZ: ");  
//   Serial.println(IMU.getAccelZ_mss(),6);
  
//   Serial.print("GyroX: ");
//   Serial.print(IMU.getGyroX_rads(),6);
//   Serial.print("  ");
//   Serial.print("GyroY: ");  
//   Serial.print(IMU.getGyroY_rads(),6);
//   Serial.print("  ");
//   Serial.print("GyroZ: ");  
//   Serial.println(IMU.getGyroZ_rads(),6);

//   Serial.print("MagX: ");  
//   Serial.print(IMU.getMagX_uT(),6);
//   Serial.print("  ");  
//   Serial.print("MagY: ");
//   Serial.print(IMU.getMagY_uT(),6);
//   Serial.print("  ");
//   Serial.print("MagZ: ");  
//   Serial.println(IMU.getMagZ_uT(),6);
  
//   Serial.print("Temperature in C: ");
//   Serial.println(IMU.getTemperature_C(),6);
//   Serial.println();
  delay(2);
  nh_imu.spinOnce();
} 
 
