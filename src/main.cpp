// Define ROS Serial Pins
#define RX2 18
#define TX2 19
#define ROSSERIAL_ARDUINO_SERIAL 1

#include <Arduino.h>

#include <string.h>
#include <Wire.h>
#include <ESP32Servo.h>

#include "ros.h"
#include "ros/time.h"
//header file for publishing velocities for odom
#include "neo_msgs/Velocities.h"
//header file for cmd_subscribing to "cmd_vel"
#include "geometry_msgs/Twist.h"
//header file for pid server
#include "neo_msgs/PID.h"
//header file for imu
#include "neo_msgs/Imu.h"
#include "neo_msgs/ImuCal.h"
#include "neo_msgs/ImuCalState.h"

//#include "neo_base_config.h"
//#include "Motor.h"
//#include "Kinematics.h"
//#include "PID.h"
#include "imu.h"

#include <ESP32Encoder.h>

#define IMU_PUBLISH_RATE 20 //hz
#define COMMAND_RATE 20     //hz
#define DEBUG_RATE 5

//callback function prototypes
void imuCalStateCallback(const neo_msgs::ImuCalState &cmd_msg);
void commandCallback(const geometry_msgs::Twist &cmd_msg);
void PIDCallback(const neo_msgs::PID &pid);

ros::NodeHandle nh;
int imu_status = 0;
short imu_cal_state = CalibCmds::END;

IMU imu;
neo_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

neo_msgs::ImuCal cal_imu_msg;
ros::Publisher cal_imu_pub("cal_imu", &cal_imu_msg);

ros::Subscriber<neo_msgs::ImuCalState> imu_cal_state_sub("imu_cal_state", imuCalStateCallback);

void imuCalStateCallback(const neo_msgs::ImuCalState &cmd)
{
  imu_cal_state = cmd.imu_state;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  }


  nh.initSerialNode(&Serial2, 57600, SERIAL_8N1, RX2, TX2);
  nh.subscribe(imu_cal_state_sub);

  // nh.subscribe(pid_sub);
  //nh.subscribe(cmd_sub);
  // nh.advertise(raw_vel_pub);
  nh.advertise(raw_imu_pub);

  while (!nh.connected())
  {
    nh.spinOnce();
  }
  nh.loginfo("CONNECTED to ROS");
  Serial.println("ESP32 IMU running");

  // start communication with IMU
  imu_status = imu.begin();
  raw_imu_msg.imu_status = static_cast<int8_t>(imu_status);
  if (imu_status < 0)
  {
    // IMU initialization unsuccessful
    nh.loginfo("IMU Error: initialization unsuccessful");
    raw_imu_pub.publish(&raw_imu_msg);
    nh.spinOnce();
  }
  else
  {
    nh.loginfo("IMU Running");
  }
}

void loop()
{
  //static unsigned long prev_control_time = 0;
  static unsigned long prev_imu_time = 0;
  //static unsigned long prev_debug_time = 0;

  if (imu_status)
  {
    if (imu_cal_state != CalibCmds::END)
    {
      // Calibrating IMU
      switch (imu_cal_state)
      {
      case CalibCmds::CAL_ACCEL: 
        nh.loginfo("Calling Accel Cal");
        imu.calibrate_accelerometer(cal_imu_msg);
         nh.loginfo("publishing cal_imu_msg");
         cal_imu_pub.publish(&cal_imu_msg);
         nh.loginfo("Done");
        break;
      case CalibCmds::CAL_MAG: 
        
        break;

        case CalibCmds::SAVE: 
        
        break;

      default:
        break;
      }
    }
    else
    {
      //this block publishes the IMU data based on defined rate
      if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
      {
        // read the sensor
        imu.readSensor();
        // store imu data in msg
        imu.fetch_imu_data(raw_imu_msg);
        //publish raw_imu_msg
        raw_imu_pub.publish(&raw_imu_msg);

        prev_imu_time = millis();
      }
    }
  }
  //call all the callbacks waiting to be called
  nh.spinOnce();
}
