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
#include "neo_msgs/Velocities.h"  //header file for publishing velocities for odom
#include "geometry_msgs/Twist.h"  //header file for cmd_subscribing to "cmd_vel"
#include "neo_msgs/PID.h"         //header file for pid server
#include "neo_msgs/Imu.h"
#include "neo_msgs/ImuCal.h"
#include "neo_msgs/ImuCmd.h"
#include "cal_imu.h"


//#include "neo_base_config.h"
//#include "Motor.h"
//#include "Kinematics.h"
//#include "PID.h"
#include "imu.h"

#include <ESP32Encoder.h>

#define IMU_PUBLISH_RATE 50 //hz

//callback function prototypes
void commandCallback(const geometry_msgs::Twist &cmd_msg);
void PIDCallback(const neo_msgs::PID &pid);
void imuCallback(const neo_msgs::ImuCmd &imu_mode);

ros::NodeHandle nh;
bool calibration_mode_ = false;
int imu_status = 0;


//ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
//ros::Subscriber<neo_msgs::PID> pid_sub("pid", PIDCallback);
ros::Subscriber<neo_msgs::ImuCmd> imu_mode_sub("imu_cmd", imuCallback);

IMU imu;
neo_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

neo_msgs::ImuCal cal_imu_msg;
ros::Publisher cal_imu_pub("cal_imu", &cal_imu_msg);

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  }

  nh.initSerialNode(&Serial2, 57600, SERIAL_8N1, RX2, TX2);

  // nh.subscribe(pid_sub);
  //nh.subscribe(cmd_sub);
    nh.subscribe(imu_mode_sub);

  // nh.advertise(raw_vel_pub);
  nh.advertise(raw_imu_pub);

  while (!nh.connected())
  {
    nh.spinOnce();
  }
  nh.loginfo("CONNECTED to ROS");

  // start communication with IMU
  if (!imu.init())
  {
    // IMU initialization unsuccessful
    nh.loginfo("IMU Error: initialization unsuccessful");
    raw_imu_pub.publish(&raw_imu_msg);
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

  if (imu.isValid())
  {
    if (calibration_mode_)
    {
      // Calibrate IMU
    CalImu calImu(imu.getRTIMU(), imu.getSettings());
    calImu.doCalibration();
    calibration_mode_ = false;

     // cal_imu_pub.publish(&cal_imu_msg);
    }
    else
    {
      // Normal IMU Processing
      /*
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
      */
    }
  }
  //call all the callbacks waiting to be called
  nh.spinOnce();
}


void imuCallback(const neo_msgs::ImuCmd &imu_mode)
{
  calibration_mode_ = imu_mode.imu_mode;
}