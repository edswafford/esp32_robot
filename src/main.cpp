// Define ROS Serial Pins
#define RX2 18
#define TX2 19
#define ROSSERIAL_ARDUINO_SERIAL 1
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <sstream>
#include <string>
#include <Wire.h>

#include "ros.h"
#include "ros/time.h"
#include "neo_msgs/Velocities.h" //header file for publishing velocities for odom
#include "geometry_msgs/Twist.h" //header file for cmd_subscribing to "cmd_vel"
#include "neo_msgs/PID.h"        //header file for pid server
#include "neo_msgs/Imu.h"
#include "neo_msgs/ImuCal.h"
#include "neo_msgs/ImuCmd.h"
#include "cal_imu.h"

#include "neo_base_config.h"
#include "Encoder.h"
#include "Motor.h"
#include "Kinematics.h"
#include "PID.h"
#include "imu.h"
#include "neo_wifi.h"

#define IMU_PUBLISH_RATE 50 //hz
#define COMMAND_RATE 20     //hz
#define DEBUG_RATE 5

void printDebug();
void stopBase();
void moveBase();


Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV);

Controller motor1_controller(Controller::PWM_CHANNEL_1, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2_controller(Controller::PWM_CHANNEL_2, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

unsigned long g_prev_command_time = 0;
bool cmd_vel_timed_out = true;
bool prev_cmd_vel_timed_out = false;

//callback function prototypes
void commandCallback(const geometry_msgs::Twist &cmd_msg);
void PIDCallback(const neo_msgs::PID &pid);
void imuCallback(const neo_msgs::ImuCmd &imu_mode);

ros::NodeHandle nh;
NeoWifi neo_wifi;
bool calibration_mode_ = false;
int imu_status = 0;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<neo_msgs::PID> pid_sub("pid", PIDCallback);
ros::Subscriber<neo_msgs::ImuCmd> imu_mode_sub("imu_cmd", imuCallback);

IMU imu;
neo_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

neo_msgs::ImuCal cal_imu_msg;
ros::Publisher cal_imu_pub("cal_imu", &cal_imu_msg);

neo_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

    void
    setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  }

  // Initialize SPIFFS
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  else
  {
    Serial.println("SPIFFS Mount succesfull");
  }

  // List all files
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while (file)
  {
    Serial.print("FILE: ");
    Serial.println(file.name());
    file = root.openNextFile();
  }

  //
  // WIFI
  //
  Serial.print("WIFI status = ");
  Serial.println(WiFi.getMode());

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(NeoWifi::ssid, NeoWifi::password);

  neo_wifi.init();

  Serial.println("");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors = UP;

  nh.initSerialNode(&Serial2, 57600, SERIAL_8N1, RX2, TX2);

  nh.subscribe(pid_sub);
  nh.subscribe(cmd_sub);
  nh.subscribe(imu_mode_sub);

  nh.advertise(raw_vel_pub);
  nh.advertise(raw_imu_pub);

  Serial.println("Waiting on ROS connection");
  while (!nh.connected())
  {
    nh.spinOnce();
  }
  nh.loginfo("CONNECTED to ROS");

  // start communication with IMU
  imu_status = imu.begin();
  raw_imu_msg.imu_status = static_cast<int8_t>(imu_status);
  if (imu_status < 0)
  {
    if (imu_status == -12)
    {
      nh.loginfo("IMU Calibration data is INVALID.");
      nh.loginfo("Please Calibrate IMU by running the following cmd.");
      nh.loginfo("rostopic pub -1 /imu_cmd neo_msgs/ImuCmd -- 1");
      imu_status = 1;
    }
    else
    {
      // IMU initialization unsuccessful
      nh.loginfo("IMU Error: initialization unsuccessful");
      Serial.print("IMU Error: initialization unsuccessful");
      raw_imu_pub.publish(&raw_imu_msg);
      nh.spinOnce();
    }
  }
  else
  {
    nh.loginfo("IMU Running");
  }
}

void loop()
{
  static unsigned long prev_control_time = 0;
  static unsigned long prev_imu_time = 0;
  static unsigned long prev_debug_time = 0;

  //this block drives the robot based on defined rate
  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
  {
    moveBase();
    prev_control_time = millis();
  }

  //this block stops the motor when no command is received
  if ((millis() - g_prev_command_time) >= 400)
  {
    cmd_vel_timed_out = true;
    stopBase();
    if (cmd_vel_timed_out != prev_cmd_vel_timed_out)
    {
      nh.loginfo("Robot stopped.");
    }
  }

  //this block publishes the IMU data based on defined rate
  if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
  {
    if (imu_status)
    {
      if (calibration_mode_)
      {
        // Calibrate IMU
        CalImu calImu(imu);
        calImu.doCalibration();
        calibration_mode_ = false;

        // cal_imu_pub.publish(&cal_imu_msg);
      }
      else
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
    prev_imu_time = millis();
  }

  //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
  if (DEBUG)
  {
    if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
    {
      // printDebug();
      prev_debug_time = millis();
    }
  }

  //call all the callbacks waiting to be called
  nh.spinOnce();
  prev_cmd_vel_timed_out = cmd_vel_timed_out;
}

void imuCallback(const neo_msgs::ImuCmd &imu_mode)
{
  calibration_mode_ = imu_mode.imu_mode;
}

void PIDCallback(const neo_msgs::PID &pid)
{
  //callback function every time PID constants are received from neo_pid for tuning
  //this callback receives pid object where P,I, and D constants are stored
  motor1_pid.updateConstants(pid.p, pid.i, pid.d);
  motor2_pid.updateConstants(pid.p, pid.i, pid.d);
}

void commandCallback(const geometry_msgs::Twist &cmd_msg)
{
  //callback function every time linear and angular speed is received from 'cmd_vel' topic
  //this callback function receives cmd_msg object where linear and angular speed are stored
  g_req_linear_vel_x = cmd_msg.linear.x;
  g_req_linear_vel_y = cmd_msg.linear.y;
  g_req_angular_vel_z = cmd_msg.angular.z;
  
  g_prev_command_time = millis();
  cmd_vel_timed_out = false;
  if (cmd_vel_timed_out != prev_cmd_vel_timed_out)
  {
    nh.loginfo("Robot received cmd_vel.");
  }
}

void moveBase()
{
  char buffer[128];

  if (neo_wifi.wifi_has_control())
  {
    int linear_x = 0;
    int angular_z = 0;
    neo_wifi.update_vel(linear_x, angular_z);
    g_req_linear_vel_x = linear_x/100.0f;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = angular_z/10.0f;
  }

  //get the required rpm for each motor based on required velocities, and base used
  Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

  //get the current speed of each motor
  int current_rpm1 = motor1_encoder.getRPM();
  int current_rpm2 = motor2_encoder.getRPM();

  //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
  //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
  int pwm1 = motor1_pid.compute(req_rpm.motor1, current_rpm1);
  int pwm2 = motor2_pid.compute(req_rpm.motor2, current_rpm2);

  if (cmd_vel_timed_out)
  {
    pwm1 = 0;
    pwm2 = 0;
  }

  if (DEBUG && (req_rpm.motor1 != 0 || req_rpm.motor2 != 0))
  {
    sprintf(buffer, "rpm 1 req/actual: %d / %d rpm 2 rerq/act: %d / %d", req_rpm.motor1, current_rpm1,
            req_rpm.motor2, current_rpm2);
    nh.loginfo(buffer);
  }
  if (DEBUG && (pwm1 != 0 && pwm2 !=0))
  {
    sprintf(buffer, "pwm %d  %d", pwm1, pwm2);
    nh.loginfo(buffer);
  }

  motor1_controller.spin(pwm1);
  motor2_controller.spin(pwm2);

  Kinematics::velocities current_vel;

  current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, 0, 0);

  if (DEBUG && (g_req_linear_vel_x != 0 || g_req_linear_vel_y != 0 || g_req_angular_vel_z != 0))
  {
    sprintf(buffer, "cmd/act vel: %.1f / %.1f  %.1f / %.1f  %.1f / %.1f", g_req_linear_vel_x, current_vel.linear_x, g_req_linear_vel_y, current_vel.linear_y,
            g_req_angular_vel_z, current_vel.angular_z);
    nh.loginfo(buffer);
  }
  //pass velocities to publisher object
  raw_vel_msg.linear_x = current_vel.linear_x;
  raw_vel_msg.linear_y = current_vel.linear_y;
  raw_vel_msg.angular_z = current_vel.angular_z;

  //publish raw_vel_msg
  raw_vel_pub.publish(&raw_vel_msg);
}

void stopBase()
{
  g_req_linear_vel_x = 0;
  g_req_linear_vel_y = 0;
  g_req_angular_vel_z = 0;
}

void printDebug()
{
  char buffer[256];

  sprintf(buffer, "Encoder FrontLeft  : %d", motor1_encoder.getTicks());
  nh.loginfo(buffer);

  sprintf(buffer, "Encoder FrontRight : %d", motor2_encoder.getTicks());
  nh.loginfo(buffer);
}
