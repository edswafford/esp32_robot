#include "imu.h"

#include "imu.h"

void IMU::fetch_imu_data(neo_msgs::Imu& raw_imu_msg) {

        raw_imu_msg.linear_acceleration.x = _ax;  // accelerometer measurement _ax in the x direction, m/s/s
         raw_imu_msg.linear_acceleration.y = _ay;  // accelerometer measurement _ay in the y direction, m/s/s
         raw_imu_msg.linear_acceleration.z = _az;  // accelerometer measurement _az in the z direction, m/s/s

        raw_imu_msg.angular_velocity.x = _gx;   //  gyroscope measurement in the x direction, rad/s
        raw_imu_msg.angular_velocity.y = _gy;   //  gyroscope measurement in the y direction, rad/s
        raw_imu_msg.angular_velocity.z = _gz;   //  gyroscope measurement in the z direction, rad/s

         raw_imu_msg.magnetic_field.x = _hx;    //  magnetometer measurement in the x direction, uT
         raw_imu_msg.magnetic_field.y = _hy;    //  magnetometer measurement in the y direction, uT
         raw_imu_msg.magnetic_field.z = _hz;    //  magnetometer measurement in the z direction, uT

    }
