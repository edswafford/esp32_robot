#ifndef _ROS_neo_msgs_Imu_h
#define _ROS_neo_msgs_Imu_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace neo_msgs
{

  class Imu : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _linear_acceleration_type;
      _linear_acceleration_type linear_acceleration;
      typedef geometry_msgs::Vector3 _angular_velocity_type;
      _angular_velocity_type angular_velocity;
      typedef geometry_msgs::Vector3 _magnetic_field_type;
      _magnetic_field_type magnetic_field;
      typedef int8_t _imu_status_type;
      _imu_status_type imu_status;

    Imu():
      linear_acceleration(),
      angular_velocity(),
      magnetic_field(),
      imu_status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->linear_acceleration.serialize(outbuffer + offset);
      offset += this->angular_velocity.serialize(outbuffer + offset);
      offset += this->magnetic_field.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_imu_status;
      u_imu_status.real = this->imu_status;
      *(outbuffer + offset + 0) = (u_imu_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->linear_acceleration.deserialize(inbuffer + offset);
      offset += this->angular_velocity.deserialize(inbuffer + offset);
      offset += this->magnetic_field.deserialize(inbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_imu_status;
      u_imu_status.base = 0;
      u_imu_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_status = u_imu_status.real;
      offset += sizeof(this->imu_status);
     return offset;
    }

    const char * getType(){ return "neo_msgs/Imu"; };
    const char * getMD5(){ return "1eca2f100d745e22e80fa28798d1b5e8"; };

  };

}
#endif
