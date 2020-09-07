#ifndef _ROS_neo_msgs_ImuCal_h
#define _ROS_neo_msgs_ImuCal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace neo_msgs
{

  class ImuCal : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _acceleration_bias_type;
      _acceleration_bias_type acceleration_bias;
      typedef geometry_msgs::Vector3 _acceleration_scale_factor_type;
      _acceleration_scale_factor_type acceleration_scale_factor;
      typedef geometry_msgs::Vector3 _magnetic_field_bias_type;
      _magnetic_field_bias_type magnetic_field_bias;
      typedef geometry_msgs::Vector3 _magnetic_field_scale_factor_type;
      _magnetic_field_scale_factor_type magnetic_field_scale_factor;
      typedef int8_t _imu_status_type;
      _imu_status_type imu_status;

    ImuCal():
      acceleration_bias(),
      acceleration_scale_factor(),
      magnetic_field_bias(),
      magnetic_field_scale_factor(),
      imu_status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->acceleration_bias.serialize(outbuffer + offset);
      offset += this->acceleration_scale_factor.serialize(outbuffer + offset);
      offset += this->magnetic_field_bias.serialize(outbuffer + offset);
      offset += this->magnetic_field_scale_factor.serialize(outbuffer + offset);
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
      offset += this->acceleration_bias.deserialize(inbuffer + offset);
      offset += this->acceleration_scale_factor.deserialize(inbuffer + offset);
      offset += this->magnetic_field_bias.deserialize(inbuffer + offset);
      offset += this->magnetic_field_scale_factor.deserialize(inbuffer + offset);
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

    const char * getType(){ return "neo_msgs/ImuCal"; };
    const char * getMD5(){ return "041daa81b02152b04acd7ab8e7324af6"; };

  };

}
#endif
