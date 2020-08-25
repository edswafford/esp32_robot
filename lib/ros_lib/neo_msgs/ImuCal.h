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
      typedef geometry_msgs::Vector3 _min_acceleration_type;
      _min_acceleration_type min_acceleration;
      typedef geometry_msgs::Vector3 _max_acceleration_type;
      _max_acceleration_type max_acceleration;
      typedef geometry_msgs::Vector3 _min_magnetic_field_type;
      _min_magnetic_field_type min_magnetic_field;
      typedef geometry_msgs::Vector3 _max_magnetic_field_type;
      _max_magnetic_field_type max_magnetic_field;
      typedef int8_t _imu_status_type;
      _imu_status_type imu_status;

    ImuCal():
      min_acceleration(),
      max_acceleration(),
      min_magnetic_field(),
      max_magnetic_field(),
      imu_status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->min_acceleration.serialize(outbuffer + offset);
      offset += this->max_acceleration.serialize(outbuffer + offset);
      offset += this->min_magnetic_field.serialize(outbuffer + offset);
      offset += this->max_magnetic_field.serialize(outbuffer + offset);
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
      offset += this->min_acceleration.deserialize(inbuffer + offset);
      offset += this->max_acceleration.deserialize(inbuffer + offset);
      offset += this->min_magnetic_field.deserialize(inbuffer + offset);
      offset += this->max_magnetic_field.deserialize(inbuffer + offset);
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
    const char * getMD5(){ return "2678d2dd208c377a822a2197c815d8f1"; };

  };

}
#endif
