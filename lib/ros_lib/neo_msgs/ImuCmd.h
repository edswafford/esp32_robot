#ifndef _ROS_neo_msgs_ImuCmd_h
#define _ROS_neo_msgs_ImuCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace neo_msgs
{

  class ImuCmd : public ros::Msg
  {
    public:
      typedef int8_t _imu_mode_type;
      _imu_mode_type imu_mode;

    ImuCmd():
      imu_mode(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_imu_mode;
      u_imu_mode.real = this->imu_mode;
      *(outbuffer + offset + 0) = (u_imu_mode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_mode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_imu_mode;
      u_imu_mode.base = 0;
      u_imu_mode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_mode = u_imu_mode.real;
      offset += sizeof(this->imu_mode);
     return offset;
    }

    const char * getType(){ return "neo_msgs/ImuCmd"; };
    const char * getMD5(){ return "4428248fe858f51541d36ca1e2e2e211"; };

  };

}
#endif
