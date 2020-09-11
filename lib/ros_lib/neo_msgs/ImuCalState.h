#ifndef _ROS_neo_msgs_ImuCalState_h
#define _ROS_neo_msgs_ImuCalState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace neo_msgs
{

  class ImuCalState : public ros::Msg
  {
    public:
      typedef int8_t _imu_state_type;
      _imu_state_type imu_state;

    ImuCalState():
      imu_state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_imu_state;
      u_imu_state.real = this->imu_state;
      *(outbuffer + offset + 0) = (u_imu_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_imu_state;
      u_imu_state.base = 0;
      u_imu_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->imu_state = u_imu_state.real;
      offset += sizeof(this->imu_state);
     return offset;
    }

    const char * getType(){ return "neo_msgs/ImuCalState"; };
    const char * getMD5(){ return "4efba3ea876079a2f1497589cb91944f"; };

  };

}
#endif
