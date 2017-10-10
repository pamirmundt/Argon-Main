#ifndef _ROS_geometry_msgs_TwistWithCovariance_32_h
#define _ROS_geometry_msgs_TwistWithCovariance_32_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "argon_msgs/Twist_32.h"

namespace argon_msgs
{

  class TwistWithCovariance_32 : public ros::Msg
  {
    public:
      argon_msgs::Twist_32 twist;
      float covariance[36];

    TwistWithCovariance_32():
      twist(),
      covariance()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->twist.serialize(outbuffer + offset);
      for( uint8_t i = 0; i < 36; i++){
      union {
        float real;
        uint32_t base;
      } u_covariancei;
      u_covariancei.real = this->covariance[i];
      *(outbuffer + offset + 0) = (u_covariancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_covariancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_covariancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_covariancei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->covariance[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->twist.deserialize(inbuffer + offset);
      for( uint8_t i = 0; i < 36; i++){
      union {
        float real;
        uint32_t base;
      } u_covariancei;
      u_covariancei.base = 0;
      u_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->covariance[i] = u_covariancei.real;
      offset += sizeof(this->covariance[i]);
      }
     return offset;
    }

    const char * getType(){ return "argon_msgs/TwistWithCovariance_32"; };
    const char * getMD5(){ return "13259c3207663c8c9e3df454d5041aaf"; };

  };

}
#endif