#ifndef _ROS_argon_msgs_PoseWithCovariance_32_h
#define _ROS_argon_msgs_PoseWithCovariance_32_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "argon_msgs/Pose_32.h"

namespace argon_msgs
{

  class PoseWithCovariance_32 : public ros::Msg
  {
    public:
      argon_msgs::Pose_32 pose;
      float covariance[36];

    PoseWithCovariance_32():
      pose(),
      covariance()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
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
      offset += this->pose.deserialize(inbuffer + offset);
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

    const char * getType(){ return "argon_msgs/PoseWithCovariance_32"; };
    const char * getMD5(){ return "694fc7bed28ed294b2bffba0a6b0ec90"; };

  };

}
#endif