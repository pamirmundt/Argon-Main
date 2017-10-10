#ifndef _ROS_argon_msgs_Twist_32_h
#define _ROS_argon_msgs_Twist_32_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "argon_msgs/Vector3_32.h"

namespace argon_msgs
{

  class Twist_32 : public ros::Msg
  {
    public:
      argon_msgs::Vector3_32 linear;
      argon_msgs::Vector3_32 angular;

    Twist_32():
      linear(),
      angular()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->linear.serialize(outbuffer + offset);
      offset += this->angular.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->linear.deserialize(inbuffer + offset);
      offset += this->angular.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "argon_msgs/Twist_32"; };
    const char * getMD5(){ return "6d107193b261039abb32b01ddb75189b"; };

  };

}
#endif