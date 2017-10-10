#ifndef _ROS_geometry_msgs_TwistWithoutCovariance_32_h
#define _ROS_geometry_msgs_TwistWithoutCovariance_32_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "argon_msgs/Twist_32.h"

namespace argon_msgs
{

  class TwistWithoutCovariance_32 : public ros::Msg
  {
    public:
      argon_msgs::Twist_32 twist;

    TwistWithoutCovariance_32():
      twist()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->twist.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->twist.deserialize(inbuffer + offset);
      return offset;
    }

    const char * getType(){ return "argon_msgs/TwistWithoutCovariance_32"; };
    const char * getMD5(){ return "17d1fb87fdfd292aad3e3e41ed4ca7a1"; };

  };

}
#endif