#ifndef _ROS_argon_msgs_PoseWithoutCovariance_32_h
#define _ROS_argon_msgs_PoseWithoutCovariance_32_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "argon_msgs/Pose_32.h"

namespace argon_msgs
{

  class PoseWithoutCovariance_32 : public ros::Msg
  {
    public:
      argon_msgs::Pose_32 pose;

    PoseWithoutCovariance_32():
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
      return offset;
    }

    const char * getType(){ return "argon_msgs/PoseWithoutCovariance_32"; };
    const char * getMD5(){ return "13367334b75fe3f9369451d6f9c29c1c"; };

  };

}
#endif