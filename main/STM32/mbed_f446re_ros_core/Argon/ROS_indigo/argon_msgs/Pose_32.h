#ifndef _ROS_argon_msgs_Pose_32_h
#define _ROS_argon_msgs_Pose_32_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "argon_msgs/Point_32.h"
#include "argon_msgs/Quaternion_32.h"

namespace argon_msgs
{

  class Pose_32 : public ros::Msg
  {
    public:
      argon_msgs::Point_32 position;
      argon_msgs::Quaternion_32 orientation;

    Pose_32():
      position(),
      orientation()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->position.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->position.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "argon_msgs/Pose_32"; };
    const char * getMD5(){ return "1ec5a94781ee76f5aa190025baf9a1a4"; };

  };

}
#endif