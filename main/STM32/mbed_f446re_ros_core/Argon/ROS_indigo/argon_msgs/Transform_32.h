#ifndef _ROS_argon_msgs_Transform_32_h
#define _ROS_argon_msgs_Transform_32_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "argon_msgs/Vector3_32.h"
#include "argon_msgs/Quaternion_32.h"

namespace argon_msgs
{

  class Transform_32 : public ros::Msg
  {
    public:
      argon_msgs::Vector3_32 translation;
      argon_msgs::Quaternion_32 rotation;

    Transform_32():
      translation(),
      rotation()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->translation.serialize(outbuffer + offset);
      offset += this->rotation.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->translation.deserialize(inbuffer + offset);
      offset += this->rotation.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "argon_msgs/Transform_32"; };
    const char * getMD5(){ return "ba485d639b2ab8864239876f74d59ce7"; };

  };

}
#endif