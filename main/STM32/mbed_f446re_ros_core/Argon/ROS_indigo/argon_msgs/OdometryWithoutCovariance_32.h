#ifndef _ROS_argon_msgs_OdometryWithoutCovariance_32_h
#define _ROS_argon_msgs_OdometryWithoutCovariance_32_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "argon_msgs/PoseWithoutCovariance_32.h"
#include "argon_msgs/TwistWithoutCovariance_32.h"

namespace argon_msgs
{

  class OdometryWithoutCovariance_32 : public ros::Msg
  {
    public:
      std_msgs::Header header;
      const char* child_frame_id;
      argon_msgs::PoseWithoutCovariance_32 pose;
      argon_msgs::TwistWithoutCovariance_32 twist;

    OdometryWithoutCovariance_32():
      header(),
      child_frame_id(""),
      pose(),
      twist()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_child_frame_id = strlen(this->child_frame_id);
      memcpy(outbuffer + offset, &length_child_frame_id, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->child_frame_id, length_child_frame_id);
      offset += length_child_frame_id;
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->twist.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_child_frame_id;
      memcpy(&length_child_frame_id, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_child_frame_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_child_frame_id-1]=0;
      this->child_frame_id = (char *)(inbuffer + offset-1);
      offset += length_child_frame_id;
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->twist.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "argon_msgs/OdometryWithoutCovariance_32"; };
    const char * getMD5(){ return "197bc1db9ac65813e0a9f6b0ba029c9e"; };

  };

}
#endif