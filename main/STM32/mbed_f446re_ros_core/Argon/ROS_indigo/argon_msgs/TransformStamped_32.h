#ifndef _ROS_argon_msgs_TransformStamped_32_h
#define _ROS_argon_msgs_TransformStamped_32_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "argon_msgs/Transform_32.h"

namespace argon_msgs
{

  class TransformStamped_32 : public ros::Msg
  {
    public:
      std_msgs::Header header;
      const char* child_frame_id;
      argon_msgs::Transform_32 transform;

    TransformStamped_32():
      header(),
      child_frame_id(""),
      transform()
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
      offset += this->transform.serialize(outbuffer + offset);
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
      offset += this->transform.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "argon_msgs/TransformStamped_32"; };
    const char * getMD5(){ return "4ba18f31ea958702facc2f14eca40b3e"; };

  };

}
#endif