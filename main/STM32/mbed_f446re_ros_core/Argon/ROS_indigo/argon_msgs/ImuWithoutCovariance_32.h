#ifndef _ROS_argon_msgs_ImuWithoutCovariance_32_h
#define _ROS_argon_msgs_ImuWithoutCovariance_32_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "argon_msgs/Quaternion_32.h"
#include "argon_msgs/Vector3_32.h"

namespace argon_msgs
{

  class ImuWithoutCovariance_32 : public ros::Msg
  {
    public:
      std_msgs::Header header;
      argon_msgs::Quaternion_32 orientation;
      //float orientation_covariance[9];
      argon_msgs::Vector3_32 angular_velocity;
      //float angular_velocity_covariance[9];
      argon_msgs::Vector3_32 linear_acceleration;
      //float linear_acceleration_covariance[9];

    ImuWithoutCovariance_32():
      header(),
      orientation(),
      //orientation_covariance(),
      angular_velocity(),
      //angular_velocity_covariance(),
      linear_acceleration()
      //linear_acceleration_covariance()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      offset += this->angular_velocity.serialize(outbuffer + offset);
      offset += this->linear_acceleration.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
      offset += this->angular_velocity.deserialize(inbuffer + offset);
      offset += this->linear_acceleration.deserialize(inbuffer + offset);
      return offset;
    }


    const char * getType(){ return "argon_msgs/ImuWithoutCovariance_32"; };
    const char * getMD5(){ return "90221191271542df64b732b0eb4ee945"; };

  };

}
#endif