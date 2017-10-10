#ifndef _ROS_argon_msgs_Imu_32_h
#define _ROS_argon_msgs_Imu_32_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "argon_msgs/Quaternion_32.h"
#include "argon_msgs/Vector3_32.h"

namespace argon_msgs
{

  class Imu_32 : public ros::Msg
  {
    public:
      std_msgs::Header header;
      argon_msgs::Quaternion_32 orientation;
      float orientation_covariance[9];
      argon_msgs::Vector3_32 angular_velocity;
      float angular_velocity_covariance[9];
      argon_msgs::Vector3_32 linear_acceleration;
      float linear_acceleration_covariance[9];

    Imu_32():
      header(),
      orientation(),
      orientation_covariance(),
      angular_velocity(),
      angular_velocity_covariance(),
      linear_acceleration(),
      linear_acceleration_covariance()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      for( uint8_t i = 0; i < 9; i++){
      union {
        float real;
        uint32_t base;
      } u_orientation_covariancei;
      u_orientation_covariancei.real = this->orientation_covariance[i];
      *(outbuffer + offset + 0) = (u_orientation_covariancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_orientation_covariancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_orientation_covariancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_orientation_covariancei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->orientation_covariance[i]);
      }
      offset += this->angular_velocity.serialize(outbuffer + offset);
      for( uint8_t i = 0; i < 9; i++){
      union {
        float real;
        uint32_t base;
      } u_angular_velocity_covariancei;
      u_angular_velocity_covariancei.real = this->angular_velocity_covariance[i];
      *(outbuffer + offset + 0) = (u_angular_velocity_covariancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angular_velocity_covariancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angular_velocity_covariancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angular_velocity_covariancei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angular_velocity_covariance[i]);
      }
      offset += this->linear_acceleration.serialize(outbuffer + offset);
      for( uint8_t i = 0; i < 9; i++){
      union {
        float real;
        uint32_t base;
      } u_linear_acceleration_covariancei;
      u_linear_acceleration_covariancei.real = this->linear_acceleration_covariance[i];
      *(outbuffer + offset + 0) = (u_linear_acceleration_covariancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linear_acceleration_covariancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_linear_acceleration_covariancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_linear_acceleration_covariancei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->linear_acceleration_covariance[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
      for( uint8_t i = 0; i < 9; i++){
      union {
        float real;
        uint32_t base;
      } u_orientation_covariancei;
      u_orientation_covariancei.base = 0;
      u_orientation_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_orientation_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_orientation_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_orientation_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->orientation_covariance[i] = u_orientation_covariancei.real;
      offset += sizeof(this->orientation_covariance[i]);
      }
      offset += this->angular_velocity.deserialize(inbuffer + offset);
      for( uint8_t i = 0; i < 9; i++){
      union {
        float real;
        uint32_t base;
      } u_angular_velocity_covariancei;
      u_angular_velocity_covariancei.base = 0;
      u_angular_velocity_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angular_velocity_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angular_velocity_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angular_velocity_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angular_velocity_covariance[i] = u_angular_velocity_covariancei.real;
      offset += sizeof(this->angular_velocity_covariance[i]);
      }
      offset += this->linear_acceleration.deserialize(inbuffer + offset);
      for( uint8_t i = 0; i < 9; i++){
      union {
        float real;
        uint32_t base;
      } u_linear_acceleration_covariancei;
      u_linear_acceleration_covariancei.base = 0;
      u_linear_acceleration_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_linear_acceleration_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_linear_acceleration_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_linear_acceleration_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->linear_acceleration_covariance[i] = u_linear_acceleration_covariancei.real;
      offset += sizeof(this->linear_acceleration_covariance[i]);
      }
     return offset;
    }


    const char * getType(){ return "argon_msgs/Imu_32"; };
    const char * getMD5(){ return "d77efde226c69d8bb876df453841af91"; };

  };

}
#endif