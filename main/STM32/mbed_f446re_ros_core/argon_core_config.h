#ifndef ARGON_CORE_CONFIG_H_
#define ARGON_CORE_CONFIG_H_

#include <math.h>

#include <ros.h>
#include <ros/time.h>
#include "tf/tf.h"
#include "argon_msgs/Twist_32.h"
#include "argon_msgs/JointState_32.h"
#include "argon_msgs/TransformStamped_32.h"
#include "argon_msgs/ImuWithoutCovariance_32.h"
#include "argon_msgs/OdometryWithoutCovariance_32.h"

#include "Argon.h"
#include "base_kinematics.h"
#include "PID_control.h"

#define ROS_SERIAL_BAUD                         500000

#define DRIVE_INFORMATION_PUBLISH_PERIOD        35.0f//hz
#define IMU_PUBLISH_PERIOD                      35.0f//hz
#define LOG_INFO_PUBLISH_PERIOD                 0.5f//hz

#define WHEEL_VELOCITY_KP                       80.0f
#define WHEEL_VELOCITY_KI                       0.0f
#define WHEEL_VELOCITY_KD                       160.0f
#define WHEEL_VELOCITY_CONT_PERIOD              0.01f

#define ACCEL_OFFSET_X                          3
#define ACCEL_OFFSET_Y                          65473
#define ACCEL_OFFSET_Z                          12
#define GYRO_OFFSET_X                           65473
#define GYRO_OFFSET_Y                           1
#define GYRO_OFFSET_Z                           0
#define MAG_OFFSET_X                            65406
#define MAG_OFFSET_Y                            362
#define MAG_OFFSET_Z                            65071

// Callback function prototypes
void updateTF(void);
void updateJoint(void);
void publishImuMsg(void);
void updateOdometry(void);
void initIMU(void);
void publishDriveInformation(void);
void commandVelocityCallback(const argon_msgs::Twist_32& cmd_vel_msg);


#endif // ARGON_CORE_CONFIG_H_