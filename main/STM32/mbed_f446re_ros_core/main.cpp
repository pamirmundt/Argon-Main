/*
 * rosserial Time and TF Example
 * Publishes a transform at current time
 */

#include "mbed.h"
#include "argon_core_config.h"

ros::NodeHandle nh;

Serial pc(USBTX, USBRX);

/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[4];
Timer t;

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<argon_msgs::Twist_32> cmd_vel_sub("cmd_vel_32", commandVelocityCallback);

/*******************************************************************************
* Publishers
*******************************************************************************/
// IMU_32
argon_msgs::ImuWithoutCovariance_32 imu_msg;
ros::Publisher imu_pub("/imu_32", &imu_msg);
//IMU_TF_32
argon_msgs::TransformStamped_32 imu_tf;
ros::Publisher imu_tf_pub("/imu_tf_32", &imu_tf);
//TF_32
argon_msgs::TransformStamped_32 odom_tf;
ros::Publisher odom_tf_pub("/tf_32", &odom_tf);
//Odom
argon_msgs::OdometryWithoutCovariance_32 odom;
ros::Publisher odom_pub("/odom_32", &odom);
//JointState
argon_msgs::JointState_32 joint_states;
ros::Publisher joint_states_pub("/joint_states_32", &joint_states);

/*******************************************************************************
* Declaration for IMU
*******************************************************************************/
Adafruit_BNO055 bno(BNO055_SDA, BNO055_SCL);

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
char *joint_states_name[] = {"wheel_front_left_joint",
                             "wheel_front_right_joint",
                             "wheel_rear_left_joint",
                             "wheel_rear_right_joint"};
float joint_states_pos[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float joint_states_vel[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float joint_states_eff[4] = {0.0f, 0.0f, 0.0f, 0.0f};

/*******************************************************************************
* Main
*******************************************************************************/
Base myBase;
velocityPID FLVelocityControl(myBase.frontLeft, WHEEL_VELOCITY_KP, WHEEL_VELOCITY_KI, WHEEL_VELOCITY_KD, WHEEL_VELOCITY_CONT_PERIOD);
velocityPID FRVelocityControl(myBase.frontRight, WHEEL_VELOCITY_KP, WHEEL_VELOCITY_KI, WHEEL_VELOCITY_KD, WHEEL_VELOCITY_CONT_PERIOD);
velocityPID RLVelocityControl(myBase.rearLeft, WHEEL_VELOCITY_KP, WHEEL_VELOCITY_KI, WHEEL_VELOCITY_KD, WHEEL_VELOCITY_CONT_PERIOD);
velocityPID RRVelocityControl(myBase.rearRight, WHEEL_VELOCITY_KP, WHEEL_VELOCITY_KI, WHEEL_VELOCITY_KD, WHEEL_VELOCITY_CONT_PERIOD);

int main() {
    //Mecanum Base Controllers Start
    FLVelocityControl.start();
    FRVelocityControl.start();
    RLVelocityControl.start();
    RRVelocityControl.start();
        
    //ROS Timer
    t.start();
    nh.getHardware()->setBaud(ROS_SERIAL_BAUD);
    nh.initNode();
    
    nh.subscribe(cmd_vel_sub);
    nh.advertise(imu_pub);
    //nh.advertise(imu_tf_pub);
    nh.advertise(odom_pub);
    nh.advertise(odom_tf_pub);
    nh.advertise(joint_states_pub);
    
    //Joint States
    joint_states.header.frame_id = "base_footprint";
    joint_states.name            = joint_states_name;
    
    joint_states.name_length     = 4;
    joint_states.position_length = 4;
    joint_states.velocity_length = 4;
    joint_states.effort_length   = 4;
    
    // Setting for IMU
    initIMU();
           
    while (1) {
        if (t.read_us() - tTime[1] >= 1000000/DRIVE_INFORMATION_PUBLISH_PERIOD){
            //publishSensorStateMsg();
            publishDriveInformation();
            tTime[1] = t.read_us();
        }

        if (t.read_us() - tTime[2] >= 1000000/IMU_PUBLISH_PERIOD){
            publishImuMsg();
            tTime[2] = t.read_us();
        }
        
        //LOG INFO
        if (t.read_us() - tTime[3] >= 1000000/LOG_INFO_PUBLISH_PERIOD){
            //Print Temp
            char buffer [50];
            //sprintf (buffer, "Temperature: %d", bno.getTemp());
            //nh.loginfo(buffer);
            //Print Cal
            uint8_t system = 0, gyro = 0, accel = 0, mag = 0;
            bno.getCalibration(&system, &gyro, &accel, &mag);
            sprintf (buffer, "Sys: %d G: %d A: %d M: %d", system, gyro, accel, mag);
            nh.loginfo(buffer);
            tTime[3] = t.read_us();
        }

        //pc.printf("%f %f %f %f\r\n", myBase.frontLeft.getJointRPM(),myBase.frontRight.getJointRPM(),myBase.rearLeft.getJointRPM(),myBase.rearRight.getJointRPM());
        
        nh.spinOnce();
    }
}


/*******************************************************************************
* Initialize BNO055 9-Dof IMU
*******************************************************************************/
void initIMU(){
    bno.begin();
    
    adafruit_bno055_offsets_t calibrationData;
    
    calibrationData.accel_offset_x = ACCEL_OFFSET_X;
    calibrationData.accel_offset_y = ACCEL_OFFSET_Y;
    calibrationData.accel_offset_z = ACCEL_OFFSET_Z;
    
    calibrationData.gyro_offset_x = GYRO_OFFSET_X;
    calibrationData.gyro_offset_y = GYRO_OFFSET_Y;
    calibrationData.gyro_offset_z = GYRO_OFFSET_Z;
    
    calibrationData.mag_offset_x = MAG_OFFSET_X;
    calibrationData.mag_offset_y = MAG_OFFSET_Y;
    calibrationData.mag_offset_z = MAG_OFFSET_Z;
    
    bno.setSensorOffsets(calibrationData);
    
    bno.setExtCrystalUse(true);
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const argon_msgs::Twist_32& cmd_vel_msg)
{
    float refLongVel = cmd_vel_msg.linear.x;
    float refTransVel = cmd_vel_msg.linear.y;
    float refAngVel = cmd_vel_msg.angular.z;
    
    float rpmFL, rpmFR, rpmRL, rpmRR;    
    cartesianVelocityToWheelVelocities(refLongVel, refTransVel, refAngVel, &rpmFL, &rpmFR, &rpmRL, &rpmRR);
    
    FLVelocityControl.setRef(rpmFL);
    FRVelocityControl.setRef(rpmFR);
    RLVelocityControl.setRef(rpmRL);
    RRVelocityControl.setRef(rpmRR);
}

/*******************************************************************************
* Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void){
  ros::Time stamp_now = nh.now();

  // odom
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // joint_states
  updateJoint();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);

  // tf
  updateTF();
  odom_tf.header.stamp = stamp_now;
  odom_tf_pub.publish(&odom_tf);
}

/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
    imu_msg.header.stamp = nh.now();
    imu_msg.header.frame_id = "imu_link";
    
    imu::Quaternion quat = bno.getQuat();                                       //Quaternian
    
    imu_msg.orientation.w = quat.w();
    imu_msg.orientation.x = quat.x();
    imu_msg.orientation.y = quat.y();
    imu_msg.orientation.z = quat.z();
    
    imu::Vector<3> angVel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);   //rad/s
    
    imu_msg.angular_velocity.x = angVel.x();
    imu_msg.angular_velocity.y = angVel.y();
    imu_msg.angular_velocity.z = angVel.z();
    
    imu::Vector<3> linAcc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); //m/s^2
    
    imu_msg.linear_acceleration.x = linAcc.x();
    imu_msg.linear_acceleration.y = linAcc.y();
    imu_msg.linear_acceleration.z = linAcc.z();
    
    imu_pub.publish(&imu_msg);

    /*
    imu_tf.header.stamp    = nh.now();
    imu_tf.header.frame_id = "base_link";
    imu_tf.child_frame_id  = "imu_link";
    imu_tf.transform.rotation.w = 1.0f;
    imu_tf.transform.rotation.x = 0.0f;
    imu_tf.transform.rotation.y = 0.0f;
    imu_tf.transform.rotation.z = 0.0f;
    
    imu_tf.transform.translation.x = -0.032;
    imu_tf.transform.translation.y = 0.0;
    imu_tf.transform.translation.z = 0.068;
    
    imu_tf_pub.publish(&imu_tf);
    */
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
void updateOdometry(void){
    float odom_vel[3];
    //Linear Velocity
    wheelVelocitiesToCartesianVelocity(myBase.frontLeft.getJointRPM(),
                                        myBase.frontRight.getJointRPM(),
                                        myBase.rearLeft.getJointRPM(),
                                        myBase.frontRight.getJointRPM(),
                                        &odom_vel[0],
                                        &odom_vel[1],
                                        &odom_vel[2]);
    
    //Update Odometry
    wheelPositionsToCartesianPosition(myBase.frontLeft.getEncoderCount(),
                                        myBase.frontRight.getEncoderCount(),
                                        myBase.rearLeft.getEncoderCount(),
                                        myBase.rearRight.getEncoderCount());
    
    //Odom
    odom.header.frame_id = "odom";
    //odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = getLongitudinalPosition();
    odom.pose.pose.position.y = getTransversalPosition();
    odom.pose.pose.position.z = 0.0f;
    
    float orien = getOrientation();
    odom.pose.pose.orientation.x = (float)(tf::createQuaternionFromYaw(orien).x);
    odom.pose.pose.orientation.y = (float)(tf::createQuaternionFromYaw(orien).y);
    odom.pose.pose.orientation.z = (float)(tf::createQuaternionFromYaw(orien).z);
    odom.pose.pose.orientation.w = (float)(tf::createQuaternionFromYaw(orien).w);
    
    odom.twist.twist.linear.x = odom_vel[0];
    odom.twist.twist.linear.y = odom_vel[1];
    odom.twist.twist.linear.z = 0.0f;
    
    odom.twist.twist.angular.x = 0.0f;
    odom.twist.twist.angular.y = 0.0f;
    odom.twist.twist.angular.z = odom_vel[2];
}

/*******************************************************************************
* Calculate the joint states
*******************************************************************************/
void updateJoint(void){
    //Joint States
    joint_states_pos[0] = myBase.frontLeft.getJointPosition();
    joint_states_pos[1] = myBase.frontRight.getJointPosition();
    joint_states_pos[2] = myBase.rearLeft.getJointPosition();
    joint_states_pos[3] = myBase.rearRight.getJointPosition();

    joint_states_vel[0] = myBase.frontLeft.getJointAngVel();
    joint_states_vel[1] = myBase.frontRight.getJointAngVel();
    joint_states_vel[2] = myBase.rearLeft.getJointAngVel();
    joint_states_vel[3] = myBase.rearRight.getJointAngVel();
    
    joint_states.position = joint_states_pos;
    joint_states.velocity = joint_states_vel;
    joint_states.effort = joint_states_eff;
}

/*******************************************************************************
* Calculate the TF
*******************************************************************************/
void updateTF(void){
    //TF
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_footprint";
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation = odom.pose.pose.orientation;
}
