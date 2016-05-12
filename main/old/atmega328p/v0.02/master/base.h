#ifndef BASE_H
#define BASE_H

#include <stdint.h>
#include "BaseJoint.h"
#include "BaseKinematics.h"

class Base{
  public:
    //Base constructor
    Base(Wheel frontLeftWheel, Wheel frontRightWheel, Wheel rearLeftWheel, Wheel rearRightWheel);

    //Resets every wheel with @.reset()
    //W0.reset() - W1.reset() - W2.reset() - W3.reset()
    void reset();

    //Sets the control mode of the base
    //0 - Manuel Mode (setPower, setPWM, etc.)
    //1 - Position Control Mode with PID
    //2 - Velocity Control Mode with PID
    void setMode(uint8_t);
    
    //Gets the cartesian base position
    //@param longitudinalPosition - forward/backward position
    //@param transversalPosition - sideway position
    //@param orientation - orientation
    void getBasePosition(float& longitudinalPosition, float& transversalPosition, float& orientation);

    //Sets the cartesian base position
    //@param longitudinalPosition - forward/backward position
    //@param transversalPosition - sideway position
    //@param orientation - orientation
    void setBasePosition(float longitudinalPosition, float transversalPosition, float angularPosition, float intervalTime);

    //Gets the cartesian base velocity
    //@param longitudinalVelocity - forward/backward velocity
    //@param transversalVelocity - sideway velocity
    //@param angularVelocity - rotational velocity
    void getBaseVelocity(float& longitudinalVelocity, float& transversalVelocity, float& angularVelocity);

    //Sets the cartesian base velocity
    //@param longitudinalVelocity - forward/backward velocity
    //@param transversalVelocity - sideway velocity
    //@param angularVelocity - rotational velocity
    void setBaseVelocity(float longitudinalVelocity, float transversalVelocity, float angularVelocity);
  
  private:
    Wheel frontLeftWheel;
    Wheel frontRightWheel;
    Wheel rearLeftWheel;
    Wheel rearRightWheel;

    int32_t lastEncoderPositions[4];
    //float longitudinalPosition;
    //float transversalPosition;
    //float orientation;
    float prevLongitudinalPosition;
    float prevTransversalPosition;
    float prevAngularPosition;
};

#endif
