#ifndef MECANUMBASEAPI_H
#define MECANUMBASEAPI_H

#include <stdint.h>

//  Initialize Mecanum Base API
//@Desc: Check if mecanum-base-driver is working, check for FIFO files
//@param NONE
//@retVal: NONE - Exits on error
void mecanumBaseAPI();

//  Finalize Mecanum Base API
//@Desc: Close  FIFO files - Call this function at the end of main.c
//@param NONE
//@retVal: NONE - Exits on error
void mecanumBaseAPI_Finalize();

//***************************************
//  IPC Motor Set Functions
//***************************************

//  Motor Reset
//@Desc: Resets a motor, clears all encoder data and other stored values.
//      Sets control mode to velocity mode (default).
//@param: Motor Number
//          Front Left Wheel:   0
//          Front Right Wheel:  1
//          Rear Left Wheel:    2
//          Rear Right Wheel:   3
//@retval: Returns 0 on success, returns -1 and prints error on failure
int motor_reset(uint8_t motorNumber);

//  Motor Set RPM
//@Desc: Sets a wheel's speed to desired RPM with "motor velocity control with PID"
//@param: Motor Number
//@param: Desired Wheel RPM
//@retval: Returns 0 on success, returns -1 and prints error on failure
int motor_set_RPM(uint8_t motorNumber, float RPM);

//  Motor Set Velocity PID
//@Desc: Sets a wheel's velocity PID constants (Kp, Ki, Kd)
//@param: Motor Number
//@param: Proportional Coefficient - Kp
//@param: Integral Coefficient - Ki
//@param: Derivarive Coefficient - Kd
//@retval: Returns 0 on success, returns -1 and prints error on failure
int motor_set_velocity_pid(uint8_t wheelNumber, float Kp, float Ki, float Kd);

//  Motor Set Power
//@Desc: Sets a wheel's power as percentage (min:-100% max:+100%)
//@param: Motor Number
//@param:   Power min: -100% (reverse)
//          Power max: +100% (forward)
//@retval: Returns 0 on success, returns -1 and prints error on failure
int motor_set_power(uint8_t wheelNumber, float power);

//  Motor Set PWM
//@Desc: Sets a wheel's PWM
//          PWM Resolution: 2047 (2^11)
//@param: Motor Number
//@param: PWM (min: -2047, max: +2047)
//@retval: Returns 0 on success, returns -1 and prints error on failure
int motor_set_pwm(uint8_t wheelNumber, float PWM);

//  Motor Set Control Mode
//@Desc: There are 2 different control modes for wheel class
//          Manuel Mode (0): Velocity Control mode is DISABLED and PWM, POWER and etc. can be entered manually.
//          Velocity Mode (1): Velocity control mode is ENABLED with default PID coefficients. PWM, POWER and etc. cannot be entered manually.
//@param: Motor Number
//@param: Mode (Manual Mode: 0, Velocity PID Control: 1)
//@retval: Returns 0 on success, returns -1 and prints error on failure
int motor_set_ctrl_mode(uint8_t wheelNumber, uint8_t mode);

//***************************************
//  IPC Motor Get Functions
//***************************************
//  Motor Get Velocity PID Coefficients
//@Desc: Returns a motor's velocity PID Coefficient
//@param: Motor Number
//@param: Proportional Coefficient (Pass by reference)
//@param: Integral Coefficient (Pass by reference)
//@param: Derivative Coefficient (Pass by reference)
//@retval: Returns 0 on success, returns -1 and prints error on failure
float motor_get_velocity_pid(uint8_t wheelNumber, float * Kp, float * Ki, float * Kd);

//  Motor Get Encoder Count/Position
//@Desc: Returns a motor's encoder position
//@param: Motor Number
//@retval: (int32_t) Returns encoder count on success, returns -1 and prints error on failure
int32_t motor_get_pos(uint8_t wheelNumber);

//  Motor Get RPM
//@Desc: Returns a motor's RPM
//@param: Motor Number
//@retval: (float) Returns motor RPM on success, returns NAN and prints error on failure
float motor_get_RPM(uint8_t wheelNumber);

//  Motor Get Reference RPM
//@Desc: Returns a motor's RPM
//@param: Motor Number
//@retval: (float) Returns motor RPM on success, returns NAN and prints error on failure
float motor_get_ref_RPM(uint8_t wheelNumber);

//  Motor Get I2C Address
//@Desc: Returns a motor's I2C Address (7-bit adressing)
//@param: Motor Number
//@retval: (int16_t) Returns motor I2C address on success, returns -1 and prints error on failure
int motor_get_i2c_addr(uint8_t wheelNumber);

//***************************************
//  IPC Base Set Functions
//***************************************
//  Base Reset
//@Desc: Resets a base, clears all encoder data and other stored values (base position, speed and etc.)
//      Sets control mode to base velocity mode (default).
//@param: NONE
//@retval: Returns 0 on success, returns -1 and prints error on failure
int base_reset();

//  Base Set Velocity
//@Desc: Sets base velocity (mode: (2)PID base velocity mode)
//@param: Longitudinal Velocity (m/sec)
//@param: Transversal Velocity (m/sec)
//@param: Angular Velocity (rad/sec)
//@retval: Returns 0 on success, returns -1 and prints error on failure
int base_set_velocity(float longitudinalVelocity, float transversalVelocity, float angularVelocity);

//  Base Set Control Mode
//@Desc: Sets base control mode/type
//          (0) Manuel Mode: Control each motor's PWM and power individually
//          (1) Wheel Velocity Control Mode: Control each motor's velocity with PID control
//          (2) Base Velocity Control Mode: Control base velocity with PID control
//          (3) Base Position Control Mode: Control base position with PID control
//@param: Control Mode (0-3)
//@retval: Returns 0 on success, returns -1 and prints error on failure
int base_set_ctrl_mode(uint8_t mode);


//  Base Motor Set Velocity PID Coefficients
//@Desc: Returns a motor's velocity PID Coefficient
//@param: Motor Number
//@param: Proportional Coefficient (Pass by reference)
//@param: Integral Coefficient (Pass by reference)
//@param: Derivative Coefficient (Pass by reference)
//@retval: Returns 0 on success, returns -1 and prints error on failure
int base_set_velocity_PID(uint8_t wheelNumber, float Kp, float Ki, float Kd);

//***************************************
//  IPC Base Get Functions
//***************************************
//  Base Get Control Mode
//@Desc: Gets base control mode/type
//          (0) Manuel Mode: Control each motor's PWM and power individually
//          (1) Wheel Velocity Control Mode: Control each motor's velocity with PID control
//          (2) Base Velocity Control Mode: Control base velocity with PID control
//          (3) Base Position Control Mode: Control base position with PID control
//@param: NONE
//@retval: (uint8_t) Returns Control Mode on success, returns -1 and prints error on failure
uint8_t base_get_ctrl_mode();

//  Base Get Velocity
//@Desc: Gets base velocity
//@param: Longitudinal Velocity (m/sec) - pass by reference
//@param: Transversal Velocity (m/sec) - pass by reference
//@param: Angular Velocity (rad/sec) - pass by reference
//@retval: Returns 0 on success, returns -1 and prints error on failure
int base_get_velocity(float * longitudinalVelocity, float * transversalVelocity, float * angularVelocity);

//  Base Get Reference Velocity
//@Desc: Gets base velocity
//@param: Longitudinal Velocity (m/sec) - pass by reference
//@param: Transversal Velocity (m/sec) - pass by reference
//@param: Angular Velocity (rad/sec) - pass by reference
//@retval: Returns 0 on success, returns -1 and prints error on failure
int base_get_ref_velocity(float * refLongitudinalVelocity, float * refTransversalVelocity, float *refAngularVelocity);

//  Base Get Position
//@Desc: Gets base velocity
//@param: Longitudinal Position (meters) - pass by reference
//@param: Transversal Position (meters) - pass by reference
//@param: Angular Position (radians) - pass by reference
//@retval: Returns 0 on success, returns -1 and prints error on failure
int base_get_position(float * longitudinalPosition, float * transversalPosition, float * angularPosition);

//  Base Get Velocity PID coefficients
//@Desc: Sets base control mode/type
//@Desc: Returns a motor's velocity PID Coefficient
//@param: Motor Number
//@param: Proportional Coefficient (Pass by reference)
//@param: Integral Coefficient (Pass by reference)
//@param: Derivative Coefficient (Pass by reference)
//@retval: Returns 0 on success, returns -1 and prints error on failure
int base_get_velocity_PID(uint8_t wheelNumber, float * Kp, float * Ki, float * Kd);

#endif
