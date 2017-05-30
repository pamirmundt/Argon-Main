/* Exported variables ------------------------------------------------------- */

extern float contPeriod;
extern float Imax, Imin;
extern float wheel1_Kp, wheel1_Ki, wheel2_Kp, wheel2_Ki, wheel3_Kp, wheel3_Ki, wheel4_Kp, wheel4_Ki;
extern float wheel1_refRPM, wheel2_refRPM , wheel3_refRPM, wheel4_refRPM;

extern float wheel1_integral, wheel2_integral, wheel3_integral, wheel4_integral;

/* Exported functions ------------------------------------------------------- */
void calcPID(void);