/* Exported variables ------------------------------------------------------- */

//Motor Velocity Control Variables
extern float velocityContPeriod;
extern float Imax, Imin;

extern float wheel1_Kp, wheel1_Ki;
extern float wheel2_Kp, wheel2_Ki;
extern float wheel3_Kp, wheel3_Ki;
extern float wheel4_Kp, wheel4_Ki;

extern volatile float wheel1_refRPM, wheel2_refRPM , wheel3_refRPM, wheel4_refRPM;


//Base Position Control Variables
extern float positionContPeriod;

extern float KLp, KLd, KLi;
extern float KTp, KTd, KTi;
extern float KOp, KOd, KOi;

extern volatile float refLongitudinalPosition, refTransversalPosition, refOrientation;

extern volatile float longitudinalPosition, transversalPosition, orientation;

/* Exported functions ------------------------------------------------------- */
void calcVelocityPID(void);
void calcPositionPID(void);