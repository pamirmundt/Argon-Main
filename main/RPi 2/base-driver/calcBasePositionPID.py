#Base Position PID Constants
#Longitude position control constants
KLp = float(13.5); KLi = float(0.0); KLd = float(0.01);
#Transversal position control constants
KTp = float(13.5); KTi = float(0.0); KTd = float(0.01);
#Angular(orientation) position control constants
KOp = float(0.3); KOi = float(0.0); KOd = float(0.0001);


def calcBasePositionPID(contPeriod, refLongitudinalPosition, refTransversalPosition, refOrientation, longitudinalPosition, transversalPosition, orientation, errPrevLong, errPrevTrans, errPrevOrien):
    #, KLp, KLi, KLd, KTp, KTi, KTd, KOp, KOi, KOd

    #Base Longitudinal Position PD
    errLong = refLongitudinalPosition - longitudinalPosition
    #integralLong = integralLong + errLong * contPeriod
    derivativeLong = (errLong - errPrevLong) / contPeriod
    controlLong = KLp * errLong + KLd * derivativeLong #+ Ki * integralLong
    errPrevLong = errLong
    
    
    #Base Transversal Position PD
    errTrans = refTransversalPosition - transversalPosition;
    #mecanumBase.integralTrans = mecanumBase.integralTrans + mecanumBase.errTrans * contPeriod
    derivativeTrans = (errTrans - errPrevTrans) / contPeriod
    controlTrans = KTp * errPrevTrans + KTd * derivativeTrans #+ Ki * integralTrans
    errPrevTrans = errTrans
    
    #Base Orientation PD
    errOrien = refOrientation - orientation
    #mecanumBase.integralOrien = mecanumBase.integralOrien + mecanumBase.errOrien * contPeriod
    derivativeOrien = (errOrien - errPrevOrien) / contPeriod
    controlOrien = KOp * errOrien + KOd * derivativeOrien #+ mecanumBase.KOi * mecanumBase.integralOrien
    errPrevOrien = errOrien

    #Create Tuple Object
    pyReturnArgs = (errPrevLong, errPrevTrans, errPrevOrien, controlLong, controlTrans, controlOrien)
    return pyReturnArgs