import math

encChannelRes = 4.0   						#4x Encoding - Channal A/B
encoderRes = 334.0							#Encoder Resolution 334 pulse/rotation
wheelRadius = 0.03   						#meter - r:30mm
gearRatio = 13.552							#1:13.552 Ratio
lengthBetweenFrontAndRearWheels = 0.2289 	#meter - a:228.9mm
lengthBetweenFrontWheels = 0.214  			#meter - b:214mm
geom_factor = (lengthBetweenFrontAndRearWheels + lengthBetweenFrontWheels / 2.0)


def cartesianVelocityToWheelVelocities(a,b):
    print "Will compute", a, "times", b
    c = 0
    for i in range(0, a):
        c = c + b
    return c


def wheelVelocitiesToCartesianVelocity(a,b):
    print "Will compute", a, "times", b
    c = 0
    for i in range(0, a):
        c = c + b
    return c


def wheelPositionsToCartesianPosition(encPos0, encPos1, encPos2, encPos3, lastEncPos0, lastEncPos1, lastEncPos2, lastEncPos3, longitudinalPosition, transversalPosition, orientation):
	#Calculate Delta Encoder Position
	dPosW0 = float(encPos0 - lastEncPos0)
	dPosW1 = float(encPos1 - lastEncPos1)
	dPosW2 = float(encPos2 - lastEncPos2)
	dPosW3 = float(encPos3 - lastEncPos3)

	#Store Encoder Count for next iteration
	lastEncPos0 = encPos0
	lastEncPos1 = encPos1
	lastEncPos2 = encPos2
	lastEncPos3 = encPos3

	#Calculate Delta Longitudial Position in meters
	deltaLongitudinalPos = (dPosW0 + dPosW1 + dPosW2 + dPosW3) * 2.0 * math.pi * wheelRadius / 4.0 / encoderRes / gearRatio / encChannelRes
	#Calculate Delta Transversal Position in meters
	deltaTransversalPos = (dPosW0 - dPosW1 - dPosW2 + dPosW3) * 2.0 * math.pi * wheelRadius / 4.0 / encoderRes / gearRatio / encChannelRes

	#Calculate Base Position (x, y, theta)
	#Radians
	orientation += (dPosW0 - dPosW1 + dPosW2 - dPosW3) * 2.0 * math.pi * wheelRadius / geom_factor / encoderRes / gearRatio / encChannelRes

	#Meters
	longitudinalPosition += deltaLongitudinalPos * math.cos(orientation) - deltaTransversalPos * math.sin(orientation)

	#Meters
	transversalPosition += deltaLongitudinalPos * math.sin(orientation) + deltaTransversalPos * math.cos(orientation)



	#Create Tuple Object
	pyReturnArgs = (lastEncPos0, lastEncPos1, lastEncPos2, lastEncPos3, longitudinalPosition, transversalPosition, orientation)
	return pyReturnArgs


def calcJacobianT(baseLongitudinalForce, baseTransversalForce, baseOrientationForce):
	#Calculate Wheel Torques from Base Force with Jacobian Transpose
	W0Torque = (baseLongitudinalForce + baseTransversalForce + baseOrientationForce * (1.0 / geom_factor)) * (wheelRadius / 4.0)
	W1Torque = (baseLongitudinalForce - baseTransversalForce - baseOrientationForce * (1.0 / geom_factor)) * (wheelRadius / 4.0)
	W2Torque = (baseLongitudinalForce - baseTransversalForce + baseOrientationForce * (1.0 / geom_factor)) * (wheelRadius / 4.0)
	W3Torque = (baseLongitudinalForce + baseTransversalForce - baseOrientationForce * (1.0 / geom_factor)) * (wheelRadius / 4.0)



	#Create Tuple Object
	pyReturnArgs = (W0Torque, W1Torque, W2Torque, W3Torque)
	return pyReturnArgs