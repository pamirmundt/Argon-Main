import math

encChannelRes = float(4.0)   						#4x Encoding - Channal A/B
encoderRes = float(334.0)							#Encoder Resolution 334 pulse/rotation
wheelRadius = float(0.03)   						#meter - r:30mm
gearRatio = float(13.552)							#1:13.552 Ratio
lengthBetweenFrontAndRearWheels = float(0.2289) 	#meter - a:228.9mm
lengthBetweenFrontWheels = float(0.214)  			#meter - b:214mm
geom_factor = float(lengthBetweenFrontAndRearWheels + lengthBetweenFrontWheels / 2.0)



def cartesianVelocityToWheelVelocities(longitudinalVelocity, transversalVelocity, angularVelocity):
	#Base Velocity to wheel velocities
	W0_angVel = float(longitudinalVelocity + transversalVelocity + geom_factor * angularVelocity)
	W1_angVel = float(longitudinalVelocity - transversalVelocity - geom_factor * angularVelocity)
	W2_angVel = float(longitudinalVelocity - transversalVelocity + geom_factor * angularVelocity)
	W3_angVel = float(longitudinalVelocity + transversalVelocity - geom_factor * angularVelocity)

	#Angular velocity to RPM
	W0_RPM = float(W0_angVel * 60.0 / (2.0 * math.pi * wheelRadius))
	W1_RPM = float(W1_angVel * 60.0 / (2.0 * math.pi * wheelRadius))
	W2_RPM = float(W2_angVel * 60.0 / (2.0 * math.pi * wheelRadius))
	W3_RPM = float(W3_angVel * 60.0 / (2.0 * math.pi * wheelRadius))


	#Create Return (Tuple) Object
	pyReturnArgs = (W0_RPM, W1_RPM, W2_RPM,W3_RPM)
	return pyReturnArgs


def wheelVelocitiesToCartesianVelocity(W0_RPM, W1_RPM, W2_RPM, W3_RPM):	
	#RPM to rad/s
	W0_angVel = float((W0_RPM / gearRatio) * 2.0 * math.pi / 60.0)
	W1_angVel = float((W1_RPM / gearRatio) * 2.0 * math.pi / 60.0)
	W2_angVel = float((W2_RPM / gearRatio) * 2.0 * math.pi / 60.0)
	W3_angVel = float((W3_RPM / gearRatio) * 2.0 * math.pi / 60.0)

	longitudinalVelocity = float((W0_angVel + W1_angVel + W2_angVel + W3_angVel) * wheelRadius / 4.0)
	transversalVelocity = float((W0_angVel - W1_angVel - W2_angVel + W3_angVel) * wheelRadius / 4.0)
	angularVelocity = float((W0_angVel - W1_angVel + W2_angVel - W3_angVel) * wheelRadius / (4.0 * geom_factor))

	#Create Return (Tuple) Object
	pyReturnArgs = (longitudinalVelocity, transversalVelocity, angularVelocity)
	return pyReturnArgs


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



	#Create Return (Tuple) Object
	pyReturnArgs = (lastEncPos0, lastEncPos1, lastEncPos2, lastEncPos3, longitudinalPosition, transversalPosition, orientation)
	return pyReturnArgs


def calcJacobianT(baseLongitudinalForce, baseTransversalForce, baseOrientationForce):
	#Calculate Wheel Torques from Base Force with Jacobian Transpose
	W0Torque = (baseLongitudinalForce + baseTransversalForce + baseOrientationForce * (1.0 / geom_factor)) * (wheelRadius / 4.0)
	W1Torque = (baseLongitudinalForce - baseTransversalForce - baseOrientationForce * (1.0 / geom_factor)) * (wheelRadius / 4.0)
	W2Torque = (baseLongitudinalForce - baseTransversalForce + baseOrientationForce * (1.0 / geom_factor)) * (wheelRadius / 4.0)
	W3Torque = (baseLongitudinalForce + baseTransversalForce - baseOrientationForce * (1.0 / geom_factor)) * (wheelRadius / 4.0)



	#Create Return (Tuple) Object
	pyReturnArgs = (W0Torque, W1Torque, W2Torque, W3Torque)
	return pyReturnArgs