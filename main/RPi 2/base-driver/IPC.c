//ICP FIFO
//User App <-> Driver

#include "IPC.h"

int fifo_read(char rx[], uint8_t size){
    if(read(fd, rx, size) != size){
        printf("Failed to read Fifo: /tmp/mecanumBaseDriverOutput \n");
        printf("Read failed and returned errno: %s \n", strerror(errno));
        printf("Make sure base-driver is working! \n");
        return -1;
    }
    return 0;
}

int fifo_write(char tx[], uint8_t size){
    if(write(fd2, tx, size) < 0){
        printf("Failed to write Fifo: /tmp/mecanumBaseDriverInput \n");
        printf("Write failed and returned errno: %s \n", strerror(errno));
        printf("Make sure base-driver is working! \n");
        return -1;
    }
    return 0;
}

int IPCHandler(){
	//Fifo Incoming Data Buffer
	uint8_t rxMsgSize = 1;
    char incomingMsg[rxMsgSize];

    if(fifo_read(incomingMsg, rxMsgSize) == 0){
    	IPCParser(incomingMsg);
    	return 0;
    }
    return -1;
}

Motor motorSelect(uint8_t wheelNumber){
	Motor motor;
	switch(wheelNumber){
		//Front Left Wheel
		case 0:{
			motor = mecanumBase.frontLeftWheel;
			break;
		}
		//Front Right Wheel
		case 1:{
			motor = mecanumBase.frontRightWheel;
			break;
		}
		//Rear Left Wheel
		case 2:{
			motor = mecanumBase.rearLeftWheel;
			break;
		}
		//Rear Rght Wheel
		case 3:{
			motor = mecanumBase.rearRightWheel;
			break;
		}
	}
	return motor;
}

int IPCParser(char* IPCMSG){
    //First byte is Command
    //Parse first byte
    uint8_t CMD;
    memcpy(&CMD, &IPCMSG[0], sizeof(CMD));

    printf("%d \n", CMD);
    
    switch (CMD){
    	//IPC Motor Set CMDs
    	case CMD_MOTOR_RESET:{
    		printf("Motor Reset \n");

    		uint8_t rxMsgSize = 1;
    		char rx[rxMsgSize];
    		fifo_read(rx, rxMsgSize);

    		uint8_t wheelNumber;
    		memcpy(&wheelNumber, &rx[0], sizeof(wheelNumber));

    		motor_reset(motorSelect(wheelNumber));
    		break;
    	}     
		case CMD_MOTOR_SET_RPM:{
			printf("Motor Set RPM\n");

    		uint8_t rxMsgSize = 5;
    		char rx[rxMsgSize];
    		fifo_read(rx, rxMsgSize);

    		uint8_t wheelNumber;
    		float RPM;
    		memcpy(&wheelNumber, &rx[0], sizeof(wheelNumber));
    		memcpy(&RPM, &rx[1], sizeof(RPM));

    		motor_setRPM(motorSelect(wheelNumber), RPM);
			break;
		}
 		case CMD_MOTOR_SET_VELOCITY_PID:{
 			printf("Set Motor Velocity PID \n");
        	
        	uint8_t rxMsgSize = 13;
        	char rx[rxMsgSize];
        	fifo_read(rx, rxMsgSize);

        	uint8_t wheelNumber;
        	float PIDparams[3];
        	memcpy(&wheelNumber, &rx[0], sizeof(wheelNumber));
        	memcpy(&PIDparams[0], &rx[1], sizeof(float));
        	memcpy(&PIDparams[1], &rx[5], sizeof(float));
        	memcpy(&PIDparams[2], &rx[9], sizeof(float));

        	motor_setPID(motorSelect(wheelNumber), PIDparams[0], PIDparams[1], PIDparams[2]);
        	break;
 		}
 		case CMD_MOTOR_SET_POWER:{
 			printf("Motor Set Power \n");

 			uint8_t rxMsgSize = 5;
 			char rx[rxMsgSize];
 			fifo_read(rx, rxMsgSize);

 			uint8_t wheelNumber;
 			float power;
 			memcpy(&wheelNumber, &rx[0], sizeof(wheelNumber));
 			memcpy(&power, &rx[1], sizeof(power));

 			motor_setPower(motorSelect(wheelNumber), power);
 			break;
 		}
		case CMD_MOTOR_SET_PWM:{
			printf("Motor Set Power\n");

 			uint8_t rxMsgSize = 5;
 			char rx[rxMsgSize];
 			fifo_read(rx, rxMsgSize);

 			uint8_t wheelNumber;
 			float PWM;
 			memcpy(&wheelNumber, &rx[0], sizeof(wheelNumber));
 			memcpy(&PWM, &rx[1], sizeof(PWM));

 			motor_setPWM(motorSelect(wheelNumber), PWM);
 			break;
		}
		case CMD_MOTOR_SET_MODE:{
			printf("Motor Set Mode\n");

			uint8_t rxMsgSize = 2;
			char rx[rxMsgSize];
			fifo_read(rx, rxMsgSize);

			uint8_t wheelNumber;
			uint8_t mode;
			memcpy(&wheelNumber, &rx[0], sizeof(wheelNumber));
			memcpy(&mode, &rx[1], sizeof(mode));

			motor_setMode(motorSelect(wheelNumber), mode);
			break;
		}
		//IPC Motor Get Functions
 		case CMD_MOTOR_GET_VELOCITY_PID:{
 			printf("Motor Get PID \n");
			
			uint8_t rxMsgSize = 1;
		    char rx[rxMsgSize];
		    fifo_read(rx, rxMsgSize);
		    uint8_t wheelNumber;
		    memcpy(&wheelNumber, &rx[0], sizeof(wheelNumber));
		    
		    float PIDparams[3];
    		motor_getPID(motorSelect(wheelNumber), &PIDparams[0], &PIDparams[1], &PIDparams[2]);
		    //Send Message
		    uint8_t txMsgSize = 12;
			char tx[txMsgSize];
			memcpy(&tx[0], &PIDparams[0], sizeof(float));
			memcpy(&tx[4], &PIDparams[1], sizeof(float));
			memcpy(&tx[8], &PIDparams[2], sizeof(float));

        	fifo_write(tx, sizeof(tx));
 		}

		case CMD_MOTOR_GET_POS:{
			printf("Motor Get Position \n");

			uint8_t rxMsgSize = 1;
			char rx[rxMsgSize];
			fifo_read(rx, rxMsgSize);

			uint8_t wheelNumber;
			memcpy(&wheelNumber, &rx[0], sizeof(wheelNumber));

			int32_t pos = motor_getPos(motorSelect(wheelNumber));

			//Send Message
			uint8_t txMsgSize = 4;
			char tx[txMsgSize];
			memcpy(&tx[0], &pos, sizeof(pos));

        	fifo_write(tx, sizeof(tx));
			break;
		}
		case CMD_MOTOR_GET_RPM:{
			printf("Motor Get RPM \n");

			uint8_t rxMsgSize = 1;
			char rx[rxMsgSize];
			fifo_read(rx, rxMsgSize);

			uint8_t wheelNumber;
			memcpy(&wheelNumber, &rx[0], sizeof(wheelNumber));

			float rpm = motor_getSpeed(motorSelect(wheelNumber));

			//Send Message
			uint8_t txMsgSize = 4;
			char tx[txMsgSize];
			memcpy(&tx[0], &rpm, sizeof(rpm));

        	fifo_write(tx, sizeof(tx));
			break;
		}
		case CMD_MOTOR_GET_PWM:{
			printf("Motor Get PWM \n");

			uint8_t rxMsgSize = 1;
			char rx[rxMsgSize];
			fifo_read(rx, rxMsgSize);

			uint8_t wheelNumber;
			memcpy(&wheelNumber, &rx[0], sizeof(wheelNumber));

			float PWM = motor_getPWM(motorSelect(wheelNumber));

			//Send Message
			uint8_t txMsgSize = 4;
			char tx[txMsgSize];
			memcpy(&tx[0], &PWM, sizeof(PWM));

        	fifo_write(tx, sizeof(tx));
			break;
		}
		case CMD_MOTOR_GET_REF_RPM:{
			printf("Motor Get Ref RPM \n");

			uint8_t rxMsgSize = 1;
			char rx[rxMsgSize];
			fifo_read(rx, rxMsgSize);

			uint8_t wheelNumber;
			memcpy(&wheelNumber, &rx[0], sizeof(wheelNumber));

			float refRPM = motor_getRefSpeed(motorSelect(wheelNumber));

			//Send Message
			uint8_t txMsgSize = 4;
			char tx[txMsgSize];
			memcpy(&tx[0], &refRPM, sizeof(refRPM));

        	fifo_write(tx, sizeof(tx));
			break;
		}

		case CMD_MOTOR_GET_I2C_ADDR:{
			printf("Motor Get I2C Addr \n");

			uint8_t rxMsgSize = 1;
			char rx[rxMsgSize];
			fifo_read(rx, rxMsgSize);

			uint8_t wheelNumber;
			memcpy(&wheelNumber, &rx[0], sizeof(wheelNumber));

			uint16_t i2cAddr = motor_getAddr(motorSelect(wheelNumber));

			//Send Message
			uint8_t txMsgSize = 2;
			char tx[txMsgSize];
			memcpy(&tx[0], &i2cAddr, sizeof(i2cAddr));

        	fifo_write(tx, sizeof(tx));
			break;
		}
        //IPC Base Set functions
        case CMD_BASE_RESET:{
        	printf("Reset Base \n");
        	base_reset(&mecanumBase);
        	mecanumBase.longitudinalPosition = 0.0f;
        	mecanumBase.transversalPosition = 0.0f;
        	mecanumBase.orientation = 0.0f;
            break;
        }
        case CMD_BASE_SET_VELOCITY:{
            printf("Set Velocity \n");

            uint8_t rxMsgSize = 12;
            char rx[rxMsgSize];
            fifo_read(rx, rxMsgSize);

            float params[3];
            memcpy(&params[0], &rx[0], sizeof(float));
            memcpy(&params[1], &rx[4], sizeof(float));
            memcpy(&params[2], &rx[8], sizeof(float));
            //Set Base Velocity

            mecanumBase.refLongitudinalVelocity = params[0];
            mecanumBase.refTransversalVelocity = params[1];
            mecanumBase.refAngularVelocity = params[2];
            break;
        }
        case CMD_BASE_SET_CTRL_MODE:{
        	printf("Set Control Mode \n");

        	uint8_t rxMsgSize = 1;
        	char rx[rxMsgSize];
        	fifo_read(rx, rxMsgSize);

            uint8_t params;
            memcpy(&params, &rx[0], sizeof(uint8_t));
            base_setControlMode(&mecanumBase, params);
            break;
        }
        case CMD_BASE_SET_VELOCITY_PID:{
        	printf("Set Velocity PID \n");
        	
        	uint8_t rxMsgSize = 13;
        	char rx[rxMsgSize];
        	fifo_read(rx, rxMsgSize);

        	uint8_t wheelNumber;
        	float PIDparams[3];
        	memcpy(&wheelNumber, &rx[0], sizeof(wheelNumber));
        	memcpy(&PIDparams[0], &rx[1], sizeof(float));
        	memcpy(&PIDparams[1], &rx[5], sizeof(float));
        	memcpy(&PIDparams[2], &rx[9], sizeof(float));

        	motor_setPID(motorSelect(wheelNumber), PIDparams[0], PIDparams[1], PIDparams[2]);
            break;
        }
        case CMD_BASE_SET_POSITION_PID:{
            break;
        }
        case CMD_BASE_SET_GOAL:{
            break;
        }

        //IPC Base Get Functions
        case CMD_BASE_GET_CTRL_MODE:{
        	printf("Get Control Mode \n");
        	uint8_t txMsgSize = 1;
            char tx[txMsgSize];
            memcpy(&tx[0], &mecanumBase.controlMode, sizeof(mecanumBase.controlMode));

            fifo_write(tx, sizeof(tx));
            break;
        }
        case CMD_BASE_GET_VELOCITY:{
        	printf("Get Velocity\n");
        	uint8_t txMsgSize = 12;
            char tx[txMsgSize];
            float params[] = {mecanumBase.longitudinalVelocity, mecanumBase.transversalVelocity, mecanumBase.angularVelocity};
            memcpy(&tx[0], &params[0], sizeof(float));
            memcpy(&tx[4], &params[1], sizeof(float));
            memcpy(&tx[8], &params[2], sizeof(float));

            fifo_write(tx, sizeof(tx));
            break;
        }
        case CMD_BASE_GET_REF_VELOCITY:{  
        	printf("Get Ref Velocity \n");
        	uint8_t txMsgSize = 12;
            char tx[txMsgSize];
            float params[] = {mecanumBase.refLongitudinalVelocity, mecanumBase.refTransversalVelocity, mecanumBase.refAngularVelocity};
            memcpy(&tx[0], &params[0], sizeof(float));
            memcpy(&tx[4], &params[1], sizeof(float));
            memcpy(&tx[8], &params[2], sizeof(float));

            fifo_write(tx, sizeof(tx));
            break;
        }
        case CMD_BASE_GET_POSITION:{
        	printf("Get Position \n");
        	uint8_t txMsgSize = 12;
            char tx[txMsgSize];
            float params[] = {mecanumBase.longitudinalPosition, mecanumBase.transversalPosition, mecanumBase.orientation};
            memcpy(&tx[0], &params[0], sizeof(float));
            memcpy(&tx[4], &params[1], sizeof(float));
            memcpy(&tx[8], &params[2], sizeof(float));

            fifo_write(tx, sizeof(tx));
            break;
        }
        case CMD_BASE_GET_VELOCITY_PID:{
        	printf("Get Velocity PID \n");
        	//Read Message (Wheel Number)
        	uint8_t rxMsgSize = 1;
		    char rx[rxMsgSize];
		    fifo_read(rx, rxMsgSize);
		    uint8_t wheelNumber;
		    memcpy(&wheelNumber, &rx[0], sizeof(wheelNumber));
		    
		    float PIDparams[3];
    		motor_getPID(motorSelect(wheelNumber), &PIDparams[0], &PIDparams[1], &PIDparams[2]);
		    //Send Message
		    uint8_t txMsgSize = 12;
			char tx[txMsgSize];
			memcpy(&tx[0], &PIDparams[0], sizeof(float));
			memcpy(&tx[4], &PIDparams[1], sizeof(float));
			memcpy(&tx[8], &PIDparams[2], sizeof(float));

        	fifo_write(tx, sizeof(tx));
            break;
        }
        case CMD_BASE_GET_POSITION_PID:{
            break;
        }
        case CMD_BASE_GET_GOAL:{
            break;
        }

    }

    return 0;
}