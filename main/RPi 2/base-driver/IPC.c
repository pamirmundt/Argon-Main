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

int IPCParser(char* IPCMSG){
    //First byte is Command
    //Parse first byte
    uint8_t CMD;
    memcpy(&CMD, &IPCMSG[0], sizeof(CMD));

    printf("%d \n", CMD);
    
    switch (CMD){
        //IPC Set functions
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

        	switch(wheelNumber){
        		//Front Left Wheel
        		case 0:{
        			motor_setPID(mecanumBase.frontLeftWheel, PIDparams[0], PIDparams[1], PIDparams[2]);
        			break;
        		}
        		//Front Right Wheel
        		case 1:{
        			motor_setPID(mecanumBase.frontRightWheel, PIDparams[0], PIDparams[1], PIDparams[2]);
        			break;
        		}
        		//Rear Left Wheel
        		case 2:{
        			motor_setPID(mecanumBase.rearLeftWheel, PIDparams[0], PIDparams[1], PIDparams[2]);
        			break;
        		}
        		//Rear Right Wheel
        		case 3:{
        			motor_setPID(mecanumBase.rearRightWheel, PIDparams[0], PIDparams[1], PIDparams[2]);
        			break;
        		}
        	}

            break;
        }
        case CMD_BASE_SET_POSITION_PID:{
            break;
        }
        case CMD_BASE_SET_GOAL:{
            break;
        }

        //IPC Get Functions
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
		    //Send Message
		    switch (wheelNumber){
		    	case 0:{
		    		float PIDparams[3];
		    		motor_getPID(mecanumBase.frontLeftWheel, &PIDparams[0], &PIDparams[1], &PIDparams[2]);

		    		uint8_t txMsgSize = 12;
        			char tx[txMsgSize];
        			memcpy(&tx[0], &PIDparams[0], sizeof(float));
        			memcpy(&tx[4], &PIDparams[1], sizeof(float));
        			memcpy(&tx[8], &PIDparams[2], sizeof(float));

                	fifo_write(tx, sizeof(tx));
		    		break;
		    	}
		    	case 1:{
		    		float PIDparams[3];
		    		motor_getPID(mecanumBase.frontRightWheel, &PIDparams[0], &PIDparams[1], &PIDparams[2]);

		    		uint8_t txMsgSize = 12;
        			char tx[txMsgSize];
        			memcpy(&tx[0], &PIDparams[0], sizeof(float));
        			memcpy(&tx[4], &PIDparams[1], sizeof(float));
        			memcpy(&tx[8], &PIDparams[2], sizeof(float));

                	fifo_write(tx, sizeof(tx));
		    		break;
		    	}
		    	case 2:{
		    		float PIDparams[3];
		    		motor_getPID(mecanumBase.rearLeftWheel, &PIDparams[0], &PIDparams[1], &PIDparams[2]);

		    		uint8_t txMsgSize = 12;
        			char tx[txMsgSize];
        			memcpy(&tx[0], &PIDparams[0], sizeof(float));
        			memcpy(&tx[4], &PIDparams[1], sizeof(float));
        			memcpy(&tx[8], &PIDparams[2], sizeof(float));

                	fifo_write(tx, sizeof(tx));
		    		break;
		    	}
		    	case 3:{
		    		float PIDparams[3];
		    		motor_getPID(mecanumBase.rearRightWheel, &PIDparams[0], &PIDparams[1], &PIDparams[2]);

		    		uint8_t txMsgSize = 12;
        			char tx[txMsgSize];
        			memcpy(&tx[0], &PIDparams[0], sizeof(float));
        			memcpy(&tx[4], &PIDparams[1], sizeof(float));
        			memcpy(&tx[8], &PIDparams[2], sizeof(float));

                	fifo_write(tx, sizeof(tx));
		    		break;
		    	}
		    }
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