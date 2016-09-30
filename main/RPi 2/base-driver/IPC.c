//ICP FIFO
//User App <-> Driver

#include "IPC.h"

int IPCHandler(){
	//Fifo Incoming Data Buffer
    char incomingMsg[13] = {0};
    if(read(fd, &incomingMsg, sizeof(incomingMsg)) > 0){
    	IPCParser(incomingMsg);
    }

    if(errno != 0)
    	printf("IPCHandler failed to read FIFO and returned errno: %s \n", strerror(errno));

    return errno;
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
            float params[3] = {0.0f};
            memcpy(&params[0], &IPCMSG[1], sizeof(float));
            memcpy(&params[1], &IPCMSG[5], sizeof(float));
            memcpy(&params[2], &IPCMSG[9], sizeof(float));
            //Set Base Velocity
            mecanumBase.refLongitudinalVelocity = params[0];
            mecanumBase.refTransversalVelocity = params[1];
            mecanumBase.refAngularVelocity = params[2];
            break;
        }
        case CMD_BASE_SET_CTRL_MODE:{
        	printf("Set Control Mode \n");
            uint8_t params = 0;
            memcpy(&params, &IPCMSG[1], sizeof(uint8_t));
            base_setControlMode(&mecanumBase, params);
            break;
        }
        case CMD_BASE_SET_VELOCITY_PID:{
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
            char tx[12] = {0};
            memcpy(&tx[0], &mecanumBase.controlMode, sizeof(mecanumBase.controlMode));
            if(write(fd2, &tx, sizeof(tx)) < 0)
                printf("Err \n");
            
            break;
        }
        case CMD_BASE_GET_VELOCITY:{
        	printf("Get Velocity\n");       
            char tx[12] = {0};
            float params[] = {mecanumBase.longitudinalVelocity, mecanumBase.transversalVelocity, mecanumBase.angularVelocity};
            memcpy(&tx[0], &params[0], sizeof(float));
            memcpy(&tx[4], &params[1], sizeof(float));
            memcpy(&tx[8], &params[2], sizeof(float));

            if(write(fd2, &tx, sizeof(tx)) < 0)
                printf("Err \n");

            break;
        }
        case CMD_BASE_GET_REF_VELOCITY:{  
        	printf("Get Ref Velocity \n");          
            char tx[12] = {0};
            float params[] = {mecanumBase.refLongitudinalVelocity, mecanumBase.refTransversalVelocity, mecanumBase.refAngularVelocity};
            memcpy(&tx[0], &params[0], sizeof(float));
            memcpy(&tx[4], &params[1], sizeof(float));
            memcpy(&tx[8], &params[2], sizeof(float));

            if(write(fd2, &tx, sizeof(tx)) < 0)
                printf("Err \n");

            break;
        }
        case CMD_BASE_GET_POSITION:{
        	printf("Get Position \n");
            char tx[12] = {0};
            float params[] = {mecanumBase.longitudinalPosition, mecanumBase.transversalPosition, mecanumBase.orientation};
            memcpy(&tx[0], &params[0], sizeof(float));
            memcpy(&tx[4], &params[1], sizeof(float));
            memcpy(&tx[8], &params[2], sizeof(float));

            if(write(fd2, &tx, sizeof(tx)) < 0)
                printf("Err \n");

            break;
        }
        case CMD_BASE_GET_VELOCITY_PID:{
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