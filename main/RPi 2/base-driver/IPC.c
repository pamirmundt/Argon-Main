//ICP FIFO
//User App <-> Driver

#include "IPC.h"

int IPCHandler(){
	//Fifo Incoming Data Buffer
    char incomingMsg[13] = {0};
    if(read(fd, &incomingMsg, sizeof(incomingMsg)) > 0){
    	IPCParser(incomingMsg);
    }

    return 0;
}

int IPCParser(char* IPCMSG){
    //First byte is Command
    //Parse first byte
    uint8_t CMD;
    memcpy(&CMD, &IPCMSG[0], sizeof(CMD));

    printf("%d \n", CMD);

    switch (CMD){
        //IPC Set functions
        case base_reset:{
            break;
        }
        case base_set_velocity:{
            printf("Set Velocity \n");
            float params[3] = {0.0f};
            memcpy(&params[0], &IPCMSG[1], sizeof(float));
            memcpy(&params[1], &IPCMSG[5], sizeof(float));
            memcpy(&params[2], &IPCMSG[9], sizeof(float));
            //Set Base Velocity
            printf("%f %f %f \n", params[0], params[1], params[2]);
            mecanumBase.refLongitudinalVelocity = params[0];
            mecanumBase.refTransversalVelocity = params[1];
            mecanumBase.refAngularVelocity = params[2];
            break;
        }
        case base_set_ctrl_mode:{
            uint8_t params = 0;
            memcpy(&params, &IPCMSG[1], sizeof(uint8_t));
            base_setControlMode(&mecanumBase, params);
            break;
        }
        case base_set_velocity_PID:{
            break;
        }
        case base_set_position_PID:{
            break;
        }
        case base_set_goal:{
            break;
        }

        //IPC Get Functions
        case base_get_ctrl_mode:{
            char tx[12] = {0};

            memcpy(&tx[0], &mecanumBase.controlMode, sizeof(mecanumBase.controlMode));
            //if(write(fd2, &tx, sizeof(tx)) < 0)
            //    printf("Err \n");
            
            break;
        }
        case base_get_velocity:{            
            char tx[12] = {0};

            memcpy(&tx[0], &mecanumBase.longitudinalVelocity, sizeof(mecanumBase.longitudinalVelocity));
            memcpy(&tx[4], &mecanumBase.transversalVelocity, sizeof(mecanumBase.transversalVelocity));
            memcpy(&tx[8], &mecanumBase.angularVelocity, sizeof(mecanumBase.angularVelocity));

            //if(write(fd2, &tx, sizeof(tx)) < 0)
            //    printf("Err \n");

            break;
        }
        case base_get_position:{
            char tx[12] = {0};

            memcpy(&tx[0], &mecanumBase.longitudinalPosition, sizeof(mecanumBase.longitudinalPosition));
            memcpy(&tx[4], &mecanumBase.transversalPosition, sizeof(mecanumBase.transversalPosition));
            memcpy(&tx[8], &mecanumBase.orientation, sizeof(mecanumBase.orientation));

            //if(write(fd2, &tx, sizeof(tx)) < 0)
            //    printf("Err \n");

            break;
        }
        case base_get_velocity_PID:{
            break;
        }
        case base_get_position_PID:{
            break;
        }
        case base_get_goal:{
            break;
        }

    }

    return 0;
}