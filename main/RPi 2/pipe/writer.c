#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>

/*
void base_reset
base_set_goal
base_set_control_mode
base_set_position_PID
base_set_velocity_PID
base_set_velocity

base_get_goal
base_get_control_mode
base_get_position_PID
base_get_velocity_PID
base_get_velocity
base_get_position
*/

//Base Set CMDs
#define CMD_BASE_RESET              0x30
#define CMD_BASE_SET_VELOCITY       0x31
#define CMD_BASE_SET_CTRL_MODE      0x32
#define CMD_BASE_SET_VELOCITY_PID   0x33

//Base get CMDs
#define CMD_BASE_GET_CTRL_MODE      0x40
#define CMD_BASE_GET_VELOCITY       0x41
#define CMD_BASE_GET_REF_VELOCITY   0x42
#define CMD_BASE_GET_POSITION       0X43
#define CMD_BASE_GET_VELOCITY_PID   0x44


int fd, fd2;
char * inputFifo = "/tmp/mecanumBaseDriverInput";
char * outputFifo = "/tmp/mecanumBaseDriverOutput";



int fifo_write(char tx[], uint8_t size){
    if(write(fd, tx, size) < 0){
        printf("Failed to write Fifo: /tmp/mecanumBaseDriverInput \n");
        printf("Write failed and returned errno: %s \n", strerror(errno));
        printf("Make sure base-driver is working! \n");
        return -1;
    }
    return 0;
}

int fifo_read(char rx[], uint8_t size){
    if(read(fd2, rx, size) != size){
        printf("Failed to read Fifo: /tmp/mecanumBaseDriverOutput \n");
        printf("Read failed and returned errno: %s \n", strerror(errno));
        printf("Make sure base-driver is working! \n");
        return -1;
    }
    return 0;
}

int base_reset(){
    //Send CMD
    char cmd[1] = {CMD_BASE_RESET};
    return (fifo_write(cmd, 1));
}

int base_set_velocity(float longitudinalVelocity, float transversalVelocity, float angularVelocity){
    //Send CMD
    char cmd[1] = {CMD_BASE_SET_VELOCITY};
    if(fifo_write(cmd, 1) < 0)
        return -1;
    //Send message
    uint8_t msgSize = 12;
    char tx[msgSize];
    memcpy(&tx[0], &longitudinalVelocity, sizeof(longitudinalVelocity));
    memcpy(&tx[4], &transversalVelocity, sizeof(transversalVelocity));
    memcpy(&tx[8], &angularVelocity, sizeof(angularVelocity));
    return(fifo_write(tx, msgSize));
}

int base_set_ctrl_mode(uint8_t mode){
    //Send CMD
    char cmd[1] = {CMD_BASE_SET_CTRL_MODE};
    if(fifo_write(cmd, 1) < 0)
        return -1;
    //Send Message
    uint8_t msgSize = 1;
    char tx[msgSize];
    memcpy(&tx[0], &mode, sizeof(mode));
    return (fifo_write(tx, msgSize));
}

int base_set_velocity_PID(uint8_t wheelNumber, float Kp, float Ki, float Kd){
    //Send CMD
    char cmd[1] = {CMD_BASE_SET_VELOCITY_PID};
    if(fifo_write(cmd, 1) < 0)
        return -1;
    //Send Message
    uint8_t msgSize = 13;
    char tx[msgSize];

    memcpy(&tx[0], &wheelNumber, sizeof(wheelNumber));
    memcpy(&tx[1], &Kp, sizeof(Kp));
    memcpy(&tx[5], &Ki, sizeof(Ki));
    memcpy(&tx[9], &Kd, sizeof(Kd));

    return (fifo_write(tx, msgSize));
}

int base_get_ctrl_mode(uint8_t * mode){
    //Send CMD
    char cmd[1] = {CMD_BASE_GET_CTRL_MODE};
    if(fifo_write(cmd, 1) < 0)
        return -1;
    //Read Message
    uint8_t msgSize = 1;
    char rx[msgSize];
    if(fifo_read(rx, msgSize) < 0)
        return -1;
    memcpy(mode, &rx[0], sizeof(uint8_t));
    return 0;   
}

int base_get_velocity(float * longitudinalVelocity, float * transversalVelocity, float * angularVelocity){
    //Send CMD
    char cmd[1] = {CMD_BASE_GET_VELOCITY};
    if(fifo_write(cmd, 1) < 0)
        return -1;
    //Read message
    uint8_t msgSize = 12;
    char rx[msgSize];
    if(fifo_read(rx, msgSize) < 0)
        return -1;
    memcpy(longitudinalVelocity, &rx[0], sizeof(float));
    memcpy(transversalVelocity, &rx[4], sizeof(float));
    memcpy(angularVelocity, &rx[8], sizeof(float));
    return 0;
}

int base_get_ref_velocity(float * refLongitudinalVelocity, float * refTransversalVelocity, float *refAngularVelocity){
    //Send CMD
    char cmd[1] = {CMD_BASE_GET_REF_VELOCITY};
    if(fifo_write(cmd, 1) < 0)
        return -1;
    //Read Message
    uint8_t msgSize = 12;
    char rx[msgSize];
    if(fifo_read(rx, msgSize) < 0)
        return -1;
    memcpy(refLongitudinalVelocity, &rx[0], sizeof(float));
    memcpy(refTransversalVelocity, &rx[4], sizeof(float));
    memcpy(refAngularVelocity, &rx[8], sizeof(float));
    return 0;
}


int base_get_position(float * longitudinalPosition, float * transversalPosition, float * angularPosition){
    //Send CMD
    char cmd[1] = {CMD_BASE_GET_POSITION};
    if(fifo_write(cmd, 1) < 0)
        return -1;
    //Read Message
    uint8_t msgSize = 12;
    char rx[msgSize];
    if(fifo_read(rx, msgSize) < 0)
        return -1;
    memcpy(longitudinalPosition, &rx[0], sizeof(float));
    memcpy(transversalPosition, &rx[4], sizeof(float));
    memcpy(angularPosition, &rx[8], sizeof(float));
    return 0;
}

int base_get_velocity_PID(uint8_t wheelNumber, float * Kp, float * Ki, float * Kd){
    //Send CMD
    char cmd[1] = {CMD_BASE_GET_VELOCITY_PID};
    if(fifo_write(cmd, 1) < 0)
        return -1;
    //Send Message (Wheel Number)
    uint8_t txMsgSize = 1;
    char tx[txMsgSize];
    memcpy(&tx[0], &wheelNumber, sizeof(wheelNumber));
    if(fifo_write(tx, txMsgSize) < 0)
        return -1;
    //Read Message (PID Values)
    uint8_t rxMsgSize = 12;
    char rx[rxMsgSize];
    if(fifo_read(rx, rxMsgSize) < 0)
        return -1;

    memcpy(Kp, &rx[0], sizeof(float));
    memcpy(Ki, &rx[4], sizeof(float));
    memcpy(Kd, &rx[8], sizeof(float));
    return 0;
}

int main()
{

    fd = open(inputFifo, O_RDWR);
    fd2 = open(outputFifo, O_RDWR);

    base_set_ctrl_mode(0x02);
    //base_set_velocity(0.1f, 0.0f, 0.0f);
    //base_reset();

    while(1){
        float sp[3] ={0};
        base_get_position(&sp[0], &sp[1], &sp[2]);
        printf("%f %f %f \n", sp[0], sp[1], sp[2]);
        //base_set_velocity_PID(3, 10.0f, 0.0f, 0.0f);
        //float Kp, Ki, Kd;
        //base_get_velocity_PID(0, &Kp, &Ki, &Kd);
        
        usleep(100000);
    }

    close(fd);
    close(fd2);

    return 0;
}