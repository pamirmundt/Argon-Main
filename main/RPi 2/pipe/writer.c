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

//Base get CMDs
#define CMD_BASE_GET_CTRL_MODE      0x40
#define CMD_BASE_GET_VELOCITY       0x41
#define CMD_BASE_GET_REF_VELOCITY   0x42
#define CMD_BASE_GET_POSITION       0X43


#define txBufferSize 13
#define rxBufferSize 12

int fd, fd2;
char * inputFifo = "/tmp/mecanumBaseDriverInput";
char * outputFifo = "/tmp/mecanumBaseDriverOutput";



int fifo_write(char tx[], uint8_t size){
    if(write(fd, tx, txBufferSize) < 0){
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
    char tx[txBufferSize] = {0};

    uint8_t cmd = CMD_BASE_RESET;
    memcpy(&tx[0], &cmd, sizeof(cmd));
    return (fifo_write(tx, txBufferSize));
}

int base_set_velocity(float longitudinalVelocity, float transversalVelocity, float angularVelocity){
    char tx[txBufferSize] = {0};

    uint8_t cmd = CMD_BASE_SET_VELOCITY;
    memcpy(&tx[0], &cmd, sizeof(cmd));
    memcpy(&tx[1], &longitudinalVelocity, sizeof(longitudinalVelocity));
    memcpy(&tx[5], &transversalVelocity, sizeof(transversalVelocity));
    memcpy(&tx[9], &angularVelocity, sizeof(angularVelocity));
    return (fifo_write(tx, txBufferSize));
}

int base_set_ctrl_mode(uint8_t mode){
    char tx[txBufferSize] = {0};

    uint8_t cmd = CMD_BASE_SET_CTRL_MODE;
    memcpy(&tx[0], &cmd, sizeof(cmd));
    memcpy(&tx[1], &mode, sizeof(mode));

    return (fifo_write(tx, txBufferSize));
}

int base_get_ctrl_mode(uint8_t * mode){
    char tx[txBufferSize] = {0};
    uint8_t cmd = CMD_BASE_GET_CTRL_MODE;
    memcpy(&tx[0], &cmd, sizeof(cmd));
    if (fifo_write(tx, txBufferSize) < 0)
        return -1;

    char rx[rxBufferSize];
    if(fifo_read(rx, rxBufferSize) < 0)
        return -1;
    memcpy(mode, &rx[0], sizeof(uint8_t));
    return 0;   
}

int base_get_velocity(float * longitudinalVelocity, float * transversalVelocity, float * angularVelocity){
    char tx[txBufferSize] = {0};
    uint8_t cmd = CMD_BASE_GET_VELOCITY;
    memcpy(&tx[0], &cmd, sizeof(cmd));
    if (fifo_write(tx, txBufferSize) < 0)
        return -1;

    char rx[rxBufferSize];
    if(fifo_read(rx, rxBufferSize) < 0)
        return -1;
    memcpy(longitudinalVelocity, &rx[0], sizeof(float));
    memcpy(transversalVelocity, &rx[4], sizeof(float));
    memcpy(angularVelocity, &rx[8], sizeof(float));
}

int base_get_ref_velocity(float * refLongitudinalVelocity, float * refTransversalVelocity, float *refAngularVelocity){
    char tx[txBufferSize] = {0};
    uint8_t cmd = CMD_BASE_GET_REF_VELOCITY;
    memcpy(&tx[0], &cmd, sizeof(cmd));
    if (fifo_write(tx, txBufferSize) < 0)
        return -1;

    char rx[rxBufferSize];
    if(fifo_read(rx, rxBufferSize) < 0)
        return -1;
    memcpy(refLongitudinalVelocity, &rx[0], sizeof(float));
    memcpy(refTransversalVelocity, &rx[4], sizeof(float));
    memcpy(refAngularVelocity, &rx[8], sizeof(float));
}


int base_get_position(float * longitudinalPosition, float * transversalPosition, float * angularPosition){
    char tx[txBufferSize] = {0};
    uint8_t cmd = CMD_BASE_GET_POSITION;
    memcpy(&tx[0], &cmd, sizeof(cmd));
    if (fifo_write(tx, txBufferSize) < 0)
        return -1;

    char rx[rxBufferSize];
    if(fifo_read(rx, rxBufferSize) < 0)
        return -1;
    memcpy(longitudinalPosition, &rx[0], sizeof(float));
    memcpy(transversalPosition, &rx[4], sizeof(float));
    memcpy(angularPosition, &rx[8], sizeof(float));
}


int main()
{

    fd = open(inputFifo, O_RDWR);
    fd2 = open(outputFifo, O_RDWR);

    base_set_ctrl_mode(0x01);
    base_set_velocity(0.1f, 0.0f, 0.0f);

    while(1){
        float sp[3] ={0};
        base_get_position(&sp[0], &sp[1], &sp[2]);
        printf("%f %f %f \n", sp[0], sp[1], sp[2]);

        usleep(100000);
    }

    close(fd);
    close(fd2);

    return 0;
}