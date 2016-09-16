#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include "RPi-i2c.h"

static int fd;

int init_I2C(void){
	const char *device;

	//RPI-1: "/dev/i2c-0"
	//RPI-2: "/dev/i2c-1"
	int piRev = 2;
	
	if(piRev == 1)
 		device = "/dev/i2c-0";
	else
		device = "/dev/i2c-1";


	if ((fd = open (device, O_RDWR)) < 0){
    		printf( "Unable to open I2C device \n");
    		printf("ioctl failed and returned errno: %s \n", strerror(errno));
			exit(1);
	}

	return fd;
}

int terminate_I2C(void){
	if(close(fd) < 0)
		printf("ioctl failed and returned errno: %s \n", strerror(errno));

	return 0;
}

int open_Port(int devId){
	if(ioctl (fd, I2C_SLAVE, devId) < 0){
		printf("Unable to select I2C device \n");
		printf("ioctl failed and returned errno: %s \n", strerror(errno));
		exit(1);
	}

	return fd;
}

inline int i2cMasterTransmit(uint16_t addr){
	
	open_Port(addr);

	if(write(fd, i2cTXBuffer, TXBUFFERSIZE) != TXBUFFERSIZE){
		printf("Failed to i2c-write \n");
		printf("ioctl failed and returned errno: %s \n", strerror(errno));
		exit(1);
	}

	return 0;
}

inline int i2cMasterReceive(uint16_t addr){

	open_Port(addr);

	if(read(fd, i2cRXBuffer, RXBUFFERSIZE) != RXBUFFERSIZE){
		printf("Failed to i2c-write \n");
		printf("ioctl failed and returned errno: %s \n", strerror(errno));
		exit(1);
	}
	
	return 0;
}