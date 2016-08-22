#include "i2c-dev.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <linux/i2c-dev.h>

int main(){
	const char *device;

	static int fd;

	int devId = 0x22;


	//RPI-1: "/dev/i2c-0"
	//RPI-2: "/dev/i2c-1"
	int piRev = 2;
	
	if(piRev == 1)
 		device = "/dev/i2c-0";
	else
		device = "/dev/i2c-1";


	if ((fd = open (device, O_RDWR)) < 0){
    		printf( "Unable to open I2C device \n");
		return -1;
	}

	

	if(ioctl (fd, I2C_SLAVE_FORCE, devId) < 0){
		printf("Unable to select I2C device \n");
		return -1;
	}


	//TX 13
	//RX 12	
	
	uint8_t size = 13;
	uint8_t RXsize = 12;
	uint8_t buff[size];
	uint8_t rxbuff[RXsize];

	float speed = 10.0f;
	uint8_t mode = 0x00;
	uint8_t CMD = 0x26;

	memcpy(&buff[0], &CMD, sizeof(CMD));
	//memcpy(&buff[1], &speed, sizeof(speed));
	
	if(write(fd, buff, size) != 1)
		printf("fail \n");


	read(fd, rxbuff, RXsize);
	int32_t a;
	memcpy(&a, &rxbuff[0], sizeof(int32_t));
	printf("%d \n", a);	
}
