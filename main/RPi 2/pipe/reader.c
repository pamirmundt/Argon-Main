//This code is a test code..

#include <fcntl.h>
#include <stdio.h>
//#include <sys/stat.h>
#include <unistd.h>
#include <string.h>

#include <sys/select.h>


int main()
{
    int fd;
    char * myfifo = "/tmp/myfifo";
    char msg[12] = {0};

	/* create the FIFO (named pipe) */
    mkfifo(myfifo, 0666);

    //Pipe has be open from both sides
    /* open, read, and display the message from the FIFO */
    fd = open(myfifo, O_RDWR);
    printf("%d \n", fd);


    

    while(1){

	    if(read(fd, &msg, sizeof(msg)) > 0){
        float test = 0;
        memcpy(&test, &msg[0], sizeof(msg));
	    	printf("Received: %f \n", test);
	    }
	}


    //printf("Received: %f \n", test);

    close(fd);

    /* remove the FIFO */
    unlink(myfifo);

    return 0;
}