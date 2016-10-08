#include "mecanumBaseAPI.h"
#include <unistd.h>	//usleep()
#include <stdio.h>	//printf()
//DEMO

int main()
{
    mecanumBaseAPI();

    base_reset();
    usleep(200000);
    base_set_ctrl_mode(0x00);
    base_set_velocity(0.1f, 0.1f, 0.1f);

    while(1){
        float sp[3] ={0};
        base_get_velocity(&sp[0], &sp[1], &sp[2]);
        printf("%f %f %f \n", sp[0], sp[1], sp[2]);
        usleep(100000);
    }

    mecanumBaseAPI_Finalize();

    return 0;
}