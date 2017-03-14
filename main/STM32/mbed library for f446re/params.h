/**
  ******************************************************************************
  * @file    Params.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMS_H
#define __PARAMS_H

#ifdef __cplusplus
 extern "C" {
#endif 

//PLLCLK Speed
#define PLLCLK_speed 180000000  //DO NOT CHANGE THIS VALUE

//Encoder Settings
#define encoder_resolution 16   //16 CPR
#define encoder_mode 4          //x4 Encoding

//Wait for the next encoder tick - If there is no encoder tick in XX secs, set RPM to zero.
#define timeout_msec 500

//Timer 9 Settings
// Default: 180Mhz/1000/900 = 200Hz
#define timer9_prescaler 999    //0 <= timer9_prescaler <= 65535
#define timer9_period 899       //0 <= timer9_period <= 65535

//Timer 1 and 8 Settings
// Default: 180Mhz/1500 = 120kHz
#define IC_prescaler 1499       //0 <= IC_prescaler <= 65535
#define IC_period 65535         //0 <= IC_period <= 65535


#ifdef __cplusplus
}
#endif

#endif /* __PARAMS_H */