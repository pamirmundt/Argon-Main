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


/* Argon: Mecanum Base Parameters --------------------------------------------*/
#define wheelRadius                       0.03f     //meter - r:30mm
#define lengthBetweenFrontAndRearWheels   0.160f    //meter - a:160mm
#define lengthBetweenFrontWheels          0.148f    //meter - b:148mm
#define geomFactor                       (lengthBetweenFrontAndRearWheels / 2.0f + lengthBetweenFrontWheels / 2.0f)
#define gearRatio 4096.0f/27.0f
/* Encoder Parameters --------------------------------------------------------*/
#define encoder_resolution 16   //16 CPR
#define encoder_mode 4          //x4 Quadrature Reading



/* STM32F446RE Parameters -----------------------------------------------------*/
//PLLCLK Speed
#define PLLCLK_speed 180000000  //DO NOT CHANGE THIS VALUE

//Wait for the next encoder tick - If there is no encoder tick in XX secs, set RPM to zero.
#define timeout_msec 500

//Timer 2
//PWM Generation
//90Mhz / 4096 ~= 21.972kHz

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