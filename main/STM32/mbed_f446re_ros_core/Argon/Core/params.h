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
#define lengthBetweenFrontAndRearWheels   0.150f    //meter - a:150mm
#define lengthBetweenFrontWheels          0.127f    //meter - b:127mm
#define geomFactor                       (lengthBetweenFrontAndRearWheels / 2.0f + lengthBetweenFrontWheels / 2.0f)
#define gearRatio                        ((35.0f*37.0f*35.0f*38.0f)/(12.0f*11.0f*13.0f*10.0f))
/* Encoder Parameters --------------------------------------------------------*/
#define encoder_resolution 3.0f   //3 CPR
#define encoder_mode 4.0f          //x4 Quadrature Reading
/* RPM Calculation Parameters ------------------------------------------------*/
//RPM Moving Average Filter (MAF)
#define RPM_MAF true
#define RPM_MAF_Alpha 0.125f

//PinName BNO055_SDA = PB_7;
//PinName BN0055_SCL = PB_6;

#define BNO055_SDA PB_9
#define BNO055_SCL PB_8

/* STM32F446RE Parameters -----------------------------------------------------*/
//PLLCLK Speed
#define PLLCLK_speed 180000000  //DO NOT CHANGE THIS VALUE

//PWM Generation
//Timer2 - 90Mhz
//12Bit Resolution
//90Mhz / (4095 + 1) ~= 21.972kHz
#define Timer2CounterPeriod 4095

//Encoder Reading
//Timer 1 - Motor 1
//Timer 3 - Motor 2
//Timer 4 - Motor 3
//Timer 8 - Motor 4s
//16 Bit Counters
//Timer counts up to 65535
#define EncoderTimerCounterPeriod 65535

//RPM Calculation
//Timer 6 - 90Mhz
//90Mhz/(999+1)/(449+1) = 200Hz
#define Timer6Prescaler 999
#define Timer6CounterPeriod 449
//Timer 6 Period
#define Timer6Frequency (90000000.0f/1000.0f/450.0f)


#ifdef __cplusplus
}
#endif

#endif /* __PARAMS_H */
