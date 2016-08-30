Base Control with Nucleo-f303k8
------------
- Slaves - Nucleo-f303k8
- Master - Raspberry Pi


Toolchain
------------
######STM32CubeMX -> Keil uVision5

I2C Setup
------------
**Baudwidth:** 400 kHz (fast-mode)  
**Rise-time:** 300 ns (~294ns)  
**RPi Pullups:** 1.8 kOhms  
**Board Pullups:** 430 Ohms (parallel with 1.8k)  
**Total resistance on bus:** 347.08 Ohms  
**Current Sink:** 9,5 mA (0,009507 Amps)  
**I2C Bus Capacitance:** ~1 nF (10^-9 Farad)  

####Useful Links for I2C
	http://www.ti.com/lit/an/slva689/slva689.pdf
