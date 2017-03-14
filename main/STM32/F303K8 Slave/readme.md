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
**Rise-time:** 300 ns (~271ns)  
**RPi Pullups:** 1.8 kOhms  
**Board Pullups:** 390 Ohms (parallel with 1.8k)  
**Total resistance on bus:** 320.54 Ohms  
**Current Sink:** 10.29mA (0,01029 Amps)  
**I2C Bus Capacitance:** ~1 nF (10^-9 Farad)  

####Useful Links for I2C
	http://www.ti.com/lit/an/slva689/slva689.pdf
