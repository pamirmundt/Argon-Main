Base Control with Nucleo-f303k8
------------
- Slaves - Nucleo-f303k8
- Master - Raspberry Pi


Toolchain
------------
######STM32CubeMX -> Keil uVision5

I2C Setup
------------
Baudwidth: 200kHz (fast-mode)
Rise-time: 300ns
RPi Pullups: 1.8kOhms
Board Pullups: 470 Ohms (parallel with 1.8k)
Total resistance on bus: 372 Ohms
Current Sink: 8,87mA (0,00887 Amps)
I2C Bus Capacitance: 1 nanoFarad (10^-9 Farad)

######Useful Links for I2C
	http://www.ti.com/lit/an/slva689/slva689.pdf
