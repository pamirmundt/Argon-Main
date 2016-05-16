#include "i2c.h"

extern Motor m;
extern uint16_t PWM_resolution;
extern const uint16_t RXBUFFERSIZE;		//Receive buffer size
extern const uint16_t TXBUFFERSIZE;		//Transmit buffer size
extern uint8_t txBuffer[];
extern uint8_t rxBuffer[];

extern void motorInit(void);

//Set Motor Configuration and Function Commands
#define CMD_RESET 				0x02
#define CMD_SET_SPEED			0x03	//Velocity control mode Only
#define CMD_SET_PID_CONST	0x04
#define CMD_SET_POWER			0x05	//Manuel mode only
#define CMD_SET_PWM				0x06	//Manuel mode only
#define CMD_SET_MODE			0x07

//Get Motor Configurations and Status
#define CMD_GET_PID 			0x10
#define CMD_GET_POWER 		0x11
#define CMD_GET_POS 			0x12
#define CMD_GET_SPEED 		0x13
#define CMD_GET_PWM 			0x14
#define CMD_GET_REF_SPEED 0x15

//rxBuffer - Receive Buffer - 13 Byte
//	1 Byte	|				4 Bytes				|					4 Bytes			|				4 Bytes				|
// int8 cmd | f/int32 parameter 1 | f/int32 parameter 2 | f/int32 parameter 3	|

//txBuffer - Transmit Buffer - 12 Byte
// 				4 Bytes			 |				4 Bytes			 |				4 Bytes			 |
// f/int32 parameter 1 | f/int32 parameter 2 | f/int32 parameter 3 |

void cmdParser(){
	//Clean TX Buffer before writing new messages
	memset(&txBuffer, 0, sizeof(TXBUFFERSIZE));

	//Get Command(CMD)
	uint8_t cmd[1];
	memcpy(&cmd[0], &rxBuffer[0], sizeof(cmd));
	
	switch(cmd[0]){
    //***** SET Commands *****
    case CMD_RESET: {
			motorInit();
			memset(&m, 0, sizeof(m));
			
    } break;
		
		case CMD_SET_SPEED: {
			//@Set Reference velocity as RPM (Float)
			float params[1] = {0};
			memcpy(&params[0], &rxBuffer[1], sizeof(float));
			m.refRPM = params[0];
		} break;
		
		case CMD_SET_PID_CONST: {
			//@Set proportional Constant (Float)
			//@Set integral Constant (Float)
			//@Set derivative Constant (Float)
			float params[3] = {0};
			memcpy(&params[0], &rxBuffer[1], sizeof(float));
			memcpy(&params[1], &rxBuffer[5], sizeof(float));
			memcpy(&params[2], &rxBuffer[9], sizeof(float));
			m.Kp = params[0];
			m.Ki = params[1];
			m.Kd = params[2];
		} break;

		case CMD_SET_POWER: {
			//@Set Power between -100% 100% (Float)
			float params[1] = {0};
			memcpy(&params[0], &rxBuffer[1], sizeof(float));
			//Saturate
			if(params[0] > 100.0f)
				params[0] = 100.f;
			else if(params[0] < -100.0f)
				params[0] = -100.0f;
			
			m.PWM = (PWM_resolution/100.0f)*params[0];			
		} break;
		
		case CMD_SET_PWM: {
			//@Set PWM <-> Direction (Float)
			float params[1] = {0};
			memcpy(&params[0], &rxBuffer[1], sizeof(float));
			m.PWM = params[0];
		} break;
		
		case CMD_SET_MODE: {
			//@Set Mode (uint8_t)
			//Manuel Mode: 				0x00
			//Velocity PID Mode: 	0x01
			uint8_t params[1] = {0};
			memcpy(&params[0], &rxBuffer[1], sizeof(uint8_t));
			m.mode = params[0];
		} break;
		
		//***** GET Commands *****
		case CMD_GET_PID: {
			//@Get proportional Constant (Float)
			//@Get integral Constant (Float)
			//@Get derivative Constant (Float)
			float params[3] = {0};
			params[0] = m.Kp;
			params[1] = m.Ki;
			params[2] = m.Kd;
			memcpy(&txBuffer[0], &params[0], sizeof(float));
			memcpy(&txBuffer[4], &params[1], sizeof(float));
			memcpy(&txBuffer[8], &params[2], sizeof(float));
		} break;
		
		case CMD_GET_POWER: {
			//@Get Power (float) -100% 100%
			float params[1] = {0};
			params[0] = (m.PWM/PWM_resolution)*100.0f;
			memcpy(&txBuffer[0], &params[0], sizeof(float));
		} break;
		
		case CMD_GET_POS: {
			//@Get encoder position (int32_t)
			int32_t params[1] = {0};
			params[0] = m.encPos;
			memcpy(&txBuffer[0], &params[0], sizeof(int32_t));
		} break;
		
		case CMD_GET_SPEED: {
			//@Get speed(RPM) (float)
			float params[1] = {0};
			params[0] = m.RPM;
			memcpy(&txBuffer[0], &params[0], sizeof(float));
		} break;
		
		case CMD_GET_PWM: {
			//@Get PWM (float)
			float params[1] = {0};
			params[0] = m.PWM;
			memcpy(&txBuffer[0], &params[0], sizeof(float));
		} break;
			
		case CMD_GET_REF_SPEED: {
			//@Get Reference Speed (float)
			float params[1] = {0};
			params[0] = m.refRPM;
			memcpy(&txBuffer[0], &params[0], sizeof(float));
		} break;
	}
	
	//Clean RX Buffer before writing new messages
	memset(&rxBuffer, 0, sizeof(RXBUFFERSIZE));
}
