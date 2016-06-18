#include "I2CParser.h"

extern Motor m;
extern uint16_t PWM_resolution;
extern const uint16_t RXBUFFERSIZE;		//Receive buffer size
extern const uint16_t TXBUFFERSIZE;		//Transmit buffer size
extern uint8_t txBuffer[];
extern uint8_t rxBuffer[];

extern void motorInit(void);

//Set Base Configuration and Function Commands
//Joint Commands
#define CMD_RESET_JOINT 						0x02
#define CMD_SET_JOINT_PWM						0x03	//Manuel mode
#define CMD_SET_JOINT_SPEED					0x04	//Velocity mode
#define CMD_SET_JOINT_POWER					0x05	//Manuel mode

//Control Mode
#define CMD_SET_CONT_MODE						0x10	//Manuel/Velocity/Position Mode
//PID
#define CMD_SET_VEL_PID_CONST				0x20
#define CMD_GET_VEL_PID_CONST				0x22

//Get Motor Configurations and Status
//Joint Commands
#define CMD_GET_JOINT_PWM 					0x25
#define CMD_GET_JOINT_POS						0x26	//Encoder count
#define CMD_GET_JOINT_SPEED					0x27	//RPM
#define CMD_GET_JOINT_POWER					0x28	//0-100%
#define CMD_GET_JOINT_REF_SPEED			0x29


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
    case CMD_RESET_JOINT: {
			__HAL_TIM_SET_COUNTER(&htim2, 0);	//RESET encoder counter
			memset(&m, 0, sizeof(Motor));
			motorInit();
    } break;
		
		case CMD_SET_JOINT_SPEED: {
			//@Set Reference velocity as RPM (Float)
			float params[1] = {0};
			memcpy(&params[0], &rxBuffer[1], sizeof(float));
			m.refRPM = params[0];
		} break;
		
		case CMD_SET_VEL_PID_CONST: {
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

		case CMD_SET_JOINT_POWER: {
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
		
		case CMD_SET_JOINT_PWM: {
			//@Set PWM <-> Direction (Float)
			float params[1] = {0};
			memcpy(&params[0], &rxBuffer[1], sizeof(float));
			m.PWM = params[0];
		} break;
		
		case CMD_SET_CONT_MODE: {
			//@Set Mode (uint8_t)
			//Manuel Mode: 				0x00
			//Velocity PID Mode: 	0x01
			uint8_t params[1] = {0};
			memcpy(&params[0], &rxBuffer[1], sizeof(uint8_t));
			m.mode = params[0];
		} break;
		
		//***** GET Commands *****
		case CMD_GET_VEL_PID_CONST: {
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
		
		case CMD_GET_JOINT_POWER: {
			//@Get Power (float) -100% 100%
			float params[1] = {0};
			params[0] = (m.PWM/PWM_resolution)*100.0f;
			memcpy(&txBuffer[0], &params[0], sizeof(float));
		} break;
		
		case CMD_GET_JOINT_POS: {
			//@Get encoder position (int32_t)
			int32_t params[1] = {0};
			params[0] = m.encPos;
			memcpy(&txBuffer[0], &params[0], sizeof(int32_t));
		} break;
		
		case CMD_GET_JOINT_SPEED: {
			//@Get speed(RPM) (float)
			float params[1] = {0};
			params[0] = m.RPM;
			memcpy(&txBuffer[0], &params[0], sizeof(float));
		} break;
		
		case CMD_GET_JOINT_PWM: {
			//@Get PWM (float)
			float params[1] = {0};
			params[0] = m.PWM;
			memcpy(&txBuffer[0], &params[0], sizeof(float));
		} break;
			
		case CMD_GET_JOINT_REF_SPEED: {
			//@Get Reference Speed (float)
			float params[1] = {0};
			params[0] = m.refRPM;
			memcpy(&txBuffer[0], &params[0], sizeof(float));
		} break;
	}
	
	//Clean RX Buffer before writing new messages
	memset(&rxBuffer, 0, sizeof(RXBUFFERSIZE));
}
