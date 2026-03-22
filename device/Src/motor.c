#include "bsp_fdcan.h"
#include "dji_motor.h"
#include "mi_motor.h"

void fdcan1_data_interpret(FDCAN_RxHeaderTypeDef *header, uint8_t *buff)
{
	switch (header->Identifier) {
	case 0x201:
		dji_motor_interpret(buff, &dji3508_1);
		break;
	case 0x202:
		dji_motor_interpret(buff, &dji3508_2);
		break;
	case 0x203:
		dji_motor_interpret(buff, &dji3508_3);
		break;
	case 0x204:
		dji_motor_interpret(buff, &dji3508_4);
		break;
	case 0x205:
		dji_motor_interpret(buff, &dji6020_1);
		break;
	case 0x206:
		dji_motor_interpret(buff, &dji6020_2);
		break;
	case 0x207:
		dji_motor_interpret(buff, &dji6020_3);
		break;
	case 0x208:
		dji_motor_interpret(buff, &dji6020_4);
		break;
	default:
		break;
	}
}

void fdcan2_data_interpret(FDCAN_RxHeaderTypeDef *header, uint8_t *buff)
{
	switch (header->Identifier) {
	case 0x201:
		dji_motor_interpret(buff, &dji3508_5);
		break;
	case 0x202:
		dji_motor_interpret(buff, &dji3508_6);
		break;
	case 0x203:
		dji_motor_interpret(buff, &dji3508_7);
		break;
	case 0x204:
		dji_motor_interpret(buff, &dji3508_8);
		break;
	case 0x205:
		dji_motor_interpret(buff, &dji3508_9);
		break;
	case 0x206:
		dji_motor_interpret(buff, &dji3508_10);
		break;
	default:
		break;
	}
}

void fdcan3_data_interpret(FDCAN_RxHeaderTypeDef *header, uint8_t *buff)
{
	mi_motor_interpret(header, buff, &mi_motor);
}
