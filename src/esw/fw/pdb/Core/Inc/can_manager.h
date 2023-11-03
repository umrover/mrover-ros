#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include "stm32g4xx_hal.h"

#define CAN_MSG_PDB_DATA 16

typedef struct CANManager {
	FDCAN_HandleTypeDef* hfdcan1;
} CANManager;

void send_pdb_data(CANManager* can, float* temperature_data, float* current_data);

#endif // CAN_MANAGER_H
