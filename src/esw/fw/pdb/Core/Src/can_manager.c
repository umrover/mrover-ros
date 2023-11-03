#include "can_manager.h"

void send_pdb_data(CANManager* can, float* temperature_data, float* current_data) {
	FDCAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[12];

	// TODO
}
