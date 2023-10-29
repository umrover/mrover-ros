typedef struct CANManager {
	FDCAN_HandleTypeDef* hfdcan1;
	FDCAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[12];

} CANManager;
