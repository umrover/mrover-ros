#pragma once

#include "main.h"
#include "messaging.hpp"

#define NOOP 0x50

namespace mrover {

    class Pin {
    public:
        explicit Pin() = default;
        Pin(GPIO_TypeDef *port, uint16_t pin) : port(port), pin(pin) { }

        inline GPIO_PinState read() {
            return HAL_GPIO_ReadPin(this->port, this->pin);
        }

        inline void write(GPIO_PinState val) {
            HAL_GPIO_WritePin(this->port, this->pin, val);
        }

    private:
        GPIO_TypeDef *port{};
        uint16_t pin{};
    };

    class LimitSwitch {
    public:
        LimitSwitch(Pin _pin)
            : m_pin(_pin)
            , m_enabled(0)
            , m_is_pressed(0)
            , m_active_high(0)
            , m_associated_count(0)
            { }

        void update_limit_switch() {
            // This suggests active low
            if (this->m_enabled) {
                this->m_is_pressed = (this->m_active_high == this->m_pin.read());
            } else {
                this->m_is_pressed = 0;
            }
        }

    private:
        Pin m_pin;
        uint8_t m_enabled;
        uint8_t m_is_pressed;
        uint8_t m_valid;
        uint8_t m_active_high;
        int32_t m_associated_count;

    };

    class SMBus {
    public:
        SMBus() = default;

        explicit SMBus(I2C_HandleTypeDef* hi2c)
            : m_i2c(hi2c) {
            for (unsigned char & i : m_buf) {
                i = 0;
            }
        }

        uint64_t read_word_data(uint8_t addr, char cmd) {
            this->m_buf[0] = cmd;
            this->m_ret = HAL_I2C_Master_Transmit(this->m_i2c, addr << 1, this->m_buf, 1, 500);

            //reads from address sent above
            this->m_ret = HAL_I2C_Master_Receive(this->m_i2c, (addr << 1) | 1, this->m_buf, 2, 500);

            long data = this->m_buf[0] | (this->m_buf[1] << 8);
            if (this->m_ret != HAL_OK) {
                HAL_I2C_DeInit(this->m_i2c);
                HAL_Delay(5);
                HAL_I2C_Init(this->m_i2c);
                data = 0;
            }

            return data;
        }

    private:
        I2C_HandleTypeDef *m_i2c{};
        HAL_StatusTypeDef m_ret{};
        uint8_t m_buf[30]{};
    };

    class CANWrapper {
    public:
        CANWrapper() = default;

        explicit CANWrapper(FDCAN_HandleTypeDef* hfdcan)
            : m_fdcan(hfdcan) {

        	if (HAL_FDCAN_Start(m_fdcan) != HAL_OK) {
			  Error_Handler();
			}
			if (HAL_FDCAN_ActivateNotification(m_fdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
			  /* Notification Error */
			  Error_Handler();
			}

        	TxHeader.Identifier = 0x11;
			TxHeader.IdType = FDCAN_STANDARD_ID;
			TxHeader.TxFrameType = FDCAN_DATA_FRAME;
			TxHeader.DataLength = FDCAN_DLC_BYTES_12;
			TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
			TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
			TxHeader.FDFormat = FDCAN_FD_CAN;
			TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
			TxHeader.MessageMarker = 0;
        }


        // Need to define the function
        // void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
        // in main
        // and call this function
        void insert_HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
        {
          if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
          {
            /* Retrieve messages from RX FIFO0 */
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &m_RxHeader, m_rxdata) != HAL_OK)
            {
            /* Reception Error */
            Error_Handler();
            }

            // TODO - process information
            // Refer to ICD



            if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
            {
              /* Notification Error */
              Error_Handler();
            }
          }
        }

        void send_data(uint8_t data[], uint8_t data_length) {

			if (data_length == 0) {
        		m_TxHeader.DataLength = FDCAN_DLC_BYTES_0;
        	}
        	else if (data_length == 1) {
        		m_TxHeader.DataLength = FDCAN_DLC_BYTES_1;
        		m_txdata[0] = data[0];
        	}
        	else if (data_length == 2) {
        		m_tx_header.DataLength = FDCAN_DLC_Bytes_2;
        		memcpy(m_txdata, data, data_length);
        	}
        	else if (data_length == 3) {
        		m_tx_header.DataLength = FDCAN_DLC_Bytes_3;
				memcpy(m_txdata, data, data_length);
        	}
        	else if (data_length == 4) {
        		m_tx_header.DataLength = FDCAN_DLC_Bytes_4;
				memcpy(m_txdata, data, data_length);
        	}
        	else if (data_length == 5) {
        		m_tx_header.DataLength = FDCAN_DLC_Bytes_5;
				memcpy(m_txdata, data, data_length);
        	}
        	else if (data_length == 6) {
        		m_tx_header.DataLength = FDCAN_DLC_Bytes_6;
				memcpy(m_txdata, data, data_length);
        	}
        	else if (data_length == 7) {
        		m_tx_header.DataLength = FDCAN_DLC_Bytes_7;
				memcpy(m_txdata, data, data_length);
        	}
        	else if (data_length == 8) {
        		m_tx_header.DataLength = FDCAN_DLC_Bytes_8;
				memcpy(m_txdata, data, data_length);
        	}
        	else if (data_length <= 12) {
        		m_tx_header.DataLength = FDCAN_DLC_Bytes_12;
				memcpy(m_txdata, data, data_length);
				for (int i = data_length; i < 12; ++i) {
					m_txdata[i] = NOOP;
				}
        	}
        	else if (data_length <= 16) {
				m_tx_header.DataLength = FDCAN_DLC_Bytes_16;
				memcpy(m_txdata, data, data_length);
				for (int i = data_length; i < 16; ++i) {
					m_txdata[i] = NOOP;
				}
			}
        	else if (data_length <= 20) {
				m_tx_header.DataLength = FDCAN_DLC_Bytes_20;
				memcpy(m_txdata, data, data_length);
				for (int i = data_length; i < 20; ++i) {
					m_txdata[i] = NOOP;
				}
			}
        	else if (data_length <= 24) {
				m_tx_header.DataLength = FDCAN_DLC_Bytes_24;
				memcpy(m_txdata, data, data_length);
				for (int i = data_length; i < 24; ++i) {
					m_txdata[i] = NOOP;
				}
			}
        	else if (data_length <= 32) {
				m_tx_header.DataLength = FDCAN_DLC_Bytes_32;
				memcpy(m_txdata, data, data_length);
				for (int i = data_length; i < 32; ++i) {
					m_txdata[i] = NOOP;
				}
			}
        	else if (data_length <= 48) {
				m_tx_header.DataLength = FDCAN_DLC_Bytes_48;
				memcpy(m_txdata, data, data_length);
				for (int i = data_length; i < 48; ++i) {
					m_txdata[i] = NOOP;
				}
			}
        	else if (data_length <= 64) {
				m_tx_header.DataLength = FDCAN_DLC_Bytes_64;
				memcpy(m_txdata, data, data_length);
				for (int i = data_length; i < 64; ++i) {
					m_txdata[i] = NOOP;
				}
			}
        	else {
        		// Invalid length
        		return;
        	}

        	if (HAL_FDCAN_AddMessageToTxFifoQ(m_fdcan, &TxHeader, TxData)!= HAL_OK)
			{
				Error_Handler();
			}
        }

    private:
        FDCAN_HandleTypeDef *m_fdcan{};
        HAL_StatusTypeDef m_ret{};
        FDCAN_TxHeaderTypeDef m_TxHeader;
        FDCAN_RxHeaderTypeDef m_RxHeader;
        uint8_t m_rxdata[64]{};
        uint8_t m_txdata[64]{};
    };

}
