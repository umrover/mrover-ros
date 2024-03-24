// AS7262 Spectral sensor
// Datasheet: https://www.mouser.com/datasheet/2/588/AS7262_DS000486_2-00-1082195.pdf

#pragma once
#include "stm32g4xx_hal.h"
#include "FreeRTOS.h"

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "cmsis_os2.h"
#include "hardware.hpp"
#include "i2c_mux.hpp"
#include <memory>

namespace mrover {

    class Spectral {
    public:
    	Spectral() = default;

    	Spectral(std::shared_ptr<SMBus<uint8_t, uint8_t>> i2c_bus, std::shared_ptr<I2CMux> i2c_mux, uint8_t i2c_mux_channel);

        enum I2C_OP{
        	READ,
			WRITE
        };
        void poll_status_reg(I2C_OP rw);

    	bool update_channel_data(); // updates all of the channels

    	float get_channel_data(uint8_t channel);

        void reboot();

        bool is_error();

        void init();

        void virtual_write(uint8_t virtual_reg, uint8_t data);
        uint8_t virtual_read(uint8_t virtual_reg);// -> std::optional<uint16_t>;

        constexpr static std::uint16_t SPECTRAL_7b_ADDRESS = 0x49;
        constexpr static std::uint8_t I2C_AS72XX_SLAVE_STATUS_REG = 0x00;
        constexpr static std::uint8_t I2C_AS72XX_WRITE_REG = 0x01;
        constexpr static std::uint8_t I2C_AS72XX_READ_REG = 0x02;
        constexpr static std::uint8_t I2C_AS72XX_SLAVE_TX_VALID = 0x02;
        constexpr static std::uint8_t I2C_AS72XX_SLAVE_RX_VALID = 0x01;
        constexpr static std::uint8_t CONTROL_SETUP_REG = 0x04;
        constexpr static std::uint8_t INT_TIME_REG = 0x05;

    private:
        bool m_error{};
        bool m_initialized{};

        std::shared_ptr<SMBus<uint8_t, uint8_t>> m_i2c_bus;
        std::shared_ptr<I2CMux> m_i2c_mux;
        uint8_t m_i2c_mux_channel;
        constexpr static std::uint8_t CHANNEL_DATA_LENGTH = 6;
        std::array<float, CHANNEL_DATA_LENGTH> channel_data {};
        // Sensor Raw Data Registers Start, 6 channels, 2 bytes each.
        // See pg. 22 of datasheet for more info.
//        constexpr static std::uint8_t CHANNEL_V_HIGH = 0x08;
        // Sensor Calibrated Data Registers Start (p. 23 on datasheet)
        constexpr static std::uint8_t CHANNEL_V_HIGH = 0x08;
        constexpr static std::uint8_t CHANNEL_V_CAL = 0x14;
        
    };

} // namespace mrover

