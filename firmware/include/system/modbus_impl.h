//
// Created by depavlov on 26.04.2023.
//

#ifndef OPEN_PNP_HEAD_MODBUS_IMPL_H
#define OPEN_PNP_HEAD_MODBUS_IMPL_H

#include "system/modbus.h"

#define NOZZLE_N                        4
#define REGISTER_RELAYS                 1       // 16-bit coil register with relay state (read/write)
#define REGISTER_END_STOPS              10011   // 16-bit input register with end-stop state (read-only)
#define REGISTER_STATE                  30001   // 16-bit register holding current state (read-only)
#define REGISTER_VAC_SENSOR_BASE        30002   // n * 2x16-bit registers with current vacuum level (read-only)
#define REGISTER_POSITION_BASE          40001   // n * 2x16-bit registers holding position (read/write)
#define REGISTER_LIGHT_MSW              40101   // 2x16-bit register holding 24-bit RGB value
#define REGISTER_LIGHT_LSW              40102   // ---

class ModBusSlave : public ModBus {
private:
    Settings *settings;
    State *state;

    bool    get_bit_reg_value(uint16_t reg) override;
    uint8_t set_bit_reg_value(uint16_t reg, bool value) override;
    uint8_t set_16bit_reg_value(uint16_t reg, uint16_t value) override;
    uint16_t get_16bit_reg_value(uint16_t reg) override;

public:
    explicit ModBusSlave(uint8_t address, Settings *settings, State *state);
};


#endif //OPEN_PNP_HEAD_MODBUS_IMPL_H
