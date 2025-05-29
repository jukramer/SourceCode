#pragma once
#include "hardware/i2c.h"

class VL6180X {
public:
    VL6180X();
    bool init(i2c_inst_t *i2c, uint8_t addr);
    bool set_address(uint8_t new_addr);
    uint8_t read_range();

private:
    i2c_inst_t *i2c_bus;
    uint8_t i2c_addr;
    void load_default_settings();

    bool write_reg(uint16_t reg, uint8_t value);
    uint8_t read_reg(uint16_t reg);
};