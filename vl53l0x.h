#pragma once
#include "hardware/i2c.h"

class VL53L0X {
public:
    VL53L0X();
    bool init(i2c_inst_t *i2c, uint8_t addr);
    bool set_address(uint8_t new_addr);
    uint16_t read_range();

private:
    i2c_inst_t *i2c_bus;
    uint8_t i2c_addr;
    uint8_t stop_variable;

    bool write_reg(uint8_t reg, uint8_t value);
    bool read_reg(uint8_t reg, uint8_t &value);
    bool write_multi(uint8_t reg, const uint8_t *data, uint8_t count);
    bool read_multi(uint8_t reg, uint8_t *data, uint8_t count);
};