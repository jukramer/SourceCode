#include "vl53l0x.h"
#include "pico/stdlib.h"

#define SYSRANGE_START 0x00
#define RESULT_RANGE_STATUS 0x14
#define I2C_SLAVE_DEVICE_ADDRESS 0x8A

VL53L0X::VL53L0X() : i2c_bus(nullptr), i2c_addr(0x29) {}

bool VL53L0X::init(i2c_inst_t *i2c, uint8_t addr) {
    this->i2c_bus = i2c;
    this->i2c_addr = addr;
    sleep_ms(50);

    // Init sequence
    if (!write_reg(0x88, 0x00)) return false;
    if (!write_reg(0x80, 0x01)) return false;
    if (!write_reg(0xFF, 0x01)) return false;
    if (!write_reg(0x00, 0x00)) return false;

    uint8_t temp = 0;
    if (!read_reg(0x91, temp)) return false;
    stop_variable = temp;

    if (!write_reg(0x00, 0x01)) return false;
    if (!write_reg(0xFF, 0x00)) return false;
    if (!write_reg(0x80, 0x00)) return false;

    if (!write_reg(0x44, 0x00)) return false;

    return true;
}


bool VL53L0X::set_address(uint8_t new_addr) {
    if (!write_reg(I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F)) {
        return false;
    }
    this->i2c_addr = new_addr;
    return true;
}

uint16_t VL53L0X::read_range() {
    if (!write_reg(SYSRANGE_START, 0x01)) return 0xFFFF;

    uint8_t status = 0;
    int attempts = 0;
    do {
        if (!read_reg(RESULT_RANGE_STATUS, status)) return 0xFFFF;
        status = status & 0x01;
        sleep_ms(5);
        attempts++;
    } while (status == 0 && attempts < 20);

    uint8_t buf[2] = {0};
    if (!read_multi(0x1E, buf, 2)) return 0xFFFF;

    return (uint16_t)buf[0] << 8 | buf[1];
}


bool VL53L0X::write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    int result = i2c_write_blocking(i2c_bus, i2c_addr, buf, 2, false);
    return result == 2;
}

bool VL53L0X::read_reg(uint8_t reg, uint8_t &value) {
    int result = i2c_write_blocking(i2c_bus, i2c_addr, &reg, 1, true);
    if (result != 1) return false;
    result = i2c_read_blocking(i2c_bus, i2c_addr, &value, 1, false);
    return result == 1;
}


bool VL53L0X::write_multi(uint8_t reg, const uint8_t *data, uint8_t count) {
    if (count > 63) return false;
    uint8_t buf[64];
    buf[0] = reg;
    for (uint8_t i = 0; i < count; ++i) {
        buf[i + 1] = data[i];
    }
    int result = i2c_write_blocking(i2c_bus, i2c_addr, buf, count + 1, false);
    return result == count + 1;
}

bool VL53L0X::read_multi(uint8_t reg, uint8_t *data, uint8_t count) {
    int result = i2c_write_blocking(i2c_bus, i2c_addr, &reg, 1, true);
    if (result != 1) return false;
    result = i2c_read_blocking(i2c_bus, i2c_addr, data, count, false);
    return result == count;
}