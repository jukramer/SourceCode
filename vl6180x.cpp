#include "vl6180x.h"
#include "pico/stdlib.h"

// Register definitions
#define VL6180X_REG_IDENTIFICATION_MODEL_ID       0x000
#define VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET     0x016
#define VL6180X_REG_SYSRANGE_START                0x018
#define VL6180X_REG_RESULT_RANGE_VAL              0x062
#define VL6180X_REG_RESULT_RANGE_STATUS           0x04d
#define VL6180X_REG_I2C_SLAVE_DEVICE_ADDRESS      0x212

VL6180X::VL6180X() : i2c_bus(nullptr), i2c_addr(0x29) {}

bool VL6180X::init(i2c_inst_t *i2c, uint8_t addr) {
    this->i2c_bus = i2c;
    this->i2c_addr = addr;

    int8_t reset_flag = read_reg(VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET);
    if (reset_flag < 0) {
        return false; // I2C error
    }

    if (reset_flag == 1) {
        load_default_settings();
    }

    if (write_reg(VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET, 0x00) < 0) {
        return false; // Write failed
    }

    return true;  // All OK
}


bool VL6180X::set_address(uint8_t new_addr) {
    if (!write_reg(VL6180X_REG_I2C_SLAVE_DEVICE_ADDRESS, new_addr)) {
        return false;
    };

    this->i2c_addr = new_addr;
    return true;
}

uint8_t VL6180X::read_range() {
    write_reg(VL6180X_REG_SYSRANGE_START, 0x01); // Start single-shot measurement
    sleep_ms(10); // Wait for measurement to complete

    uint8_t status = read_reg(VL6180X_REG_RESULT_RANGE_STATUS) >> 4;
    if (status == 0 || status == 6) { // 0 = valid, 6 = valid but wraparound
        return read_reg(VL6180X_REG_RESULT_RANGE_VAL);
    } else {
        return 255; // Invalid reading
    }
}

bool VL6180X::write_reg(uint16_t reg, uint8_t value) {
    uint8_t buffer[3];
    buffer[0] = (reg >> 8) & 0xFF;
    buffer[1] = reg & 0xFF;
    buffer[2] = value;

    int result = i2c_write_blocking(this->i2c_bus, this->i2c_addr, buffer, 3, false);
    return result == 3;
}


uint8_t VL6180X::read_reg(uint16_t reg) {
    uint8_t addr[2] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF)
    };
    uint8_t value = 0;
    i2c_write_blocking(i2c_bus, i2c_addr, addr, 2, true);
    i2c_read_blocking(i2c_bus, i2c_addr, &value, 1, false);
    return value;
}

// Minimal default settings (derived from ST/Pololu setup)
void VL6180X::load_default_settings() {
    // Required settings (you can expand this if needed)
    write_reg(0x0207, 0x01);
    write_reg(0x0208, 0x01);
    write_reg(0x0096, 0x00);
    write_reg(0x0097, 0xfd);
    write_reg(0x00e3, 0x00);
    write_reg(0x00e4, 0x04);
    write_reg(0x00e5, 0x02);
    write_reg(0x00e6, 0x01);
    write_reg(0x00e7, 0x03);
    write_reg(0x00f5, 0x02);
    write_reg(0x00d9, 0x05);
    write_reg(0x00db, 0xce);
    write_reg(0x00dc, 0x03);
    write_reg(0x00dd, 0xf8);
    write_reg(0x009f, 0x00);
    write_reg(0x00a3, 0x3c);
    write_reg(0x00b7, 0x00);
    write_reg(0x00bb, 0x3c);
    write_reg(0x00b2, 0x09);
    write_reg(0x00ca, 0x09);
    write_reg(0x0198, 0x01);
    write_reg(0x01b0, 0x17);
    write_reg(0x01ad, 0x00);
    write_reg(0x00ff, 0x05);
    write_reg(0x0100, 0x05);
    write_reg(0x0199, 0x05);
    write_reg(0x01a6, 0x1b);
    write_reg(0x01ac, 0x3e);
    write_reg(0x01a7, 0x1f);
    write_reg(0x0030, 0x00);
}

