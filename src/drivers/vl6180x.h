#pragma once

#include <stdbool.h>
#include <stdint.h>

#define VL6180X_DEFAULT_I2C_ADDR 0x29 ///< The fixed I2C addres

///! Device model identification number
#define VL6180X_REG_IDENTIFICATION_MODEL_ID 0x000
///! Interrupt configuration
#define VL6180X_REG_SYSTEM_INTERRUPT_CONFIG 0x014
///! Interrupt clear bits
#define VL6180X_REG_SYSTEM_INTERRUPT_CLEAR 0x015
///! Fresh out of reset bit
#define VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET 0x016
///! Trigger Ranging
#define VL6180X_REG_SYSRANGE_START 0x018
///! Part to part range offset
#define VL6180X_REG_SYSRANGE_PART_TO_PART_RANGE_OFFSET 0x024
///! Trigger Lux Reading
#define VL6180X_REG_SYSALS_START 0x038
///! Lux reading gain
#define VL6180X_REG_SYSALS_ANALOGUE_GAIN 0x03F
///! Integration period for ALS mode, high byte
#define VL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI 0x040
///! Integration period for ALS mode, low byte
#define VL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO 0x041
///! Specific error codes
#define VL6180X_REG_RESULT_RANGE_STATUS 0x04d
///! Interrupt status
#define VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO 0x04f
///! Light reading value
#define VL6180X_REG_RESULT_ALS_VAL 0x050
///! Ranging reading value
#define VL6180X_REG_RESULT_RANGE_VAL 0x062
///! I2C Slave Device Address
#define VL6180X_REG_SLAVE_DEVICE_ADDRESS 0x212

#define VL6180X_ALS_GAIN_1 0x06    ///< 1x gain
#define VL6180X_ALS_GAIN_1_25 0x05 ///< 1.25x gain
#define VL6180X_ALS_GAIN_1_67 0x04 ///< 1.67x gain
#define VL6180X_ALS_GAIN_2_5 0x03  ///< 2.5x gain
#define VL6180X_ALS_GAIN_5 0x02    ///< 5x gain
#define VL6180X_ALS_GAIN_10 0x01   ///< 10x gain
#define VL6180X_ALS_GAIN_20 0x00   ///< 20x gain
#define VL6180X_ALS_GAIN_40 0x07   ///< 40x gain

#define VL6180X_ERROR_NONE 0        ///< Success!
#define VL6180X_ERROR_SYSERR_1 1    ///< System error
#define VL6180X_ERROR_SYSERR_5 5    ///< Sysem error
#define VL6180X_ERROR_ECEFAIL 6     ///< Early convergence estimate fail
#define VL6180X_ERROR_NOCONVERGE 7  ///< No target detected
#define VL6180X_ERROR_RANGEIGNORE 8 ///< Ignore threshold check failed
#define VL6180X_ERROR_SNR 11        ///< Ambient conditions too high
#define VL6180X_ERROR_RAWUFLOW 12   ///< Raw range algo underflow
#define VL6180X_ERROR_RAWOFLOW 13   ///< Raw range algo overflow
#define VL6180X_ERROR_RANGEUFLOW 14 ///< Raw range algo underflow
#define VL6180X_ERROR_RANGEOFLOW 15 ///< Raw range algo overflow

///! Class for managing connection and state to a VL6180X sensor
class Adafruit_VL6180X
{
public:
    Adafruit_VL6180X(uint8_t i2caddr = VL6180X_DEFAULT_I2C_ADDR);
    ~Adafruit_VL6180X();

    bool begin();
    bool setAddress(uint8_t newAddr);
    uint8_t getAddress();

    uint8_t readRange();
    float readLux(uint8_t gain);
    uint8_t readRangeStatus();
    void checkRangeStatus();

    bool startRange();
    bool isRangeComplete();
    bool waitRangeComplete();
    uint8_t readRangeResult();

    // readRangeResult and isRangeComplete apply here is well
    void startRangeContinuous(uint16_t period_ms = 50);
    void stopRangeContinuous();

    void setOffset(uint8_t offset);
    void getID(uint8_t *id_ptr);

private:
    void loadSettings();

    uint8_t _i2caddr; 

    uint8_t read8(uint16_t reg);
    uint16_t read16(uint16_t reg);
    void write8(uint16_t reg, uint8_t value);
    void write16(uint16_t reg, uint16_t value);
};