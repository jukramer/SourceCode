#include <stdio.h>

#include "../i2c.h"

#include "vl6180x.h"

// Read 1 byte from the VL6180X at 'reg'
uint8_t Adafruit_VL6180X::read8(uint16_t reg)
{
    uint8_t buffer[1];
    uint8_t regbuf[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};

    i2c_write_blocking(I2C_CHANNEL_TOF, _i2caddr, regbuf, 2, true); // no stop
    i2c_read_blocking(I2C_CHANNEL_TOF, _i2caddr, buffer, 1, false); // with stop
    return buffer[0];
}

// Read 2 bytes from the VL6180X at 'reg'
uint16_t Adafruit_VL6180X::read16(uint16_t reg)
{
    uint8_t buffer[2];
    uint8_t regbuf[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
    i2c_write_blocking(I2C_CHANNEL_TOF, _i2caddr, regbuf, 2, true); // no stop
    i2c_read_blocking(I2C_CHANNEL_TOF, _i2caddr, buffer, 2, false); // with stop
    return ((uint16_t)buffer[0] << 8) | buffer[1];
}

// Write 1 byte to 'reg'
void Adafruit_VL6180X::write8(uint16_t reg, uint8_t value)
{
    uint8_t buffer[3] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), value};
    i2c_write_blocking(I2C_CHANNEL_TOF, _i2caddr, buffer, 3, false); // with stop
}

// Write 2 bytes to 'reg'
void Adafruit_VL6180X::write16(uint16_t reg, uint16_t value)
{
    uint8_t buffer[4] = {
        (uint8_t)(reg >> 8),
        (uint8_t)(reg & 0xFF),
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF)};
    i2c_write_blocking(I2C_CHANNEL_TOF, _i2caddr, buffer, 4, false); // with stop
}


// Define some additional registers mentioned in application notes and we use
///! period between each measurement when in continuous mode
#define SYSRANGE__INTERMEASUREMENT_PERIOD 0x001b // P19 application notes

Adafruit_VL6180X::~Adafruit_VL6180X()
{
}

/**************************************************************************/
/*!
    @brief  Instantiates a new VL6180X class
    @param  i2caddr Optional initial i2c address of the chip,
   VL6180X_DEFAULT_I2C_ADDR is used by default
*/
/**************************************************************************/
Adafruit_VL6180X::Adafruit_VL6180X(uint8_t i2caddr) : _i2caddr(i2caddr) {}

/**************************************************************************/
/*!
    @brief  Initializes I2C interface, checks that VL6180X is found and resets
   chip.
    @param  theWire Optional pointer to I2C interface, &Wire is used by default
    @returns True if chip found and initialized, False otherwise
*/
/**************************************************************************/
bool Adafruit_VL6180X::begin()
{
    printf("VL6180X: Initializing...\n");

    // check for expected model id
    if (read8(VL6180X_REG_IDENTIFICATION_MODEL_ID) != 0xB4)
    {
        return false;
    }

    // fresh out of reset?
    if (read8(VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET) & 0x01)
    {
        loadSettings();
        write8(VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET, 0x00);
    }

    return true;
}

/**************************************************************************/
/*!
    @brief  Sets the address of the device to a different address.
   chip.
    @param newAddr new I2C address for the device.
    @returns True if write succeeded.
*/
/**************************************************************************/
bool Adafruit_VL6180X::setAddress(uint8_t newAddr)
{
    // BUGBUG - not sure if we detect errors or not...
    // The register is mentioned in app notes.
    write8(VL6180X_REG_SLAVE_DEVICE_ADDRESS, newAddr & 0x7F);
    _i2caddr = newAddr;

    return true;
}

/**************************************************************************/
/*!
    @brief  gets the address of the device
   chip.
    @returns the address
*/
/**************************************************************************/
uint8_t Adafruit_VL6180X::getAddress()
{
    return _i2caddr;
}

/**************************************************************************/
/*!
    @brief  Load the settings for proximity/distance ranging
*/
/**************************************************************************/

void Adafruit_VL6180X::loadSettings()
{
    // load settings!

    // private settings from page 24 of app note
    write8(0x0207, 0x01);
    write8(0x0208, 0x01);
    write8(0x0096, 0x00);
    write8(0x0097, 0xfd);
    write8(0x00e3, 0x00);
    write8(0x00e4, 0x04);
    write8(0x00e5, 0x02);
    write8(0x00e6, 0x01);
    write8(0x00e7, 0x03);
    write8(0x00f5, 0x02);
    write8(0x00d9, 0x05);
    write8(0x00db, 0xce);
    write8(0x00dc, 0x03);
    write8(0x00dd, 0xf8);
    write8(0x009f, 0x00);
    write8(0x00a3, 0x3c);
    write8(0x00b7, 0x00);
    write8(0x00bb, 0x3c);
    write8(0x00b2, 0x09);
    write8(0x00ca, 0x09);
    write8(0x0198, 0x01);
    write8(0x01b0, 0x17);
    write8(0x01ad, 0x00);
    write8(0x00ff, 0x05);
    write8(0x0100, 0x05);
    write8(0x0199, 0x05);
    write8(0x01a6, 0x1b);
    write8(0x01ac, 0x3e);
    write8(0x01a7, 0x1f);
    write8(0x0030, 0x00);

    // Recommended : Public registers - See data sheet for more detail
    write8(0x0011, 0x10); // Enables polling for 'New Sample ready'
                          // when measurement completes
    write8(0x010a, 0x30); // Set the averaging sample period
                          // (compromise between lower noise and
                          // increased execution time)
    write8(0x003f, 0x46); // Sets the light and dark gain (upper
                          // nibble). Dark gain should not be
                          // changed.
    write8(0x0031, 0xFF); // sets the # of range measurements after
                          // which auto calibration of system is
                          // performed
    write8(0x0041, 0x63); // Set ALS integration time to 100ms
    write8(0x002e, 0x01); // perform a single temperature calibration
                          // of the ranging sensor

    // Optional: Public registers - See data sheet for more detail
    write8(SYSRANGE__INTERMEASUREMENT_PERIOD,
           0x09);         // Set default ranging inter-measurement
                          // period to 100ms
    write8(0x003e, 0x31); // Set default ALS inter-measurement period
                          // to 500ms
    write8(0x0014, 0x24); // Configures interrupt on 'New Sample
                          // Ready threshold event'
}

/**************************************************************************/
/*!
    @brief  Single shot ranging. Be sure to check the return of {@link
   readRangeStatus} to before using the return value!
    @return Distance in millimeters if valid
*/
/**************************************************************************/

uint8_t Adafruit_VL6180X::readRange()
{
    // wait for device to be ready for range measurement
    while (!(read8(VL6180X_REG_RESULT_RANGE_STATUS) & 0x01))
        ;

    // Start a range measurement
    write8(VL6180X_REG_SYSRANGE_START, 0x01);

    // Poll until bit 2 is set
    while (!(read8(VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) & 0x04))
        ;

    // read range in mm
    uint8_t range = read8(VL6180X_REG_RESULT_RANGE_VAL);

    // clear interrupt
    write8(VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07);

    return range;
}

/**************************************************************************/
/*!
    @brief  start Single shot ranging. The caller of this should have code
    that waits until the read completes, by either calling
    {@link waitRangeComplete} or calling {@link isRangeComplete} until it
    returns true.  And then the code should call {@link readRangeResult}
    to retrieve the range value and clear out the internal status.
    @return true if range completed.
*/
/**************************************************************************/

bool Adafruit_VL6180X::startRange()
{
    // wait for device to be ready for range measurement
    while (!(read8(VL6180X_REG_RESULT_RANGE_STATUS) & 0x01))
        ;

    // Start a range measurement
    write8(VL6180X_REG_SYSRANGE_START, 0x01);

    return true;
}

/**************************************************************************/
/*!
    @brief  Check to see if the range command completed.
    @return true if range completed.
*/
/**************************************************************************/

bool Adafruit_VL6180X::isRangeComplete()
{

    // Poll until bit 2 is set
    if ((read8(VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) & 0x04))
        return true;

    return false;
}

/**************************************************************************/
/*!
    @brief  Wait until Range completed
    @return true if range completed.
*/
/**************************************************************************/

bool Adafruit_VL6180X::waitRangeComplete()
{

    // Poll until bit 2 is set
    while (!(read8(VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) & 0x04))
        ;

    return true;
}

/**************************************************************************/
/*!
    @brief  Return results of read reqyest also clears out the interrupt
    Be sure to check the return of {@link readRangeStatus} to before using
    the return value!
    @return if range started.
*/
/**************************************************************************/

uint8_t Adafruit_VL6180X::readRangeResult()
{

    // read range in mm
    uint8_t range = read8(VL6180X_REG_RESULT_RANGE_VAL);

    // clear interrupt
    write8(VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07);

    return range;
}

/**************************************************************************/
/*!
    @brief  Start continuous ranging
    @param  period_ms Optional Period between ranges in ms.  Values will
    be rounded down to 10ms units with minimum of 10ms.  Default is 50
*/
/**************************************************************************/

void Adafruit_VL6180X::startRangeContinuous(uint16_t period_ms)
{
    uint8_t period_reg = 0;
    if (period_ms > 10)
    {
        if (period_ms < 2550)
            period_reg = (period_ms / 10) - 1;
        else
            period_reg = 254;
    }
    // Set  ranging inter-measurement
    write8(SYSRANGE__INTERMEASUREMENT_PERIOD, period_reg);

    // Start a continuous range measurement
    write8(VL6180X_REG_SYSRANGE_START, 0x03);
}

/**************************************************************************/
/*!
    @brief stop continuous range operation.
*/
/**************************************************************************/

void Adafruit_VL6180X::stopRangeContinuous()
{
    // stop the continuous range operation, by setting the range register
    // back to 1, Page 7 of appication notes
    write8(VL6180X_REG_SYSRANGE_START, 0x01);
}

/**************************************************************************/
/*!
    @brief  Request ranging success/error message (retreive after ranging)
    @returns One of possible VL6180X_ERROR_* values
*/
/**************************************************************************/

uint8_t Adafruit_VL6180X::readRangeStatus()
{
    return (read8(VL6180X_REG_RESULT_RANGE_STATUS) >> 4);
}

void Adafruit_VL6180X::checkRangeStatus()
{
    uint8_t status = readRangeStatus();

    if ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5))
    {
        printf("System error");
    }
    else if (status == VL6180X_ERROR_ECEFAIL)
    {
        printf("ECE failure");
    }
    else if (status == VL6180X_ERROR_NOCONVERGE)
    {
        printf("No convergence");
    }
    else if (status == VL6180X_ERROR_RANGEIGNORE)
    {
        printf("Ignoring range");
    }
    else if (status == VL6180X_ERROR_SNR)
    {
        printf("Signal/noise error");
    }
    else if (status == VL6180X_ERROR_RAWUFLOW)
    {
        printf("Raw reading underflow");
    }
    else if (status == VL6180X_ERROR_RAWOFLOW)
    {
        printf("Raw reading overflow");
    }
    else if (status == VL6180X_ERROR_RANGEUFLOW)
    {
        printf("Range reading underflow");
    }
    else if (status == VL6180X_ERROR_RANGEOFLOW)
    {
        printf("Range reading overflow");
    }
}

/**************************************************************************/
/*!
    @brief  Single shot lux measurement
    @param  gain Gain setting, one of VL6180X_ALS_GAIN_*
    @returns Lux reading
*/
/**************************************************************************/

float Adafruit_VL6180X::readLux(uint8_t gain)
{
    uint8_t reg;

    reg = read8(VL6180X_REG_SYSTEM_INTERRUPT_CONFIG);
    reg &= ~0x38;
    reg |= (0x4 << 3); // IRQ on ALS ready
    write8(VL6180X_REG_SYSTEM_INTERRUPT_CONFIG, reg);

    // 100 ms integration period
    write8(VL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI, 0);
    write8(VL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO, 100);

    // analog gain
    if (gain > VL6180X_ALS_GAIN_40)
    {
        gain = VL6180X_ALS_GAIN_40;
    }
    write8(VL6180X_REG_SYSALS_ANALOGUE_GAIN, 0x40 | gain);

    // start ALS
    write8(VL6180X_REG_SYSALS_START, 0x1);

    // Poll until "New Sample Ready threshold event" is set
    while (4 != ((read8(VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) >> 3) & 0x7))
        ;

    // read lux!
    float lux = read16(VL6180X_REG_RESULT_ALS_VAL);

    // clear interrupt
    write8(VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07);

    lux *= 0.32; // calibrated count/lux
    switch (gain)
    {
    case VL6180X_ALS_GAIN_1:
        break;
    case VL6180X_ALS_GAIN_1_25:
        lux /= 1.25;
        break;
    case VL6180X_ALS_GAIN_1_67:
        lux /= 1.67;
        break;
    case VL6180X_ALS_GAIN_2_5:
        lux /= 2.5;
        break;
    case VL6180X_ALS_GAIN_5:
        lux /= 5;
        break;
    case VL6180X_ALS_GAIN_10:
        lux /= 10;
        break;
    case VL6180X_ALS_GAIN_20:
        lux /= 20;
        break;
    case VL6180X_ALS_GAIN_40:
        lux /= 40;
        break;
    }
    lux *= 100;
    lux /= 100; // integration time in ms

    return lux;
}

/**************************************************************************/
/*!
    @brief  Set the offset
    @param  offset Offset setting
*/
/**************************************************************************/

void Adafruit_VL6180X::setOffset(uint8_t offset)
{
    // write the offset
    write8(VL6180X_REG_SYSRANGE_PART_TO_PART_RANGE_OFFSET, offset);
}

/**************************************************************************/
/*!
    @brief  Get the 7 bytes of id
    @param  id_ptr Pointer to array of id bytes
*/
/**************************************************************************/

void Adafruit_VL6180X::getID(uint8_t *id_ptr)
{

    id_ptr[0] = read8(VL6180X_REG_IDENTIFICATION_MODEL_ID + 0);
    id_ptr[1] = read8(VL6180X_REG_IDENTIFICATION_MODEL_ID + 1);
    id_ptr[2] = read8(VL6180X_REG_IDENTIFICATION_MODEL_ID + 2);
    id_ptr[3] = read8(VL6180X_REG_IDENTIFICATION_MODEL_ID + 3);
    id_ptr[4] = read8(VL6180X_REG_IDENTIFICATION_MODEL_ID + 4);
    id_ptr[6] = read8(VL6180X_REG_IDENTIFICATION_MODEL_ID + 6);
    id_ptr[7] = read8(VL6180X_REG_IDENTIFICATION_MODEL_ID + 7);
}
