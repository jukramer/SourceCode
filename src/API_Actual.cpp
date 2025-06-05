#include "API.h"

#include <stdlib.h>

#include "i2c.h"

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "pins.hpp"
#include "pico/time.h"
#include <math.h>

#include "drivers/vl53l1x.h"
#include "drivers/vl6180x.h"
#include "drivers/driver_mpu6500_fifo.h"

// Motor Constants/Variables
#define TICKS_PER_REV 7.0
#define GEAR_RATIO 25.0

volatile int64_t totalTicksL = 0;
volatile int64_t totalTicksR = 0;

volatile int lastEncodedL = 0;
volatile int lastEncodedR = 0;

void analogWrite(uint gpio, uint level)
{
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(gpio), level);
}

void c1_callback(uint gpio, uint32_t events)
{
    if (events & GPIO_IRQ_EDGE_RISE)
    {
        int MSB = gpio_get(mlencPin); 
        int LSB = gpio_get(mlencPin2);

        int encoded = (MSB << 1) |LSB;
        int sum = (lastEncodedL << 2) | encoded;

        printf("c1_callback: Encoded: %d, Last Encoded: %d, Sum: %d, MSB: %d, LSB: %d\n", 
               encoded, lastEncodedL, sum, MSB, LSB);

        if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) totalTicksL -= 1;
        if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) totalTicksL += 1;

        lastEncodedL = encoded;
    }
}

void c2_callback(uint gpio, uint32_t events)
{
    if (events & GPIO_IRQ_EDGE_RISE)
    {
        int MSB = gpio_get(mrencPin); 
        int LSB = gpio_get(mrencPin2);

        int encoded = (MSB << 1) |LSB;
        int sum = (lastEncodedR << 2) | encoded;

        printf("c2_callback: Encoded: %d, Last Encoded: %d, Sum: %d, MSB: %d, LSB: %d\n", 
               encoded, lastEncodedR, sum, MSB, LSB);

        if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) totalTicksR -= 1;
        if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) totalTicksR += 1;

        lastEncodedR = encoded;
    }
}

Motor::Motor(Motor_Choice choice)
{
    this->choice = choice;

    int pinForward = choice == Motor_Choice::LEFT ? dirA1Pin : dirB1Pin;
    int pinBackward = choice == Motor_Choice::LEFT ? dirA2Pin : dirB2Pin;
    int pinPWM = choice == Motor_Choice::LEFT ? spdAPin : spdBPin;
    int pinENC = choice == Motor_Choice::LEFT ? mlencPin : mrencPin;
    int pinENC2 = choice == Motor_Choice::LEFT ? mlencPin2 : mrencPin2;

    const volatile int64_t *totalTicks = choice == Motor_Choice::LEFT ? &totalTicksL : &totalTicksR;
    gpio_irq_callback_t callback = choice == Motor_Choice::LEFT ? &c1_callback : &c2_callback;

    tPrev = time_us_64();

    this->pinForward = pinForward;
    this->pinBackward = pinBackward;
    this->pinPWM = pinPWM;
    this->pinENC = pinENC;

    this->totalTicks = totalTicks;
    this->prevTicks = *totalTicks;

    gpio_init(pinENC);
    gpio_set_dir(pinENC, GPIO_IN);
    gpio_pull_up(pinENC);

    gpio_init(pinENC2);
    gpio_set_dir(pinENC2, GPIO_IN);
    gpio_pull_up(pinENC2);

    gpio_set_irq_enabled_with_callback(pinENC, GPIO_IRQ_EDGE_RISE, true, callback);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void Motor::setPWM(float pwm)
{
    printf("Setting PWM...\n");

    pwm = fmaxf(fminf(pwm, 100.0f), -100.0f); // Cap PWM to [-100, 100]
    this->PWM = pwm;
    
    int sign = 1;
    if (pwm < 0.0)
    {
        sign = -1; // To flip to positive later
    }

    // Scale PWM from 0 to 100 to 45 to 255
    int actualPWM = sign * (pwm * 215) / 100 + 40;

    if (sign == 1)
    {
        gpio_put(pinForward, 1);
        gpio_put(pinBackward, 0);
    }
    else if (sign == -1)
    {
        gpio_put(pinForward, 0);
        gpio_put(pinBackward, 1);
    }

    if (actualPWM > 255 || actualPWM < 0)
    {
        printf("Invalid PWM calculated...\n");
    }
    analogWrite(pinPWM, actualPWM);
}

void Motor::setPWMRaw(int pwm)
{
    
}

void Motor::update()
{
    uint64_t now = time_us_64();

    float pulses = float(*totalTicks - prevTicks);
    printf("Pulses: %d, ticks: %d, prev ticks: %d ", pulses, *totalTicks, prevTicks);
    prevTicks = *totalTicks;

    float dt_us = (now - tPrev);
    printf("tNow: %lld tPrev: %lld dt: %f\n", now, tPrev, dt_us);

    tPrev = now;

    if (dt_us > 0)
    {
        // float rps = (pulses / TICKS_PER_REV) * (1000000.0f / dt_us) / GEAR_RATIO;
        float rpm = (pulses / TICKS_PER_REV) * (60000000.0f / dt_us) / GEAR_RATIO; // Convert to RPM
        printf("RPM reading is: %f\n", rpm);
        
        this->RPM = rpm;
    } else {
        printf("dt_us is zero or negative, skipping RPM update.\n");
        this->RPM = 0.0f; // Avoid division by zero
    }

    float deltaPos = pulses / TICKS_PER_REV * 2 * M_PI * WHEEL_RADIUS_MM;
    this->DELTA_POS = deltaPos;
}

uint pwm_setup(uint gpio)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_enabled(slice, true);
    pwm_set_wrap(slice, 255); // 8-bit PWM
    return slice;
}

void pico_led_init()
{
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
}

void pico_set_led(bool on)
{
    gpio_put(25, on);
}

Adafruit_VL6180X TOF_XS[4] = {
    Adafruit_VL6180X(),
    Adafruit_VL6180X(),
    Adafruit_VL6180X(),
    Adafruit_VL6180X()};

VL53L1X TOF_XL;

void init_gpio(uint8_t gpio)
{
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_OUT);
    gpio_put(gpio, false); // Set GPIO low to indicate the program has started
    printf("GPIO %d initialized and set to LOW.\n", gpio);
}

void xshut(uint8_t gpio, bool state)
{
    gpio_put(gpio, state);
    printf("  -> GPIO %d set to %s\n", gpio, state ? "HIGH" : "LOW");
    sleep_us(400);
}

void imu_int(uint gpio, uint32_t events)
{
    if (gpio == 21 && (events & GPIO_IRQ_EDGE_FALL))
    {
        printf("IMU interrupt on GPIO %d, events: %u\n", gpio, events);
        mpu6500_fifo_irq_handler();
    }
}

#define MPU_FILTER_DELAY_MS 11.8f
#define MPU_FILTER_DELAY_S (MPU_FILTER_DELAY_MS / 1000.0f)

bool scan_address(i2c_inst_t *i2c, uint8_t addr)
{
    uint8_t buf;
    int ret = i2c_read_timeout_us(i2c, addr, &buf, 1, false, 2000);
    return ret >= 0;
}
void global_init()
{
    stdio_init_all();

    while (!stdio_usb_connected())
    {
        sleep_ms(1000);
    }

    //
    // Turn on LEDs
    //
    gpio_init(22);
    gpio_set_dir(22, GPIO_OUT);
    gpio_put(22, 1); // Set GPIO 22 high to indicate the program has started

    pico_led_init();
    pico_set_led(true);

    //
    // Initialize XSHUT pins for VL6180X sensors
    //
    for (int i = tofXL1Pin; i <= tofXS4Pin; i++)
    {
        init_gpio(i);
    }
    printf("All TOF sensors are now in hardware standby.\n");
    sleep_us(100);

    //
    // Initialize I2C for VL6180X sensors
    //
    gpio_init(tofsdaPin);
    gpio_init(tofsclPin);

    i2c_deinit(I2C_CHANNEL_TOF);
    i2c_init(I2C_CHANNEL_TOF, 400 * 1000);

    gpio_set_function(tofsdaPin, GPIO_FUNC_I2C);
    gpio_set_function(tofsclPin, GPIO_FUNC_I2C);

    gpio_pull_up(tofsdaPin);
    gpio_pull_up(tofsclPin);

    //
    // Initialize I2C for IMU
    //
    gpio_init(imusdaPin);
    gpio_init(imusclPin);

    i2c_deinit(I2C_CHANNEL_IMU);
    i2c_init(I2C_CHANNEL_IMU, 400 * 1000);

    gpio_set_function(imusdaPin, GPIO_FUNC_I2C);
    gpio_set_function(imusclPin, GPIO_FUNC_I2C);

    gpio_pull_up(imusdaPin);
    gpio_pull_up(imusclPin);

    //
    // Initialize IMU
    //
    gpio_init(imuIntPin);
    gpio_set_dir(imuIntPin, GPIO_IN);
    gpio_pull_up(imuIntPin); // optional, depending on your MPU circuit

    gpio_set_irq_enabled_with_callback(imuIntPin, GPIO_IRQ_EDGE_FALL, true, &imu_int);

    if (0 != mpu6500_fifo_init(MPU6500_INTERFACE_IIC, (mpu6500_address_t)0x68)) // Default address for MPU6500
    {
        printf("mpu6500_fifo_init failed.\n");
    }
    else
    {
        printf("mpu6500_fifo_init succeeded.\n");
    }

    //
    // Initialize VL6180X sensors
    //
    int tofSensorIdx = 0;
    int tofSensorAddresses[] = {0x3A, 0x3B, 0x3C, 0x3D, 0x3E}; // Addresses for VL6180X sensors

    for (int gpio = tofXS1Pin; gpio <= tofXS4Pin; gpio++)
    {
        xshut(gpio, true);

        int retryCount = 0;

        while (!TOF_XS[tofSensorIdx].begin())
        {
            sleep_ms(100);

            retryCount++;
            if (retryCount > 3)
            {
                printf("Failed to initialize VL6180X on GPIO %d after 100 retries.\n", gpio);
                xshut(gpio, false); // Turn off the sensor

                break;
            }
        }
        TOF_XS[tofSensorIdx].setAddress(tofSensorAddresses[tofSensorIdx]);
        tofSensorIdx += 1;
    }

    //
    // Initialize VL53L0X sensor
    //
    xshut(tofXL1Pin, true);

    int retryCount = 0;

    while (!TOF_XL.init(true))
    {
        sleep_ms(1);

        retryCount++;
        if (retryCount > 100)
        {
            printf("Failed to initialize VL53L0X after 100 retries.\n");
            break;
        }

        for (uint8_t addr = 0x00; addr <= 0x77; ++addr)
        {
            // printf("  -> 0x%02X\n", addr);
            if (scan_address(I2C_CHANNEL_TOF, addr))
            {
                printf("  -> Found device at 0x%02X\n", addr);
            }
        }
    }
    TOF_XL.setAddress(0x4A);

    gpio_init(dirA1Pin);
    gpio_set_dir(dirA1Pin, GPIO_OUT);
    gpio_init(dirA2Pin);
    gpio_set_dir(dirA2Pin, GPIO_OUT);
    pwm_setup(spdAPin);

    gpio_init(dirB1Pin);
    gpio_set_dir(dirB1Pin, GPIO_OUT);
    gpio_init(dirB2Pin);
    gpio_set_dir(dirB2Pin, GPIO_OUT);
    pwm_setup(spdBPin);

    //
    // Start TOF continuous ranging
    //
    TOF_XL.setDistanceMode(TOF_XL.Short);
    TOF_XL.setMeasurementTimingBudget(30000);
    TOF_XL.startContinuous(1); // Start back-to-back measurements

    for (int i = 0; i < 4; i++)
    {
        TOF_XS[i].startRangeContinuous(50);
    }
}

#define LOG 1

constexpr TOF_Direction INDEX_TO_TOF_DIRECTION[4] = {
    TOF_Direction::RIGHT,
    TOF_Direction::FRONT_RIGHT_45,
    TOF_Direction::FRONT_LEFT_45,
    TOF_Direction::LEFT,
};

void global_read_tofs()
{
    static uint64_t start_time = time_us_64();
    static int counts_XS[4] = {0};
    static int count_XL = 0;

    auto frameBegin = time_us_64();

    for (int i = 0; i < 4; i++)
    {
        int index = (int) INDEX_TO_TOF_DIRECTION[i];
        if (TOF_XS[i].isRangeComplete())
        {
            byte mm = TOF_XS[i].readRangeResult();
            byte status = TOF_XS[i].readRangeStatus();

            MM[index] = mm;
            MM_VALID[index] = (status == VL6180X_ERROR_NONE);

            // Count only if reading is valid
            if (MM_VALID[index])
                counts_XS[i]++;
        }
#if LOG
        printf("%d - Range: %d mm, Valid: %d | ", i, MM[index], MM_VALID[index]);
#endif
    }

    int index = (int) TOF_Direction::FRONT;
    if (TOF_XL.dataReady())
    {
        uint16_t mm = TOF_XL.read(true); // non-blocking read

        MM[index] = mm;
        MM_VALID[index] = (TOF_XL.ranging_data.range_status == VL53L1X::RangeValid);

        if (MM_VALID[index])
            count_XL++;
    }

#if LOG
    printf("XL Distance: %u mm, Valid: %d | ", MM[index], MM_VALID[index]);
    auto frameEnd = time_us_64();
    int64_t frameDuration = frameEnd - frameBegin;
    printf(" Frame Duration: %lld us\n", frameDuration);
#endif

    uint64_t now = time_us_64();
    if (now - start_time >= 2 * 1000 * 1000)
    {
        printf("=== Sensor Frequencies (Hz) over 2 sec ===\n");
        for (int i = 0; i < 4; i++)
            printf("XS[%d]: %.2f Hz | ", i, counts_XS[i] / 2.0);
        printf("XL: %.2f Hz\n", count_XL / 2.0);

        for (int i = 0; i < 4; i++)
            counts_XS[i] = 0;
        count_XL = 0;
        start_time = now;
    }
}

static int16_t gs_accel_raw[128][3];
static float gs_accel_g[128][3];
static int16_t gs_gyro_raw[128][3];
static float gs_gyro_dps[128][3];

void global_read_imu()
{
    static uint64_t t_start = time_us_64();
    static uint64_t t_prev = time_us_64();
    static uint64_t t_end_calib = t_prev + 3000000; // 3s calibration

    static int calib_samples = 0;
    static bool calibrated = false;

    static int imu_sample_count = 0;

    float ax_bias = 0.0f, ay_bias = 0.0f, gyro_z_bias = 0.0f;

    auto t1 = time_us_64();

    uint16_t len = 128;
    mpu6500_fifo_read(gs_accel_raw, gs_accel_g, gs_gyro_raw, gs_gyro_dps, &len);
    if (len == 0)
        return;

    if (len != 1) {
        printf("Warning: IMU usually returns 1 sample, got %d samples.\n", len);
    }

    auto t2 = time_us_64();

    float dt = (t2 - t_prev) / 1e6f;
    t_prev = t2;

    AX = gs_accel_g[0][0];
    AY = gs_accel_g[0][1];
    GYRO_Z = gs_gyro_dps[0][2];

    if (!calibrated)
    {
        ax_bias += AX;
        ay_bias += AY;
        gyro_z_bias += GYRO_Z;

        calib_samples++;

        if (t2 > t_end_calib)
        {
            ax_bias /= calib_samples;
            ay_bias /= calib_samples;
            gyro_z_bias /= calib_samples;
            calibrated = true;
        }
        return;
    }

    AX -= ax_bias;
    AY -= ay_bias;
    GYRO_Z -= gyro_z_bias;

    imu_sample_count++;

#if LOG
    printf("IMU ax: %.2f, ay: %.2f, gyro_z: %.2f, dt: %.4f s, fifo len: %d\n", AX, AY, GYRO_Z, dt, len);
#endif

    if (t2 - t_start >= 2 * 1000 * 1000)
    {
        float hz = imu_sample_count / 2.0f;
        printf("=== IMU Frequency: %.2f Hz ===\n", hz);

        imu_sample_count = 0;
        t_start = t2;
    }
}

void setWall_UI(int x, int y, Direction direction)
{
}