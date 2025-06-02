#include "i2c.h"

#include "vl53l0x.h"
#include "vl6180x.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/cyw43_arch.h"

#include "driver_mpu6500_fifo.h"

#include "IMU_EKF/ESKF.h"

using Eigen::Vector3f;

int pico_led_init()
{
    return cyw43_arch_init();
}

void pico_set_led(bool on)
{
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
}

Adafruit_VL6180X TOF_XS[4] = {
    Adafruit_VL6180X(),
    Adafruit_VL6180X(),
    Adafruit_VL6180X(),
    Adafruit_VL6180X()};

VL53L0X TOF_XL;

void init_gpio(uint8_t gpio)
{
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_OUT);
    gpio_put(gpio, false); // Set GPIO low to indicate the program has started
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

static int16_t gs_accel_raw[128][3];
static float gs_accel_g[128][3];
static int16_t gs_gyro_raw[128][3];
static float gs_gyro_dps[128][3];

IMU_EKF::ESKF<float> filter;

#define MPU_FILTER_DELAY_MS 11.8f
#define MPU_FILTER_DELAY_S  (MPU_FILTER_DELAY_MS / 1000.0f)

int main()
{
    stdio_init_all();

    while (!stdio_usb_connected())
    {
        sleep_ms(1000);
    }

    //
    // Turn on LEDs
    //
    // gpio_init(22);
    // gpio_set_dir(22, GPIO_OUT);
    // gpio_put(22, 1); // Set GPIO 22 high to indicate the program has started

    pico_led_init();
    pico_set_led(true);

#if 0
    printf("Starting I2C scan across all valid pin pairs...\n");

    gpio_init(17); // GPIO 17 for first VL6180X sensor
    gpio_set_dir(17, GPIO_OUT);
    gpio_put(17, true); // Enable first VL6180X sensor

    for (int i = 0; i < sizeof(i2c_pin_pairs) / sizeof(i2c_pin_pair); ++i)
    {
        i2c_pin_pair pair = i2c_pin_pairs[i];
        printf("\nTrying I2C%d on SDA=%d, SCL=%d\n", pair.i2c == i2c0 ? 0 : 1, pair.sda, pair.scl);

        i2c_deinit(pair.i2c); // Reset in case it was already used
        i2c_init(pair.i2c, 100000);
        gpio_set_function(pair.sda, GPIO_FUNC_I2C);
        gpio_set_function(pair.scl, GPIO_FUNC_I2C);
        gpio_pull_up(pair.sda);
        gpio_pull_up(pair.scl);

        sleep_ms(500);

        for (uint8_t addr = 0x00; addr <= 0x77; ++addr)
        {
            // printf("  -> 0x%02X\n", addr);
            if (scan_address(pair.i2c, addr))
            {
                printf("  -> Found device at 0x%02X\n", addr);
            }
        }
    }
    return 0;
#endif

    //
    // Initialize XSHUT pins for VL6180X sensors
    //
    for (int i = 16; i <= 20; i++)
    {
        init_gpio(i);
    }
    printf("All TOF sensors are now in hardware standby.\n");
    sleep_us(100);

    //
    // Initialize I2C for VL6180X sensors
    //
    gpio_init(GPIO_TOF_SDA);
    gpio_init(GPIO_TOF_SCL);

    i2c_deinit(I2C_CHANNEL_TOF);
    i2c_init(I2C_CHANNEL_TOF, 400 * 1000);

    gpio_set_function(GPIO_TOF_SDA, GPIO_FUNC_I2C);
    gpio_set_function(GPIO_TOF_SCL, GPIO_FUNC_I2C);

    gpio_pull_up(GPIO_TOF_SDA);
    gpio_pull_up(GPIO_TOF_SCL);

    //
    // Initialize I2C for IMU
    //
    gpio_init(GPIO_IMU_SDA);
    gpio_init(GPIO_IMU_SCL);

    i2c_deinit(I2C_CHANNEL_IMU);
    i2c_init(I2C_CHANNEL_IMU, 400 * 1000);

    gpio_set_function(GPIO_IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(GPIO_IMU_SCL, GPIO_FUNC_I2C);

    gpio_pull_up(GPIO_IMU_SDA);
    gpio_pull_up(GPIO_IMU_SCL);

//
// Initialize IMU
//
#define GPIO_MPU6500_INT 21

    gpio_init(GPIO_MPU6500_INT);
    gpio_set_dir(GPIO_MPU6500_INT, GPIO_IN);
    gpio_pull_up(GPIO_MPU6500_INT); // optional, depending on your MPU circuit

    gpio_set_irq_enabled_with_callback(GPIO_MPU6500_INT,
                                       GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &imu_int);

    if (0 != mpu6500_fifo_init(MPU6500_INTERFACE_IIC, (mpu6500_address_t)0x68)) // Default address for MPU6500
    {
        printf("mpu6500_fifo_init failed.\n");
    }

    //
    // Initialize VL6180X sensors
    //
    int tofSensorIdx = 0;
    int tofSensorAddresses[] = {0x2A, 0x2B, 0x2C, 0x2D, 0x2E}; // Addresses for VL6180X sensors

    for (int gpio = 17; gpio <= 20; gpio++)
    {
        xshut(gpio, true);

        int retryCount = 0;

        while (!TOF_XS[tofSensorIdx].begin())
        {
            sleep_ms(1);

            retryCount++;
            if (retryCount > 100)
            {
                printf("Failed to initialize VL6180X on GPIO %d after 100 retries.\n", gpio);
                break;
            }
        }
        TOF_XS[tofSensorIdx].setAddress(tofSensorAddresses[tofSensorIdx]);
        tofSensorIdx += 1;
    }

    //
    // Initialize VL53L0X sensor
    //
    /*xshut(16, true);

    sleep_ms(50);
    for (uint8_t addr = 0x00; addr <= 0x77; ++addr)
        {
            // printf("  -> 0x%02X\n", addr);
            if (scan_address(I2C_CHANNEL_TOF, addr))
            {
                printf("  -> Found device at 0x%02X\n", addr);
            }
        }

    int retryCount = 0;
    while (!TOF_XL.init(false))
    {
        sleep_ms(1);

        retryCount++;
        if (retryCount > 100)
        {
            printf("Failed to initialize VL53L0X after 100 retries.\n");
            break;
        }
    }*/

    //
    // Main loop
    //

    // TOF_XL.startContinuous(50); // Start continuous ranging with 50ms period

    for (int i = 0; i < 4; i++)
    {
        TOF_XS[i].startRangeContinuous(50);
    }

    /**
     *         for (int i = 0; i < 0; i++)
        {
            if (TOF_XS[i].isRangeComplete())
            {
                mm[i] = TOF_XS[i].readRangeResult();
            }

            // float lux = TOF_XS[i].readLux(VL6180X_ALS_GAIN_5);
            // uint8_t range = TOF_XS[i].readRange();
            // uint8_t status = TOF_XS[i].readRangeStatus();
            // printf("Sensor %d - Lux: %.2f, Range: %d mm, Status: %d, ", i, lux, range, status);

            // printf("%d: Range: %d mm | ", i, mm[i]);
        }

                /*if (TOF_XL.readRangeContinuousMillimeters())
        {
            mmXL = TOF_XL.readRangeContinuousMillimeters();
            printf("XL Range: %d mm ", (int)mmXL);
        }*/
    
    uint8_t mm[4] = {};
    float mmXL = 0;

    auto t_prev = time_us_64();
    auto t_end_calib = t_prev + 3000000; // 3s calibration

    int calib_samples = 0;
    bool calibrated = false;

    filter.initWithAcc(0.0f, 0.0f, 0.0f);
    Vector3f accel_bias, gyro_bias;

    while (true)
    {
        auto t1 = time_us_64();

        uint16_t len = 128;
        mpu6500_fifo_read(gs_accel_raw, gs_accel_g, gs_gyro_raw, gs_gyro_dps, &len);
        auto t2 = time_us_64();

        float dt = (t2 - t_prev) / 1e6f;
        t_prev = t2;

        Vector3f accel_raw(gs_accel_g[0][0], gs_accel_g[0][1], gs_accel_g[0][2]);
        Vector3f gyro_raw(gs_gyro_dps[0][0], gs_gyro_dps[0][1], gs_gyro_dps[0][2]);

        if (!calibrated)
        {
            accel_raw[2] -= 1.0f; // Remove gravity from Z axis

            accel_bias += accel_raw;
            gyro_bias += gyro_raw;
            calib_samples++;

            if (t2 > t_end_calib)
            {
                accel_bias *= (1.0f / calib_samples);
                gyro_bias *= (1.0f / calib_samples);
                calibrated = true;
            }

            continue;
        }

        accel_raw -= accel_bias;
        gyro_raw -= gyro_bias;

        filter.predict(dt);
        filter.correctGyr(gyro_raw[0], gyro_raw[1], gyro_raw[2]);
        filter.correctAcc(accel_raw[0], accel_raw[1], accel_raw[2]);

        float roll, pitch, yaw;
        filter.getAttitude(roll, pitch, yaw);

        printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f | "
               "Accel: [%.2f, %.2f, %.2f] m/s^2 | "
               "Gyro: [%.2f, %.2f, %.2f] rad/s | "
               "dt: %.3f s\n",
               roll * (180.0 / M_PI), pitch * (180.0 / M_PI), yaw * (180.0 / M_PI),
               accel_raw[0], accel_raw[1], accel_raw[2],
               gyro_raw[0], gyro_raw[1], gyro_raw[2],
               dt);

    }

    return 0;
}