#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>

#define I2C_FREQ 100000

#define I2C_PORT i2c0
#define SDA_PIN 15
#define SCL_PIN 14

#define VL6180_DEFAULT_ADDR 0x29
#define TOF_COUNT 4

const uint XSHUT_PINS[TOF_COUNT] = {17, 18, 19, 20};         // GPIOs for XSHUT
const uint8_t TOF_ADDRESSES[TOF_COUNT] = {0x30, 0x31, 0x32, 0x33}; // New I2C addresses

typedef struct {
    i2c_inst_t *i2c;
    uint8_t sda;
    uint8_t scl;
} i2c_pin_pair;

// i2c_pin_pair i2c_pin_pairs[] = {
//     {i2c0, 0, 1}, {i2c0, 4, 5}, {i2c0, 8, 9}, {i2c0, 12, 13}, {i2c0, 16, 17}, {i2c0, 20, 21},
//     {i2c1, 2, 3}, {i2c1, 6, 7}, {i2c1, 10, 11}, {i2c1, 14, 15}, {i2c1, 18, 19}, {i2c1, 26, 27}
// };

i2c_pin_pair i2c_pin_pairs[] = {
    {i2c1, 15, 14}, //{i2c0, 1, 2}, {i2c0, 6, 7}, {i2c0, 11, 12}, {i2c0, 16, 17}, {i2c0, 21, 22}, {i2c0, 26, 27},
    //{i2c1, 4, 5}, {i2c1, 9, 10}, {i2c1, 15, 14}, {i2c1, 19, 20}, {i2c1, 24, 25}, {i2c1, 31, 32}
};

bool scan_address(i2c_inst_t *i2c, uint8_t addr) {
    uint8_t buf;
    int ret = i2c_read_timeout_us(i2c, addr, &buf, 1, false, 2000);
    return ret >= 0;
}



// Write a single byte to a 16-bit register
bool vl6180_write_reg(uint8_t addr, uint16_t reg, uint8_t value) {
    uint8_t buf[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), value };
    int res = i2c_write_blocking(I2C_PORT, addr, buf, 3, false);
    return res >= 0;
}

// Read a single byte from a 16-bit register
bool vl6180_read_reg(uint8_t addr, uint16_t reg, uint8_t *value) {
    uint8_t reg_buf[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    if (i2c_write_blocking(I2C_PORT, addr, reg_buf, 2, true) < 0) return false;
    return i2c_read_blocking(I2C_PORT, addr, value, 1, false) >= 0;
}

// Change the I2C address of a sensor
bool vl6180_set_address(uint8_t old_addr, uint8_t new_addr) {
    return vl6180_write_reg(old_addr, 0x0212, new_addr);
}

// Boot check: 0x016 == 0x01 means ready
bool vl6180_wait_boot(uint8_t addr) {
    uint8_t status = 0;
    for (int i = 0; i < 100; ++i) {
        if (vl6180_read_reg(addr, 0x016, &status) && status == 0x01)
            return true;
        sleep_ms(5);
    }
    return false;
}

// Start a single range measurement and read result
int vl6180_read_range(uint8_t addr) {
    vl6180_write_reg(addr, 0x018, 0x01);  // SYSRANGE__START = 0x01
    sleep_ms(10);                         // Wait for measurement (usually ~10ms)
    uint8_t range = 0;
    if (vl6180_read_reg(addr, 0x062, &range))
        return range;
    return -1;
}

// Initialize I2C and GPIO
void tof_init_all() {
    i2c_init(I2C_PORT, 100000); // 100 kHz
    gpio_init(SDA_PIN);
    gpio_init(SCL_PIN);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Reset all XSHUT lines
    for (int i = 0; i < TOF_COUNT; ++i) {
        gpio_init(XSHUT_PINS[i]);
        gpio_set_dir(XSHUT_PINS[i], GPIO_OUT);
        gpio_put(XSHUT_PINS[i], 0); // Shut down
    }
    sleep_ms(10);

    for (int i = 0; i < TOF_COUNT; ++i) {
        // Enable only one sensor
        gpio_put(XSHUT_PINS[i], 1);
        sleep_ms(10);

        if (!vl6180_wait_boot(VL6180_DEFAULT_ADDR)) {
            printf("Sensor %d failed to boot\n", i);
            continue;
        }

        if (!vl6180_set_address(VL6180_DEFAULT_ADDR, TOF_ADDRESSES[i])) {
            printf("Sensor %d address change failed\n", i);
            continue;
        }

        printf("Sensor %d set to 0x%02X\n", i, TOF_ADDRESSES[i]);
    }
}


int main() {
    stdio_init_all();
    printf("STDIO Init done...");
    tof_init_all();
    printf("TOF Init done...");

    gpio_init(22);
    gpio_set_dir(22, GPIO_OUT);
    gpio_put(22, 1);

    
    while (!stdio_usb_connected()) {
        sleep_ms(1000);
    }
    printf("Starting I2C scan across all valid pin pairs...\n");
    
    cyw43_arch_init();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    // for (int i = 0; i < sizeof(i2c_pin_pairs)/sizeof(i2c_pin_pair); ++i) {
    //     i2c_pin_pair pair = i2c_pin_pairs[i];
    //     printf("\nTrying I2C%d on SDA=%d, SCL=%d\n", pair.i2c == i2c0 ? 0 : 1, pair.sda, pair.scl);

    //     i2c_deinit(pair.i2c);  // Reset in case it was already used
    //     i2c_init(pair.i2c, I2C_FREQ);
    //     gpio_set_function(pair.sda, GPIO_FUNC_I2C);
    //     gpio_set_function(pair.scl, GPIO_FUNC_I2C);
    //     gpio_pull_up(pair.sda);
    //     gpio_pull_up(pair.scl);

    //     sleep_ms(500);

    //     for (uint8_t addr = 0x00; addr <= 0x77; ++addr) {
    //         printf("  -> 0x%02X\n", addr);
    //         if (scan_address(pair.i2c, addr)) {
    //             printf("  -> Found device at 0x%02X\n", addr);
    //         }
    //     }
    // }

    while (true) {
        for (int i = 0; i < TOF_COUNT; ++i) {
            int range = vl6180_read_range(TOF_ADDRESSES[i]);
            printf("Sensor %d (0x%02X): %d mm\n", i, TOF_ADDRESSES[i], range);
        }
        sleep_ms(500);
    }

    // printf("\nScan complete.\n");
    return 0;
}