#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>

#define I2C_FREQ 100000

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
    // {i2c0, 1, 2}, {i2c0, 6, 7}, {i2c0, 11, 12}, {i2c0, 16, 17}, {i2c0, 21, 22}, {i2c0, 26, 27},
    // {i2c1, 4, 5}, {i2c1, 9, 10}, {i2c1, 14, 15}, {i2c1, 19, 20}, {i2c1, 24, 25}, {i2c1, 31, 32}
    {i2c0, 14, 15}, {i2c1, 14, 15}, {i2c0, 15, 14}, {i2c1, 15, 14}
};

bool scan_address(i2c_inst_t *i2c, uint8_t addr) {
    uint8_t buf;
    int ret = i2c_read_timeout_us(i2c, addr, &buf, 1, false, 2000);
    return ret >= 0;
}

int main() {
    stdio_init_all();

    gpio_init(14);
    gpio_init(15);
    gpio_init(17);
    gpio_set_dir(17, GPIO_OUT);
    gpio_put(17, 1);
    
    while (!stdio_usb_connected()) {
        sleep_ms(1000);
    }
    printf("Starting I2C scan across all valid pin pairs...\n");

    for (int i = 0; i < sizeof(i2c_pin_pairs)/sizeof(i2c_pin_pair); ++i) {
        i2c_pin_pair pair = i2c_pin_pairs[i];
        printf("\nTrying I2C%d on SDA=%d, SCL=%d\n", pair.i2c == i2c0 ? 0 : 1, pair.sda, pair.scl);

        i2c_deinit(pair.i2c);  // Reset in case it was already used
        i2c_init(pair.i2c, I2C_FREQ);
        gpio_set_function(pair.sda, GPIO_FUNC_I2C);
        gpio_set_function(pair.scl, GPIO_FUNC_I2C);
        gpio_pull_up(pair.sda);
        gpio_pull_up(pair.scl);

        sleep_ms(500);

        for (uint8_t addr = 0x00; addr <= 0x77; ++addr) {
            // printf("  -> 0x%02X\n", addr);
            if (scan_address(pair.i2c, addr)) {
                printf("  -> Found device at 0x%02X\n", addr);
            }
        }
    }

    printf("\nScan complete.\n");
    return 0;
}