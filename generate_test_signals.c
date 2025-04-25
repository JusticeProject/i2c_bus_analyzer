#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "generate_test_signals.h"

//*************************************************************************************************

#define I2C_PORT i2c0
// GPIO 4,5 is actually pins 6,7 on the pico board
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

//*************************************************************************************************

void test_signal_init()
{
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

//*************************************************************************************************

int test_signal_read(void)
{
    uint8_t data;
    int ret = i2c_read_blocking(I2C_PORT, 0x27, &data, 1, false);
    return ret;
}
