#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "generate_test_signals.h"

//*************************************************************************************************

#define I2C_PORT i2c1
// GPIO 2,3 is actually pins 4,5 on the pico board
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3

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

int test_signal_read(uint8_t* buffer, size_t len)
{
    int ret = i2c_read_blocking(I2C_PORT, 0x27, buffer, len, false);
    return ret;
}

//*************************************************************************************************

int test_signal_write(void)
{
    uint8_t data = 0xaa;
    int ret = i2c_write_blocking(I2C_PORT, 0x27, &data, 1, false);
    return ret;
}
