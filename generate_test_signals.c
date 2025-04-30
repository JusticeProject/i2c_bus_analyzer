#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "generate_test_signals.h"

//*************************************************************************************************

#define I2C_PORT i2c1
// GPIO 2,3 is actually pins 4,5 on the pico board
#define I2C_SDA_GPIO_PIN 2
#define I2C_SCL_GPIO_PIN 3

#define MPU6050_ADDR 0x68

//*************************************************************************************************

void test_signal_init()
{
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    gpio_set_function(I2C_SDA_GPIO_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO_PIN);
    gpio_pull_up(I2C_SCL_GPIO_PIN);
}

//*************************************************************************************************

int test_signal_read(uint8_t* buffer, size_t len)
{
    int ret = i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, len, false);
    return ret;
}

//*************************************************************************************************

int test_signal_write(void)
{
    uint8_t data = 0xaa;
    int ret = i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &data, 1, false);
    return ret;
}

//*************************************************************************************************

int test_signal_read_write(uint8_t buffer[], size_t len)
{
    // Start reading acceleration registers from register 0x3B for 2 bytes
    uint8_t val = 0x3B;
    int ret = i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &val, 1, true); // true to keep master control of bus
    ret += i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, len, false);
    return ret;
}

//*************************************************************************************************

void mpu6050_init() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
    sleep_ms(100); // Allow device to reset and stabilize

    // Clear sleep mode (0x6B register, 0x00 value)
    buf[1] = 0x00;  // Clear sleep mode by writing 0x00 to the 0x6B register
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false); 
    sleep_ms(10); // Allow stabilization after waking up
}

//*************************************************************************************************

void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 6, false);

    printf("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &val, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 6, false);  // False - finished with bus

    printf("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &val, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 2, false);  // False - finished with bus

    printf("0x%x 0x%x\n", buffer[0], buffer[1]);

    *temp = buffer[0] << 8 | buffer[1];
}

//*************************************************************************************************

void mpu6050_read_raw_test(uint8_t buffer[14])
{
    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 6, false);

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &val, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer + 6, 6, false);  // False - finished with bus

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &val, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer + 12, 2, false);  // False - finished with bus
}

//*************************************************************************************************

bool reserved_addr(uint8_t addr)
{
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

//*************************************************************************************************

void scan_bus()
{
    printf("\nI2C Bus Scan\n");
    printf("@ = device found\n");
    printf(". = no device found\n");
    printf("  = not scanned (reserved address)\n\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr)
    {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a peripheral
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        if (reserved_addr(addr))
        {
            printf(" ");
        }
        else
        {
            uint8_t rxdata;
            int ret = i2c_read_blocking(I2C_PORT, addr, &rxdata, 1, false);
            printf(ret < 0 ? "." : "@");
        }

        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n");
}
