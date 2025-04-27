#ifndef _GENERATE_TEST_SIGNALS_H_
#define _GENERATE_TEST_SIGNALS_H_

//*************************************************************************************************

void test_signal_init();
int test_signal_read(uint8_t* buffer, size_t len);
int test_signal_write(void);
int test_signal_read_write(uint8_t buffer[], size_t len);

void mpu6050_init();
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);

#endif