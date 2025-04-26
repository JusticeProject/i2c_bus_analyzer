#ifndef _GENERATE_TEST_SIGNALS_H_
#define _GENERATE_TEST_SIGNALS_H_

//*************************************************************************************************

void test_signal_init();
int test_signal_read(uint8_t* buffer, size_t len);
int test_signal_write(void);

#endif