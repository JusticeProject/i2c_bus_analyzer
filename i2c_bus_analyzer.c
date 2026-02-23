#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/pio.h"

// auto generated header file
#include "i2c_bus_analyzer.pio.h"

#include "generate_test_signals.h"

//*************************************************************************************************

// NOTE!!!!
// This code works with the PicoW but not the Pico. This is because GPIOs 24 and 25 are
// different between the two and it causes problems in the PIO state machine code when 
// moving all the pins to the x/y registers. Could try switching to in instructions but we
// might start running out of space in the PIO program. Or could try setting GPIOs 24 and
// 25 to outputs and driving them low - would need to check the schematic.
// 
// We will actually use GPIO 0 and 1, which are pins 1,2 on the pico board.
// Connect the I2C device being tested to GPIO 0 (SDA) and GPIO 1 (SCL).
// We also need to connect GPIO 0,1 to another I2C bus on the pico. This is used 
// for some of the debug features but also the PIO now expects them to be connected. Thus:
// Connect a wire from GPIO 0 to GPIO 2. (physical pin 1 to 4)
// Also connect a wire from GPIO 1 to GPIO 3. (physical pin 2 to 5)
// Don't connect anything to the other GPIO pins on the pico.
#define BUS_ANALYZER_SDA_GPIO_PIN 0
#define BUS_ANALYZER_SCL_GPIO_PIN 1

//*************************************************************************************************

// global variables
PIO pio;
uint sm;
uint offset;

queue_t msg_queue;

//*************************************************************************************************

// function declarations
bool self_test();
bool check_byte(uint32_t msg_queue_data, uint8_t system_call_data);

//*************************************************************************************************

void i2c_bus_analyzer_program_init(PIO pio, uint sm, uint offset)
{
    pio_sm_config c = i2c_bus_analyzer_program_get_default_config(offset);

    sm_config_set_in_pins(&c, BUS_ANALYZER_SDA_GPIO_PIN); // set the base for the input pins
    sm_config_set_jmp_pin(&c, BUS_ANALYZER_SDA_GPIO_PIN);
    sm_config_set_in_shift(&c, false, false, 0); // shift left when inputting the data, it's MSB first
    //sm_config_set_in_shift(&c, false, true, 9); // shift left when inputting the data, it's MSB first
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_gpio_init(pio, BUS_ANALYZER_SDA_GPIO_PIN);
    pio_gpio_init(pio, BUS_ANALYZER_SCL_GPIO_PIN);

    // Want high impedance because they are inputs. By default it uses pull-downs, which combined
    // with pull-ups from the i2c devices causes a voltage divider and the bus voltage stays around 1.6V.
    // Do I need gpio_set_dir(pin, GPIO_IN)??
    //gpio_pull_up(pin);
    //gpio_pull_up(pin + 1);
    gpio_disable_pulls(BUS_ANALYZER_SDA_GPIO_PIN);
    gpio_disable_pulls(BUS_ANALYZER_SCL_GPIO_PIN);

    int result = pio_sm_set_consecutive_pindirs(pio, sm, BUS_ANALYZER_SDA_GPIO_PIN, 2, false);
    //printf("set_consecutive_pindirs returned %d\n", result);

    result = pio_sm_init(pio, sm, offset, &c);
    //printf("pio_sm_init returned %d\n", result);
    pio_sm_set_enabled(pio, sm, true);
}

//*************************************************************************************************

void i2c_bus_analyzer_program_restart(PIO pio, uint sm)
{
    uint8_t pc = pio_sm_get_pc(pio, sm);
    printf("Restarting PIO, addr = %u\n", pc);

    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);

    // drain the inter-core queue
    bool data_in_queue = true;
    while (data_in_queue)
    {
        uint32_t dummy_data;
        data_in_queue = queue_try_remove(&msg_queue, &dummy_data);
    }

    pio_sm_restart(pio, sm);

    // jump to the beginning of the program when we restart
    pio_sm_exec(pio, sm, pio_encode_jmp(0));

    pio_sm_set_enabled(pio, sm, true);
    printf("Done restarting the PIO\n");

    pc = pio_sm_get_pc(pio, sm);
    printf("PIO addr = %u\n", pc);
}

//*************************************************************************************************

void send_data_over_usb(uint32_t data)
{
    //printf("pio_sm_get returned 0x%x    ", data);
    
    if (0xfffffff8 == data)
    {
        printf("STOP,,,\n");
    }
    else
    {
        if (data & 0x400)
        {
            uint8_t addr = (data >> 2) & 0x7F;
            char* read_write = (data & 0x2) ? "READ" : "WRITE";
            char* ack_nack = (data & 0x1) ? "NACK" : "ACK";

            printf("START,0x%x,%s,%s\n", addr, read_write, ack_nack);
        }
        else
        {
            uint8_t byte_data = (data >> 1) & 0xFF;
            char* ack_nack = (data & 0x1) ? "NACK" : "ACK";

            printf("BYTE,0x%x,,%s\n", byte_data, ack_nack);
        }
    }
}

//*************************************************************************************************

void core1_entry()
{
    while (true)
    {
        uint32_t rx = pio_sm_get_blocking(pio, sm);
        queue_try_add(&msg_queue, &rx);
    }
}

//*************************************************************************************************

int main() {
    stdio_init_all();

    // This will activate pull-ups, so the i2c bus will be in a known state when we start PIO right after this.
    // If the i2c bus pins are pulled low by default then rise it could cause problems in the PIO state machine logic.
    test_signal_init();

    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(
        &i2c_bus_analyzer_program, &pio, &sm, &offset, BUS_ANALYZER_SDA_GPIO_PIN, 2, true);
    //printf("pio_claim returned success=%d sm=%u offset=%u\n", (int)success, sm, offset);
    hard_assert(success);
    i2c_bus_analyzer_program_init(pio, sm, offset);

    queue_init(&msg_queue, sizeof(uint32_t), 1000);
    multicore_launch_core1(core1_entry);

    // the host PC will send a go command when it is ready to receive data, until that time the data 
    // will accumulate in the inter-core queue
    bool host_pc_is_ready = false;

    while (true)
    {
        // only pull data from the queue if the host PC said so
        if (host_pc_is_ready)
        {
            uint32_t rx;
            bool newData = queue_try_remove(&msg_queue, &rx);
            if (newData)
            {
                send_data_over_usb(rx);
            }
        }

        int c = getchar_timeout_us(0);
        if (PICO_ERROR_TIMEOUT == c)
        {
            // no data received from host PC, back to beginning of loop
            continue;
        }

        // check which command was received from host PC
        if ('g' == c)
        {
            host_pc_is_ready = true;
        }
        else if ('h' == c)
        {
            // h for hold/halt the sending of data to the host PC
            host_pc_is_ready = false;
        }
        else if ('s' == c)
        {
            i2c_bus_analyzer_program_restart(pio, sm);
        }
        else if ('a' == c)
        {
            // scan the i2c bus to look for available devices
            scan_bus();
        }
        else if ('r' == c)
        {
            uint8_t data = 0;
            int result = test_signal_read(&data, 1);
            //printf("test_signal_read returned %d with data = 0x%x\n", result, data);
        }
        else if ('w' == c)
        {
            int result = test_signal_write();
            //printf("test_signal_write returned %d\n", result);
        }
        else if ('e' == c)
        {
            uint8_t buffer[2];
            int ret = test_signal_read_write(buffer, 2);
            //printf("test_signal_read_write returned %d buffer[0] = 0x%x buffer[1] = 0x%x\n", ret, buffer[0], buffer[1]);
        }
        else if ('m' == c)
        {
            // test the MPU6050 IMU
            mpu6050_init();
        }
        else if ('n' == c)
        {
            int16_t acceleration[3], gyro[3], temp;
            mpu6050_read_raw(acceleration, gyro, &temp);
            printf("Temp. = %f\n", (temp / 340.0) + 36.53);
        }
        else if ('t' == c)
        {
            // Self test with a ton of data.
            // To run this:
            // send g cmd to empty the queue
            // then h cmd
            // then t cmd
            bool pass = self_test();
            printf("Self Test result: %s\n", pass ? "PASS" : "FAIL");
        }
        else if ('z' == c)
        {
            // simple debug test of communication with host PC
            printf("hello\n");
        }
    }

    // This will free resources and unload our program
    pio_remove_program_and_unclaim_sm(&i2c_bus_analyzer_program, pio, sm, offset);
}

//*************************************************************************************************

bool check_byte(uint32_t msg_queue_data, uint8_t system_call_data)
{
    uint8_t extracted_data = ((msg_queue_data >> 1) & 0xFF);
    if (extracted_data == system_call_data)
    {
        //printf("pass\n");
        return true;
    }
    else
    {
        printf("fail, extracted_data = 0x%x, system_call_data = 0x%x\n", extracted_data, system_call_data);
        return false;
    }
}

//*************************************************************************************************

bool self_test()
{
    mpu6050_init();

    const int NUM_RAW_READS = 38;
    uint8_t buffer[14 * NUM_RAW_READS];

    for (int i = 0; i < NUM_RAW_READS; i++)
    {
        mpu6050_read_raw_test(buffer + i*14);
    }

    sleep_ms(1);
    printf("before checking data, msg_queue level is at %u\n", queue_get_level(&msg_queue));

    bool pass = true;

    uint32_t msg_queue_data;

    // start bit, address
    queue_remove_blocking(&msg_queue, &msg_queue_data);

    queue_remove_blocking(&msg_queue, &msg_queue_data);
    pass &= check_byte(msg_queue_data, 0x6b);
    
    queue_remove_blocking(&msg_queue, &msg_queue_data);
    pass &= check_byte(msg_queue_data, 0x80);
    
    // stop bit
    queue_remove_blocking(&msg_queue, &msg_queue_data);

    ///////////////////////////////////////////////////////////////////////////

    // start bit, address
    queue_remove_blocking(&msg_queue, &msg_queue_data);

    queue_remove_blocking(&msg_queue, &msg_queue_data);
    pass &= check_byte(msg_queue_data, 0x6b);
    
    queue_remove_blocking(&msg_queue, &msg_queue_data);
    pass &= check_byte(msg_queue_data, 0x0);
    
    // stop bit
    queue_remove_blocking(&msg_queue, &msg_queue_data);

    ///////////////////////////////////////////////////////////////////////////

    int buffer_offset = 0;
    for (int i = 0; i < NUM_RAW_READS; i++)
    {
        // start bit, address
        queue_remove_blocking(&msg_queue, &msg_queue_data);

        // byte
        queue_remove_blocking(&msg_queue, &msg_queue_data);
        pass &= check_byte(msg_queue_data, 0x3b);

        // restart, address
        queue_remove_blocking(&msg_queue, &msg_queue_data);

        // 6 bytes
        for (int j = 0; j < 6; j++)
        {
            queue_remove_blocking(&msg_queue, &msg_queue_data);
            pass &= check_byte(msg_queue_data, buffer[buffer_offset]);
            buffer_offset++;
        }

        // stop
        queue_remove_blocking(&msg_queue, &msg_queue_data);

        ////////////////////////////////////////////

        // start bit, address
        queue_remove_blocking(&msg_queue, &msg_queue_data);

        // byte
        queue_remove_blocking(&msg_queue, &msg_queue_data);
        pass &= check_byte(msg_queue_data, 0x43);

        // restart, address
        queue_remove_blocking(&msg_queue, &msg_queue_data);

        // 6 bytes
        for (int j = 0; j < 6; j++)
        {
            queue_remove_blocking(&msg_queue, &msg_queue_data);
            pass &= check_byte(msg_queue_data, buffer[buffer_offset]);
            buffer_offset++;
        }

        // stop
        queue_remove_blocking(&msg_queue, &msg_queue_data);

        ////////////////////////////////////////////

        // start bit, address
        queue_remove_blocking(&msg_queue, &msg_queue_data);

        // byte
        queue_remove_blocking(&msg_queue, &msg_queue_data);
        pass &= check_byte(msg_queue_data, 0x41);

        // restart, address
        queue_remove_blocking(&msg_queue, &msg_queue_data);

        // 2 bytes
        for (int j = 0; j < 2; j++)
        {
            queue_remove_blocking(&msg_queue, &msg_queue_data);
            pass &= check_byte(msg_queue_data, buffer[buffer_offset]);
            buffer_offset++;
        }

        // stop
        queue_remove_blocking(&msg_queue, &msg_queue_data);
    }

    return pass;
}
