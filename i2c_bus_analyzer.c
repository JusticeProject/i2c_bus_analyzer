#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "i2c_bus_analyzer.pio.h"
#include "generate_test_signals.h"

//*************************************************************************************************

// We will actually use GPIO 0 and 1, which are pins 1,2 on the pico board.
// For the debug features connect a wire from GPIO 0 to GPIO 4. (physical pin 1 to 6)
// Also connect a wire from GPIO 1 to GPIO 5. (physical pin 2 to 7)
#define BUS_ANALYZER_INPUT_SDA 0
#define BUS_ANALYZER_INPUT_SCL 1

//*************************************************************************************************

void i2c_bus_analyzer_program_init(PIO pio, uint sm, uint offset)
{
    pio_sm_config c = i2c_bus_analyzer_program_get_default_config(offset);

    sm_config_set_in_pins(&c, BUS_ANALYZER_INPUT_SDA); // set the base for the input pins
    sm_config_set_jmp_pin(&c, BUS_ANALYZER_INPUT_SDA);
    sm_config_set_in_shift(&c, false, false, 0); // shift left when inputting the data, it's MSB first
    //sm_config_set_in_shift(&c, false, true, 9); // shift left when inputting the data, it's MSB first
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_gpio_init(pio, BUS_ANALYZER_INPUT_SDA);
    pio_gpio_init(pio, BUS_ANALYZER_INPUT_SCL);

    // Want high impedance because they are inputs. By default it uses pull-downs, which combined
    // with pull-ups from the i2c devices causes a voltage divider and the bus voltage stays around 1.6V.
    // Do I need gpio_set_dir(pin, GPIO_IN)??
    //gpio_pull_up(pin);
    //gpio_pull_up(pin + 1);
    gpio_disable_pulls(BUS_ANALYZER_INPUT_SDA);
    gpio_disable_pulls(BUS_ANALYZER_INPUT_SCL);

    int result = pio_sm_set_consecutive_pindirs(pio, sm, BUS_ANALYZER_INPUT_SDA, 2, false);
    printf("set_consecutive_pindirs returned %d\n", result);

    result = pio_sm_init(pio, sm, offset, &c);
    printf("pio_sm_init returned %d\n", result);
    pio_sm_set_enabled(pio, sm, true);
}

//*************************************************************************************************

void i2c_bus_analyzer_program_restart(PIO pio, uint sm)
{
    uint8_t pc = pio_sm_get_pc(pio, sm);
    printf("before restart pc = %u\n", pc);

    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);

    // it looks like the program counter doesn't start at 0
    pio_sm_exec(pio, sm, pio_encode_jmp(32 - i2c_bus_analyzer_program.length)); // jump to the beginning of the program when we restart

    pio_sm_set_enabled(pio, sm, true);
    printf("done restarting the PIO\n");

    pc = pio_sm_get_pc(pio, sm);
    printf("after restart pc = %u\n", pc);
}

//*************************************************************************************************

int main() {
    stdio_init_all();

    test_signal_init();

    PIO pio;
    uint sm;
    uint offset;

    while (1)
    {
        int c = getchar_timeout_us(0);
        if (c == 'h')
        {
            printf("hello\n");
        }
        else if (c == 'i')
        {
            bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&i2c_bus_analyzer_program, &pio, &sm, &offset, BUS_ANALYZER_INPUT_SDA, 2, true);
            printf("pio_claim returned %d\n", (int)success);
            hard_assert(success);
            i2c_bus_analyzer_program_init(pio, sm, offset);
        }
        else if (c == 'r')
        {
            int result = test_signal_read();
            printf("test_signal_read returned %d\n", result);
        }
        else if (c == 'w')
        {
            int result = test_signal_write();
            printf("test_signal_write returned %d\n", result);
        }
        else if (c == 'f')
        {
            uint level = pio_sm_get_rx_fifo_level(pio, sm);
            printf("pio_sm_get_rx_fifo_level returned %u\n", level);
            if (level > 0)
            {
                uint32_t rx = pio_sm_get(pio, sm);
                printf("pio_sm_get returned 0x%x\n", rx);
            }
        }
        else if (c == 's')
        {
            i2c_bus_analyzer_program_restart(pio, sm);
        }

        sleep_ms(10);
    }

    // This will free resources and unload our program
    pio_remove_program_and_unclaim_sm(&i2c_bus_analyzer_program, pio, sm, offset);
}
