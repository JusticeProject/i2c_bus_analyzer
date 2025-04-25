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
#define BUS_ANALYZER_INPUT_BASE_PIN 0

//*************************************************************************************************

void i2c_bus_analyzer_program_init(PIO pio, uint sm, uint offset, uint pin)
{
    pio_sm_config c = i2c_bus_analyzer_program_get_default_config(offset);

    sm_config_set_in_pins(&c, pin);
    sm_config_set_in_shift(&c, false, false, 0);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_gpio_init(pio, pin);
    pio_gpio_init(pio, pin + 1);

    // TODO: not sure why this is necessary, I want high impedance because they are inputs.
    // Without these calls pins 1 and 2 were pulling the i2c bus down to ~1.5V.
    // Do I need gpio_set_dir(pin, GPIO_IN)??
    gpio_pull_up(pin);
    gpio_pull_up(pin + 1);

    int result = pio_sm_set_consecutive_pindirs(pio, sm, pin, 2, false);
    printf("set_consecutive_pindirs returned %d\n", result);

    result = pio_sm_init(pio, sm, offset, &c);
    printf("pio_sm_init returned %d\n", result);
    pio_sm_set_enabled(pio, sm, true);
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
        else if (c == 'a')
        {
            bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&i2c_bus_analyzer_program, &pio, &sm, &offset, BUS_ANALYZER_INPUT_BASE_PIN, 2, true);
            printf("pio_claim returned %d\n", (int)success);
            hard_assert(success);
            i2c_bus_analyzer_program_init(pio, sm, offset, BUS_ANALYZER_INPUT_BASE_PIN);
        }
        else if (c == 's')
        {
            int result = test_signal_read();
            printf("test_signal_read returned %d\n", result);
        }
        else if (c == 't')
        {
            uint level = pio_sm_get_rx_fifo_level(pio, sm);
            printf("pio_sm_get_rx_fifo_level returned %u\n", level);
            if (level > 0)
            {
                uint32_t rx = pio_sm_get(pio, sm);
                printf("pio_sm_get returned %u\n", rx);
            }
        }

        sleep_ms(10);
    }

    // This will free resources and unload our program
    pio_remove_program_and_unclaim_sm(&i2c_bus_analyzer_program, pio, sm, offset);
}
