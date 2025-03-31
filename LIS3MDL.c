#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define DRDY_PIN 3
#define SLAVE_ADDRESS 0x1C
#define WHO_AM_I_REG 0x0F
#define CTRL_REG1 0x20
#define OUT_X_L 0x28

volatile bool data_ready = false;

void gpio_irq_handler(uint gpio, uint32_t event_mask)
{
    data_ready = true;
}

int i2c_write_register(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    return i2c_write_timeout_us(
        i2c_get_instance(PICO_DEFAULT_I2C),
        SLAVE_ADDRESS,
        buffer,
        sizeof(buffer) / sizeof(buffer[0]),
        false,
        1000 * 500);
}

int i2c_read_register(uint8_t reg)
{
    uint8_t value;
    int write = i2c_write_timeout_us(i2c_get_instance(PICO_DEFAULT_I2C), SLAVE_ADDRESS, &reg, sizeof(reg), true, 1000 * 500);
    if (write < 0)
    {
        return write;
    }
    int read = i2c_read_timeout_us(i2c_get_instance(PICO_DEFAULT_I2C), SLAVE_ADDRESS, &value, sizeof(value), true, 1000 * 500);
    if (read < 0)
    {
        return read;
    }
    return value;
}

void lis3mdl_init()
{
    uint8_t who_am_i = i2c_read_register(WHO_AM_I_REG);

    if (who_am_i == 0x3D)
    {
        printf("LIS3MDL detected!\n");
    }
    else
    {
        printf("LIS3MDL not detected! ID: 0x%X\n", who_am_i);
    }

    // Configure LIS3MDL: 10Hz, High-resolution mode, X/Y/Z enabled
    i2c_write_register(CTRL_REG1, 0x70);
}

int main()
{
    stdio_init_all();

    gpio_init(DRDY_PIN);
    gpio_set_dir(DRDY_PIN, GPIO_IN);
    gpio_pull_up(DRDY_PIN);

    // Add ISR to handle whenever new data is ready to be read from the sensor
    // ! Use gpio_add_raw_irq_handler() instead if there are other GPIO IRQ enabled
    gpio_set_irq_enabled_with_callback(DRDY_PIN, GPIO_IRQ_EDGE_FALL, true, gpio_irq_handler);

    // I2C initialisation, 100 Khz (LIS3MDL standard mode)
    i2c_init(i2c_get_instance(PICO_DEFAULT_I2C), 100000);

    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    sleep_ms(5000);

    puts("Init.");

    lis3mdl_init();

    while (true)
    {
        if (data_ready)
        {
            uint8_t buffer;
            int result = i2c_read_timeout_us(
                i2c_get_instance(PICO_DEFAULT_I2C),
                SLAVE_ADDRESS,
                &buffer,
                sizeof(buffer),
                false,
                1000 * 500);

            if (result == PICO_ERROR_GENERIC)
            {
                puts("Generic error while reading.");
            }
            else if (result == PICO_ERROR_TIMEOUT)
            {
                puts("Timeout error while reading.");
            }
            else
            {
                printf("Read %d bytes.\n", result);
            }

            data_ready = false;
        }

        sleep_ms(100);
    }
}
