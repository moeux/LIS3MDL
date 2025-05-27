#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define DRDY_PIN 3
#define SLAVE_ADDRESS 0x1C
#define WHO_AM_I_REG 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG4 0x23
#define INT_CFG 0x30
#define INT_SRC 0x31
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D

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

int i2c_read_register(uint8_t reg, uint8_t *value)
{
    int write = i2c_write_timeout_us(i2c_get_instance(PICO_DEFAULT_I2C), SLAVE_ADDRESS, &reg, sizeof(reg), true, 1000 * 500);
    if (write < 0)
    {
        return write;
    }

    int read = i2c_read_timeout_us(i2c_get_instance(PICO_DEFAULT_I2C), SLAVE_ADDRESS, *value, sizeof(*value), true, 1000 * 500);
    if (read < 0)
    {
        return read;
    }

    return write + read;
}

bool lis3mdl_init()
{
    int who_am_i;
    int result = i2c_read_register(WHO_AM_I_REG, &who_am_i);

    if (result < 0 || who_am_i != 0x3D)
    {
        return false;
    }

    // Configuration:
    //  - Temperature Sensor disabled
    //  - X/Y axes operative mode: Ultrahigh-performance mode
    //  - Output data rate selection: 80 Hz (ignored due to FAST_ODR)
    //  - FAST_ODR enabled
    //  - Self-Test disabled
    if (i2c_write_register(CTRL_REG1, 0x7E) < 0)
    {
        return false;
    }
    // Configuration:
    //  - Full-scale: +/- Gauss
    //  - REBOOT: no reboot, normal mode
    //  - SOFT_RST: disabled
    if (i2c_write_register(CTRL_REG2, 0x0) < 0)
    {
        return false;
    }
    // Configuration:
    //  - Z axis operative mode: Ultrahigh-performance mode
    //  - Big/little Endian: Little Endian (data MSb at lower address)
    if (i2c_write_register(CTRL_REG4, 0xE) < 0)
    {
        return false;
    }
    // Configuration:
    //  - X-axis interrupt generation enabled
    //  - Y-axis interrupt generation enabled
    //  - X-axis interrupt generation enabled
    //  - Interrupt active configuration: low
    //  - Latch interrupt enabled
    //  - Interrupt enabled
    if (i2c_write_register(INT_CFG, 0xE9) < 0)
    {
        return false;
    }

    return true;
}

void lis3mdl_read(uint16_t *x, uint16_t *y, uint16_t *z)
{
    uint8_t x_high;
    uint8_t x_low;
    uint8_t y_high;
    uint8_t y_low;
    uint8_t z_high;
    uint8_t z_low;

    i2c_read_register(OUT_X_H, &x_high);
    i2c_read_register(OUT_X_L, &x_low);
    *x = ((uint16_t)x_high << 8) | x_low;

    i2c_read_register(OUT_Y_H, &y_high);
    i2c_read_register(OUT_Y_L, &y_low);
    *y = ((uint16_t)y_high << 8) | y_low;

    i2c_read_register(OUT_Z_H, &z_high);
    i2c_read_register(OUT_Z_L, &z_low);
    *z = ((uint16_t)z_high << 8) | z_low;
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

    i2c_init(i2c_get_instance(PICO_DEFAULT_I2C), 155);

    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    if (lis3mdl_init())
    {
        puts("LIS3MDL initialized.");
    }
    else
    {
        puts("LIS3MDL initialization failed.");
    }

    while (true)
    {
        if (data_ready)
        {
            // Reading the INT_SRC register to reset the INT pin.
            uint8_t int_src;
            uint8_t result = i2c_read_register(INT_SRC, &int_src);
            switch (result)
            {
            case PICO_ERROR_GENERIC:
                puts("Generic error reading the INT_SRC.");
                break;
            case PICO_ERROR_TIMEOUT:
                puts("Timeout error reading the INT_SRC.");
                break;
            default:
                printf("Successfully read the INT_SRC register: %x (%d bytes)", int_src, result);
                break;
            };

            uint16_t x;
            uint16_t y;
            uint16_t z;

            lis3mdl_read(&x, &y, &z);

            printf("%x | %x | %x", x, y, z);

            data_ready = false;
        }

        sleep_ms(100);
    }
}
