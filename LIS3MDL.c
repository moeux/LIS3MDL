#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define SLAVE_ADDRESS _u(0x1C)
#define WHO_AM_I_REG _u(0x0F)
#define WHO_AM_I _u(0x3D)
#define CTRL_REG1 _u(0x20)
#define CTRL_REG2 _u(0x21)
#define CTRL_REG3 _u(0x22)
#define CTRL_REG4 _u(0x23)
#define CTRL_REG5 _u(0x24)
#define OUT_X_L _u(0x28)

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
    int write = i2c_write_timeout_us(i2c_get_instance(PICO_DEFAULT_I2C), SLAVE_ADDRESS, &reg, sizeof(reg), false, 1000 * 500);
    if (write < 0)
    {
        printf("Read: Write failed - %d\n", write);
        return write;
    }
    int read = i2c_read_timeout_us(i2c_get_instance(PICO_DEFAULT_I2C), SLAVE_ADDRESS, &value, sizeof(value), false, 1000 * 500);
    if (read < 0)
    {
        printf("Read: Read failed - %d\n", write);
        return read;
    }
    return value;
}

void lis3mdl_init()
{
    uint8_t who_am_i = i2c_read_register(WHO_AM_I_REG);

    if (who_am_i == WHO_AM_I)
    {
        puts("LIS3MDL detected!");
    }
    else
    {
        printf("LIS3MDL not detected! ID: 0x%X\n", who_am_i);
        return;
    }

    /* CTRL_REG1 - Configuration:
        - TEMP_EN: disabled                 (0)
        - OM: ultrahigh-performance mode    (11)
        - DO: 10 Hz                         (100)
        - FAST_ODR: disabled                (0)
        - ST: disabled                      (0)
    */
    i2c_write_register(CTRL_REG1, 0x70);

    /* CTRL_REG2 - Configuration:
        - FS: +/- 4 Gauss                   (00)
        - REBOOT: normal mode               (0)
        - SOFT_RST: default                 (0)
    */
    i2c_write_register(CTRL_REG2, 0x0 & 0x6C);

    /* CTRL_REG3 - Configuration:
        - LP: disabled                      (0)
        - SIM: 4-wire interface             (0)
        - MD: continuous-conversion mode    (00)
    */
    i2c_write_register(CTRL_REG3, 0x0 & 0x27);

    /* CTRL_REG4 - Configuration:
        - OMZ: Ultrahigh-performance mode   (11)
        - BLE: Big-Endian                   (0)
    */
    i2c_write_register(CTRL_REG4, 0xC & 0xE);

    /* CTRL_REG5 - Configuration:
        - FAST_READ: disabled               (0)
        - BDU: continuous update            (0)
    */
    // i2c_write_register(CTRL_REG5, 0x0 & 0xC0);
}

int main()
{
    stdio_init_all();

    // I2C initialisation, 10 Hz (LIS3MDL standard mode)
    i2c_init(i2c_get_instance(PICO_DEFAULT_I2C), 10);

    gpio_init(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_init(PICO_DEFAULT_I2C_SCL_PIN);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    sleep_ms(5000);

    puts("Initializing LIS3MDL module.");

    lis3mdl_init();

    puts("Getting readings.");

    while (true)
    {
        int x = i2c_read_register(OUT_X_L);
        printf(">x_l:%d\r\n", x);
        sleep_ms(100);
    }
}
