#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define MERGE(low, high) ((high < 8) | low)

#define SLAVE_ADDRESS _u(0x1C)
#define OFFSET_X_REG_L_M _u(0x05)
#define OFFSET_X_REG_H_M _u(0x06)
#define OFFSET_Y_REG_L_M _u(0x07)
#define OFFSET_Y_REG_H_M _u(0x08)
#define OFFSET_Z_REG_L_M _u(0x09)
#define OFFSET_Z_REG_H_M _u(0x0A)
#define WHO_AM_I_REG _u(0x0F)
#define CTRL_REG1 _u(0x20)
#define CTRL_REG2 _u(0x21)
#define CTRL_REG3 _u(0x22)
#define CTRL_REG4 _u(0x23)
#define CTRL_REG5 _u(0x24)
#define STATUS_REG _u(0x27)
#define OUT_X_L _u(0x28)
#define OUT_X_H _u(0x29)
#define OUT_Y_L _u(0x2A)
#define OUT_Y_H _u(0x2B)
#define OUT_Z_L _u(0x2C)
#define OUT_Z_H _u(0x2D)

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} axes_data_t;

typedef struct
{
    // ZYX overrun -> a new set of data has overwritten the previous data
    bool overrun;
    bool z_overrun;
    bool y_overrun;
    bool x_overrun;
    // A new set of data is available
    bool data_available;
    bool z_data_available;
    bool y_data_available;
    bool x_data_available;
} status_t;

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
        return write;
    }
    int read = i2c_read_timeout_us(i2c_get_instance(PICO_DEFAULT_I2C), SLAVE_ADDRESS, &value, sizeof(value), false, 1000 * 500);
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
    i2c_write_register(CTRL_REG5, 0x0 & 0xC0);
}

void lis3mdl_read_status(status_t *status)
{
    uint8_t raw = i2c_read_register(STATUS_REG);

    status->overrun = raw & 0x80;
    status->z_overrun = raw & 0x40;
    status->y_overrun = raw & 0x20;
    status->x_overrun = raw & 0x10;
    status->data_available = raw & 0x8;
    status->z_data_available = raw & 0x4;
    status->y_data_available = raw & 0x2;
    status->x_data_available = raw & 0x1;
}

void lis3mdl_read_raw_offsets(axes_data_t *data)
{
    data->x = MERGE(i2c_read_register(OFFSET_X_REG_L_M), i2c_read_register(OFFSET_X_REG_H_M));
    data->y = MERGE(i2c_read_register(OFFSET_Y_REG_L_M), i2c_read_register(OFFSET_Y_REG_H_M));
    data->z = MERGE(i2c_read_register(OFFSET_Z_REG_L_M), i2c_read_register(OFFSET_Z_REG_H_M));
}

void lis3mdl_read_raw_axes(axes_data_t *data)
{
    data->x = MERGE(i2c_read_register(OUT_X_L), i2c_read_register(OUT_X_H));
    data->y = MERGE(i2c_read_register(OUT_Y_L), i2c_read_register(OUT_Y_H));
    data->z = MERGE(i2c_read_register(OUT_Z_L), i2c_read_register(OUT_Z_H));
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
        axes_data_t offsets = {0, 0, 0};
        lis3mdl_read_raw_offsets(&offsets);
        printf(">x_offset:%d,y_offset:%d,z_offset:%d\r\n", offsets.x, offsets.y, offsets.z);
        status_t status = {false, false, false, false, false, false, false, false};
        lis3mdl_read_status(&status);
        printf(">overrun:%d,x_overrun:%d,y_overrun:%d,z_overrun:%d,data_available:%d,x_data_available:%d,y_data_available:%d,z_data_available:%d\r\n",
               status.overrun, status.x_overrun, status.y_overrun, status.z_overrun,
               status.data_available, status.x_data_available, status.y_data_available, status.z_data_available);
        axes_data_t data = {0, 0, 0};
        lis3mdl_read_raw_axes(&data);
        printf(">x:%d,y:%d,z:%d\r\n", data.x, data.y, data.z);
        sleep_ms(100);
    }
}
