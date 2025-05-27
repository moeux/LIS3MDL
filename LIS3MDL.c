#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define MERGE(low, high) ((high << 8) | low)
#define MAP(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

#define I2C_TIMEOUT_US 500000
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

typedef enum
{
    GAUSS_4,
    GAUSS_8,
    GAUSS_12,
    GAUSS_16
} gauss_scale_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} axes_raw_data_t;

typedef struct
{
    float x;
    float y;
    float z;
} axes_data_t;

typedef union
{
    uint8_t raw;
    struct
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
    };
} status_t;

static inline void split_int16(const int16_t val, uint8_t *low, uint8_t *high)
{
    *low = (uint8_t)(val & 0xFF);
    *high = (uint8_t)((val >> 8) & 0xFF);
}

static int i2c_write_register(const uint8_t reg, const uint8_t value)
{
    uint8_t buffer[2] = {reg, value};

    return i2c_write_timeout_us(
        i2c_get_instance(PICO_DEFAULT_I2C),
        SLAVE_ADDRESS,
        buffer,
        sizeof(buffer) / sizeof(buffer[0]),
        false,
        I2C_TIMEOUT_US);
}

static int i2c_read_register(const uint8_t reg)
{
    uint8_t value;
    int write = i2c_write_timeout_us(i2c_get_instance(PICO_DEFAULT_I2C), SLAVE_ADDRESS, &reg, sizeof(reg), true, I2C_TIMEOUT_US);

    if (write < 0)
    {
        return write;
    }

    int read = i2c_read_timeout_us(i2c_get_instance(PICO_DEFAULT_I2C), SLAVE_ADDRESS, &value, sizeof(value), false, I2C_TIMEOUT_US);

    if (read < 0)
    {
        return read;
    }

    return value;
}

static int i2c_read_multiple_registers(uint8_t reg, uint8_t *data, size_t length)
{
    reg = reg | 0x80; // set auto-increment bit
    int write = i2c_write_timeout_us(i2c_get_instance(PICO_DEFAULT_I2C), SLAVE_ADDRESS, &reg, sizeof(reg), true, I2C_TIMEOUT_US);

    if (write < 0)
    {
        return write;
    }

    int read = i2c_read_timeout_us(i2c_get_instance(PICO_DEFAULT_I2C), SLAVE_ADDRESS, data, length, false, I2C_TIMEOUT_US);

    if (read < 0)
    {
        return read;
    }

    return write + read;
}

static bool lis3mdl_read_axes_data(uint8_t reg, axes_raw_data_t *data)
{
    uint8_t raw[6];

    if (i2c_read_multiple_registers(reg, raw, sizeof(raw) / sizeof(raw[0])) < 0)
    {
        return false;
    }

    data->x = MERGE(raw[0], raw[1]);
    data->y = MERGE(raw[2], raw[3]);
    data->z = MERGE(raw[4], raw[5]);

    return true;
}

bool lis3mdl_init()
{
    if (i2c_read_register(WHO_AM_I_REG) != 0x3D)
    {
        return false;
    }

    /* CTRL_REG1 - Configuration:
        - TEMP_EN: disabled                 (0)
        - OM: ultrahigh-performance mode    (11)
        - DO: 10 Hz                         (100)
        - FAST_ODR: disabled                (0)
        - ST: disabled                      (0)
    */
    if (i2c_write_register(CTRL_REG1, 0x70) < 0)
    {
        return false;
    }

    /* CTRL_REG2 - Configuration:
        - FS: +/- 4 Gauss                   (00)
        - REBOOT: normal mode               (0)
        - SOFT_RST: default                 (0)
    */
    if (i2c_write_register(CTRL_REG2, 0x0 & 0x6C) < 0)
    {
        return false;
    }

    /* CTRL_REG3 - Configuration:
        - LP: disabled                      (0)
        - SIM: 4-wire interface             (0)
        - MD: continuous-conversion mode    (00)
    */
    if (i2c_write_register(CTRL_REG3, 0x0 & 0x27) < 0)
    {
        return false;
    }

    /* CTRL_REG4 - Configuration:
        - OMZ: Ultrahigh-performance mode   (11)
        - BLE: Little-Endian                (1)
    */
    if (i2c_write_register(CTRL_REG4, 0xE & 0xE) < 0)
    {
        return false;
    }

    /* CTRL_REG5 - Configuration:
        - FAST_READ: disabled               (0)
        - BDU: continuous update            (0)
    */
    if (i2c_write_register(CTRL_REG5, 0x0 & 0xC0) < 0)
    {
        return false;
    }

    return true;
}

bool lis3mdl_set_offsets(const int16_t x, const int16_t y, const int16_t z)
{
    uint8_t data[6];

    split_int16(x, &data[0], &data[1]);
    split_int16(y, &data[2], &data[3]);
    split_int16(z, &data[4], &data[5]);

    for (int i = 0; i < 6; i++)
    {
        if (i2c_write_register(OFFSET_X_REG_L_M + i, data[i]) < 0)
        {
            return false;
        }
    }

    return true;
}

bool lis3mdl_read_status(status_t *status)
{
    int read = i2c_read_register(STATUS_REG);

    if (read < 0)
    {
        return false;
    }

    status->raw = read;

    return true;
}

bool lis3mdl_read_raw_offsets(axes_raw_data_t *data)
{
    return lis3mdl_read_axes_data(OFFSET_X_REG_L_M, data);
}

bool lis3mdl_read_raw_axes(axes_raw_data_t *data)
{
    return lis3mdl_read_axes_data(OUT_X_L, data);
}

bool lis3mdl_read_microteslas(axes_data_t *data, const gauss_scale_t gauss)
{
    float scale;
    axes_raw_data_t raw = {0, 0, 0};

    switch (gauss)
    {
    case GAUSS_4:
        scale = 6842;
        break;
    case GAUSS_8:
        scale = 3421;
        break;
    case GAUSS_12:
        scale = 2281;
        break;
    case GAUSS_16:
        scale = 1711;
        break;
    default:
        scale = 1;
        break;
    }

    if (!lis3mdl_read_raw_axes(&raw))
    {
        return false;
    }

    data->x = (float)raw.x / scale * 100;
    data->y = (float)raw.y / scale * 100;
    data->z = (float)raw.z / scale * 100;

    return true;
}

float lis3mdl_get_heading(const float x, const float y)
{
    float heading = atan2f(y, x) * (180.0f / M_PI);

    if (heading < 0)
    {
        heading += 360.0f;
    }

    return heading;
}

void lis3mdl_calibrate()
{
    int16_t x_max = INT16_MIN;
    int16_t x_min = INT16_MAX;
    int16_t y_max = INT16_MIN;
    int16_t y_min = INT16_MAX;
    int16_t z_max = INT16_MIN;
    int16_t z_min = INT16_MAX;

    puts("Please rotate the sensor in all directions for calibration.");

    for (int i = 0; i < 1000; i++)
    {
        axes_raw_data_t raw;

        if (lis3mdl_read_raw_axes(&raw))
        {
            x_max = MAX(raw.x, x_max);
            x_min = MIN(raw.x, x_min);
            y_max = MAX(raw.y, y_max);
            y_min = MIN(raw.y, y_min);
            z_max = MAX(raw.z, z_max);
            z_min = MIN(raw.z, z_min);

            printf("Sample %d\n", i);
            printf(">x:%d,y:%d,z:%d\r\n", raw.x, raw.y, raw.z);
        }

        sleep_ms(20);
    }

    int16_t x_offset = -(x_min + x_max) / 2;
    int16_t y_offset = -(y_min + y_max) / 2;
    int16_t z_offset = -(z_min + z_max) / 2;

    printf("Computed Offsets:\n     X: %d\n     Y: %d\n     Z: %d\n", x_offset, y_offset, z_offset);

    if (lis3mdl_set_offsets(x_offset, y_offset, z_offset))
    {
        puts("Offsets applied successfully.");
    }
    else
    {
        puts("Failed to apply offsets.");
    }
}

int main()
{
    stdio_init_all();

    // I2C initialisation, 10 Hz (LIS3MDL standard mode)
    i2c_init(i2c_get_instance(PICO_DEFAULT_I2C), 100000);

    gpio_init(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_init(PICO_DEFAULT_I2C_SCL_PIN);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    sleep_ms(5000);

    puts("Initializing LIS3MDL module.");

    if (!lis3mdl_init())
    {
        puts("LIS3MDL initialization failed!");

        while (true)
        {
            sleep_ms(10);
        }
    }

    // lis3mdl_calibrate();
    // Pre-calibrated values
    lis3mdl_set_offsets(3939, -8614, 9060);

    while (true)
    {
        /*status_t status = {0};
        lis3mdl_read_status(&status);
        printf(">overrun:%d,x_overrun:%d,y_overrun:%d,z_overrun:%d,data_available:%d,x_data_available:%d,y_data_available:%d,z_data_available:%d\r\n",
               status.overrun, status.x_overrun, status.y_overrun, status.z_overrun,
               status.data_available, status.x_data_available, status.y_data_available, status.z_data_available);

        axes_raw_data_t raw_data = {0, 0, 0};
        lis3mdl_read_raw_offsets(&raw_data);
        printf(">x_offset:%d,y_offset:%d,z_offset:%d\r\n", raw_data.x, raw_data.y, raw_data.z);*/

        axes_raw_data_t raw_data = {0, 0, 0};
        lis3mdl_read_raw_axes(&raw_data);
        printf(">x_raw:%d,y_raw:%d,z_raw:%d\r\n", raw_data.x, raw_data.y, raw_data.z);

        axes_data_t data = {0.0, 0.0, 0.0};
        lis3mdl_read_microteslas(&data, GAUSS_4);
        printf(">x_ut:%.2f,y_ut:%.2f,z_ut:%.2f\r\n", data.x, data.y, data.z);

        float heading = lis3mdl_get_heading(data.x, data.y);
        printf(">heading:%.2f\r\n", heading);

        sleep_ms(10);
    }
}