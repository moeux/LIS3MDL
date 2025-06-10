#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "LIS3MDL.h"

#define MERGE(low, high) ((int16_t)((high << 8) | low))

static int16_t x_max = INT16_MIN;
static int16_t x_min = INT16_MAX;
static int16_t y_max = INT16_MIN;
static int16_t y_min = INT16_MAX;
static int16_t z_max = INT16_MIN;
static int16_t z_min = INT16_MAX;

static inline void split_int16(const int16_t val, uint8_t *low, uint8_t *high)
{
    *low = (uint8_t)(val & 0xFF);
    *high = (uint8_t)((val >> 8) & 0xFF);
}

static bool i2c_write_register(const uint8_t reg, const uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    uint8_t length = sizeof(buffer) / sizeof(buffer[0]);
    int result = i2c_write_timeout_us(
        i2c_get_instance(PICO_DEFAULT_I2C),
        SLAVE_ADDRESS,
        buffer,
        length,
        false,
        I2C_TIMEOUT_US);

    return result == length;
}

static bool i2c_read_register(const uint8_t reg, uint8_t *value)
{
    if (i2c_write_timeout_us(i2c_get_instance(PICO_DEFAULT_I2C), SLAVE_ADDRESS, &reg, sizeof(reg), true, I2C_TIMEOUT_US) != sizeof(reg))
    {
        return false;
    }

    if (i2c_read_timeout_us(i2c_get_instance(PICO_DEFAULT_I2C), SLAVE_ADDRESS, value, sizeof(*value), false, I2C_TIMEOUT_US) != sizeof(*value))
    {
        return false;
    }

    return true;
}

static bool i2c_read_multiple_registers(uint8_t reg, uint8_t *data, size_t length)
{
    reg = reg | 0x80; // set auto-increment bit

    if (i2c_write_timeout_us(i2c_get_instance(PICO_DEFAULT_I2C), SLAVE_ADDRESS, &reg, sizeof(reg), true, I2C_TIMEOUT_US) != sizeof(reg))
    {
        return false;
    }

    if (i2c_read_timeout_us(i2c_get_instance(PICO_DEFAULT_I2C), SLAVE_ADDRESS, data, length, false, I2C_TIMEOUT_US) != length)
    {
        return false;
    }

    return true;
}

static bool lis3mdl_read_axes_data(uint8_t reg, axes_raw_data_t *data)
{
    uint8_t raw[6];

    if (!i2c_read_multiple_registers(reg, raw, sizeof(raw) / sizeof(raw[0])))
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
    uint8_t id = 0;

    if (!i2c_read_register(WHO_AM_I_REG, &id) || id != 0x3D)
    {
        return false;
    }

    /* CTRL_REG1 - Configuration:
        - TEMP_EN:      enabled                         (1)
        - OM:           ultrahigh-performance mode      (11)
        - DO:           10 Hz                           (100)
        - FAST_ODR:     disabled                        (0)
        - ST:           disabled                        (0)
    */
    if (!i2c_write_register(CTRL_REG1, 0xF0))
    {
        return false;
    }

    /* CTRL_REG2 - Configuration:
        - FS:           +/- 4 Gauss                     (00)
        - REBOOT:       normal mode                     (0)
        - SOFT_RST:     default                         (0)
    */
    if (!i2c_write_register(CTRL_REG2, 0x0 & 0x6C))
    {
        return false;
    }

    /* CTRL_REG3 - Configuration:
        - LP:           disabled                        (0)
        - SIM:          4-wire interface                (0)
        - MD:           continuous-conversion mode      (00)
    */
    if (!i2c_write_register(CTRL_REG3, 0x0 & 0x27))
    {
        return false;
    }

    /* CTRL_REG4 - Configuration:
        - OMZ:          ultrahigh-performance mode      (11)
        - BLE:          big-Endian                      (0)
    */
    if (!i2c_write_register(CTRL_REG4, 0xC & 0xE))
    {
        return false;
    }

    /* CTRL_REG5 - Configuration:
        - FAST_READ:    disabled                        (0)
        - BDU:          continuous update               (0)
    */
    if (!i2c_write_register(CTRL_REG5, 0x0 & 0xC0))
    {
        return false;
    }

    /* INT_CFG - Configration:
        - XIEN:         enabled                         (1)
        - YIEN:         enabled                         (1)
        - ZIEN:         enabled                         (1)
        - IEA:          high                            (1)
        - LIR:          latched                         (0)
        - IEN:          disabled                        (0)
     */
    if (!i2c_write_register(INT_CFG, 0xEC & 0xEF | 0x8))
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

    for (int i = 0; i < sizeof(data) / sizeof(data[0]); i++)
    {
        if (!i2c_write_register(OFFSET_X_REG_L_M + i, data[i]))
        {
            return false;
        }
    }

    return true;
}

bool lis3mdl_set_threshold(const uint16_t threshold)
{
    uint8_t low = 0;
    uint8_t high = 0;

    split_int16(threshold, &low, &high);

    if (!i2c_write_register(INT_THS_L, low))
    {
        return false;
    }

    if (!i2c_write_register(INT_THS_H, high & 0x7F))
    {
        return false;
    }

    return true;
}

bool lis3mdl_read_status(status_t *status)
{
    uint8_t read = 0;

    if (!i2c_read_register(STATUS_REG, &read))
    {
        return false;
    }

    status->overrun = (read & 0x80) >> 7;
    status->z_overrun = (read & 0x40) >> 6;
    status->y_overrun = (read & 0x20) >> 5;
    status->x_overrun = (read & 0x10) >> 4;
    status->data_available = (read & 8) >> 3;
    status->z_data_available = (read & 4) >> 2;
    status->y_data_available = (read & 2) >> 1;
    status->x_data_available = read & 1;

    return true;
}

bool lis3mdl_read_interrupt_source(int_src_t *source)
{
    uint8_t read = 0;

    if (!i2c_read_register(INT_SRC, &read))
    {
        return false;
    }

    source->x_positive_exceeded = (read & 0x80) >> 7;
    source->y_positive_exceeded = (read & 0x40) >> 6;
    source->z_positive_exceeded = (read & 0x20) >> 5;
    source->x_negative_exceeded = (read & 0x10) >> 4;
    source->y_negative_exceeded = (read & 8) >> 3;
    source->z_negative_exceeded = (read & 4) >> 2;
    source->overflow = (read & 2) >> 1;
    source->interrupted = read & 1;

    return true;
}

bool lis3mdl_read_raw_offsets(axes_raw_data_t *data)
{
    return lis3mdl_read_axes_data(OFFSET_X_REG_L_M, data);
}

bool lis3mdl_read_raw_threshold(uint16_t *threshold)
{
    uint8_t low = 0;
    uint8_t high = 0;

    if (!i2c_read_register(INT_THS_L, &low) || !i2c_read_register(INT_THS_H, &high))
    {
        return false;
    }

    *threshold = MERGE(low, high);

    return true;
}

bool lis3mdl_read_raw_axes(axes_raw_data_t *data)
{
    if (lis3mdl_read_axes_data(OUT_X_L, data))
    {
        x_max = MAX(data->x, x_max);
        x_min = MIN(data->x, x_min);
        y_max = MAX(data->y, y_max);
        y_min = MIN(data->y, y_min);
        z_max = MAX(data->z, z_max);
        z_min = MIN(data->z, z_min);

        data->x = data->x - ((x_max + x_min) / 2);
        data->y = data->y - ((y_max + y_min) / 2);
        data->z = data->z - ((z_max + z_min) / 2);

        return true;
    }

    return false;
}

bool lis3mdl_read_raw_temperature(int16_t *temp)
{
    uint8_t low = 0;
    uint8_t high = 0;

    if (!i2c_read_register(TEMP_OUT_L, &low) || !i2c_read_register(TEMP_OUT_H, &high))
    {
        return false;
    }

    *temp = MERGE(low, high);

    return true;
}

axes_data_t lis3mdl_get_microteslas(const axes_raw_data_t raw, const gauss_scale_t gauss)
{
    float scale;
    axes_data_t data = {0.0, 0.0, 0.0};

    switch (gauss)
    {
    case GAUSS_4:
        scale = 6842.0f;
        break;
    case GAUSS_8:
        scale = 3421.0f;
        break;
    case GAUSS_12:
        scale = 2281.0f;
        break;
    case GAUSS_16:
        scale = 1711.0f;
        break;
    default:
        scale = 6842.0f;
        break;
    }

    data.x = ((float)raw.x / scale) * 100.0f;
    data.y = ((float)raw.y / scale) * 100.0f;
    data.z = ((float)raw.z / scale) * 100.0f;

    return data;
}

float lis3mdl_get_heading(const int16_t x, const int16_t y)
{
    float heading = atan2f(y, x) * (180.0f / M_PI);

    if (heading < 0)
    {
        heading += 360.0f;
    }

    return heading;
}

float lis3mdl_get_celcius(const int16_t temp)
{
    int16_t raw = 0;

    if (!lis3mdl_read_raw_temperature(&raw))
    {
        return false;
    }

    return (float)temp / 8.0;
}

int main()
{
    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    i2c_init(i2c_get_instance(PICO_DEFAULT_I2C), 100000);

    gpio_init(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_init(PICO_DEFAULT_I2C_SCL_PIN);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    sleep_ms(5000);

    puts("Initializing LIS3MDL module.");

    while (!lis3mdl_init())
    {
        puts("LIS3MDL initialization failed!");
        sleep_ms(500);
    }

    lis3mdl_set_offsets(0, 0, 0);
    lis3mdl_set_threshold(0);

    puts("LIS3MDL initialized.");

    while (true)
    {
        status_t status = {false, false, false, false, false, false, false, false};

        if (lis3mdl_read_status(&status) && (status.data_available || status.x_data_available || status.y_data_available || status.z_data_available))
        {
            axes_raw_data_t raw_data = {0, 0, 0};
            axes_data_t data = {0, 0, 0};
            int16_t temp = 0;

            if (lis3mdl_read_raw_axes(&raw_data) && lis3mdl_read_raw_temperature(&temp))
            {
                printf(">x_raw:%d,y_raw:%d,z_raw:%d\r\n", raw_data.x, raw_data.y, raw_data.z);
                data = lis3mdl_get_microteslas(raw_data, GAUSS_4);
                printf(">x_ut:%.2f,y_ut:%.2f,z_ut:%.2f\r\n", data.x, data.y, data.z);
                printf(">heading:%.2f\r\n", lis3mdl_get_heading(raw_data.x, raw_data.y));
                printf(">temp_raw:%d,temp_c:%f\r\n", temp, lis3mdl_get_celcius(temp));
            }
        }

        sleep_ms(10);
    }
}