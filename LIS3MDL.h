#ifndef LIS3MDL_H
#define LIS3MDL_H

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

/**
 * Initializes the LIS3MDL, by writing into the control registers.
 * @return `true` on successful initialization, otherwise `false`.
 */
bool lis3mdl_init();
/**
 * Sets the hard iron offset registers.
 * @param x X-axis offset.
 * @param y Y-axis offset.
 * @param z Z-axis offset.
 * @return `true` on successful offset calibration, otherwise `false`.
 */
bool lis3mdl_set_offsets(const int16_t x, const int16_t y, const int16_t z);
/**
 * Reads the status register.
 * @param status Struct to read into.
 * @return `true` on successful status retrieval, otherwise `false`.
 */
bool lis3mdl_read_status(status_t *status);
/**
 * Reads the raw hard iron offset registers.
 * @param data Struct to read into.
 * @return `true` on successful offset retrieval, otherwise `false`.
 */
bool lis3mdl_read_raw_offsets(axes_raw_data_t *data);
/**
 * Reads the raw axes data.
 * @param data Struct to read into.
 * @return `true` on successful axes data retrieval, otherwise `false`.
 */
bool lis3mdl_read_raw_axes(axes_raw_data_t *data);
/**
 * Reads the axes data in microtesla.
 * @param data Struct to read into.
 * @param gauss The scale the LIS3MDL is operating in.
 * @return `true` on successful axes data retrieval, otherwise `false`.
 */
bool lis3mdl_read_microteslas(axes_data_t *data, const gauss_scale_t gauss);
/**
 * Calculates the compass heading in degrees.
 * @param x The raw x-axis value.
 * @param y The raw y-axis value.
 * @return The compass heading.
 */
float lis3mdl_get_heading(const int16_t x, const int16_t y);

#endif