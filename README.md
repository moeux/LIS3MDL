# LIS3MDL Pico Library

A lightweight C library for interfacing the STMicroelectronics LIS3MDL 3-axis magnetometer via I²C on the Raspberry Pi Pico (and other RP2040 boards) using the Pico C/C++ SDK.

---

## Table of Contents

- [Features](#features)  
- [Hardware Requirements](#hardware-requirements)  
- [Software Requirements](#software-requirements)  
- [Getting Started](#getting-started)  
  - [Cloning & Building](#cloning--building)  
  - [Wiring](#wiring)  
- [Usage](#usage)  
  - [Initialization & Configuration](#initialization--configuration)  
  - [Reading Data](#reading-data)  
- [API](#api)  
- [Example](#example)  
- [License](#license)  
- [Contributing](#contributing)

---

## Features

- I²C read/write helpers with timeout  
- Automatic register address auto-increment for multi-byte reads  
- Initialization/configuration of all LIS3MDL control registers  
- Hard-iron offset & threshold setup  
- Raw & calibrated (µT) axis data  
- Heading (°) calculation  
- On-chip temperature reading (°C)  
- Status & interrupt‐source register decoding  

---

## Hardware Requirements

- Raspberry Pi Pico (or any RP2040 board)  
- LIS3MDL magnetometer breakout or module  
- Pull-up resistors on SDA/SCL if not provided on your breakout  

---

## Software Requirements

- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) (tested on v1.5.0+)  
- CMake 3.13+  
- A GCC toolchain for ARM Cortex-M0+  

---

## Getting Started

### Cloning & Building

1. Clone the Pico project (or add as a submodule):

   ```bash
   git clone https://github.com/moeux/LIS3MDL.git
   cd LIS3MDL
   git submodule add https://github.com/moeux/LIS3MDL.git librares/LIS3MDL
   
2. In your `CMakeLists.txt`, add:

   ```cmake
   add_subdirectory(libraries/LIS3MDL)
   target_link_libraries(<your_executable> PRIVATE LIS3MDL)

3. Build as usual

   ```bash
   mkdir build
   cd build
   cmake ..
   make

### Wiring

|     Pico Pin    | LIS3MDL Pin |
| :-------------: | :---------: |
|  I2C0 SDA (GP4) |     SDA     |
|  I2C1 SCL (GP5) |     SCL     |
|       3V3       |     VCC     |
|       GND       |     GND     |

## Usage

### Initialization & Configuration

  ```c
  #include "LIS3MDL.h"
  
  // Initialize I²C and LIS3MDL
  i2c_init(i2c0, 100 * 1000);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
  
  if (!lis3mdl_init()) {
      printf("LIS3MDL init failed!\n");
      return;
  }
  
  // Optional: calibrate offsets (hard-iron)
  lis3mdl_set_offsets(0, 0, 0);
  // Optional: set interrupt threshold
  lis3mdl_set_threshold(1000);
  ```

### Reading Data

  ```c
  status_t st;
  axes_raw_data_t raw;
  axes_data_t data;
  int16_t temp_raw;
  
  // Poll for data-ready
  if (lis3mdl_read_status(&st) && st.data_available) {
      lis3mdl_read_raw_axes(&raw);
      data = lis3mdl_get_microteslas(raw, GAUSS_4);
      float heading = lis3mdl_get_heading(raw.x, raw.y);
      lis3mdl_read_raw_temperature(&temp_raw);
      float temperature = lis3mdl_get_celcius(temp_raw);
  
      printf("X: %.2f μT, Y: %.2f μT, Z: %.2f μT\n", data.x, data.y, data.z);
      printf("Heading: %.1f°\n", heading);
      printf("Temp: %.1f °C\n", temperature);
  }
  ```

### API

| Function                                             | Description                                                 |
| ---------------------------------------------------- | ----------------------------------------------------------- |
| `bool lis3mdl_init(void)`                            | Initialize sensor & control registers                       |
| `bool lis3mdl_set_offsets(int16_t x,y,z)`            | Write hard-iron offsets                                     |
| `bool lis3mdl_set_threshold(uint16_t th)`            | Set interrupt threshold                                     |
| `bool lis3mdl_read_status(status_t *st)`             | Read status register flags                                  |
| `bool lis3mdl_read_interrupt_source(int_src_t *src)` | Read interrupt source register                              |
| `bool lis3mdl_read_raw_axes(axes_raw_data_t *d)`     | Read & auto-center raw axis data                            |
| `axes_data_t lis3mdl_get_microteslas(...)`           | Convert raw data to microteslas based on chosen gauss scale |
| `float lis3mdl_get_heading(int16_t x,int16_t y)`     | Compute 2D compass heading (°)                              |
| `bool lis3mdl_read_raw_temperature(int16_t *t)`      | Read raw temperature                                        |
| `float lis3mdl_get_celcius(int16_t t)`               | Convert raw temp to Celsius                                 |

*(See header comments for full prototypes.)*

### Example

See the `main()` in `LIS3MDL.c` for a complete demo:

- Turns the on-board LED on while initializing
- Prints raw & scaled axes, heading, and temperature every sample

### License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

---

## Contributing

1. Fork the repo
2. Create a feature branch (`git checkout -b feature/awesome`)
3. Commit your changes
4. Open a Pull Request
