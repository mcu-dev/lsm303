# LSM303DLHC Support Library in C

This project is designed to interface the [LSM303DLHC](https://www.st.com/resource/en/datasheet/lsm303dlhc.pdf) using C language.

## Usage

### Shims
First update I2C shim functions to let this library communicate with your system:

>**Note**: These shims are initially design to work in Zephyr RTOS using the build flag PLATFORM_ZEPHYR.

For I2C Write: 

```
/**
 * @brief Writes data to a register of the LSM303 device over I2C.
 *
 * This function writes data to a specified register of the LSM303 device
 * using the provided data buffer.
 *
 * @param device      Pointer to the LSM303 device structure.
 * @param address     I2C address of the LSM303 device.
 * @param data_buffer Pointer to the buffer containing the data to be written.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t lsm303_i2c_write(lsm303_dev *device, uint8_t address,
                        uint8_t *data_buffer) {
#ifdef PLATFORM_ZEPHYR
  uint32_t bytecount = 2;
  return i2c_write(device->i2c0_dev, data_buffer, bytecount, address);
#endif
}
```

FOr I2C Read:

```
/**
 * @brief Reads a register value from the LSM303 device via I2C.
 *
 * Reads a single register from the LSM303 device and stores the retrieved value
 * in the specified buffer.
 *
 * @param device      Pointer to the LSM303 device structure.
 * @param address     I2C address of the LSM303 device.
 * @param reg         Register address to read from.
 * @param read_data   Pointer to the buffer where the read value will be stored.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t lsm303_i2c_read(lsm303_dev *device, uint8_t address, uint8_t reg,
                       uint8_t *read_data) {
#ifdef PLATFORM_ZEPHYR
  uint32_t bytecount = 1;

  if (i2c_write(device->i2c0_dev, &reg, bytecount, address) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  if (i2c_read(device->i2c0_dev, read_data, sizeof(*read_data), address) != 0) {
    return LSM303_STATUS_API_ERR;
  }

  return LSM303_STATUS_SUCCESS;
#endif
```

### Application

You may refer to [mcu-dev/dev-apps](https://github.com/mcu-dev/dev-apps) repository written for Zephyr RTOS for a sample application.

```
static lsm303_dev dev;
static lsm303_init_param dev_param;
static lsm303_axes_data lsm303_acc_data;
uint8_t addr = 0x00;

int main(void){
    dev_param.acc_power_mode = ACC_NORMAL;
    dev_param.acc_odr = ACC_ODR_1HZ;
    dev_param.acc_scale = ACC_SCALE_8G;
    dev_param.acc_resolution = ACC_RESOLUTION_LOW;
    dev_param.acc_axes_config.acc_axes = ACC_AXES_ENABLE_XYZ;

    if (lsm303_setup(&dev, dev_param) == LSM303_STATUS_SUCCESS) {
        /* Successful */
    } else {
        /* Error occur */
    }

    while(1){
        if (lsm303_data_ready(&dev) == LSM303_STATUS_SUCCESS) {
            if (dev.acc_axes_config.enable.x) {
                if (dev.acc_axes_config.ready.x) {
                    if (lsm303_get_x_raw_data(&dev, &lsm303_acc_data) ==
                        LSM303_STATUS_SUCCESS) {
                        /* Get raw value or get G equivalent */
                    }
                }
            }

            if (dev.acc_axes_config.enable.y) {
                if (dev.acc_axes_config.ready.y) {
                    if (lsm303_get_y_raw_data(&dev, &lsm303_acc_data) ==
                        LSM303_STATUS_SUCCESS) {
                        /* Get raw value or get G equivalent */
                    }
                }
            }

            if (dev.acc_axes_config.enable.z) {
                if (dev.acc_axes_config.ready.z) {
                    if (lsm303_get_z_raw_data(&dev, &lsm303_acc_data) ==
                        LSM303_STATUS_SUCCESS) {
                        /* Get raw value or get G equivalent */
                    }
                }
            }
        }
        /* Delay to handle polling */
    }
    return 0;
}

```

## License

Licensed under the MIT license.