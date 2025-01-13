# LSM303DLHC Support Library in C

This project is designed to interface the [LSM303DLHC](https://www.st.com/resource/en/datasheet/lsm303dlhc.pdf) using C language.

## Usage

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