/**
 ******************************************************************************
 * @file    LSM303.c
 * @author  - Anthony E.Raterta
 * @version V1.0.0
 * @date    13-September-2024
 * @brief   Contains all the functionalities to control the LSM303DLHC
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 mcu-dev
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "lsm303.h"

/**
 * @brief Initializes and sets up the LSM303 device.
 *
 * Allocates and initializes the device structure and configures it with the
 * specified initialization parameters.
 *
 * @param device        Double pointer to the LSM303 device structure to be
 * allocated and initialized.
 * @param lsm303_params Initialization parameters for the LSM303 device.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t lsm303_setup(lsm303_dev *dev, lsm303_init_param lsm303_params) {
  int8_t ret = 0;

  if (!i2c_init()) {
    return LSM303_STATUS_INIT_ERR;
  }

  dev->acc_power_mode  = lsm303_params.acc_power_mode;
  dev->acc_odr         = lsm303_params.acc_odr;
  dev->acc_axes_config = lsm303_params.acc_axes_config;
  dev->acc_scale       = lsm303_params.acc_scale;
  dev->acc_resolution  = lsm303_params.acc_resolution;

  ret |= lsm303_set_power_mode(dev, dev->acc_power_mode);
  ret |= lsm303_acc_enable_axes(dev, dev->acc_axes_config);
  ret |= lsm303_acc_set_odr(dev, dev->acc_odr);
  ret |= lsm303_acc_set_scale(dev, dev->acc_scale);
  ret |= lsm303_acc_set_resolution(dev, dev->acc_resolution);

  if (ret == LSM303_STATUS_SUCCESS) {
    dev->is_Setup = true;
  } else {
    dev->is_Setup = false;
  }

  return ret;
}

/**
 * @brief Sets the accelerometer power mode.
 *
 * Configures the accelerometer's power mode to optimize power consumption
 * or performance based on the specified mode.
 *
 * @param device Pointer to the LSM303 device structure.
 * @param mode   Power mode to be configured for the accelerometer.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t lsm303_set_power_mode(lsm303_dev *device,
                             enum lsm303_acc_power_mode mode) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ACC_I2C_ADDRESS, CTRL_REG1_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  if (mode == ACC_POWER_DOWN) {
    val &= ~0xF0;
  } else {
    val &= ~0x08;
    val = val | mode << ACC_POWER_MODE_MASK;
  }
  device->acc_power_mode = mode;

  uint8_t data_buffer[] = {CTRL_REG1_A, val};
  return i2c_write_bytes(ACC_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Enables or disables specific accelerometer axes.
 *
 * Configures the accelerometer to enable or disable the specified axes
 * for data measurement.
 *
 * @param device Pointer to the LSM303 device structure.
 * @param axes   Axes configuration to enable or disable specific accelerometer
 * axes.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t lsm303_acc_enable_axes(lsm303_dev *device, lsm303_acc_axes_config axes) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ACC_I2C_ADDRESS, CTRL_REG1_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  val &= ~0x07;
  val           = val | axes.acc_axes << ACC_AXES_MASK;
  axes.enable.z = (val & (1 << 2)) >> 2;
  axes.enable.y = (val & (1 << 1)) >> 1;
  axes.enable.x = (val & 1);

  device->acc_axes_config = axes;

  uint8_t data_buffer[] = {CTRL_REG1_A, val};
  return i2c_write_bytes(ACC_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Configures the Output Data Rate (ODR) for the accelerometer.
 *
 * Sets the output data rate of the accelerometer to control how frequently
 * the device outputs data.
 *
 * @param device Pointer to the LSM303 device structure.
 * @param odr    Desired output data rate setting for the accelerometer.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t lsm303_acc_set_odr(lsm303_dev *device, enum lsm303_acc_odr odr) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ACC_I2C_ADDRESS, CTRL_REG1_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  if (odr == ACC_ODR_1K620HZ || odr == ACC_ODR_5K376HZ) {
    if (device->acc_power_mode != ACC_LOW_POWER) {
      return LSM303_STATUS_INPUT_ERR;
    }
  }

  val &= ~0xF0;
  if (odr == ACC_ODR_5K376HZ) {
    val = val | (odr - 1) << ACC_ODR_MASK;
  } else {
    val = val | odr << ACC_ODR_MASK;
  }

  device->acc_odr = odr;

  uint8_t data_buffer[] = {CTRL_REG1_A, val};
  return i2c_write_bytes(ACC_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Configures the accelerometer scale for the LSM303 device.
 *
 * Sets the full-scale range of the accelerometer based on the specified scale
 * parameter.
 *
 * @param device Pointer to the LSM303 device structure.
 * @param scale  Full-scale range setting for the accelerometer.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t lsm303_acc_set_scale(lsm303_dev *device,
                            enum lsm303_acc_full_scale scale) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ACC_I2C_ADDRESS, CTRL_REG4_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  val &= ~0x30;
  val = val | scale << ACC_SCALE_MASK;

  device->acc_scale = scale;

  uint8_t data_buffer[] = {CTRL_REG4_A, val};
  return i2c_write_bytes(ACC_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Configures the accelerometer resolution for the LSM303 device.
 *
 * Sets the resolution of the accelerometer based on the specified
 * resolution parameter.
 *
 * @param device Pointer to the LSM303 device structure.
 * @param resolution  Resolution setting for the accelerometer.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t lsm303_acc_set_resolution(lsm303_dev *device,
                                 enum lsm303_acc_resolution resolution) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ACC_I2C_ADDRESS, CTRL_REG4_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  val &= ~0x08;
  val = val | resolution << ACC_RESOLUTION_MASK;

  device->acc_resolution = resolution;

  uint8_t data_buffer[] = {CTRL_REG4_A, val};
  return i2c_write_bytes(ACC_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Check individual axes data ready
 *
 * Checks individual axes data ready depending on the axes enabled
 *
 * @param device Pointer to the LSM303 device structure.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t lsm303_data_ready(lsm303_dev *device) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ACC_I2C_ADDRESS, STATUS_REG_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  device->acc_axes_config.ready.z = val & (1 << 2);
  device->acc_axes_config.ready.y = val & (1 << 1);
  device->acc_axes_config.ready.x = val & 1;

  return LSM303_STATUS_SUCCESS;
}

/**
 * @brief Reads x-axis accelerometer data from the LSM303 device.
 *
 * Retrieves the X accelerometer data from the LSM303 device and stores
 * the values in the provided `lsm303_axes_data` structure.
 *
 * @param device      Pointer to the LSM303 device structure.
 * @param accel_data  Pointer to a structure where the accelerometer data (X, Y,
 * Z) will be stored.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t lsm303_get_x_raw_data(lsm303_dev *device, lsm303_axes_data *accel_data) {
  uint8_t val_l = 0x00;
  uint8_t val_h = 0x00;

  if (i2c_read_byte(ACC_I2C_ADDRESS, OUT_X_H_A, &val_h) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  if (i2c_read_byte(ACC_I2C_ADDRESS, OUT_X_L_A, &val_l) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  if (device->acc_resolution == ACC_RESOLUTION_LOW) {
    accel_data->x = (int16_t)(val_l | (val_h << 8)) >> 4;
  } else {
    if (device->acc_power_mode == ACC_NORMAL) {
      accel_data->x = (int16_t)(val_l | (val_h << 8));
    } else {
      accel_data->x = (int16_t)(val_l | (val_h << 8)) >> 2;
    }
  }

  return LSM303_STATUS_SUCCESS;
}

/**
 * @brief Reads y-axis accelerometer data from the LSM303 device.
 *
 * Retrieves the Y accelerometer data from the LSM303 device and stores
 * the values in the provided `lsm303_axes_data` structure.
 *
 * @param device      Pointer to the LSM303 device structure.
 * @param accel_data  Pointer to a structure where the accelerometer data (X, Y,
 * Z) will be stored.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t lsm303_get_y_raw_data(lsm303_dev *device, lsm303_axes_data *accel_data) {
  uint8_t val_l = 0x00;
  uint8_t val_h = 0x00;

  if (i2c_read_byte(ACC_I2C_ADDRESS, OUT_Y_H_A, &val_h) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  if (i2c_read_byte(ACC_I2C_ADDRESS, OUT_Y_L_A, &val_l) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  if (device->acc_resolution == ACC_RESOLUTION_LOW) {
    accel_data->y = (int16_t)(val_l | (val_h << 8)) >> 4;
  } else {
    if (device->acc_power_mode == ACC_NORMAL) {
      accel_data->y = (int16_t)(val_l | (val_h << 8));
    } else {
      accel_data->y = (int16_t)(val_l | (val_h << 8)) >> 2;
    }
  }

  return LSM303_STATUS_SUCCESS;
}

/**
 * @brief Reads z-axis accelerometer data from the LSM303 device.
 *
 * Retrieves the Z accelerometer data from the LSM303 device and stores
 * the values in the provided `lsm303_axes_data` structure.
 *
 * @param device      Pointer to the LSM303 device structure.
 * @param accel_data  Pointer to a structure where the accelerometer data (X, Y,
 * Z) will be stored.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t lsm303_get_z_raw_data(lsm303_dev *device, lsm303_axes_data *accel_data) {
  uint8_t val_l = 0x00;
  uint8_t val_h = 0x00;

  if (i2c_read_byte(ACC_I2C_ADDRESS, OUT_Z_H_A, &val_h) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  if (i2c_read_byte(ACC_I2C_ADDRESS, OUT_Z_L_A, &val_l) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  if (device->acc_resolution == ACC_RESOLUTION_LOW) {
    accel_data->z = (int16_t)(val_l | (val_h << 8)) >> 4;
  } else {
    if (device->acc_power_mode == ACC_NORMAL) {
      accel_data->z = (int16_t)(val_l | (val_h << 8));
    } else {
      accel_data->z = (int16_t)(val_l | (val_h << 8)) >> 2;
    }
  }

  return LSM303_STATUS_SUCCESS;
}

/**
 * @brief Converts raw accelerometer data to acceleration in g units.
 *
 * This function converts a raw accelerometer value from the LSM303 device
 * to acceleration in g units, considering the device's sensitivity
 * and configuration settings.
 *
 * @param device     Pointer to the LSM303 device structure containing
 * configuration details such as sensitivity.
 * @param raw_value  The raw accelerometer data to be converted.
 *
 * @return
 *   The acceleration value in g units as a float.
 */
float lsm303_convert_raw_to_g(lsm303_dev *device, int16_t raw_value) {
  float sensitivity = 0.0;

  if (device->acc_resolution == ACC_RESOLUTION_LOW) {
    if (device->acc_power_mode == ACC_NORMAL) {
      // NORMAL MODE / LOW RESOLUTION
      switch (device->acc_scale) {
      case ACC_SCALE_2G:
        sensitivity = 1.0;
        break;
      case ACC_SCALE_4G:
        sensitivity = 2.0;
        break;
      case ACC_SCALE_8G:
        sensitivity = 4.0;
        break;
      case ACC_SCALE_16G:
        sensitivity = 12.0;
        break;
      }
    } else {
      // LOW POWER MODE / LOW RESOLUTION
      switch (device->acc_scale) {
      case ACC_SCALE_2G:
        sensitivity = 1.0;
        break;
      case ACC_SCALE_4G:
        sensitivity = 2.0;
        break;
      case ACC_SCALE_8G:
        sensitivity = 4.0;
        break;
      case ACC_SCALE_16G:
        sensitivity = 8.0;
        break;
      }
    }
  } else {
    if (device->acc_power_mode == ACC_NORMAL) {
      // NORMAL MODE / HIGH RESOLUTION
      switch (device->acc_scale) {
      case ACC_SCALE_2G:
        sensitivity = 0.0625;
        break;
      case ACC_SCALE_4G:
        sensitivity = 0.125;
        break;
      case ACC_SCALE_8G:
        sensitivity = 0.25;
        break;
      case ACC_SCALE_16G:
        sensitivity = 0.5;
        break;
      }
    } else {
      // LOW POWER MODE / LOW RESOLUTION
      switch (device->acc_scale) {
      case ACC_SCALE_2G:
        sensitivity = 0.25;
        break;
      case ACC_SCALE_4G:
        sensitivity = 0.5;
        break;
      case ACC_SCALE_8G:
        sensitivity = 1.0;
        break;
      case ACC_SCALE_16G:
        sensitivity = 2.0;
        break;
      }
    }
  }

  return (float)raw_value * sensitivity / (float)1000.0;
}
