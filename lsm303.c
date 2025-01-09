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
uint8_t lsm303_setup(lsm303_dev **device, lsm303_init_param lsm303_params) {
  uint8_t ret;

  lsm303_dev *dev;

  dev->acc_power_mode  = lsm303_params.acc_power_mode;
  dev->acc_odr         = lsm303_params.acc_odr;
  dev->acc_axes_config = lsm303_params.acc_axes_config;
  dev->acc_scale       = lsm303_params.acc_scale;

  ret |= lsm303_set_power_mode(dev, dev->acc_power_mode);
  ret |= lsm303_acc_enable_axes(dev, dev->acc_axes_config);
  ret |= lsm303_acc_set_odr(dev, dev->acc_odr);
  ret |= lsm303_acc_set_scale(dev, dev->acc_scale);

  *device = dev;

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
uint8_t lsm303_set_power_mode(lsm303_dev *device,
                              enum lsm303_acc_power_mode mode) {
  uint8_t val = 0x00;
  uint8_t data_buffer[2];

  if (lsm303_i2c_read(&device, ACC_I2C_ADDRESS, CTRL_REG1_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  if (mode == ACC_POWER_DOWN) {
    val &= ~0xF0;
  } else {
    val = val | mode << ACC_POWER_MODE_MASK;
  }

  data_buffer[1] = CTRL_REG1_A;
  data_buffer[0] = val;

  if (device->is_Setup) {
    device->acc_power_mode = mode;
  }

  return lsm303_i2c_write(&device, ACC_I2C_ADDRESS, data_buffer);
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
uint8_t lsm303_acc_enable_axes(lsm303_dev *device,
                               lsm303_acc_axes_config axes) {
  uint8_t val = 0x00;
  uint8_t data_buffer[2];

  if (lsm303_i2c_read(&device, ACC_I2C_ADDRESS, CTRL_REG1_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  val &= ~0x07;
  val = val | axes.acc_axes << ACC_AXES_MASK;

  data_buffer[1] = CTRL_REG1_A;
  data_buffer[0] = val;

  axes.enable.z = val & (1 << 2);
  axes.enable.y = val & (1 << 1);
  axes.enable.x = val & 1;

  if (device->is_Setup) {
    device->acc_axes_config = axes;
  }

  return lsm303_i2c_write(&device, ACC_I2C_ADDRESS, data_buffer);
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
uint8_t lsm303_acc_set_odr(lsm303_dev *device, enum lsm303_acc_odr odr) {
  uint8_t val = 0x00;
  uint8_t data_buffer[2];

  if (lsm303_i2c_read(&device, ACC_I2C_ADDRESS, CTRL_REG1_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  val &= ~0xF0;
  val = val | odr << ACC_ODR_MASK;

  data_buffer[1] = CTRL_REG1_A;
  data_buffer[0] = val;

  if (device->is_Setup) {
    device->acc_odr = odr;
  }

  return lsm303_i2c_write(&device, ACC_I2C_ADDRESS, data_buffer);
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
uint8_t lsm303_acc_set_scale(lsm303_dev *device,
                             enum lsm303_acc_full_scale scale) {
  uint8_t val = 0x00;
  uint8_t data_buffer[2];

  if (lsm303_i2c_read(&device, ACC_I2C_ADDRESS, CTRL_REG1_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  val &= ~0x18;
  val = val | scale << ACC_SCALE_MASK;

  data_buffer[1] = CTRL_REG4_A;
  data_buffer[0] = val;

  if (device->is_Setup) {
    device->acc_scale = scale;
  }

  return lsm303_i2c_write(&device, ACC_I2C_ADDRESS, data_buffer);
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
uint8_t lsm303_data_ready(lsm303_dev *device) {
  uint8_t val = 0x00;

  if (lsm303_i2c_read(&device, ACC_I2C_ADDRESS, STATUS_REG_A, &val) !=
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
uint8_t lsm303_get_x_data(lsm303_dev *device, lsm303_axes_data *accel_data) {
  uint8_t val = 0x00;

  if (lsm303_i2c_read(&device, ACC_I2C_ADDRESS, OUT_X_H_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  accel_data->x = (int16_t)(val << 8);

  if (lsm303_i2c_read(&device, ACC_I2C_ADDRESS, OUT_X_L_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  accel_data->x |= (int16_t)val;

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
uint8_t lsm303_get_y_data(lsm303_dev *device, lsm303_axes_data *accel_data) {
  uint8_t val = 0x00;

  if (lsm303_i2c_read(&device, ACC_I2C_ADDRESS, OUT_Y_H_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  accel_data->y = (int16_t)(val << 8);

  if (lsm303_i2c_read(&device, ACC_I2C_ADDRESS, OUT_Y_L_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  accel_data->y |= (int16_t)val;

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
uint8_t lsm303_get_z_data(lsm303_dev *device, lsm303_axes_data *accel_data) {
  uint8_t val = 0x00;

  if (lsm303_i2c_read(&device, ACC_I2C_ADDRESS, OUT_Z_H_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  accel_data->z = (int16_t)(val << 8);

  if (lsm303_i2c_read(&device, ACC_I2C_ADDRESS, OUT_Z_L_A, &val) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  accel_data->z |= (int16_t)val;

  return LSM303_STATUS_SUCCESS;
}

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
uint8_t lsm303_i2c_read(lsm303_dev *device, uint8_t address, uint8_t reg,
                        uint8_t *read_data) {
  uint8_t ret = 0;

  // TO DO: I2C communication
  return ret;
}

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
uint8_t lsm303_i2c_write(lsm303_dev *device, uint8_t address,
                         uint8_t *data_buffer) {
  uint8_t ret = 0;

  // TO DO: I2C communication
  return ret;
}
