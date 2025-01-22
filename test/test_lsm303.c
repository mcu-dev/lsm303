#include "stdint.h"

#include "cmock.h"
#include "unity.h"

#include "lsm303.h"
#include "mock_i2c.h"
#include "test_helper.h"

static lsm303_dev dev;
static lsm303_init_param init_param;
static lsm303_axes_data lsm303_acc_data;

void setUp(void) {
  init_param.acc_power_mode           = ACC_NORMAL;
  init_param.acc_odr                  = ACC_ODR_1HZ;
  init_param.acc_scale                = ACC_SCALE_8G;
  init_param.acc_resolution           = ACC_RESOLUTION_LOW;
  init_param.acc_axes_config.acc_axes = ACC_AXES_ENABLE_XYZ;
}

void test_lsm303_setup(void) {
  uint8_t power_mode_return     = 0x07;
  uint8_t enable_axes_return    = 0x07;
  uint8_t set_odr_return        = 0x07;
  uint8_t set_scale_return      = 0x00;
  uint8_t set_resolution_return = 0x20;

  uint8_t power_buffer[]      = {CTRL_REG1_A, 0x07};
  uint8_t axes_buffer[]       = {CTRL_REG1_A, 0x07};
  uint8_t odr_buffer[]        = {CTRL_REG1_A, 0x17};
  uint8_t scale_buffer[]      = {CTRL_REG4_A, 0x20};
  uint8_t resolution_buffer[] = {CTRL_REG4_A, 0x1F};

  i2c_init_ExpectAndReturn(true);

  // POWER MODE
  i2c_read_byte_ExpectAndReturn(ACC_I2C_ADDRESS, CTRL_REG1_A, NULL,
                                LSM303_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&power_mode_return);
  i2c_write_byte_ExpectAndReturn(ACC_I2C_ADDRESS, power_buffer,
                                  LSM303_STATUS_SUCCESS);

  // AXES
  i2c_read_byte_ExpectAndReturn(ACC_I2C_ADDRESS, CTRL_REG1_A, NULL,
                                LSM303_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&enable_axes_return);
  i2c_write_byte_ExpectAndReturn(ACC_I2C_ADDRESS, axes_buffer,
                                  LSM303_STATUS_SUCCESS);

  // ODR
  i2c_read_byte_ExpectAndReturn(ACC_I2C_ADDRESS, CTRL_REG1_A, NULL,
                                LSM303_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&set_odr_return);
  i2c_write_byte_ExpectAndReturn(ACC_I2C_ADDRESS, odr_buffer,
                                  LSM303_STATUS_SUCCESS);

  // SCALE
  i2c_read_byte_ExpectAndReturn(ACC_I2C_ADDRESS, CTRL_REG4_A, NULL,
                                LSM303_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&set_scale_return);
  i2c_write_byte_ExpectAndReturn(ACC_I2C_ADDRESS, scale_buffer,
                                  LSM303_STATUS_SUCCESS);

  // RESOLUTION
  i2c_read_byte_ExpectAndReturn(ACC_I2C_ADDRESS, CTRL_REG4_A, NULL,
                                LSM303_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&set_resolution_return);
  ;
  i2c_write_byte_ExpectAndReturn(ACC_I2C_ADDRESS, resolution_buffer,
                                  LSM303_STATUS_SUCCESS);

  TEST_ASSERT_EQUAL(LSM303_STATUS_SUCCESS, lsm303_setup(&dev, init_param));
  TEST_ASSERT_EQUAL(ACC_NORMAL, dev.acc_power_mode);
  TEST_ASSERT_EQUAL(ACC_ODR_1HZ, dev.acc_odr);
  TEST_ASSERT_EQUAL(ACC_AXES_ENABLE_XYZ, dev.acc_axes_config.acc_axes);
  TEST_ASSERT_EQUAL(ACC_SCALE_8G, dev.acc_scale);
  TEST_ASSERT_EQUAL(ACC_RESOLUTION_LOW, dev.acc_resolution);
  TEST_ASSERT_TRUE(dev.is_Setup);
}

void test_lsm303_set_power_mode(void) {
  uint8_t read_data_result       = 0x17;
  uint8_t expected_data_buffer[] = {CTRL_REG1_A, 0x17};
  set_expected_i2c_write_data(expected_data_buffer,
                              sizeof(expected_data_buffer));

  i2c_read_byte_ExpectAndReturn(ACC_I2C_ADDRESS, CTRL_REG1_A, NULL,
                                LSM303_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  i2c_write_byte_ExpectAndReturn(ACC_I2C_ADDRESS, NULL, LSM303_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();
  i2c_write_byte_StubWithCallback(i2c_write_byte_callback);

  TEST_ASSERT_EQUAL(LSM303_STATUS_SUCCESS,
                    lsm303_set_power_mode(&dev, ACC_NORMAL));
  TEST_ASSERT_EQUAL(dev.acc_power_mode, ACC_NORMAL);
}

void test_lsm303_acc_enable_axes(void) {
  uint8_t read_data_result       = 0x10;
  uint8_t expected_data_buffer[] = {CTRL_REG1_A, 0x17};
  set_expected_i2c_write_data(expected_data_buffer,
                              sizeof(expected_data_buffer));

  i2c_read_byte_ExpectAndReturn(ACC_I2C_ADDRESS, CTRL_REG1_A, NULL,
                                LSM303_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  i2c_write_byte_ExpectAndReturn(ACC_I2C_ADDRESS, NULL, LSM303_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();
  i2c_write_byte_StubWithCallback(i2c_write_byte_callback);

  TEST_ASSERT_EQUAL(LSM303_STATUS_SUCCESS,
                    lsm303_acc_enable_axes(&dev, init_param.acc_axes_config));
  TEST_ASSERT_EQUAL(dev.acc_axes_config.acc_axes, ACC_AXES_ENABLE_XYZ);
}

void test_lsm303_acc_set_odr(void) {
  uint8_t read_data_result       = 0x17;
  uint8_t expected_data_buffer[] = {CTRL_REG1_A, 0x77};
  set_expected_i2c_write_data(expected_data_buffer,
                              sizeof(expected_data_buffer));

  i2c_read_byte_ExpectAndReturn(ACC_I2C_ADDRESS, CTRL_REG1_A, NULL,
                                LSM303_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  i2c_write_byte_ExpectAndReturn(ACC_I2C_ADDRESS, NULL, LSM303_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();
  i2c_write_byte_StubWithCallback(i2c_write_byte_callback);

  TEST_ASSERT_EQUAL(LSM303_STATUS_SUCCESS,
                    lsm303_acc_set_odr(&dev, ACC_AXES_ENABLE_XYZ));
  TEST_ASSERT_EQUAL(dev.acc_odr, ACC_AXES_ENABLE_XYZ);
}

void test_lsm303_acc_set_scale(void) {
  uint8_t read_data_result       = 0x00;
  uint8_t expected_data_buffer[] = {CTRL_REG4_A, 0x20};
  set_expected_i2c_write_data(expected_data_buffer,
                              sizeof(expected_data_buffer));

  i2c_read_byte_ExpectAndReturn(ACC_I2C_ADDRESS, CTRL_REG4_A, NULL,
                                LSM303_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  i2c_write_byte_ExpectAndReturn(ACC_I2C_ADDRESS, NULL, LSM303_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();
  i2c_write_byte_StubWithCallback(i2c_write_byte_callback);

  TEST_ASSERT_EQUAL(LSM303_STATUS_SUCCESS,
                    lsm303_acc_set_scale(&dev, ACC_SCALE_8G));
  TEST_ASSERT_EQUAL(dev.acc_scale, ACC_SCALE_8G);
}

void test_lsm303_acc_set_resolution(void) {
  uint8_t read_data_result       = 0x00;
  uint8_t expected_data_buffer[] = {CTRL_REG4_A, 0x08};
  set_expected_i2c_write_data(expected_data_buffer,
                              sizeof(expected_data_buffer));

  i2c_read_byte_ExpectAndReturn(ACC_I2C_ADDRESS, CTRL_REG4_A, NULL,
                                LSM303_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  i2c_write_byte_ExpectAndReturn(ACC_I2C_ADDRESS, NULL, LSM303_STATUS_SUCCESS);
  i2c_write_byte_IgnoreArg_data_buffer();
  i2c_write_byte_StubWithCallback(i2c_write_byte_callback);

  TEST_ASSERT_EQUAL(LSM303_STATUS_SUCCESS,
                    lsm303_acc_set_resolution(&dev, ACC_RESOLUTION_HIGH));
  TEST_ASSERT_EQUAL(dev.acc_resolution, ACC_RESOLUTION_HIGH);
}

void test_lsm303_data_ready(void) {
  uint8_t read_data_result = 0x0F;

  i2c_read_byte_ExpectAndReturn(ACC_I2C_ADDRESS, STATUS_REG_A, NULL,
                                LSM303_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  TEST_ASSERT_EQUAL(LSM303_STATUS_SUCCESS, lsm303_data_ready(&dev));
  TEST_ASSERT_TRUE(dev.acc_axes_config.ready.x);
  TEST_ASSERT_TRUE(dev.acc_axes_config.ready.y);
  TEST_ASSERT_TRUE(dev.acc_axes_config.ready.z);
}

void test_lsm303_get_x_raw_data(void) {
  uint8_t val_l = 0x00;
  uint8_t val_h = 0x00;

  i2c_read_byte_ExpectAndReturn(ACC_I2C_ADDRESS, OUT_X_H_A, NULL,
                                LSM303_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_h);

  i2c_read_byte_ExpectAndReturn(ACC_I2C_ADDRESS, OUT_X_L_A, NULL,
                                LSM303_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_l);

  TEST_ASSERT_EQUAL(LSM303_STATUS_SUCCESS,
                    lsm303_get_x_raw_data(&dev, &lsm303_acc_data));
}

void test_lsm303_get_y_raw_data(void) {
  uint8_t val_l = 0x00;
  uint8_t val_h = 0x00;

  i2c_read_byte_ExpectAndReturn(ACC_I2C_ADDRESS, OUT_Y_H_A, NULL,
                                LSM303_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_h);

  i2c_read_byte_ExpectAndReturn(ACC_I2C_ADDRESS, OUT_Y_L_A, NULL,
                                LSM303_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_l);

  TEST_ASSERT_EQUAL(LSM303_STATUS_SUCCESS,
                    lsm303_get_y_raw_data(&dev, &lsm303_acc_data));
}

void test_lsm303_get_z_raw_data(void) {
  uint8_t val_l = 0x00;
  uint8_t val_h = 0x00;

  i2c_read_byte_ExpectAndReturn(ACC_I2C_ADDRESS, OUT_Z_H_A, NULL,
                                LSM303_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_h);

  i2c_read_byte_ExpectAndReturn(ACC_I2C_ADDRESS, OUT_Z_L_A, NULL,
                                LSM303_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&val_l);

  TEST_ASSERT_EQUAL(LSM303_STATUS_SUCCESS,
                    lsm303_get_z_raw_data(&dev, &lsm303_acc_data));
}