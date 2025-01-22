#include "test_helper.h"
#include "unity.h"

static uint8_t *expected_data    = NULL;
static size_t expected_data_size = 0;

void set_expected_i2c_write_data(uint8_t *expected_data_param, size_t size) {
  expected_data      = expected_data_param;
  expected_data_size = size;
}

signed char i2c_write_byte_callback(uint8_t address, uint8_t *data_buffer,
                                     int cmock_num_calls) {
  // Validate the address
  TEST_ASSERT_EQUAL(ACC_I2C_ADDRESS, address);

  // Validate the data buffer
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_data, data_buffer, expected_data_size);

  return LSM303_STATUS_SUCCESS;
}
