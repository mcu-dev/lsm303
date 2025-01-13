#ifndef TEST_HELPERS_H
#define TEST_HELPERS_H

#include <stddef.h>
#include <stdint.h>

#include "lsm303.h"

// Declare the generalized callback function
void set_expected_i2c_write_data(uint8_t *expected_data, size_t size);
signed char i2c_write_bytes_callback(uint8_t address, uint8_t *data_buffer,
                                     int cmock_num_calls);

#endif // TEST_HELPERS_H
