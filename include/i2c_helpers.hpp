
#pragma once

#include <stdint.h>
#include <vector>
#include "hardware/i2c.h"

int write_i2c(int i2c_port, int addr, const std::vector<uint8_t> &bytes, bool nostop = false);

int read_i2c(int i2c_port, int addr, const std::vector<uint8_t> &write_bytes,
             int bytes_to_read, std::vector<uint8_t> &read_bytes);