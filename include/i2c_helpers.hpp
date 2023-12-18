
#pragma once

#include <stdint.h>
// #include <vector>
#include <array>
#include <vector>
#include <deque>
#include <utility>
#include <cstddef>
#include "hardware/i2c.h"

template <typename T>
int write_i2c_t(int i2c_port, int addr, const T &bytes, bool nostop = false){
     i2c_inst_t *i2c;

    if (i2c_port)
    {
        i2c = i2c1;
    }
    else
    {
        i2c = i2c0;
    }
    int i2c_sdk_call_return_value =
        i2c_write_blocking_until(i2c, (uint8_t)addr, bytes.data(), bytes.size(),
                                 nostop, make_timeout_time_ms(50));
    return i2c_sdk_call_return_value == bytes.size();
}

int write_i2c(int i2c_port, int addr, const std::vector<uint8_t> &bytes, bool nostop = false);

int read_i2c(int i2c_port, int addr, const std::vector<uint8_t> &write_bytes,
             int bytes_to_read, std::vector<uint8_t> &read_bytes);

uint32_t millis();