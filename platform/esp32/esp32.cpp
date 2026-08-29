#include "platform/common.hpp"
#include "driver/i2c_master.h"

// init and deinit functions
i2c::i2c() {}
i2c::~i2c() {}

// data transfer functions
ret_val send_byte(const uint8_t dev_addr, const uint8_t data){

}
ret_val send_bytes(const uint8_t dev_addr, const trans_data_t& data){

}
ret_val recv_byte(const uint8_t dev_addr, const uint8_t data){}
ret_val recv_bytes(const uint8_t dev_addr, const trans_data_t& data){}
