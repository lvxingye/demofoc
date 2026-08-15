#include <cstdint>
#include "utils/utils.hpp"

namespace pltf{
    class i2c{
        i2c();
        ~i2c();

        ret_val send_byte(const uint8_t dev_addr,const uint8_t data);
        ret_val send_bytes(const uint8_t dev_addr,const trans_data_t& data);
        ret_val recv_byte(const uint8_t dev_addr,const uint8_t data);
        ret_val recv_bytes(const uint8_t dev_addr,const trans_data_t& data);

    };
    // class spi;
    class uart{

        uart();
        ~uart();

        ret_val send_bytes(const uint8_t dev_addr,const trans_data_t& data);
        ret_val recv_bytes(const uint8_t dev_addr,const trans_data_t& data);
        
    };
    class pwm{
        pwm();
        ~pwm();

    };



}