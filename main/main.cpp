#include "core/core.hpp"
#include "devices/encoder/common.hpp"
#include "platform/common.hpp"

void app_main(void){
    foc_t foc_instance;

    foc_instance.init();

    while (true) {
        foc_instance.loop();
    }
}