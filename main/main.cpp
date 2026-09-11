#include "core/core.hpp"
#include "devices/encoder/common.hpp"
#include "platform/common.hpp"
#include <cstdio>

extern "C" {
void app_main(void);
}

void app_main(void) {
    foc_t foc_instance;

    foc_instance.init();
    foc_instance.test_correct_phase();

    while (true) {
        foc_instance.loop();
    }
}
