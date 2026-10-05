add_library(driver STATIC ${FOC_LIBRARY_ROOT}/platform/esp32/esp32.cpp)
target_include_directories(driver PRIVATE ${FOC_LIBRARY_ROOT})

target_link_libraries(driver PRIVATE
    idf::esp_driver_i2c
    idf::esp_driver_gpio
    idf::esp_driver_mcpwm
    idf::driver
    idf::esp_adc)
