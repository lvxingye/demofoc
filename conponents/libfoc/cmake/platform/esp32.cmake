add_library(libplat STATIC ${FOC_LIBRARY_ROOT}/platform/esp32/esp32.cpp)
target_include_directories(libplat PRIVATE ${FOC_LIBRARY_ROOT})

target_link_libraries(libplat PRIVATE
    idf::esp_driver_i2c
    idf::esp_driver_gpio
    idf::esp_driver_mcpwm
    idf::driver)
