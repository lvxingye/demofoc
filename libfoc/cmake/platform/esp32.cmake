add_library(libesp32 ${CMAKE_SOURCE_DIR}/esp32.cpp)

target_link_libraries(libesp32 PRIVATE 
                                idf::esp_driver_i2c 
                                idf::esp_driver_gpio 
                                idf::esp_driver_mcpwm 
                                idf::driver)
