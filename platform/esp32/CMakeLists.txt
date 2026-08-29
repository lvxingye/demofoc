add_library(libesp32 ${CMAKE_CURRENT_LIST_DIR}/esp32.cpp)

target_link_libraries(libesp32 PRIVATE idf::esp_driver_i2c)
