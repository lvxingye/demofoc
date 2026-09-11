cmake_minimum_required(VERSION 3.10)


set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # 生成 compile_commands.json

set(PROJECT_ROOT ${CMAKE_SOURCE_DIR})

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# include(/home/lvxy/.espressif/v6.0.2/esp-idf/tools/cmake/project.cmake)

project(foc_esp32)

add_compile_options(-Wno-missing-field-initializers)

