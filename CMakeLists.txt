cmake_minimum_required(VERSION 3.10)


set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # 生成 compile_commands.json

set(PROJECT_ROOT ${CMAKE_SOURCE_DIR})

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# include(/home/lvxy/.espressif/v6.0.2/esp-idf/tools/cmake/project.cmake)

project(foc_esp32)

include(${PROJECT_ROOT}/cmake/libfoc.cmake)