cmake_minimum_required(VERSION 3.10)


set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # 生成 compile_commands.json

# 项目内所有 include 均相对仓库根目录（如 "utils/utils.hpp"、"platform/common.hpp"）。
# 放在 project() 之前，使 IDF 组件与 libesp32 都继承该包含路径。
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(foc)
include(platform/esp32/CMakeLists.txt)