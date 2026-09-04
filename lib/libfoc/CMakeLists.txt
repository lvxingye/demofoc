cmake_minimum_required(VERSION 3.22)

# set library relative variables
set(FOC_LIBRARY_ROOT ${CMAKE_CURRENT_LIST_DIR})

# platform driver
include(${FOC_LIBRARY_ROOT}/cmake/platform/esp32.cmake)

# device driver library
add_library(libfoc_dev STATIC
    ${FOC_LIBRARY_ROOT}/devices/encoder/as5600/as5600.cpp
    ${FOC_LIBRARY_ROOT}/devices/wave_gen/onchip/onchip.cpp
)
target_include_directories(libfoc_dev PRIVATE ${FOC_LIBRARY_ROOT})
target_link_libraries(libfoc_dev PRIVATE libplat)

# core algorithm library
add_library(libfoc_core STATIC
    ${FOC_LIBRARY_ROOT}/core/core.cpp
    ${FOC_LIBRARY_ROOT}/core/algorithm.cpp
)
target_include_directories(libfoc_dev PRIVATE ${FOC_LIBRARY_ROOT})
target_link_libraries(libfoc_core PRIVATE libfoc_dev)

add_library(libfoc INTERFACE)
target_include_directories(libfoc_dev PUBLIC ${FOC_LIBRARY_ROOT})
target_link_libraries(libfoc INTERFACE
    libfoc_core
    libfoc_dev
    libplat
)

