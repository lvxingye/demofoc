cmake_minimum_required(VERSION 3.22)

add_compile_options(-Wno-missing-field-initializers)

# set library relative variables
set(FOC_LIBRARY_ROOT ${CMAKE_CURRENT_LIST_DIR})

# platform driver
include(${FOC_LIBRARY_ROOT}/cmake/platform/esp32.cmake)

# device driver library
add_library(foc-device STATIC
    ${FOC_LIBRARY_ROOT}/devices/encoder/as5600/as5600.cpp
    ${FOC_LIBRARY_ROOT}/devices/wave_gen/onchip/onchip.cpp
)
target_include_directories(foc-device PRIVATE ${FOC_LIBRARY_ROOT})
target_link_libraries(foc-device PRIVATE driver)

# core algorithm library
add_library(foc-core STATIC
    ${FOC_LIBRARY_ROOT}/core/core.cpp
    ${FOC_LIBRARY_ROOT}/core/algorithm.cpp
)
target_include_directories(foc-core PRIVATE ${FOC_LIBRARY_ROOT})
target_link_libraries(foc-core PRIVATE foc-device)

add_library(foc INTERFACE)
target_include_directories(foc-device PUBLIC ${FOC_LIBRARY_ROOT})
target_link_libraries(foc INTERFACE
    foc-core
    foc-device
    driver
)

