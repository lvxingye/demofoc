idf_component_register(
    SRCS ${FOC_LIBRARY_ROOT}/core/core.cpp
    INCLUDE_DIRS 
)

add_subdirectory(${FOC_LIBRARY_ROOT}/libfoc)

target_link_libraries(main PRIVATE libfoc)