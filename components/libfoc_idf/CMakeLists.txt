idf_component_register(
    INCLUDE_DIRS "libfoc"
)

add_subdirectory(libfoc)

target_link_libraries(${COMPONENT_LIB} INTERFACE libfoc)
# target_link_libraries(main PRIVATE libfoc)