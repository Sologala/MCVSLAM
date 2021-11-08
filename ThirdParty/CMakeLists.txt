
add_subdirectory(fmt-7.1.3)
add_subdirectory(yaml_cpp)
include(
    ${yaml_cpp_INCLUDE_DIRS}
    ${fmt_INCLUDE_DIRS}
)
link_libraries(
    ${yaml_cpp_LIBS}
    ${fmt_LIBS}
)