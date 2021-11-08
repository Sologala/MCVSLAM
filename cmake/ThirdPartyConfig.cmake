add_subdirectory(ThirdParty/fmt-7.1.3)
add_subdirectory(ThirdParty/yaml_cpp)

include_directories(
    ${yaml_cpp_INCLUDE_DIRS}
    ${fmt_INCLUDE_DIRS}
)
link_libraries(
    ${yaml_cpp_LIBS}
    ${fmt_LIBS}
)