add_subdirectory(ThirdParty/fmt-7.1.3)
set(yaml_cpp_INCLUDE_DIRS ThirdParty/mini-yaml/yaml)
add_library(yaml_cpp ThirdParty/mini-yaml/yaml/Yaml.cpp)
set(yaml_cpp_LIBS yaml_cpp)

include_directories(
    ${yaml_cpp_INCLUDE_DIRS}
    ${fmt_INCLUDE_DIRS}
)
link_libraries(
    ${yaml_cpp_LIBS}
    ${fmt_LIBS}
)