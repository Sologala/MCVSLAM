
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -std=c++11 -O3 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -O3 -march=native")


file(GLOB SRC_fmt  "src/*.c*")

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories("./include")
add_library(fmt SHARED ${SRC_fmt})
