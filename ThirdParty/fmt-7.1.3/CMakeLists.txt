
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -std=c++11 -O3 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -O3 -march=native")


file(GLOB SRC_fmt  "src/*.c*")
include_directories("./include")
add_library(fmt SHARED ${SRC_fmt})

set(fmt_INCLUDE_DIRS
    ./include
    PARENT_SCOPE
)
set(fmt_LIBS
    fmt
    PARENT_SCOPE
)

