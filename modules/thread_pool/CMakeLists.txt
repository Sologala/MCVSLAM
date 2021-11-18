add_library(thread_pool STATIC 
thread_pool.cpp)
target_include_directories(
    thread_pool PUBLIC
    .
)
