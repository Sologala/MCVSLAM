find_package(Threads)

SET (SRCS
    src/BowVector.cpp
    src/Database.cpp
    src/DescManip.cpp
    src/FeatureVector.cpp
    src/QueryResults.cpp
    src/quicklz.c
    src/ScoringObject.cpp
    src/Vocabulary.cpp
    )

add_library(
    Dbow3 STATIC
    ${SRCS}
)

target_include_directories(
    Dbow3 PUBLIC
    src
)
target_link_libraries(
    Dbow3
    ${OpenCV_LIBRARIES}
)

