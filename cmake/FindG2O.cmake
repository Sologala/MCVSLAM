set(g2o_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/installed/lib/cmake/g2o)
message(${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/installed/lib/cmake/g2o)
find_package(g2o REQUIRED)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("Debug Mode")
    # set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/debug)
    set(G2O_LIBS
    g2o::freeglut_minimal g2o::stuff g2o::opengl_helper g2o::core   g2o::g2o_hierarchical_library g2o::g2o_simulator_library    g2o::types_data g2o::types_slam2d g2o::types_slam3d g2o::types_sba g2o::types_sim3 g2o::types_icp g2o::types_sclam2d g2o::types_slam2d_addons g2o::types_slam3d_addons g2o::solver_pcg g2o::solver_dense g2o::solver_structure_only g2o::solver_eigen
    )
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/release)
    message("Release Mode")
    set(G2O_LIBS
    g2o::freeglut_minimal g2o::stuff g2o::opengl_helper g2o::core   g2o::g2o_hierarchical_library g2o::g2o_simulator_library    g2o::types_data g2o::types_slam2d g2o::types_slam3d g2o::types_sba g2o::types_sim3 g2o::types_icp g2o::types_sclam2d g2o::types_slam2d_addons g2o::types_slam3d_addons g2o::solver_pcg g2o::solver_dense g2o::solver_structure_only g2o::solver_eigen
    )
endif()

# message(${G2O_LIBS})