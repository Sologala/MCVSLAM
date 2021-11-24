set(OpenGL_GL_PREFERENCE "GLVND")
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(OpenCV 4  REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(osgText)
find_package(osgViewer)
find_package(osg)

set(ALL_LIBS 
    libosgAnimation.so
    libosgDB.so
    libosgFX.so
    libosgGA.so
    libosgGA.so
    libosgManipulator.so
    libosgParticle.so
    libosgPresentation.so
    libosgShadow.so
    libosgSim.so
    libosg.so
    libosgTerrain.so
    libosgText.so
    libosgUI.so
    libosgUtil.so
    libosgViewer.so
    libosgVolume.so
    libosgWidget.so
    libOpenThreads.so;
    pyp::cmdline
    -pthread
    ${OPENGL_LIBRARIES} 
    ${GLUT_LIBRARY}
    ${OpenCV_LIBS}
)
link_libraries(pyp::cmdline pyp::yaml)

file(GLOB src_files "./*.c*")

add_library(osg_viewer ${src_files})
target_include_directories(osg_viewer PUBLIC
./
)
target_include_directories(osg_viewer PRIVATE
${Boost_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
${GLUT_INCLUDE_DIRS}
${OpenGL_INCLUDE_DIRS}
${ROS_INCLUDE_DIRS}
)
target_link_libraries(osg_viewer
    ${ALL_LIBS}
    ${ROS_LIBS}
)



add_executable(osg_camera_draw ./src/test_cam_draw.cpp)
target_include_directories(osg_camera_draw PUBLIC
./
${Boost_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
${GLUT_INCLUDE_DIRS}
${OpenGL_INCLUDE_DIRS}
${ROS_INCLUDE_DIRS}
)
target_link_libraries(osg_camera_draw
    ${ALL_LIBS}
    ${ROS_LIBS}
    osg_viewer
)
    



