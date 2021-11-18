#ifndef OSG_VIEWER_H
#define OSG_VIEWER_H
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/core/types.hpp>
#include <osg/Group>
#include <osg/Node>
#include <osg/ref_ptr>
#include <osgViewer/Viewer>
#include <string>
#pragma once
class osg_viewer {
   public:
    osg_viewer(const std::string &config_file);
    ~osg_viewer();
    void Run();
    void Draw();

   private:
    void Parse(std::string config_file);
    osg::ref_ptr<osg::Node> CreateCameraNode();
    osg::ref_ptr<osg::Node> CreateCoordinate();
    std::string model_path;
    int camera_width;
    int mappoint_size;
    int keyframe_size;
    bool is_show_model = false;

    cv::Rect wnd_rect;

    // models
    osg::ref_ptr<osg::Node> node_environment;
    osg::ref_ptr<osg::Node> node_camera;
    osg::ref_ptr<osg::Group> scene;
    osg::ref_ptr<osgViewer::Viewer> viewer;
};
#endif