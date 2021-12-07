#ifndef OSG_VIEWER_H
#define OSG_VIEWER_H
#include <sensor_msgs/PointCloud2.h>

#include <boost/thread.hpp>
#include <boost/thread/pthread/mutex.hpp>
#include <opencv2/core/types.hpp>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/Transform>
#include <osg/ref_ptr>
#include <osgViewer/Viewer>
#include <queue>
#include <string>
#include <thread>

#pragma once
class osg_viewer {
   public:
    osg_viewer(const std::string& config_file);
    ~osg_viewer();
    void Start();
    void Run();
    void Draw(const cv::Mat mps, uint r, uint g, uint b);
    void Draw(const std::vector<cv::Mat> mps, uint r, uint g, uint b);
    void DrawCam(const cv::Mat Tcw, bool ned_shape, uint r, uint g, uint b);
    void SetCurViewFollow(const cv::Mat Twc);
    void Commit();
    osg::Matrixd ShowTracjtory(const std::string& tracj_file, osg::Vec4 color);

   public:
    bool IsStoped();
    void RequestStop();

    void DrawCams(const std::vector<cv::Mat>& Tcws, const std::vector<bool>& mask, uint r, uint g, uint b);

   private:
    void Parse(std::string config_file);
    osg::ref_ptr<osg::Geode> CreateCameraNode();
    osg::ref_ptr<osg::Node> CreateCoordinate();
    std::string model_path;
    std::string gt_tracj_path;
    int camera_width;
    int mappoint_size;
    int keyframe_size;
    bool is_show_model = false;

    cv::Rect wnd_rect;

    osg::Matrixd tf;
    osg::Matrixd follow_tf;

    osg::ref_ptr<osg::Group> node_environment;
    osg::ref_ptr<osg::Node> node_curr_cam;
    osg::ref_ptr<osg::Group> scene;
    osg::ref_ptr<osgViewer::Viewer> viewer;
    osg::ref_ptr<osg::Group> draw_buffer[2];
    osg::ref_ptr<osg::MatrixTransform> points_cams;
    osg::ref_ptr<osg::MatrixTransform> points_cams_align;
    osg::ref_ptr<osg::Group> ground_trugh_traj;

    // double buffer queue
    boost::mutex mtx_commit;
    uint show_idx = 0, draw_idx = 1;

    // thread control
    boost::mutex mtx;
    bool is_commited;
    bool is_request_stop;
    bool is_stoped;

    // thread
    std::shared_ptr<std::thread> pthread;
};
#endif