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
#include <osg/Vec3>
#include <osg/ref_ptr>
#include <osgViewer/Viewer>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>

#include "osg_follower.hpp"
#pragma once

class CameraDataBase {
    using CameraDespcriptor = unsigned int;

   public:
    static osg::ref_ptr<osg::Geode> CreateCamera(uint r, uint g, uint b, uint width);
    ~CameraDataBase(){};

   private:
    static std::unordered_map<CameraDespcriptor, osg::ref_ptr<osg::Geode>> database;
    CameraDataBase(){};
};

class osg_viewer {
   public:
    osg_viewer(const std::string& config_file);
    ~osg_viewer();
    void Start();
    void Run();
    void Draw(const cv::Mat mps, uint r, uint g, uint b);
    void Draw(const std::vector<cv::Mat> mps, uint r, uint g, uint b);
    void DrawCam(const cv::Mat Tcw, bool ned_shape, uint r, uint g, uint b);
    void DrawEssentialGraph(const std::vector<std::pair<cv::Mat, cv::Mat>>& graph, uint r = 90, uint g = 90, uint b = 150);

    void Commit();
    osg::Matrixd ShowTracjtory(const std::string& tracj_file, osg::Vec4 color);

   public:
    bool IsStoped();
    void RequestStop();
    void DrawPredictTracjectories(const std::vector<cv::Mat>& Tcws, const std::vector<bool>& mask, uint r, uint g, uint b);
    void SetCurrentCamera(const cv::Mat Tcw);

   private:
    void SetCurViewFollow(const cv::Mat Twc);

    void Parse(std::string config_file);
    osg::ref_ptr<osg::Geode> CreateCameraNode();
    osg::ref_ptr<osg::Node> CreateCoordinate();
    std::string model_path;
    std::string gt_tracj_path;
    int camera_width;
    int mappoint_size;
    int keyframe_size;
    bool is_show_model = false;

    bool gate_draw_essential_graph = true;

    cv::Rect wnd_rect;

    osg::Matrixd tf;
    osg::Matrixd follow_tf;

    osg::ref_ptr<osg::Group> layer_environment;
    osg::ref_ptr<osg::MatrixTransform> layer_curr_cam;
    osg::ref_ptr<osg::MatrixTransform> node_curr_cam;
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

    // GUI shortcurt  handle
    KeyBoardBoolTriger kbtriger;
};
#endif