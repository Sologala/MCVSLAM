#include <osg/Matrix>
#include <osg/Matrixd>
#include <osg/Vec3>
#include <osg/ref_ptr>
#include <osgGA/EventHandler>
#include <osgGA/GUIEventAdapter>
#include <osgGA/OrbitManipulator>
#include <stdexcept>
#ifndef EVENT_HANDLE_H_
#define EVENT_HANDLE_H_ 1
#pragma once
#include <stdlib.h>
#include <time.h>

#include <iostream>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/LineSegment>
#include <osg/MatrixTransform>
#include <osg/Node>
#include <osg/Point>
#include <osgGA/CameraManipulator>
#include <osgGA/TrackballManipulator>
#include <osgUtil/IntersectionVisitor>
#include <osgViewer/Viewer>
#include <unordered_map>
#include <vector>

#include "pyp/fmt/fmt.hpp"
#define POSITION_DISTORTRANGE 0.005
#define ROTATION_DISTORTRANGE (osg::PI / 3600)
class Follower : public osgGA::OrbitManipulator {
   public:
    Follower(const osg::ref_ptr<osgViewer::Viewer> &_viewer, osg::Matrixd _T_layer_w, osg::ref_ptr<osg::MatrixTransform> _node_curr_cam)
        : viewer(_viewer), node_curr_cam(_node_curr_cam), T_layer_w(_T_layer_w) {
        view_center = osg::Vec3(0, 0, 0);
    };
    ~Follower(){};
    virtual bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &es);

   private:
    bool pick(float fX, float fY);

   protected:
    osg::ref_ptr<osgViewer::Viewer> viewer;
    osg::ref_ptr<osg::MatrixTransform> node_curr_cam;
    osg::Matrixd T_layer_w;
    osg::Matrixd T_v_c;
    bool reset_view_port = true;
    osg::Vec3 view_center;
};

inline bool Follower::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &es) {
    if (ea.getHandled()) return false;
    if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH) {                  // 单击
        if (ea.getButton() == osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON) {  // 右键
            if (pick(ea.getX(), ea.getY())) {
                osgGA::OrbitManipulator::setCenter(view_center);
                reset_view_port = false;
            }
            return true;
        }
    } else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP) {
        if (ea.getKey() == 32)  // SPACE
        {
            reset_view_port = true;
            osg::Matrixd T_c_w = node_curr_cam->getMatrix() * T_layer_w;
            this->setByMatrix(osg::Matrixd::translate(osg::Vec3(0, 0, 50) * node_curr_cam->getMatrix() * T_layer_w));
            osgGA::OrbitManipulator::setCenter(T_c_w.getTrans());
            osgGA::OrbitManipulator::setDistance(20);
        }
        ea.setHandled(true);
        // return true;
    }
    if (ea.getEventType() == osgGA::GUIEventAdapter::FRAME) {
        if (reset_view_port) {
            osg::Matrixd T_c_w = node_curr_cam->getMatrix() * T_layer_w;
            osgGA::OrbitManipulator::setCenter(T_c_w.getTrans());
        } else {
            osgGA::OrbitManipulator::setCenter(view_center);
        }
    }
    return osgGA::OrbitManipulator::handle(ea, es);
}

inline bool Follower::pick(float fX, float fY) {
    fmt::print("pick\n");
    osgUtil::LineSegmentIntersector::Intersections intersections;
    if (viewer->computeIntersections(fX, fY, intersections)) {
        for (auto itr = intersections.begin(); itr != intersections.end(); ++itr) {
            if (!itr->nodePath.empty()) {
                const osg::NodePath &np = itr->nodePath;
                for (int i = np.size() - 1; i >= 0; --i) {
                    osg::ref_ptr<osg::Node> node = dynamic_cast<osg::Node *>(np[i]);
                    if (NULL != node && node->getNodeMask() == 1) {
                        // node->setNodeMask(0);
                        view_center = itr->getWorldIntersectPoint();  // 得到坐标
                        fmt::print("pick success\n");
                        // view_center = node->getBound()._center;
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

class KeyBoardBoolTriger : public osgGA::GUIEventHandler {
   public:
    KeyBoardBoolTriger(){};
    virtual ~KeyBoardBoolTriger(){};
    virtual bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
        if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP) {
            const char c = ea.getKey();
            if (short_cuts.count(c) != 0) {
                const std::string _name = short_cuts[c];
                values[_name] = !values[_name];
                auto func = callbacks[c];
                func(values[_name]);
                return true;
            }
        }
        return false;
    }
    void Add(const std::string name, const char c, bool default_val, const std::function<void(bool)> &call_back) {
        values[name] = default_val;

        if (short_cuts.count(c) == 0) {
            short_cuts[c] = name;
            callbacks[c] = call_back;
        } else {
            throw std::runtime_error(fmt::format("Can not regist [{}] to [{}], Because it has been regiest to [{}] ", c, name, short_cuts[c]));
        }
    }
    bool operator[](const std::string &name) { return values[name]; }

   private:
    std::unordered_map<std::string, bool> values;
    std::unordered_map<char, std::function<void(bool)>> callbacks;
    std::unordered_map<char, std::string> short_cuts;
};
#endif