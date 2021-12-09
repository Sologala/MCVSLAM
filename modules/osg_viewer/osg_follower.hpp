#include <osg/Matrix>
#include <osg/Matrixd>
#include <osg/Vec3>
#include <osg/ref_ptr>
#include <osgGA/EventHandler>
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
class Follower : public osgGA::TrackballManipulator {
   public:
    Follower(const osg::ref_ptr<osgViewer::Viewer>& _viewer, osg::Matrixd _T_layer_w, osg::ref_ptr<osg::MatrixTransform> _node_curr_cam)
        : viewer(_viewer), node_curr_cam(_node_curr_cam), T_layer_w(_T_layer_w){};
    ~Follower(){};
    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& es);

   protected:
    osg::ref_ptr<osgViewer::Viewer> viewer;
    osg::ref_ptr<osg::MatrixTransform> node_curr_cam;
    osg::Matrixd T_layer_w;
    osg::Matrixd T_v_c;
    bool is_follow = true;
};

inline bool Follower::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& es) {
    if (ea.getHandled()) return false;

    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP) {
        if (ea.getKey() == 32)  // SPACE
        {
            is_follow = !is_follow;
            std::cout << "isfollow : " << is_follow << std::endl;

            // Calculate transform from Viewer Camera  to current cam
            osg::Matrixd T_c_w = node_curr_cam->getMatrix() * T_layer_w;
            T_v_c = this->getMatrix() * osg::Matrixd::inverse(T_c_w);
            if (is_follow == false) {
            }
        }
        ea.setHandled(true);
        return true;
    }
    if (is_follow && ea.getEventType() == osgGA::GUIEventAdapter::FRAME) {
        osg::Matrixd T_c_w = node_curr_cam->getMatrix() * T_layer_w;

        this->setByMatrix(osg::Matrixd::translate(osg::Vec3(0, 0, 50) * node_curr_cam->getMatrix() * T_layer_w));
        // ea.setHandled(true);
    }
    return osgGA::TrackballManipulator::handle(ea, es);
}

class KeyBoardBoolTriger : public osgGA::GUIEventHandler {
   public:
    KeyBoardBoolTriger(){};
    virtual ~KeyBoardBoolTriger(){};
    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) {
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
    void Add(const std::string name, const char c, bool default_val, const std::function<void(bool)>& call_back) {
        values[name] = default_val;

        if (short_cuts.count(c) == 0) {
            short_cuts[c] = name;
            callbacks[c] = call_back;
        } else {
            throw std::runtime_error(fmt::format("Can not regist [{}] to [{}], Because it has been regiest to [{}] ", c, name, short_cuts[c]));
        }
    }
    bool operator[](const std::string& name) { return values[name]; }

   private:
    std::unordered_map<std::string, bool> values;
    std::unordered_map<char, std::function<void(bool)>> callbacks;
    std::unordered_map<char, std::string> short_cuts;
};
#endif