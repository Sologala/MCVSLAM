#include <osg/Matrixd>
#include <osg/Vec3>
#include <osg/ref_ptr>
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
#include <osgUtil/IntersectionVisitor>
#include <osgViewer/Viewer>
#include <vector>

#define POSITION_DISTORTRANGE 0.005
#define ROTATION_DISTORTRANGE (osg::PI / 3600)

class Follow : public osgGA::CameraManipulator {
   public:
    Follow(std::shared_ptr<osg::Matrixd> cam_pose) {
        Tcw = cam_pose;
        tf = osg::Matrixd::translate(0, 0, 30);
        carPosition = osg::Vec3(0.0, 12.0, 0.0);
    }
    virtual void setByMatrix(const osg::Matrixd &matrix) {}

    virtual void setByInverseMatrix(const osg::Matrixd &matrix) {}

    virtual osg::Matrixd getMatrix() const { return (*Tcw) * tf; }

    virtual osg::Matrixd getInverseMatrix() const { return osg::Matrixd::inverse(getMatrix()); }
    float getrand(float range) { return -range + (range * 2) / 10 * (rand() % 10 + 1); }

    osg::MatrixTransform *CreateMT() {
        osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
        mt->setMatrix(osg::Matrix::translate(0.0, 0.0, 0.0));
        return mt.release();
    }

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us) {
        switch (ea.getEventType()) {
            case osgGA::GUIEventAdapter::KEYDOWN:
                std::cout << ea.getKey() << std::endl;
                if (ea.getKey() == 'a')  // a
                {
                    (*Tcw) *= osg::Matrixd::translate(-1, 0, 0);
                } else if (ea.getKey() == 'w')  // w
                {
                    (*Tcw) *= osg::Matrixd::translate(0, 1, 0);
                } else if (ea.getKey() == 'd') {  // d
                    (*Tcw) *= osg::Matrixd::translate(1, 0, 0);
                } else if (ea.getKey() == 's') {  // s
                    (*Tcw) *= osg::Matrixd::translate(0, -1, 0);
                }
                break;
            case osgGA::GUIEventAdapter::KEYUP:
                break;

            case osgGA::GUIEventAdapter::FRAME:
                break;
        }
        return false;
    }

   private:
    osg::Vec3 m_vPosition;
    std::shared_ptr<osg::Matrixd> Tcw;
    float m_fMoveSpeed;
    float m_ori_Speed;
    osg::Vec3 carPosition;
    osg::ref_ptr<osg::MatrixTransform> mt;
    osg::Matrixd tf;
};

#endif