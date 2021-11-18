#include <stdlib.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <osg/Camera>
#include <osg/CameraView>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/Image>
#include <osg/LineWidth>
#include <osg/Matrix>
#include <osg/MatrixTransform>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/ShapeDrawable>
#include <osg/ref_ptr>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgGA/TrackballManipulator>
#include <osgText/Text>
#include <osgUtil/GLObjectsVisitor>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/Optimizer>
#include <osgViewer/Viewer>
#include <stack>
#include <string>
using namespace std;

osg::ref_ptr<osg::Group> R(osg::ref_ptr<osg::Group> r, double angle, double x, double y, double z) {
    osg::ref_ptr<osg::MatrixTransform> trans = new osg::MatrixTransform();
    trans->setMatrix(osg::Matrix::rotate(angle, x, y, z));
    trans->addChild(r);
    return trans;
}

osg::Geode *makeVirtualCamera() {
    osg::ref_ptr<osg::Sphere> pSphereShape = new osg::Sphere(osg::Vec3(0, 0, 0), 0.1f);
    osg::ref_ptr<osg::ShapeDrawable> pShapeDrawable = new osg::ShapeDrawable(pSphereShape.get());
    pShapeDrawable->setColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));

    const float &w = 2;
    const float h = w * 0.75;
    const float z = -w * 0.6;
    //创建保存几何信息的对象
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();

    double lineLength = 100.0;

    //创建四个顶点
    osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array();
    v->push_back(osg::Vec3(0, 0, 0));
    v->push_back(osg::Vec3(w, h, z));
    v->push_back(osg::Vec3(0, 0, 0));
    v->push_back(osg::Vec3(w, -h, z));
    v->push_back(osg::Vec3(0, 0, 0));
    v->push_back(osg::Vec3(-w, -h, z));
    v->push_back(osg::Vec3(0, 0, 0));
    v->push_back(osg::Vec3(-w, h, z));
    v->push_back(osg::Vec3(0, 0, 0));
    v->push_back(osg::Vec3(w, -h, z));
    v->push_back(osg::Vec3(w, h, z));
    v->push_back(osg::Vec3(w, -h, z));

    v->push_back(osg::Vec3(-w, h, z));
    v->push_back(osg::Vec3(-w, -h, z));

    v->push_back(osg::Vec3(-w, h, z));
    v->push_back(osg::Vec3(w, h, z));
    v->push_back(osg::Vec3(-w, -h, z));
    v->push_back(osg::Vec3(w, -h, z));
    geom->setVertexArray(v.get());

    //为每个顶点指定一种颜色
    osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array();
    for (int i = 0, sz = v->size(); i < sz; ++i) {
        c->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));  //坐标原点为红色
    }
    //如果没指定颜色则会变为黑色
    geom->setColorArray(c.get());
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    //绘制 camera
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, v->size()));  // X

    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    geode->getOrCreateStateSet()->setAttribute(new osg::LineWidth(5.0), osg::StateAttribute::ON);
    geode->addDrawable(pShapeDrawable.get());
    geode->addDrawable(geom.get());
    return geode.release();
}

osg::Geode *makeCoordinate() {
    osg::ref_ptr<osg::Sphere> pSphereShape = new osg::Sphere(osg::Vec3(0, 0, 0), 0.1f);
    osg::ref_ptr<osg::ShapeDrawable> pShapeDrawable = new osg::ShapeDrawable(pSphereShape.get());
    pShapeDrawable->setColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));

    //创建保存几何信息的对象
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();

    double lineLength = 100.0;

    //创建四个顶点
    osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array();
    v->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
    v->push_back(osg::Vec3(lineLength, 0.0f, 0.0f));
    v->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
    v->push_back(osg::Vec3(0.0f, lineLength, 0.0f));
    v->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
    v->push_back(osg::Vec3(0.0f, 0.0f, lineLength));
    geom->setVertexArray(v.get());

    //为每个顶点指定一种颜色
    osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array();
    c->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));  //坐标原点为红色
    c->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));  // x red
    c->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));  //坐标原点为绿色
    c->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));  // y green
    c->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));  //坐标原点为蓝色
    c->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));  // z blue
    //如果没指定颜色则会变为黑色
    geom->setColorArray(c.get());
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    //三个轴
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));  // X
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 2, 2));  // Y
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 4, 2));  // Z

    osg::ref_ptr<osgText::Text> pTextXAuxis = new osgText::Text;
    pTextXAuxis->setText(L"X");
    pTextXAuxis->setFont("Fonts/simhei.ttf");
    pTextXAuxis->setAxisAlignment(osgText::Text::SCREEN);
    pTextXAuxis->setCharacterSize(16);
    pTextXAuxis->setPosition(osg::Vec3(lineLength, 0.0f, 0.0f));

    osg::ref_ptr<osgText::Text> pTextYAuxis = new osgText::Text;
    pTextYAuxis->setText(L"Y");
    pTextYAuxis->setFont("Fonts/simhei.ttf");
    pTextYAuxis->setAxisAlignment(osgText::Text::SCREEN);
    pTextYAuxis->setCharacterSize(16);
    pTextYAuxis->setPosition(osg::Vec3(0.0f, lineLength, 0.0f));

    osg::ref_ptr<osgText::Text> pTextZAuxis = new osgText::Text;
    pTextZAuxis->setText(L"Z");
    pTextZAuxis->setFont("Fonts/simhei.ttf");
    pTextZAuxis->setAxisAlignment(osgText::Text::SCREEN);
    pTextZAuxis->setCharacterSize(16);
    pTextZAuxis->setPosition(osg::Vec3(0.0f, 0.0f, lineLength));

    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    geode->getOrCreateStateSet()->setAttribute(new osg::LineWidth(3.0), osg::StateAttribute::ON);

    geode->addDrawable(pShapeDrawable.get());
    geode->addDrawable(geom.get());
    // geode->addDrawable(pTextXAuxis.get());
    // geode->addDrawable(pTextYAuxis.get());
    // geode->addDrawable(pTextZAuxis.get());
    return geode.release();
}

int main(int argc, char **argv) {
    std::string datapath = "/home/wen/Desktop/Production_1/Data/root.osgb";
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    viewer.getCamera()->setClearColor(osg::Vec4(1, 1, 1, 0));
    osg::ref_ptr<osg::Group> model = new osg::Group;
    osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(datapath);
    osg::ref_ptr<osg::MatrixTransform> tf_root = new osg::MatrixTransform();
    tf_root->setMatrix(osg::Matrix::translate(0, 0, -root->getBound().center().z()));
    tf_root->addChild(root);
    // osg::ref_ptr<osg::Node> root = myUtils::read(datapath);
    std::vector<std::string> vTrajectoryOutPath;
    bool showModel = true;
    if (showModel) model->addChild(tf_root);
    // 红色为 GT
    model->addChild(makeCoordinate());
    model->addChild(makeVirtualCamera());

    // viewer.addEventHandler(pHUD.get());
    osg::ref_ptr<osg::MatrixTransform> trans = new osg::MatrixTransform();
    trans->setMatrix(osg::Matrix::rotate(osg::PI_2, 1, 0, 0));
    trans->addChild(model);

    viewer.setSceneData(trans.get());
    viewer.setUpViewInWindow(0, 0, 1000, 600);
    viewer.setCameraManipulator(new osgGA::TrackballManipulator);
    viewer.realize();
    // viewer.setCameraManipulator(NULL);
    // osg::Matrixd mat(0.644403, -0.669683, 0.369147, 0.0,
    //                 0.712408, 0.701189, 0.0284349, 0.0
    //                 -0.277884, 0.24466, 0.928936, 0.0
    //                 -66.7099, 71.0084 ,-397.536, 1.0);
    osg::Vec3d eye(248.196, -87.7405, 365.554);
    osg::Vec3d center(247.81, -87.55, 364.651);
    osg::Vec3d up(0.0587382, 0.981529, 0.182076);
    osg::Matrixd mat;
    mat.makeLookAt(eye, center, up);
    // viewer.getCamera()->setViewMatrix(mat);
    viewer.run();
    return 0;
}
