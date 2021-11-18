#include "osg_viewer.hpp"

#include <opencv2/core/types.hpp>
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
#include <osgText/Text>
#include <osgViewer/Scene>
#include <osgViewer/Viewer>

#include "pyp/yaml/yaml.hpp"
using namespace std;

osg_viewer::~osg_viewer() {}
osg::ref_ptr<osg::Node> osg_viewer::CreateCameraNode() {
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
osg::ref_ptr<osg::Node> osg_viewer::CreateCoordinate() {
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
void osg_viewer::Parse(std::string config_file) {
    Yaml::Node fs;
    Yaml::Parse(fs, config_file);
    model_path = fs["model_path"].As<std::string>();
    camera_width = fs["camera_width"].As<int>();
    mappoint_size = fs["mappoint_size"].As<int>();
    keyframe_size = fs["keyframe_size"].As<int>();
    is_show_model = fs["is_show_model"].As<bool>();
    wnd_rect = cv::Rect(fs["window_x"].As<int>(), fs["window_y"].As<int>(), fs["window_width"].As<int>(), fs["window_height"].As<int>());
}
osg_viewer::osg_viewer(const std::string &config_file) {
    Parse(config_file);
    node_camera = CreateCameraNode();
    if (is_show_model) node_environment = osgDB::readRefNodeFile(model_path);

    // set windows configuration

    // Create viewer
    viewer = new osgViewer::Viewer();
    viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    // Set Background color
    viewer->getCamera()->setClearColor(osg::Vec4(1, 1, 1, 0));

    // Add model to viewer
    scene = new osg::Group;
    scene->addChild(node_camera);
    scene->addChild(CreateCoordinate());
    if (is_show_model) scene->addChild(node_environment);
    viewer->setUpViewInWindow(wnd_rect.x, wnd_rect.y, wnd_rect.width, wnd_rect.height);
    // viewer->setCameraManipulator(new osgGA::TrackballManipulator);
    viewer->setSceneData(scene);
    viewer->realize();
}