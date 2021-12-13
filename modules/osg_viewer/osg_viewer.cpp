#include "osg_viewer.hpp"

#include <opencv2/core/hal/interface.h>
#include <pyp/fmt/core.h>

#include <algorithm>
#include <boost/thread/lock_types.hpp>
#include <boost/thread/pthread/mutex.hpp>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <osg/Camera>
#include <osg/CameraView>
#include <osg/ComputeBoundsVisitor>
#include <osg/CopyOp>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/Image>
#include <osg/Light>
#include <osg/LightModel>
#include <osg/LightSource>
#include <osg/LineWidth>
#include <osg/Math>
#include <osg/Matrix>
#include <osg/MatrixTransform>
#include <osg/Matrixd>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/Point>
#include <osg/PolygonMode>
#include <osg/Quat>
#include <osg/ShapeDrawable>
#include <osg/Transform>
#include <osg/Vec3>
#include <osg/Vec3d>
#include <osg/Vec4>
#include <osg/ref_ptr>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osgText/Text>
#include <osgViewer/Scene>
#include <osgViewer/Viewer>
#include <thread>
#include <tuple>

#include "osg_follower.hpp"
#include "pyp/yaml/yaml.hpp"

using namespace std;
std::unordered_map<CameraDataBase::CameraDespcriptor, osg::ref_ptr<osg::Geode>> CameraDataBase::database;

osg::ref_ptr<osg::Geode> CameraDataBase::CreateCamera(uint r, uint g, uint b, uint width) {
    CameraDespcriptor desp = 0;
    desp <<= 8;
    desp |= r;
    desp <<= 8;
    desp |= g;
    desp <<= 8;
    desp |= b;
    desp <<= 8;
    desp |= width;

    if (database.count(desp) == 0) {
        // create new camera
        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
        osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array();
        osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array();
        osg::ref_ptr<osg::Geode> geode = new osg::Geode();
        osg::ref_ptr<osg::Sphere> pSphereShape = new osg::Sphere(osg::Vec3(0, 0, 0), 0.1f);
        osg::ref_ptr<osg::ShapeDrawable> pShapeDrawable = new osg::ShapeDrawable(pSphereShape.get());
        pShapeDrawable->setColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));
        if (width != 0) {
            float w = width * 1.f / 10.;
            const float h = w * 0.75;
            const float z = -w * 0.6;
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
            for (int i = 0, sz = v->size(); i < sz; ++i) {
                c->push_back(osg::Vec4(1.0f, r, g, b));  //坐标原点为红色
            }

            //如果没指定颜色则会变为黑色
            geom->setColorArray(c.get());
            geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
            //绘制 camera
            geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, v->size()));
            geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
            geode->getOrCreateStateSet()->setAttribute(new osg::LineWidth(5.), osg::StateAttribute::ON);
            geode->addDrawable(pShapeDrawable.get());
            geode->addDrawable(geom.get());

            // 创建 包围盒
            {
                osg::ComputeBoundsVisitor boundVisitor;
                geom->accept(boundVisitor);
                osg::BoundingBox boundingBox = boundVisitor.getBoundingBox();

                float length = boundingBox.xMax() - boundingBox.xMin();
                float width = boundingBox.yMax() - boundingBox.yMin();
                float height = boundingBox.zMax() - boundingBox.zMin();
                osg::ref_ptr<osg::ShapeDrawable> drawable = new osg::ShapeDrawable(new osg::Box(boundingBox.center(), length, width, height));
                drawable->setColor(osg::Vec4(0.1, 1.0, 0.0, 0.0));
                osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet;
                stateset = drawable->getOrCreateStateSet();
                osg::ref_ptr<osg::PolygonMode> polygonMode = new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
                stateset->setAttributeAndModes(polygonMode);
                stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
                stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
                stateset->setMode(GL_DEPTH, osg::StateAttribute::OFF);
                stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
                geode->addDrawable(drawable);
            }
            geode->setNodeMask(1);
        } else {
            v->push_back(osg::Vec3(0, 0, 0));
            for (int i = 0, sz = v->size(); i < sz; ++i) {
                c->push_back(osg::Vec4(1.0f, 0, 0, 225));  //坐标原点为红色
            }
            geom->setVertexArray(v);
            geom->setColorArray(c);
            geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
            geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, v->size()));  // X
            geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
            geode->getOrCreateStateSet()->setAttribute(new osg::Point(6), osg::StateAttribute::ON);
            geode->addDrawable(pShapeDrawable.get());
            geode->addDrawable(geom.get());
        }
        database[desp] = geode;
    }

    return dynamic_cast<osg::Geode *>(database[desp]->clone(osg::CopyOp::SHALLOW_COPY));
}
cv::Mat SE3Inverse(const cv::Mat T) {
    cv::Mat R = T.rowRange(0, 3).colRange(0, 3);
    cv::Mat t = T.rowRange(0, 3).col(3);
    cv::Mat T_inv = cv::Mat::eye(4, 4, T.type());
    cv::Mat R_inv = R.t();
    R_inv.copyTo(T_inv.rowRange(0, 3).colRange(0, 3));
    cv::Mat new_t = -R_inv * t;
    new_t.copyTo(T_inv.rowRange(0, 3).col(3));
    return new_t;
}

/* */
cv::Mat right2left(cv::Mat T) {
    static cv::Mat r2l = (cv::Mat_<float>(4, 4) << -1, 0, 0, 0, /* */ 0, 01, 0, 0,
                          /* */ 0, 0, 1, 0, /* */ 0, 0, 0, 1);
    return r2l * T;
}
void osg_viewer::Run() {
    is_commited = false;
    is_request_stop = false;
    is_stoped = false;
    // clear node_cloud_points when launch threads.

    while (1) {
        viewer->frame();
        // float x = viewer->getCamera()->getInverseViewMatrix().getTrans()._v[0];
        // float y = viewer->getCamera()->getInverseViewMatrix().getTrans()._v[1];
        // float z = viewer->getCamera()->getInverseViewMatrix().getTrans()._v[2];

        // cout << x << " " << y << " " << z << endl;
        {
            boost::unique_lock<boost::mutex> lock(mtx);
            if (is_request_stop) {
                break;
            }
        }
    }
    {  // set stop
        boost::unique_lock<boost::mutex> lock(mtx);
        is_stoped = true;
    }
}

// input cv::Mat mps must be float
void osg_viewer::Draw(const cv::Mat mps, uint r, uint g, uint b) {
    // create vertex array and colors
    osg::ref_ptr<osg::Sphere> pSphereShape = new osg::Sphere(osg::Vec3(0, 0, 0), 0.1f);
    osg::ref_ptr<osg::ShapeDrawable> pShapeDrawable = new osg::ShapeDrawable(pSphereShape.get());
    pShapeDrawable->setColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
    osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array();
    osg::ref_ptr<osg::Vec4dArray> colors = new osg::Vec4dArray();

    for (int i = 0, sz = mps.rows; i < sz; i++) {
        const cv::Point3f *ptr = mps.ptr<cv::Point3f>(i);
        v->push_back(osg::Vec3(ptr->x, -ptr->y, -ptr->z));
        colors->push_back(osg::Vec4(1., r, g, b));
    }
    geom->setVertexArray(v);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, v->size()));  // X
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    geode->getOrCreateStateSet()->setAttribute(new osg::Point(mappoint_size), osg::StateAttribute::ON);
    geode->addDrawable(pShapeDrawable.get());
    geode->addDrawable(geom.get());
    draw_buffer[draw_idx]->addChild(geode);
}

void osg_viewer::Draw(const std::vector<cv::Mat> mps, uint r, uint g, uint b) {
    // create vertex array and colors
    fmt::print("draw {} points\n", mps.size());
    osg::ref_ptr<osg::Sphere> pSphereShape = new osg::Sphere(osg::Vec3(0, 0, 0), 0.1f);
    osg::ref_ptr<osg::ShapeDrawable> pShapeDrawable = new osg::ShapeDrawable(pSphereShape.get());
    pShapeDrawable->setColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
    osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array();
    osg::ref_ptr<osg::Vec4dArray> colors = new osg::Vec4dArray();

    for (int i = 0, sz = mps.size(); i < sz; i++) {
        auto ptr = mps[i].ptr<float>(0);
        v->push_back(osg::Vec3(*ptr, -*(ptr + 1), -*(ptr + 2)));
        // fmt::print("{} {} {}\n", *ptr, *(ptr + 1), *(ptr + 2));
        colors->push_back(osg::Vec4(1., r, g, b));
    }
    geom->setVertexArray(v);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, v->size()));  // X
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    geode->getOrCreateStateSet()->setAttribute(new osg::Point(mappoint_size), osg::StateAttribute::ON);
    geode->addDrawable(pShapeDrawable.get());
    geode->addDrawable(geom.get());
    draw_buffer[draw_idx]->addChild(geode);
}

void osg_viewer::Commit() {
    {
        boost::unique_lock<boost::mutex> lock(mtx_commit);
        points_cams->removeChildren(0, points_cams->getNumChildren());
        points_cams->addChild(draw_buffer[draw_idx]);
        show_idx = draw_idx;
        draw_idx = (draw_idx + 1) % 2;
        draw_buffer[draw_idx]->removeChild(0, draw_buffer[draw_idx]->getNumChildren());
    }
}
osg::Matrixd osg_viewer::ShowTracjtory(const std::string &tracj_file, osg::Vec4 color) {
    // load from file
    ifstream ifs(tracj_file);
    std::string line;
    int nFrams = 0;
    osg::Vec3d init;
    osg::ref_ptr<osg::Vec4dArray> vecColor1 = new osg::Vec4dArray();
    std::vector<osg::Vec3d> vpose;
    std::vector<osg::Quat> vquats;
    while (getline(ifs, line)) {
        int64_t timeStamps;
        char strTime[256] = {};
        osg::Vec3d p;
        osg::Quat q;

        for (char &c : line)
            if (c == '|') c = ' ';
        sscanf(line.c_str(), "%s%lf%lf%lf%lf%lf%lf%lf\n", strTime, &p._v[0], &p._v[1], &p._v[2], &q._v[0], &q._v[1], &q._v[2], &q._v[3]);
        // printf("%lf %lf %lf \n", p._v[0], p._v[1], p._v[2]);
        // p._v[2] *= -1;
        // double temp = p._v[1];
        // p._v[1] = p._v[0];
        // p._v[0] = temp;
        // p._v[1] *= -1;
        // p._v[0] *= -1;
        // osg::Quat rot(osg::PI, osg::Vec3d(1, 0, 0));
        // p = rot * p;
        nFrams += 1;
        vpose.push_back(p);
        vquats.push_back(q);
        vecColor1->push_back(color);
    }

    osg::ref_ptr<osg::Vec3dArray> vecarry1 = new osg::Vec3dArray();
    for (osg::Vec3d p : vpose) {
        vecarry1->push_back(p);
    }
    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
    geometry->setColorArray(vecColor1);
    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(2.0);
    geometry->getOrCreateStateSet()->setAttribute(lw, osg::StateAttribute::ON);
    geometry->setVertexArray(vecarry1.get());
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vecarry1->size()));
    ground_trugh_traj->addChild(geometry);

    auto T = osg::Matrixd(vquats[0]) * osg::Matrixd::translate(vpose[0]);
    return T;
}

bool osg_viewer::IsStoped() {
    boost::unique_lock<boost::mutex> lock(mtx);
    return is_stoped;
}

void osg_viewer::RequestStop() {
    boost::unique_lock<boost::mutex> lock(mtx);
    is_request_stop = true;
}

void osg_viewer::DrawPredictTracjectories(const std::vector<cv::Mat> &Tcws, const std::vector<bool> &mask, uint r, uint g, uint b) {
    fmt::print("draw {} keyframes \n", Tcws.size());
    for (uint i = 0, sz = Tcws.size(); i < sz; i++) {
        DrawCam(Tcws[i], mask[i], r, g, b);
    }
    // if (Tcws.size()) SetCurViewFollow(Tcws.back());
}

void osg_viewer::SetCurrentCamera(const cv::Mat Tcw) {
    cv::Mat Tcw_lhs = right2left(Tcw);
    cv::Mat Tcw_t;
    cv::transpose(Tcw_lhs, Tcw_t);
    osg::Matrixd mat(Tcw_t.ptr<float>());
    node_curr_cam->setMatrix(mat);
}

void osg_viewer::DrawCam(const cv::Mat Tcw, bool ned_shape, uint r, uint g, uint b) {
    osg::ref_ptr<osg::Geode> geode = CameraDataBase::CreateCamera(r, g, b, (ned_shape ? camera_width : 1));
    osg::ref_ptr<osg::MatrixTransform> trans = new osg::MatrixTransform();
    trans->addChild(geode);
    cv::Mat Tcw_lhs = right2left(Tcw);
    cv::Mat Tcw_t;
    cv::transpose(Tcw_lhs, Tcw_t);
    osg::Matrixd mat(Tcw_t.ptr<float>());
    trans->setMatrix(mat);
    draw_buffer[draw_idx]->addChild(trans);
}

void osg_viewer::DrawEssentialGraph(const std::vector<std::pair<cv::Mat, cv::Mat>> &graph, uint r, uint g, uint b) {
    if (gate_draw_essential_graph == false) return;
    osg::ref_ptr<osg::Sphere> pSphereShape = new osg::Sphere(osg::Vec3(0, 0, 0), 0.1f);
    osg::ref_ptr<osg::ShapeDrawable> pShapeDrawable = new osg::ShapeDrawable(pSphereShape.get());
    pShapeDrawable->setColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(pShapeDrawable.get());
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    geode->getOrCreateStateSet()->setAttribute(new osg::LineWidth(2.), osg::StateAttribute::ON);
    for (const std::pair<cv::Mat, cv::Mat> &e : graph) {
        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
        osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array();
        osg::ref_ptr<osg::Vec4dArray> colors = new osg::Vec4dArray();
        auto ptr_cam0 = e.first.ptr<float>(0);
        auto ptr_cam1 = e.second.ptr<float>(0);
        v->push_back(osg::Vec3(*ptr_cam0, -*(ptr_cam0 + 1), -*(ptr_cam0 + 2)));
        v->push_back(osg::Vec3(*ptr_cam1, -*(ptr_cam1 + 1), -*(ptr_cam1 + 2)));
        // fmt::print("{} {} {}\n", *ptr, *(ptr + 1), *(ptr + 2));
        colors->push_back(osg::Vec4(1., r, g, b));
        colors->push_back(osg::Vec4(1., r, g, b));
        geom->setVertexArray(v);
        geom->setColorArray(colors);
        geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, v->size()));
        geode->addDrawable(geom.get());
    }
    draw_buffer[draw_idx]->addChild(geode);
}

void osg_viewer::SetCurViewFollow(const cv::Mat Tcw) {
    cv::Mat Tcw_lhs = right2left(Tcw);
    cv::Mat t = Tcw_lhs.rowRange(0, 3).col(3);
    cv::Mat Tcw_t;
    cv::transpose(Tcw, Tcw_t);
    // osg::Matrixd mat(Tcw_t.ptr<float>());
    osg::Matrixd mat;
    mat.makeTranslate(osg::Vec3d(t.at<float>(0), t.at<float>(1), t.at<float>(2)));

    viewer->getCameraManipulator()->setByMatrix(tf * follow_tf * mat);
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
    mappoint_size = fs["mappoint_size"].As<float>();
    camera_width = fs["camera_width"].As<float>();
    is_show_model = fs["is_show_model"].As<bool>();
    wnd_rect = cv::Rect(fs["window_x"].As<int>(), fs["window_y"].As<int>(), fs["window_width"].As<int>(), fs["window_height"].As<int>());
    gt_tracj_path = fs["gt_traj"].As<std::string>();

    vector<float> follow_tf_vec = fs["follow_tf"].AsVector<float>();
    follow_tf.makeTranslate(follow_tf_vec[0], follow_tf_vec[1], follow_tf_vec[2]);
}
osg_viewer::osg_viewer(const std::string &config_file) {
    draw_buffer[0] = new osg::Group();
    draw_buffer[1] = new osg::Group();
    ground_trugh_traj = new osg::Group();
    points_cams = new osg::MatrixTransform();
    points_cams_align = new osg::MatrixTransform();
    Parse(config_file);
    layer_environment = new osg::Group();
    layer_curr_cam = new osg::MatrixTransform();
    node_curr_cam = new osg::MatrixTransform();
    if (is_show_model) {
        osg::ref_ptr<osg::Node> model = osgDB::readRefNodeFile(model_path);
        osg::Vec3 vcenter = model->getBound().center();
        osg::ref_ptr<osg::MatrixTransform> tf = new osg::MatrixTransform();
        tf->setMatrix(osg::Matrix::translate(osg::Vec3(0, 0, -vcenter[2])));
        tf->addChild(model);

        layer_environment->addChild(tf);
    }

    // Create viewer
    viewer = new osgViewer::Viewer();
    viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    // Set Background color
    viewer->getCamera()->setClearColor(osg::Vec4(1, 1, 1, 0));
    viewer->setLightingMode(osg::View::LightingMode::NO_LIGHT);
    // Add model to viewer
    scene = new osg::Group;

    scene->addChild(CreateCoordinate());
    if (is_show_model) {
        scene->addChild(layer_environment);
    }

    // load ground truth traj
    tf = ShowTracjtory(gt_tracj_path, osg::Vec4(0, 100, 225, 100));
    // tf = osg::Matrixd::inverse(tf);
    scene->addChild(ground_trugh_traj);
    osg::Matrixd tft;
    // tf.transpose(tft);
    points_cams_align->setMatrix(tf);
    // points_cams->setMatrix(osg::Matrix(osg::Quat(0, 1, 0, 0)));
    points_cams_align->addChild(points_cams);

    // create current cam
    layer_curr_cam->addChild(node_curr_cam);
    layer_curr_cam->setMatrix(tf);
    node_curr_cam->addChild(CameraDataBase::CreateCamera(255, 255, 0, camera_width));
    // create points and cams
    scene->addChild(points_cams_align);
    scene->addChild(layer_curr_cam);

    viewer->setUpViewInWindow(wnd_rect.x, wnd_rect.y, wnd_rect.width, wnd_rect.height);
    // viewer->setCameraManipulator(nullptr);
    viewer->setSceneData(scene);
    viewer->realize();
    // viewer->setCameraManipulator(new osgGA::TrackballManipulator());
    osg::ref_ptr<Follower> follower = new Follower(viewer, layer_curr_cam->getMatrix(), node_curr_cam);
    viewer->setCameraManipulator(follower);
    // viewer->setCameraManipulator(follower);

    // add keyboard handle
    kbtriger.Add("show_essential_graph", 'e', true, [&](bool _val) { gate_draw_essential_graph = _val; });
    kbtriger.Add("show_ground_truth", 'g', true, [&](bool _val) {
        if (_val) {
            if (scene->getNumChildren() == this->scene->getChildIndex(this->ground_trugh_traj)) {
                this->scene->addChild(this->ground_trugh_traj);
            }
        } else {
            if (scene->getNumChildren() != this->scene->getChildIndex(this->ground_trugh_traj)) {
                this->scene->removeChild(this->ground_trugh_traj);
            }
        }
    });
    kbtriger.Add("show_map", 'm', true, [&](bool _val) {
        if (_val) {
            if (scene->getNumChildren() == this->scene->getChildIndex(this->layer_environment)) {
                this->scene->addChild(this->layer_environment);
            }
        } else {
            if (scene->getNumChildren() != this->scene->getChildIndex(this->layer_environment)) {
                this->scene->removeChild(this->layer_environment);
            }
        }
    });

    viewer->addEventHandler(&kbtriger);
}

osg_viewer::~osg_viewer() { pthread->join(); }

void osg_viewer::Start() {
    if (pthread) {
        fmt::print("Error: viewer thread has launched!\n");
        return;
    }
    pthread = make_shared<std::thread>(&osg_viewer::Run, this);
}
