#include <opencv2/core/hal/interface.h>
#include <stdlib.h>
#include <unistd.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <stack>
#include <string>

#include "osg_viewer.hpp"
using namespace std;

int main(int argc, char **argv) {
    std::string config_file = "../config/viewer.yaml";
    osg_viewer viewer(config_file);
    std::vector<float> arr = {0, 0, 100};
    cv::Mat pts(100, 3, CV_32F);
    cv::randn(pts, cv::Scalar(0), cv::Scalar(100));
    viewer.Draw(pts, 0, 0, 225);
    cv::Mat cam_pos = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat t = cam_pos.rowRange(0, 3).col(3);

    cout << cam_pos << endl;
    viewer.DrawCam(cam_pos, true, 0, 225, 0);
    viewer.Commit();
    viewer.Run();
    // viewer.getCamera()->setViewMatrix(mat);
    // viewer.run();
    return 0;
}
