#include "multi_msg_wraper.hpp"

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"

namespace MCVSLAM {
std::vector<cv::Mat> MsgWraper::rosMats2cvMats(const multi_msgs::multi_images &multi_msg) {
    std::vector<cv::Mat> ret;
    int nimgs = multi_msg.nimgs;
    for (int i = 0; i < nimgs; i++) {
        cv::Mat img = cv_bridge::toCvCopy(multi_msg.imgs[i], "bgr8")->image;
        ret.push_back(img);
    }
    return ret;
}

multi_msgs::multi_images MsgWraper::cvMats2rosMats(const std::vector<cv::Mat> &imgs) {
    multi_msgs::multi_images msg;
    msg.nimgs = imgs.size();
    for (cv::Mat img : imgs) {
        cv_bridge::CvImage img_msg(std_msgs::Header(), "bgr8", img);
        msg.imgs.push_back(*img_msg.toImageMsg());
    }
    return msg;
}

const sensor_msgs::ImageConstPtr MsgWraper::cvMat2rosMat(const cv::Mat &img) {
    cv_bridge::CvImage img_msg(std_msgs::Header(), "bgr8", img);
    return img_msg.toImageMsg();
}

cv::Mat MsgWraper::rosMat2cvMat(const sensor_msgs::ImageConstPtr &msg) {
    cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    return img;
}

}  // namespace MCVSLAM