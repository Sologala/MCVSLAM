#include "multi_msg_wraper.hpp"

#include "cv_bridge/cv_bridge.h"

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

}  // namespace MCVSLAM