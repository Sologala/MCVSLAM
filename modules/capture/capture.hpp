#ifndef Capture_H
#define CAPTURE_H
#pragma once
#include <opencv2/videoio.hpp>
#include <string>
namespace MCVSLAM {
class CaptureConfig;

class Capture {
   public:
    enum Work_Mode { WEBCAM = 0, VIDEO_FILE = 1, PATH = 2 };

    Capture(const std::string &_fileName);
    Capture(const int &_camId);
    ~Capture();

    bool get(cv::Mat &img);

    static CaptureConfig global_capture_config;

   private:
    void Init();

   private:
    int cam_id = 0;
    double timeStamp = 0;
    int img_path_idx = 0;
    double specified_duration = 0;
    int len;

    Work_Mode mode;

    cv::VideoCapture *cap;

    const std::string file_name;
    std::vector<std::string> img_paths;
};

class CaptureConfig {
   public:
    struct capture_info {
        std::string source;
        std::string topic_name;
    };
    void Parse(const std::string &configPath);

    int fps;
    int capture_cnt;
    std::vector<capture_info> caps;
};

}  // namespace MCVSLAM

namespace PATH_TEST {
enum PATH_TYPE { FOLDER, FILE, NO_EXIST };

PATH_TYPE isDir(const std::string &path);
}  // namespace PATH_TEST
#endif