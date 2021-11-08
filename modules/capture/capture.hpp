#ifndef Capture_H
#define CAPTURE_H
#pragma once
#include <opencv2/videoio.hpp>
#include <string>
namespace MCVSLAM {
class Capture {
private:
public:
  enum Work_Mode { WEBCAM = 0, VIDEO_FILE = 1, PATH = 2 };

  Capture(const std::string &_fileName);
  Capture(const int &_camId);
  ~Capture();

  bool get(cv::Mat &img);

private:
  void Init();

private:
  int _cam_id = 0;
  double timeStamp = 0;
  int img_path_idx = 0;
  double specified_duration = 0;
  int len;

  Work_Mode _mode;

  cv::VideoCapture *cap;

  const std::string file_name_;
  std::vector<std::string> img_paths;
};
} // namespace MCVSLAM
#endif