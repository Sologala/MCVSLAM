#include "capture.hpp"

#include "pyp/cmdline/cmdline.hpp"
#include <opencv2/core.hpp>
#include <unistd.h>
// #include <chrono>
// #include <fstream>
// #include <iostream>
// #include <opencv2/core.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/videoio.hpp>
// #include <thread>

// using namespace std;

// std::string GetFile_Extentions(const std::string file) {
//     char *p = const_cast<char *>(file.c_str());
//     char *q = ::strtok(p, ".");
//     vector<string> res;
//     while (q) {
//         res.emplace_back(string(q));
//         q = strtok(NULL, ".");
//     }
//     return res.back();
// }

// std::string GetFile_Path(const std::string file) {
//     char *p = const_cast<char *>(file.c_str());
//     char *q = ::strtok(p, "/");
//     vector<string> res;
//     while (q) {
//         res.emplace_back(string(q));
//         q = strtok(NULL, "/");
//     }
//     res.pop_back();
//     std::string ret;
//     for (auto &s : res) {
//         ret += '/';
//         ret += s;
//     }
//     return ret;
// }

// namespace MCVSLAM {

// Capture::Capture(const std::string &_fileName) : file_name_(_fileName) {

//     if (PATH_TEST::isDir(_fileName) == PATH_TEST::PATH_TYPE::FILE) {
//         CHECK_PATH(file_name_);
//         const string ext = GetFile_Extentions(_fileName);
//         if (ext == "mp4" || ext == "avi") {
//             _mode = Work_Mode::VIDEO_FILE;
//             cap = new cv::VideoCapture(_fileName);
//             if (cap->isOpened() == false) {
//                 cerr << " camera open file " << _fileName << "   faild "
//                      << endl;
//                 exit(-1);
//             }
//         } else {
//             _mode = Work_Mode::PATH;
//             img_paths.clear();
//             string path = GetFile_Path(_fileName);
//             path += ("/*." + ext);
//             printf("capture from path %s, %d imgs\n", path.c_str(),
//                    img_paths.size());
//             cv::glob(path, img_paths);
//             printf("capture from path %s, %d imgs\n", path.c_str(),
//                    img_paths.size());
//             sort(img_paths.begin(), img_paths.end());
//         }
//     } else {
//         printf("%s is not a file \n", file_name_.c_str());
//     }
//     if (int(timeStamp) % len == 0 && _mode == Work_Mode::VIDEO_FILE) {
//         cap->release();
//         cap->open(file_name_);
//     } else if (int(timeStamp) % len == 0 && _mode == Work_Mode::PATH) {
//         img_path_idx = 0;
//     }
//     Init();
// }
// void Capture::Init() {
//     specified_duration = 1000.0 / gconfig.capture_fps;
//     if (_mode == Work_Mode::VIDEO_FILE)
//         len = cap->get(7);
//     else if (_mode == Work_Mode::PATH) {
//         len = img_paths.size();
//     }
// }

// Capture::Capture(const int &_camId) {
//     _cam_id = _camId;
//     cap = new cv::VideoCapture(_camId);
//     _mode = Work_Mode::WEBCAM;
//     if (cap->isOpened() == false) {
//         cerr << " camera " << _camId << " open faild " << endl;
//         exit(-1);
//     }
//     Init();
// }

// Capture::~Capture() {
//     cap->release();
//     delete cap;
// }

// bool Capture::get(cv::Mat &img) {
//     timeStamp += 1;
//     if (int(timeStamp) % len == 0 && _mode == Work_Mode::VIDEO_FILE) {
//         cap->release();
//         cap->open(file_name_);
//     } else if (int(timeStamp) % len == 0 && _mode == Work_Mode::PATH) {
//         img_path_idx = 0;
//     }

//     if (_mode == Work_Mode::VIDEO_FILE || _mode == Work_Mode::WEBCAM) {
//         cap->grab();
//         bool ret = cap->retrieve(img);
//         if (ret == false) {
//             fmt::print("capture image from camera {} Faild\n", _cam_id);
//             return false;
//         }
//     } else if (_mode == Work_Mode::PATH) {
//         img = cv::imread(img_paths[img_path_idx]);
//         if (img.empty()) {
//             fmt::print("capture image from file {} Faild\n",
//                        img_paths[img_path_idx]);
//             return false;
//         }
//         img_path_idx++;
//     }
// }

// } // namespace MCVSLAM

// namespace PATH_TEST {
// PATH_TYPE isDir(const std::string &path) {
//     struct stat s;

//     if (stat(path.c_str(), &s) == 0) {
//         if (s.st_mode & S_IFDIR) {
//             // std::cout << "it's a directory" << std::endl;
//             return FOLDER;
//         } else if (s.st_mode & S_IFREG) {
//             // std::cout << "it's a file" << std::endl;
//             return FILE;
//         }
//     }
//     return NO_EXIST;
// }
// } // namespace PATH_TEST