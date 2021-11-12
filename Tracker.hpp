#ifndef TRACKER_H
#define TRACKER_H
#include <string>
#pragma once
namespace MCVSLAM {

class Tracker {
   private:
   public:
    Tracker();
    ~Tracker();

    //  global config
    class Tracker_Config {
       public:
        void Parse(const std::string& config_file);
    };
    static Tracker_Config global_track_config;
};

}  // namespace MCVSLAM
#endif