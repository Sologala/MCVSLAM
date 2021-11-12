#include "Tracker.hpp"

#include "pyp/yaml/yaml.hpp"
namespace MCVSLAM {
Tracker::Tracker_Config Tracker::global_track_config;

Tracker::Tracker() {}

Tracker::~Tracker() {}

void Tracker::Tracker_Config::Parse(const std::string& config_file) {
    {
        Yaml::Node fs;
        Yaml::Parse(fs, config_file);
    }
}
}  // namespace MCVSLAM