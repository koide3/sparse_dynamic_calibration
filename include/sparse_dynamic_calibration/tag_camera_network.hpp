#ifndef SIM3_ESTIMATION_TAG_CAMERA_NETWORK_HPP
#define SIM3_ESTIMATION_TAG_CAMERA_NETWORK_HPP

#include <regex>
#include <memory>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <opencv2/opencv.hpp>

#include <sparse_dynamic_calibration/static_tag.hpp>
#include <sparse_dynamic_calibration/static_camera.hpp>

namespace sparse_dynamic_calibration {

class TagCameraNetwork {
public:
    TagCameraNetwork(const std::string& data_dir, const std::string& tag_setting, const std::string& camera_setting);

    bool read_poses(ros::NodeHandle& nh, g2o::SparseOptimizer* graph, const std::string& pose_filename);

private:
    void read_tag_setting(const std::string& tag_setting);
    void read_camera_setting(const std::string& data_dir, const std::string& camera_setting);

public:
    double default_tag_size;
    std::vector<std::shared_ptr<StaticCamera>> cameras;
    std::unordered_map<long, std::shared_ptr<StaticTag>> tagmap;
    std::unordered_map<long, std::set<std::shared_ptr<StaticCamera>>> tag2camera;
};

}

#endif
