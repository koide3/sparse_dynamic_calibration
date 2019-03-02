#ifndef SIM3_ESTIMATION_SIM3_ESTIMATOR_HPP
#define SIM3_ESTIMATION_SIM3_ESTIMATOR_HPP

#include <memory>
#include <vector>
#include <unordered_map>

#include <ros/time.h>

#include <sparse_dynamic_calibration/ros_time_hash.hpp>
#include <sparse_dynamic_calibration/keyframe.hpp>
#include <sparse_dynamic_calibration/tag_detector.hpp>
#include <sparse_dynamic_calibration/tag_camera_network.hpp>
#include <sparse_dynamic_calibration/cloud_mapper.hpp>

namespace g2o {
class SparseOptimizer;
}

namespace sparse_dynamic_calibration {

class Sim3Estimator {
public:
    Sim3Estimator();
    ~Sim3Estimator();

    void update(ros::NodeHandle& nh, const ros::Time& stamp, const cv::Mat& camera_matrix, const cv::Mat& rect_image, const Eigen::Isometry3d& vodom);
    pcl::PointCloud<pcl::PointXYZ>::Ptr update(const ros::Time& stamp, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    bool optimize();

    void write(const std::string& output_filename);

public:
    bool initialize_graph(const std::string& solver_name, int max_iterations);

    bool initialize_tag_camera_network(const std::string& data_dir, const std::string& tag_setting, const std::string& camera_setting);

public:
    int max_iterations;
    std::unique_ptr<g2o::SparseOptimizer> graph;

    std::unique_ptr<TagDetector> tag_detector;
    std::unique_ptr<TagCameraNetwork> tag_camera_network;

    std::vector<std::shared_ptr<Keyframe>> keyframes;
    std::unordered_map<ros::Time, std::shared_ptr<Keyframe>, RosTimeHash> keyframes_map;

    std::unique_ptr<CloudMapper> cloud_mapper;
};

}

#endif
