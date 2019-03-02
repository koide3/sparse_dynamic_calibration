#ifndef SIM3_ESTIMATION_STATIC_CAMERA_HPP
#define SIM3_ESTIMATION_STATIC_CAMERA_HPP

#include <iostream>
#include <Eigen/Dense>
#include <boost/optional.hpp>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <sparse_dynamic_calibration/static_tag.hpp>


namespace sparse_dynamic_calibration {

struct StaticCamera {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<StaticCamera>;

    virtual ~StaticCamera() {}

    StaticCamera();

    StaticCamera(const std::string& data_dir, const std::string& camera_name, const boost::optional<Eigen::Isometry3d>& prior_pose, const std::vector<long>& detected_tag_ids, const std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& detected_tag_poses);

    StaticCamera(const std::string& data_dir, const std::string& camera_name);

    void add_to_graph(ros::NodeHandle& nh, g2o::SparseOptimizer* graph, std::unordered_map<long, std::shared_ptr<StaticTag>>& tagmap);

private:
    void read_cloud(const std::string& data_dir, const std::string& camera_name);

public:
    std::string camera_name;
    boost::optional<Eigen::Isometry3d> prior_pose;

    std::vector<long> detected_tag_ids;
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> detected_tag_poses;

    g2o::VertexSE3Expmap* vertex;
    std::vector<g2o::EdgeSE3Expmap*> tag_edges;

    cv::Mat image;
    cv::Mat camera_matrix;
    Eigen::Isometry3d depth2rgb;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

}

#endif
