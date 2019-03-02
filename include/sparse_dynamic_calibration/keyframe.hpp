#ifndef SIM3_ESTIMATION_KEYFRAME_HPP
#define SIM3_ESTIMATION_KEYFRAME_HPP

#include <memory>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/edge_sim3_se3_expmap.hpp>

#include <sparse_dynamic_calibration/static_tag.hpp>


namespace sparse_dynamic_calibration {

class Keyframe {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void add_to_graph(ros::NodeHandle& nh, g2o::SparseOptimizer* graph, std::unordered_map<long, std::shared_ptr<StaticTag>>& tagmap, Keyframe* prev_keyframe = nullptr);

public:
    ros::Time stamp;
    Eigen::Isometry3d vodom;
    std::vector<long> detected_tag_ids;
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> detected_tag_poses;

    g2o::VertexSim3Expmap* vertex;
    g2o::EdgeSim3* vodom_edge;
    std::vector<g2o::EdgeSim3SE3*> tag_edges;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

}

#endif
