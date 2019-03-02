#ifndef GRAPH_REFINEMENT_HPP
#define GRAPH_REFINEMENT_HPP

#include <memory>
#include <vector>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

#include <sparse_dynamic_calibration/static_tag.hpp>
#include <sparse_dynamic_calibration/static_camera_refinement.hpp>

namespace sparse_dynamic_calibration {

class GraphRefinement {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GraphRefinement() {}

    bool refine(ros::NodeHandle& nh, const std::string& data_dir);

    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulate_clouds() const;

private:
    bool initialize_graph(const std::string& solver_name);

    bool optimize(int max_iterations);

    bool read(const std::string& data_dir);

    bool write(const std::string& data_dir);

private:
    g2o::VertexPlane* floor_plane_vertex;
    std::unique_ptr<g2o::SparseOptimizer> graph;

    std::vector<StaticTag::Ptr> tags;
    std::vector<StaticCameraRefinement::Ptr> cameras;
};

}

#endif
