#ifndef GICP_CONSTRAINTS_HPP
#define GICP_CONSTRAINTS_HPP

#include <ros/ros.h>
#include <g2o/types/icp/types_icp.h>
#include <g2o/core/sparse_optimizer.h>
#include <sparse_dynamic_calibration/static_camera_refinement.hpp>

namespace sparse_dynamic_calibration {

struct Correspondence {
public:
    Correspondence(const StaticCameraRefinement& camera1, int pt1_index, const StaticCameraRefinement& camera2, int pt2_index)
        : camera1(camera1),
          camera2(camera2),
          pt1_index(pt1_index),
          pt2_index(pt2_index),
          edge(nullptr)
    {}

    const StaticCameraRefinement& camera1;
    const StaticCameraRefinement& camera2;

    int pt1_index;
    int pt2_index;

    g2o::OptimizableGraph::Edge* edge;
};

class ICPConstraints {
public:
    ICPConstraints(ros::NodeHandle& nh, const std::vector<StaticCameraRefinement::Ptr>& cameras);

    void add_to_graph(ros::NodeHandle& nh, g2o::SparseOptimizer* graph);
    void remove_from_graph(g2o::SparseOptimizer* graph);
private:
    std::vector<Correspondence> find_correspondences(const StaticCameraRefinement::Ptr& camera1, const StaticCameraRefinement::Ptr& camera2, float max_correspondence_distance, bool reciprocal_correspondence);

private:
    std::vector<Correspondence> correspondences;
};

}


#endif // GICP_CONSTRAINTS_HPP
