#ifndef SIM3_ESTIMATION_STATIC_TAG_HPP
#define SIM3_ESTIMATION_STATIC_TAG_HPP

#include <iostream>
#include <Eigen/Dense>
#include <boost/optional.hpp>

#include <ros/ros.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

namespace sparse_dynamic_calibration {

struct StaticTag {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<StaticTag>;

    StaticTag()
        : id(-1),
          prior_pose(boost::none),
          vertex(nullptr),
          prior_vertex(nullptr),
          prior_edge(nullptr)
    {}

    StaticTag(long id)
        : id(id),
          prior_pose(boost::none),
          vertex(nullptr),
          prior_vertex(nullptr),
          prior_edge(nullptr)
    {}

    StaticTag(long id, const boost::optional<Eigen::Isometry3d>& prior_pose)
        : id(id),
          prior_pose(prior_pose),
          vertex(nullptr),
          prior_vertex(nullptr),
          prior_edge(nullptr)
    {}

    void add_to_graph(ros::NodeHandle& nh, g2o::SparseOptimizer* graph) {
        if(vertex != nullptr) {
            return;
        }

        vertex = new g2o::VertexSE3Expmap();
        vertex->setId(graph->vertices().size());
        vertex->setEstimate(g2o::SE3Quat());
        graph->addVertex(vertex);

        prior_vertex = nullptr;
        prior_edge = nullptr;
        if(prior_pose) {
            prior_vertex = new g2o::VertexSE3Expmap();
            prior_vertex->setId(graph->vertices().size());
            prior_vertex->setEstimate(g2o::SE3Quat(prior_pose->linear(), prior_pose->translation()));
            prior_vertex->setFixed(true);
            graph->addVertex(prior_vertex);

            prior_edge = new g2o::EdgeSE3Expmap();
            prior_edge->vertices()[0] = vertex;
            prior_edge->vertices()[1] = prior_vertex;
            prior_edge->setMeasurement(g2o::SE3Quat());
            prior_edge->setInformation(Eigen::MatrixXd::Identity(6, 6) * nh.param<double>("tag_prior_inf_scale", 1e3));
            prior_edge->setId(graph->edges().size());
            graph->addEdge(prior_edge);
        }
    }
public:
    long id;
    boost::optional<Eigen::Isometry3d> prior_pose;

    g2o::VertexSE3Expmap* vertex;
    g2o::VertexSE3Expmap* prior_vertex;
    g2o::EdgeSE3Expmap* prior_edge;
};

}

#endif
