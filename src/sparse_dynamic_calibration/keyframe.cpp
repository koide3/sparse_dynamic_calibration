#include <memory>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <g2o/core/robust_kernel_factory.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/edge_sim3_se3_expmap.hpp>

#include <sparse_dynamic_calibration/keyframe.hpp>


namespace sparse_dynamic_calibration {

void Keyframe::add_to_graph(ros::NodeHandle& nh, g2o::SparseOptimizer* graph, std::unordered_map<long, std::shared_ptr<StaticTag>>& tagmap, Keyframe* prev_keyframe) {
    g2o::RobustKernelFactory* robust_kernel_factory = g2o::RobustKernelFactory::instance();

    vertex = new g2o::VertexSim3Expmap();
    vertex->setId(graph->vertices().size());
    vertex->setEstimate(g2o::Sim3(vodom.linear(), vodom.translation(), nh.param<double>("initial_scale", 6)));
    graph->addVertex(vertex);

    vodom_edge = nullptr;
    if(prev_keyframe) {
        vodom_edge = new g2o::EdgeSim3();
        vodom_edge->vertices()[0] = prev_keyframe->vertex;
        vodom_edge->vertices()[1] = vertex;

        Eigen::Isometry3d relative = vodom * prev_keyframe->vodom.inverse();
        g2o::Sim3 relative_sim3(relative.linear(), relative.translation(), 1.0);

        vodom_edge->setMeasurement(relative_sim3);
        Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(7, 7);
        vodom_edge->setInformation(inf * nh.param<double>("vodom_inf_scale", 1e3));
        graph->addEdge(vodom_edge);

        auto prev_sim3 = prev_keyframe->vertex->estimate();
        vertex->setEstimate(relative_sim3 * prev_sim3);

        vodom_edge->computeError();
    }
    else {
        vertex->setFixed(true);
    }

    for(size_t i=0; i<detected_tag_ids.size(); i++) {
        long tag_id = detected_tag_ids[i];

        const auto& tag_pose = detected_tag_poses[i];

        auto found = tagmap.find(tag_id);
        if(found == tagmap.end()) {
            tagmap[tag_id].reset(new StaticTag(tag_id));
        }

        if(!tagmap[tag_id]->vertex) {
            tagmap[tag_id]->add_to_graph(nh, graph);
        }

        g2o::EdgeSim3SE3* tag_edge = new g2o::EdgeSim3SE3();
        tag_edge->vertices()[0] = vertex;
        tag_edge->vertices()[1] = tagmap[tag_id]->vertex;
        tag_edge->setMeasurement(g2o::SE3Quat(tag_pose.linear(), tag_pose.translation()).inverse());

        Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
        tag_edge->setInformation(inf * nh.param<double>("vodom2tag_inf_scale", 1e1));

        if(found == tagmap.end()) {
            tag_edge->init_v2();
        }

        std::string kernel_type = nh.param<std::string>("vodom2tag_robust_kernel", "Huber");
        if(kernel_type != "NONE") {
            g2o::RobustKernel* kernel = robust_kernel_factory->construct(kernel_type);
            if(kernel == nullptr) {
                std::cerr << "failed to construct the robust kernel!!" << std::endl;
            } else {
                kernel->setDelta(nh.param<double>("vodom2tag_robust_kernel_delta", 1e-3));
                tag_edge->setRobustKernel(kernel);
            }
        }

        graph->addEdge(tag_edge);
        // tag_edges.push_back(tag_edge);
    }
}

}
