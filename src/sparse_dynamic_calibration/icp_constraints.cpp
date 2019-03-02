#include <sparse_dynamic_calibration/icp_constraints.hpp>

#include <g2o/edge_icp.hpp>
#include <g2o/core/robust_kernel_factory.h>


namespace sparse_dynamic_calibration {


ICPConstraints::ICPConstraints(ros::NodeHandle& nh, const std::vector<StaticCameraRefinement::Ptr>& cameras) {
    correspondences.clear();
    correspondences.reserve(2048);

    bool reciprocal_correspondence = nh.param<bool>("icp_reciprocal_correspondence", true);
    float max_correspondence_distance = nh.param<double>("icp_max_correspondence_distance", 0.25);


    for(size_t i=0; i<cameras.size(); i++) {
        for(size_t j=i+1; j<cameras.size(); j++) {
            const auto& camera1 = cameras[i];
            const auto& camera2 = cameras[j];

            std::vector<Correspondence> corr = find_correspondences(camera1, camera2, max_correspondence_distance, reciprocal_correspondence);
            std::copy(corr.begin(), corr.end(), std::back_inserter(correspondences));
        }
    }

    std::cout << "correspondences: " << correspondences.size() << std::endl;
}


std::vector<Correspondence> ICPConstraints::find_correspondences(const StaticCameraRefinement::Ptr& camera1, const StaticCameraRefinement::Ptr& camera2, float max_correspondence_distance, bool reciprocal_correspondence) {
    if(!camera1->hit_test(*camera2)) {
        return std::vector<Correspondence>();
    }

    float sqr_dist_thresh = max_correspondence_distance * max_correspondence_distance;

    std::vector<int> k_indices;
    std::vector<float> k_sqr_dists;

    std::vector<Correspondence> corrs;
    corrs.reserve(512);

    for(size_t i=0; i < camera1->transformed->size(); i++) {
        int pt1_index = i;
        const auto& pt1 = camera1->transformed->at(i);

        k_indices.clear();
        k_sqr_dists.clear();
        camera2->kdtree.nearestKSearch(pt1, 1, k_indices, k_sqr_dists);

        if(k_sqr_dists.front() > sqr_dist_thresh) {
            continue;
        }

        int pt2_index = k_indices.front();
        const auto& pt2 = camera2->transformed->at(k_indices.front());

        if(reciprocal_correspondence) {
            k_indices.clear();
            k_sqr_dists.clear();
            camera1->kdtree.nearestKSearch(pt2, 1, k_indices, k_sqr_dists);

            int index = k_indices.front();
            if(index != i) {
                continue;
            }
        }

        corrs.push_back(Correspondence(*camera1, pt1_index, *camera2, pt2_index));
    }

    std::cout << camera1->camera_name << " vs " << camera2->camera_name << " : " << corrs.size() << std::endl;
    return corrs;
}


void ICPConstraints::add_to_graph(ros::NodeHandle& nh, g2o::SparseOptimizer* graph) {
    auto factory = g2o::RobustKernelFactory::instance();

    bool point_to_point = nh.param<bool>("icp_point_to_plane", false);
    double inf_scale = nh.param<double>("icp_inf_scale", 1e2);
    std::string kernel_type = nh.param<std::string>("icp_robust_kernel", "NONE");
    double kernel_delta = nh.param<double>("icp_robust_kernel_delta", 1e-2);

    if(correspondences.size() < 1500) {
        return;
    }

    for(auto& corr: correspondences) {
        g2o::ICPCorrespondence constraint;

        const auto& pt0 = corr.camera1.filtered->at(corr.pt1_index);
        const auto& pt1 = corr.camera2.filtered->at(corr.pt2_index);

        constraint.pos0 = pt0.getVector3fMap().cast<double>();
        constraint.pos1 = pt1.getVector3fMap().cast<double>();
        constraint.normal0 = pt0.getNormalVector3fMap().cast<double>();
        constraint.normal1 = pt1.getNormalVector3fMap().cast<double>();


        if(constraint.normal0.array().isNaN().any() || constraint.normal1.array().isNaN().any()) {
            continue;
        }

        g2o::EdgeICP* edge = new g2o::EdgeICP();
        edge->point_to_plane = point_to_point;
        edge->vertices()[0] = corr.camera1.vertex;
        edge->vertices()[1] = corr.camera2.vertex;
        edge->setMeasurement(constraint);
        edge->setInformation(Eigen::Matrix3d::Identity() * inf_scale);

        if(kernel_type != "NONE") {
            g2o::RobustKernel* kernel = factory->construct(kernel_type);
            kernel->setDelta(kernel_delta);
            edge->setRobustKernel(kernel);
        }

        graph->addEdge(edge);
        corr.edge = edge;
    }
}

void ICPConstraints::remove_from_graph(g2o::SparseOptimizer* graph) {
    for(auto& corr: correspondences) {
        if(corr.edge) {
            graph->removeEdge(corr.edge);
        }
    }
}


}
