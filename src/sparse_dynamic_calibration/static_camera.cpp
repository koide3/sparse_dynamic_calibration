#include <sparse_dynamic_calibration/static_camera.hpp>

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <g2o/core/robust_kernel_factory.h>


namespace sparse_dynamic_calibration {

StaticCamera::StaticCamera()
    : camera_name("none"),
      prior_pose(boost::none),
      detected_tag_ids(),
      detected_tag_poses(),
      vertex(nullptr),
      tag_edges()
{}

StaticCamera::StaticCamera(const std::string& data_dir, const std::string& camera_name, const boost::optional<Eigen::Isometry3d>& prior_pose, const std::vector<long>& detected_tag_ids, const std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& detected_tag_poses)
    : camera_name(camera_name),
      prior_pose(prior_pose),
      detected_tag_ids(detected_tag_ids),
      detected_tag_poses(detected_tag_poses),
      vertex(nullptr),
      tag_edges()
{
    read_cloud(data_dir, camera_name);
}

StaticCamera::StaticCamera(const std::string& data_dir, const std::string& camera_name)
    : camera_name(camera_name)
{
    image = cv::imread(data_dir + "/" + camera_name + "_image.jpg");
    std::ifstream ifs(data_dir + "/" + camera_name + "_camera_matrix");
    camera_matrix.create(3, 3, CV_64FC1);
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            ifs >> camera_matrix.at<double>(i, j);
        }
    }

    read_cloud(data_dir, camera_name);
}

void StaticCamera::read_cloud(const std::string& data_dir, const std::string& camera_name) {
    depth2rgb = Eigen::Isometry3d::Identity();
    depth2rgb.translation() = Eigen::Vector3d(-0.052f, 0.0, 0.0);

    std::ifstream ifs(data_dir + "/" + camera_name + "_depth2rgb");
    if(ifs) {
        Eigen::Vector3d trans;
        ifs >> trans[0] >> trans[1] >> trans[2];

        Eigen::Quaterniond quat;
        ifs >> quat.x() >> quat.y() >> quat.z() >> quat.w();

        depth2rgb.translation() = trans;
        depth2rgb.linear() = quat.toRotationMatrix();
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::io::loadPCDFile(data_dir + "/" + camera_name + "_points.pcd", *original_cloud);

    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::transformPointCloud(*original_cloud, *cloud, depth2rgb.cast<float>());
    cloud->header.frame_id = camera_name;
}

// TODO: initial guess estimation is necessary.
//     : currently, without the robust kernel, optimization gets corrupted
void StaticCamera::add_to_graph(ros::NodeHandle& nh, g2o::SparseOptimizer* graph, std::unordered_map<long, std::shared_ptr<StaticTag>>& tagmap) {
    if(vertex != nullptr) {
        return;
    }

    g2o::RobustKernelFactory* robust_kernel_factory = g2o::RobustKernelFactory::instance();

    vertex = new g2o::VertexSE3Expmap();
    vertex->setId(graph->vertices().size());
    vertex->setEstimate(g2o::SE3Quat());
    graph->addVertex(vertex);

    for(size_t i=0; i<detected_tag_ids.size(); i++) {
        long tag_id = detected_tag_ids[i];
        const auto& tag_pose = detected_tag_poses[i];

        auto found = tagmap.find(tag_id);
        if(found == tagmap.end()) {
            tagmap[tag_id].reset(new StaticTag {tag_id});
        }

        if(!tagmap[tag_id]->vertex) {
            tagmap[tag_id]->add_to_graph(nh, graph);
        }

        g2o::EdgeSE3Expmap* tag_edge = new g2o::EdgeSE3Expmap();
        tag_edge->vertices()[0] = vertex;
        tag_edge->vertices()[1] = tagmap[tag_id]->vertex;
        tag_edge->setMeasurement(g2o::SE3Quat(tag_pose.linear(), tag_pose.translation()).inverse());
        tag_edge->setInformation(Eigen::MatrixXd::Identity(6, 6) * nh.param<double>("camera2tag_inf_scale", 1e2));

        std::string camera2tag_robust_kernel = nh.param<std::string>("camera2tag_robust_kernel", "Huber");
        if(camera2tag_robust_kernel != "NONE") {
            g2o::RobustKernel* kernel = robust_kernel_factory->construct("Huber");
            if(kernel == nullptr) {
                std::cerr << "failed to construct the robust kernel!!" << std::endl;
            } else {
                kernel->setDelta(nh.param<double>("camera2tag_robust_kernel_delta", 1e-3));
                tag_edge->setRobustKernel(kernel);
            }
        }

        graph->addEdge(tag_edge);
        tag_edges.push_back(tag_edge);
    }
}

}
