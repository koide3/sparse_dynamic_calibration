#include <sparse_dynamic_calibration/graph_refinement.hpp>

#include <memory>
#include <boost/format.hpp>

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/robust_kernel_io.hpp>
#include <g2o/edge_se3_plane.hpp>

#include <sparse_dynamic_calibration/pose_io.hpp>
#include <sparse_dynamic_calibration/icp_constraints.hpp>

namespace sparse_dynamic_calibration {

bool GraphRefinement::refine(ros::NodeHandle& nh, const std::string &data_dir) {
    if(!initialize_graph(nh.param<std::string>("g2o_optimizer_name", "lm_var_cholmod"))) {
        return false;
    }

    if(!read(data_dir)) {
        return false;
    }

    ros::Publisher points_pub = nh.advertise<sensor_msgs::PointCloud2>("/points", 1);

    auto original_clouds = accumulate_clouds();
    pcl::io::savePCDFileBinary("/tmp/original.pcd", *original_clouds);

    int icp_iterations = nh.param<int>("icp_iterations", 32);
    for(int i=0; i<icp_iterations && ros::ok(); i++) {
        ROS_INFO_STREAM("iteration " << i << "/" << icp_iterations);
        for(const auto& camera: cameras) {
            camera->update_transformed();
        }

        std::unique_ptr<ICPConstraints> icp_constraints(new ICPConstraints(nh, cameras));
        icp_constraints->add_to_graph(nh, graph.get());

        optimize(nh.param<int>("g2o_max_iterations", 32));

        icp_constraints->remove_from_graph(graph.get());

        if(points_pub.getNumSubscribers()) {
            auto accumulated = accumulate_clouds();
            points_pub.publish(accumulated);
        }
    }

    write(data_dir);

    auto refined_clouds = accumulate_clouds();
    pcl::io::savePCDFileBinary("/tmp/refined.pcd", *refined_clouds);

    return true;
}

bool GraphRefinement::initialize_graph(const std::string& solver_name) {
    graph.reset(new g2o::SparseOptimizer());
    std::cout << "construct solver... " << std::flush;
    g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty solver_property;
    g2o::OptimizationAlgorithm* solver = solver_factory->construct(solver_name, solver_property);
    graph->setAlgorithm(solver);

    if (!graph->solver()) {
        std::cerr << std::endl;
        std::cerr << "error : failed to allocate solver!!" << std::endl;
        solver_factory->listSolvers(std::cerr);
        std::cerr << "-------------" << std::endl;
        std::cin.ignore(1);
        graph.reset();
        return false;
    }

    std::cout << "done" << std::endl;

    return true;
}

bool GraphRefinement::optimize(int max_iterations) {
    std::cout << "optimizing..." << std::endl;
    std::cout << "vertices:" << graph->vertices().size() << " edges:" << graph->edges().size() << std::endl;
    graph->initializeOptimization();
    graph->setVerbose(true);

    double chi2 = graph->chi2();
    int iterations = graph->optimize(max_iterations);

    std::cout << "iterations: " << iterations << "/" << max_iterations << std::endl;
    std::cout << "chi2: (before)" << chi2 << " -> (after)" << graph->chi2() << std::endl;
    return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GraphRefinement::accumulate_clouds() const {
    g2o::SE3Quat to_world;
    auto tag0 = std::find_if(tags.begin(), tags.end(), [=](const StaticTag::Ptr& tag) { return tag->id == 0; });
    if(tag0 != tags.end() && (*tag0)->vertex) {
        g2o::SE3Quat upside_down(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix(), Eigen::Vector3d::Zero());

        to_world = upside_down * (*tag0)->vertex->estimate();
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated(new pcl::PointCloud<pcl::PointXYZ>());
    for(const auto& camera: cameras) {
        g2o::SE3Quat se3 = to_world * camera->vertex->estimate().inverse();
        Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
        pose.linear() = se3.rotation().toRotationMatrix().cast<float>();
        pose.translation() = se3.translation().cast<float>();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = camera->cloud;
        /*
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        cloud->resize(camera->filtered->size());
        for(int i=0; i<camera->filtered->size(); i++) {
            cloud->at(i).getVector4fMap() = camera->filtered->at(i).getVector4fMap();
        }
        cloud->width = cloud->size();
        cloud->height = 1;
        cloud->is_dense = false;
        */

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cloud, *transformed, pose);

        std::copy(transformed->begin(), transformed->end(), std::back_inserter(accumulated->points));
    }
    accumulated->header.frame_id = "world";
    accumulated->width = accumulated->size();
    accumulated->height = 1;
    accumulated->is_dense = false;

    return accumulated;
}

bool GraphRefinement::read(const std::string &data_dir) {
    std::string graph_filename = data_dir + "/graph.g2o";
    if(!graph->load(graph_filename.c_str())) {
        ROS_ERROR_STREAM("failed to load the pose graph!!");
        return false;
    }

    if(!load_robust_kernels(data_dir + "/robust_kernels.g2o", graph.get())) {
        ROS_ERROR_STREAM("failed to loda the robust kernels!!");
        return false;
    }

    floor_plane_vertex = new g2o::VertexPlane();
    floor_plane_vertex->setId(static_cast<int>(graph->vertices().size()));
    floor_plane_vertex->setEstimate(Eigen::Vector4d(0.0, 1.0, 0.0, -1.0));
    graph->addVertex(floor_plane_vertex);

    cv::FileStorage fs(data_dir + "/tag_camera_poses.yaml", cv::FileStorage::READ);
    if(!fs.isOpened()) {
        ROS_ERROR_STREAM("failed to load the camera network info!!");
        return false;
    }


    for(auto camera_itr = fs["cameras"].begin(); camera_itr != fs["cameras"].end(); camera_itr++) {
        std::string camera_name = (*camera_itr).name();
        int vertex_id = (*camera_itr)["vertex_id"];

        auto camera = std::make_shared<StaticCameraRefinement>(data_dir, camera_name);
        camera->vertex = dynamic_cast<g2o::VertexSE3Expmap*>(graph->vertex(vertex_id));

        if(camera->vertex == nullptr) {
            ROS_WARN_STREAM("failed to cast the vertex!!");
            continue;
        }
        camera->update_transformed();

        if(floor_plane_vertex) {
            camera->detect_floor_plane();

            if(camera->floor_plane) {
                g2o::Plane3D plane(camera->floor_plane->cast<double>());

                g2o::EdgeSE3Plane* edge = new g2o::EdgeSE3Plane();
                edge->vertices()[0] = camera->vertex;
                edge->vertices()[1] = floor_plane_vertex;
                edge->setMeasurement(plane);
                edge->setInformation(Eigen::Matrix3d::Identity() * 1e3);
                graph->addEdge(edge);
            }
        }



        ROS_INFO_STREAM("camera_name:" << camera_name << ", vertex:" << vertex_id);
        cameras.push_back(camera);
    }

    for(auto tag_itr = fs["tags"].begin(); tag_itr != fs["tags"].end(); tag_itr++) {
        std::string tag_name = (*tag_itr).name();
        long tag_id = std::stol(tag_name.substr(4));

        int vertex_id = (*tag_itr)["vertex_id"];

        auto tag = std::make_shared<StaticTag>(tag_id);
        tag->vertex = dynamic_cast<g2o::VertexSE3Expmap*>(graph->vertex(vertex_id));

        if(tag->vertex == nullptr) {
            ROS_WARN_STREAM("failed to cast the vertex!!");
            continue;
        }

        ROS_INFO_STREAM("tag_id:" << tag_id << ", vertex:" << vertex_id);
        tags.push_back(tag);
    }

    return true;
}

bool GraphRefinement::write(const std::string& data_dir) {
    ROS_INFO_STREAM("save to " << data_dir);

    for(const auto& tag: tags) {
        tag->vertex->setFixed(true);
    }
    for(const auto& camera: cameras) {
        camera->vertex->setFixed(true);
    }

    std::string output_graph_filename = data_dir + "/graph_refined.g2o";
    if(!graph->save(output_graph_filename.c_str())) {
        ROS_WARN_STREAM("failed to save the pose graph!!");
    }

    std::string output_filename = data_dir + "/tag_camera_poses_refined.yaml";
    cv::FileStorage fs(output_filename, cv::FileStorage::WRITE);
    if(!fs.isOpened()) {
        ROS_WARN_STREAM("failed to open the output file!!");
        return false;
    }

    ROS_INFO_STREAM("write camera poses");
    fs << "cameras" << "{";
    for(const auto& camera : cameras) {
        ROS_INFO_STREAM(camera->camera_name);
        if(!camera->vertex) {
            continue;
        }

        std::vector<double> posevec = pose2vec(camera->vertex->estimate().inverse());
        fs << camera->camera_name << "{";
        fs << "vertex_id" << camera->vertex->id();
        fs << "pose" << posevec;
        fs << "}";
    }
    fs << "}";

    ROS_INFO_STREAM("write tag poses");
    fs << "tags" << "{";
    for(const auto& tag : tags) {
        ROS_INFO_STREAM(boost::format("tag_%d") % tag->id);
        if(!tag->vertex) {
            fs << (boost::format("tag_%d") % tag->id).str() << std::vector<double>();
            continue;
        }

        std::vector<double> posevec = pose2vec(tag->vertex->estimate().inverse());

        fs << (boost::format("tag_%d") % tag->id).str() << "{";
        fs << "vertex_id" << tag->vertex->id();
        fs << "pose" << posevec;
        fs << "}";
    }

    fs << "}";

    fs.release();

    return true;
}

}
