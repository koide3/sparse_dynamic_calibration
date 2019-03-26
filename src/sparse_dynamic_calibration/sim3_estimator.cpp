#include <iostream>
#include <ros/ros.h>
#include <boost/format.hpp>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/robust_kernel_io.hpp>

#include <sparse_dynamic_calibration/pose_io.hpp>
#include <sparse_dynamic_calibration/keyframe.hpp>
#include <sparse_dynamic_calibration/sim3_estimator.hpp>


namespace sparse_dynamic_calibration {

Sim3Estimator::Sim3Estimator(long world_frame_tag_id)
    : world_frame_tag_id(world_frame_tag_id),
      max_iterations(0)
{}
Sim3Estimator::~Sim3Estimator() {}

bool Sim3Estimator::initialize_graph(const std::string& solver_name, int max_iterations) {
    this->max_iterations = max_iterations;

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

bool Sim3Estimator::initialize_tag_camera_network(const std::string& data_dir, const std::string &tag_setting, const std::string &camera_setting) {
    std::string graph_name = data_dir + "/graph_refined.g2o";
    tag_detector.reset(new TagDetector(tag_setting));

    tag_camera_network.reset(new TagCameraNetwork(data_dir, tag_setting, camera_setting));
    // tag_camera_network->add_to_graph(graph.get());

    return optimize();
}

void Sim3Estimator::update(ros::NodeHandle& nh, const ros::Time& stamp, const cv::Mat& camera_matrix, const cv::Mat& rect_image, const Eigen::Isometry3d& vodom) {
    std::vector<long> tag_ids;
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> tag_poses;

    cv::Mat grayscale;
    cv::cvtColor(rect_image, grayscale, CV_BGR2GRAY);
    std::shared_ptr<zarray> detected_tags = tag_detector->detect(grayscale);

    for(int i=0; i<zarray_size(detected_tags.get()); i++) {
        apriltag_detection* det;
        zarray_get(detected_tags.get(), i, &det);


        if(det->decision_margin < nh.param<double>("min_decision_margin", 100.0)) {
            det->id = -1;
            continue;
        }

        Eigen::Isometry3d pose = tag_detector->estimate_pose(camera_matrix, det);

        double max_distance = nh.param<double>("max_distance", 5.0);
        if(pose.translation().norm() > max_distance) {
            det->id = -1;
            continue;
        }

        tag_ids.push_back(det->id);
        tag_poses.push_back(pose);

        auto found = tag_detections_count.find(det->id);
        if(found == tag_detections_count.end()) {
            tag_detections_count[det->id] = 0;
        }
        tag_detections_count[det->id] ++;
    }

    keyframes.push_back(std::shared_ptr<Keyframe>(new Keyframe {stamp, vodom, tag_ids, tag_poses}));
    keyframes_map[stamp] = keyframes.back();

    Keyframe* prev_keyframe = keyframes.size() < 2 ? nullptr : keyframes[keyframes.size() - 2].get();
    keyframes.back()->add_to_graph(nh, graph.get(), tag_camera_network->tagmap, prev_keyframe);

    for(const auto& tag: tag_camera_network->tagmap) {
        if(!tag.second->vertex) {
            continue;
        }

        auto& associated_cameras = tag_camera_network->tag2camera[tag.second->id];
        for(auto& cam : associated_cameras) {
            cam->add_to_graph(nh, graph.get(), tag_camera_network->tagmap);
        }
    }

    auto found = tag_detections_count.find(world_frame_tag_id);
    if(found == tag_detections_count.end()) {
        tag_detections_count[world_frame_tag_id] = 0;
    }

    if(tag_detections_count[world_frame_tag_id] > 5) {
        keyframes.front()->vertex->setFixed(false);
        tag_camera_network->tagmap[world_frame_tag_id]->vertex->setFixed(true);
    }

    optimize();

    cv::Mat canvas = rect_image.clone();
    tag_detector->draw(canvas, camera_matrix, detected_tags);

    cv::Mat resized;
    cv::resize(canvas, resized, cv::Size(canvas.cols/2, canvas.rows/2));
    cv::imshow("canvas", resized);
    cv::waitKey(10);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Sim3Estimator::update(const ros::Time& stamp, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if(!cloud_mapper) {
        cloud_mapper.reset(new CloudMapper());
    }

    cloud_mapper->update(stamp, cloud);

    return cloud_mapper->generate(keyframes);
}

bool Sim3Estimator::optimize() {
    std::cout << "optimizing..." << std::endl;
    std::cout << "vertices:" << graph->vertices().size() << " edges:" << graph->edges().size() << std::endl;
    graph->initializeOptimization();
    graph->setVerbose(false);

    double chi2 = graph->chi2();
    int iterations = graph->optimize(max_iterations);

    std::cout << "iterations: " << iterations << "/" << max_iterations << std::endl;
    std::cout << "chi2: (before)" << chi2 << " -> (after)" << graph->chi2() << std::endl;
    return true;
}

void Sim3Estimator::write(const std::string& output_directory) {
    ROS_INFO_STREAM("save to " << output_directory);

    std::string output_graph_filename = output_directory + "/graph.g2o";
    if(!graph->save(output_graph_filename.c_str())) {
        ROS_WARN_STREAM("failed to save the pose graph!!");
    }

    if(!save_robust_kernels(output_directory + "/robust_kernels.g2o", graph.get())) {
        ROS_WARN_STREAM("failed to save the robust kernels!!");
    }

    std::string output_filename = output_directory + "/tag_camera_poses.yaml";
    cv::FileStorage fs(output_filename, cv::FileStorage::WRITE);
    if(!fs.isOpened()) {
        ROS_WARN_STREAM("failed to open the output file!!");
        return;
    }

    ROS_INFO_STREAM("write camera poses");
    fs << "cameras" << "{";
    for(const auto& camera : tag_camera_network->cameras) {
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
    for(const auto& tag : tag_camera_network->tagmap) {
        ROS_INFO_STREAM(boost::format("tag_%d") % tag.second->id);
        if(!tag.second->vertex) {
            fs << (boost::format("tag_%d") % tag.second->id).str() << std::vector<double>();
            continue;
        }

        std::vector<double> posevec = pose2vec(tag.second->vertex->estimate().inverse());

        fs << (boost::format("tag_%d") % tag.second->id).str() << "{";
        fs << "vertex_id" << tag.second->vertex->id();
        fs << "pose" << posevec;
        fs << "}";
    }

    fs << "}";

    fs.release();
}
}

