#include <sparse_dynamic_calibration/tag_camera_network.hpp>

#include <regex>
#include <sparse_dynamic_calibration/pose_io.hpp>

namespace sparse_dynamic_calibration {

TagCameraNetwork::TagCameraNetwork(const std::string& data_dir, const std::string& tag_setting, const std::string& camera_setting) {
    read_tag_setting(tag_setting);
    read_camera_setting(data_dir, camera_setting);

    for(const auto& camera : cameras) {
        for(long tag_id : camera->detected_tag_ids) {
            tag2camera[tag_id].insert(camera);
        }
    }
}

void TagCameraNetwork::read_tag_setting(const std::string& tag_setting) {
    std::cout << "tag_setting: " << tag_setting << std::endl;
    cv::FileStorage fs(tag_setting, cv::FileStorage::READ);
    if(!fs.isOpened()) {
        std::cerr << "failed to open the tag setting file!!" << std::endl;
        return;
    }

    default_tag_size = fs["default_tag_size"].real();

    for(cv::FileNodeIterator tag_itr=fs.root().begin(); tag_itr != fs.root().end(); tag_itr++) {
        std::string tag_name = (*tag_itr).name();
        std::regex pattern("tag_([0-9]+)");
        std::smatch matched;
        std::regex_search(tag_name, matched, pattern);

        if(matched.empty()) {
            // maybe it's default_tag_size
            continue;
        }

        long tag_id = std::stol(matched.str(1));
        double tag_size = (*tag_itr)["tag_size"].real();

        cv::FileNode pose_node = (*tag_itr)["pose"];
        std::vector<double> pose_elements;
        pose_node >> pose_elements;

        boost::optional<Eigen::Isometry3d> tag_pose_prior;
        if(!pose_elements.empty()) {
            tag_pose_prior = Eigen::Isometry3d::Identity();
            tag_pose_prior->translation() = Eigen::Vector3d(pose_elements[0], pose_elements[1], pose_elements[2]);
            tag_pose_prior->linear() = Eigen::Quaterniond(pose_elements[6], pose_elements[3], pose_elements[4], pose_elements[5]).toRotationMatrix();
        }

        std::shared_ptr<StaticTag> tag(new StaticTag(tag_id, tag_pose_prior));

        tagmap[tag->id] = tag;
    }
}

void TagCameraNetwork::read_camera_setting(const std::string& data_dir, const std::string& camera_setting) {
    std::cout << "camera_setting: " << camera_setting << std::endl;
    cv::FileStorage fs(camera_setting, cv::FileStorage::READ);
    if(!fs.isOpened()) {
        std::cerr << "failed to open the camera setting file!!" << std::endl;
        return;
    }

    for(cv::FileNodeIterator camera_itr=fs.root().begin(); camera_itr != fs.root().end(); camera_itr++) {
        std::string camera_name = (*camera_itr).name();

        std::vector<double> pose_vec;
        (*camera_itr)["pose"] >> pose_vec;

        boost::optional<Eigen::Isometry3d> camera_pose_prior;
        if(!pose_vec.empty()) {
            camera_pose_prior = Eigen::Isometry3d::Identity();
            camera_pose_prior->translation() = Eigen::Vector3d(pose_vec[0], pose_vec[1], pose_vec[2]);
            camera_pose_prior->linear() = Eigen::Quaterniond(pose_vec[6], pose_vec[3], pose_vec[4], pose_vec[5]).toRotationMatrix();
        }

        std::vector<long> detected_tag_ids;
        std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> detected_tag_poses;
        cv::FileNode tag_det_node = (*camera_itr)["tag_detections"];
        for(auto tag_det_itr=tag_det_node.begin(); tag_det_itr!=tag_det_node.end(); tag_det_itr++) {
            std::string tag_name = (*tag_det_itr).name();
            std::regex pattern("tag_([0-9]+)");
            std::smatch matched;
            std::regex_search(tag_name, matched, pattern);

            long tag_id = std::stol(matched.str(1));

            std::vector<double> tag_pose;
            (*tag_det_itr) >> tag_pose;

            detected_tag_ids.push_back(tag_id);

            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.translation() = Eigen::Vector3d(tag_pose[0], tag_pose[1], tag_pose[2]);
            pose.linear() = Eigen::Quaterniond(tag_pose[6], tag_pose[3], tag_pose[4], tag_pose[5]).toRotationMatrix();
            detected_tag_poses.push_back(pose);
        }

        std::shared_ptr<StaticCamera> camera(new StaticCamera(data_dir, camera_name, camera_pose_prior, detected_tag_ids, detected_tag_poses));
        cameras.push_back(camera);
    }
}


bool TagCameraNetwork::read_poses(ros::NodeHandle& nh, g2o::SparseOptimizer* graph, const std::string& pose_filename) {
    ROS_INFO_STREAM("read calibrated poses...");
    cv::FileStorage fs(pose_filename, cv::FileStorage::READ);
    if(!fs.isOpened()) {
        return false;
    }

    for(auto camera_itr = fs["cameras"].begin(); camera_itr != fs["cameras"].end(); camera_itr++) {
        std::string camera_name = (*camera_itr).name();

        auto camera = std::find_if(cameras.begin(), cameras.end(), [&](const StaticCamera::Ptr& camera) { return camera->camera_name == camera_name; });
        if(camera == cameras.end()) {
            ROS_ERROR_STREAM("failed to find the camera:" << camera_name);
            continue;
        }

        std::vector<double> posevec;
        (*camera_itr)["pose"] >> posevec;

        g2o::SE3Quat se3 = vec2se3(posevec);

        (*camera)->add_to_graph(nh, graph, tagmap);
        (*camera)->vertex->setEstimate(se3.inverse());
        (*camera)->vertex->setFixed(true);

        ROS_INFO_STREAM(camera_name << " pose loaded");
    }


    for(auto tag_itr = fs["tags"].begin(); tag_itr != fs["tags"].end(); tag_itr++) {
        std::string tag_name = (*tag_itr).name();

        std::regex pattern("tag_([0-9]+)");
        std::smatch matched;
        std::regex_search(tag_name, matched, pattern);

        if(matched.empty()) {
            // maybe it's default_tag_size
            continue;
        }

        long tag_id = std::stol(matched.str(1));

        auto tag = tagmap.find(tag_id);
        if(tag == tagmap.end()) {
            tagmap[tag_id] = StaticTag::Ptr(new StaticTag(tag_id));
            tag = tagmap.find(tag_id);
        }

        std::vector<double> posevec;
        (*tag_itr)["pose"] >> posevec;

        g2o::SE3Quat se3 = vec2se3(posevec);
        tag->second->add_to_graph(nh, graph);
        tag->second->vertex->setEstimate(se3.inverse());
        tag->second->vertex->setFixed(true);

        ROS_INFO_STREAM("tag[" << tag_id << "] pose loaded");
    }

    return true;
}

}
