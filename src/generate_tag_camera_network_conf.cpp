#include <regex>
#include <memory>
#include <iostream>
#include <unordered_map>

#include <Eigen/Dense>
#include <boost/format.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <sparse_dynamic_calibration/pose_io.hpp>
#include <sparse_dynamic_calibration/static_tag.hpp>
#include <sparse_dynamic_calibration/static_camera.hpp>
#include <sparse_dynamic_calibration/tag_detector.hpp>


namespace sparse_dynamic_calibration {

class CameraNeworkTagDetection {
public:
    CameraNeworkTagDetection(const std::string& tag_settings) {
        tag_detector.reset(new TagDetector(tag_settings));
    }

    bool detect() {
        std::vector<std::string> topics = get_topics();

        for(const auto& topic : topics) {
            std::regex pattern("(/kinect.*)/rgb/image");
            std::smatch matched;
            std::regex_match(topic, matched, pattern);

            if(matched.empty()) {
                continue;
            };

            retrieve(matched[1]);
        }

        return true;
    }

    bool read_from_file() {
        for(int i=1; ; i++) {
            std::string camera_name = (boost::format("kinect_%02d") % i).str();
            ROS_INFO_STREAM("read:" << camera_name);
            if(!read_from_file(camera_name)) {
                break;
            }
        }

        return true;
    }

    std::vector<std::string> get_topics() const {
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);

        std::vector<std::string> topics;
        for(ros::master::V_TopicInfo::iterator it=master_topics.begin(); it != master_topics.end(); it++) {
            const ros::master::TopicInfo& info = *it;
            topics.push_back(info.name);
        }

        std::sort(topics.begin(), topics.end());
        return topics;
    }

    void retrieve(const std::string& camera_name) {
        ROS_INFO_STREAM("camera_name: " << camera_name);
        ROS_INFO_STREAM("waiting for camera_info...");
        ROS_INFO_STREAM("topic: " << camera_name + "/rgb/camera_info");
        sensor_msgs::CameraInfoConstPtr camera_info_msg;
        for(int i=0; i<5; i++) {
           camera_info_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_name + "/rgb/camera_info", ros::Duration(1.0));
           if(camera_info_msg) {
               break;
           } else {
               ROS_INFO_STREAM("retry...");
           }
        }
        if(!camera_info_msg) {
            ROS_WARN_STREAM("failed to retrieve camera_info!!");
            return;
        }

        ROS_INFO_STREAM("waiting for image...");
        ROS_INFO_STREAM("topic: " << camera_name + "/rgb/image");
        sensor_msgs::ImageConstPtr image_msg;
        for(int i=0; i<5; i++) {
           image_msg = ros::topic::waitForMessage<sensor_msgs::Image>(camera_name + "/rgb/image", ros::Duration(2.5));
           if(image_msg) {
               break;
           } else {
               ROS_INFO_STREAM("retry...");
           }
        }
        if(!image_msg) {
            ROS_WARN_STREAM("failed to retrieve image!!");
            return;
        }

        ROS_INFO_STREAM("waiting for point cloud...");
        ROS_INFO_STREAM("topic: " << camera_name + "/depth_ir/points");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        for(int i=0; i<5; i++) {
            sensor_msgs::PointCloud2ConstPtr points_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(camera_name + "/depth_ir/points", ros::Duration(2.5));
            if(points_msg) {
                cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::fromROSMsg(*points_msg, *cloud);
                break;
            } else {
                ROS_INFO_STREAM("retry...");
            }
        }
        if(!cloud) {
            ROS_WARN_STREAM("failed to retrieve point cloud!!");
            return;
        }

        cv::Mat camera_matrix(3, 3, CV_64FC1);
        cv::Mat distortion(5, 1, CV_64FC1);

        for(int i=0; i<3; i++) {
            for(int j=0; j<3; j++) {
                camera_matrix.at<double>(i, j) = camera_info_msg->K[i * 3 + j];
            }
        }
        for(int i=0; i<5; i++) {
            distortion.at<double>(i) = camera_info_msg->D[i];
        }

        auto cv_image = cv_bridge::toCvCopy(image_msg, "bgr8");

        cv::Mat undistorted = cv_image->image.clone();
        cv::undistort(cv_image->image, undistorted, camera_matrix, distortion);

        cv::Mat grayscale;
        cv::cvtColor(undistorted, grayscale, CV_BGR2GRAY);

        ROS_INFO_STREAM("detecting tags...");
        auto detected_tags = tag_detector->detect(grayscale);

        std::shared_ptr<StaticCamera> camera(new StaticCamera());
        int begin = camera_name.find_first_not_of('/');
        int end = camera_name.find_first_of('/', begin);
        camera->camera_name = camera_name.substr(begin);
        ROS_INFO_STREAM("camera_name: " << camera->camera_name);

        camera->image = undistorted.clone();
        camera->camera_matrix = camera_matrix.clone();
        camera->cloud = cloud;

        ROS_INFO_STREAM("waiting for depth2rgb...");
        for(int i=0; i<5; i++) {
            if(tf_listener.waitForTransform(image_msg->header.frame_id, cloud->header.frame_id, ros::Time(0), ros::Duration(1))) {
                tf::StampedTransform depth2rgb;
                tf_listener.lookupTransform(image_msg->header.frame_id, cloud->header.frame_id, ros::Time(0), depth2rgb);
                tf::transformTFToEigen(depth2rgb, camera->depth2rgb);
                break;
            }

            if(i==4) {
                ROS_INFO_STREAM("failed");
                return;
            }
            ROS_INFO_STREAM("retry...");
        }

        cameras.push_back(camera);

        tf::StampedTransform transform;
        ROS_INFO_STREAM("waiting for transform...");
        if(tf_listener.waitForTransform("world", image_msg->header.frame_id, ros::Time(0), ros::Duration(2))) {
            tf_listener.lookupTransform("world", image_msg->header.frame_id, ros::Time(0), transform);
            camera->prior_pose = Eigen::Isometry3d::Identity();
            tf::transformTFToEigen(transform, *camera->prior_pose);
        }

        for(int i=0; i<zarray_size(detected_tags.get()); i++) {
            apriltag_detection* det;
            zarray_get(detected_tags.get(), i, &det);

            Eigen::Isometry3d pose = tag_detector->estimate_pose(camera_matrix, det);

            camera->detected_tag_ids.push_back(det->id);
            camera->detected_tag_poses.push_back(pose);

            auto found = tagmap.find(det->id);
            if(found == tagmap.end()) {
                std::shared_ptr<StaticTag> tag(new StaticTag());
                tag->id = det->id;
                tags.push_back(tag);
                tagmap[det->id] = tag;
            }

            cv::Mat obj_points(4, 1, CV_64FC3, cv::Scalar::all(0));
            obj_points.at<cv::Vec3d>(1)[0] = 1;
            obj_points.at<cv::Vec3d>(2)[1] = 1;
            obj_points.at<cv::Vec3d>(3)[2] = 1;

            cv::Mat r(3, 3, CV_64FC1);
            for(int i=0; i<3; i++) {
                for(int j=0; j<3; j++) {
                    r.at<double>(i, j) = pose.linear()(i, j);
                }
            }
            cv::Rodrigues(r, r);

            cv::Mat t(3, 1, CV_64FC1);
            for(int i=0; i<3; i++) {
                t.at<double>(i) = pose.translation()(i);
            }

            cv::Mat image_points;
            cv::projectPoints(obj_points, r, t, camera_matrix, cv::Mat(), image_points);

            cv::Vec2d org = image_points.at<cv::Vec2d>(0);
            cv::Vec2d x_axis = image_points.at<cv::Vec2d>(1);
            cv::Vec2d y_axis = image_points.at<cv::Vec2d>(2);
            cv::Vec2d z_axis = image_points.at<cv::Vec2d>(3);
            cv::line(undistorted, cv::Point(org[0], org[1]), cv::Point(x_axis[0], x_axis[1]), cv::Scalar(0, 0, 255), 2);
            cv::line(undistorted, cv::Point(org[0], org[1]), cv::Point(y_axis[0], y_axis[1]), cv::Scalar(0, 255, 0), 2);
            cv::line(undistorted, cv::Point(org[0], org[1]), cv::Point(z_axis[0], z_axis[1]), cv::Scalar(255, 0, 0), 2);
        }

        for(int i=0; i<zarray_size(detected_tags.get()); i++) {
            apriltag_detection* det;
            zarray_get(detected_tags.get(), i, &det);
            cv::line(undistorted, cv::Point(det->p[0][0], det->p[0][1]), cv::Point(det->p[1][0], det->p[1][1]), cv::Scalar(255, 0, 0), 2);
            cv::line(undistorted, cv::Point(det->p[1][0], det->p[1][1]), cv::Point(det->p[2][0], det->p[2][1]), cv::Scalar(255, 0, 0), 2);
            cv::line(undistorted, cv::Point(det->p[2][0], det->p[2][1]), cv::Point(det->p[3][0], det->p[3][1]), cv::Scalar(255, 0, 0), 2);
            cv::line(undistorted, cv::Point(det->p[3][0], det->p[3][1]), cv::Point(det->p[0][0], det->p[0][1]), cv::Scalar(255, 0, 0), 2);

            std::stringstream sst;
            sst << det->id;

            int baseline = 0;
            cv::Size textsize = cv::getTextSize(sst.str(), CV_FONT_HERSHEY_SCRIPT_SIMPLEX, 2.0,  2, &baseline);
            cv::putText(undistorted, sst.str(), cv::Point(det->c[0] - textsize.width/2, det->c[1] + textsize.height/2), CV_FONT_HERSHEY_SCRIPT_SIMPLEX, 2.0, cv::Scalar(255, 64, 64), 2);
        }

        cv::imwrite((boost::format("/tmp/%s.jpg") % camera->camera_name).str(), undistorted);
        cv::imshow("image", undistorted);
        cv::waitKey(1000);
    }

    bool read_from_file(const std::string& camera_name) {
        std::string data_dir = ros::package::getPath("sparse_dynamic_calibration") + "/data";
        ROS_INFO_STREAM("camera_name: " << camera_name);

        cv::Mat camera_matrix(3, 3, CV_64FC1);

        std::ifstream intrinsic_ifs(data_dir + "/" + camera_name + "_camera_matrix");
        if(!intrinsic_ifs) {
            ROS_INFO_STREAM("failed to open camera matrix file!!");
            return false;
        }
        for(int i=0; i<3; i++) {
            for(int j=0; j<3; j++) {
                intrinsic_ifs >> camera_matrix.at<double>(i, j);
            }
        }

        cv::Mat undistorted = cv::imread(data_dir + "/" + camera_name + "_image.jpg");
        if(!undistorted.data) {
           ROS_INFO_STREAM("failed to open the image file!!");
           return false;
        }

        cv::Mat grayscale;
        cv::cvtColor(undistorted, grayscale, CV_BGR2GRAY);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        if(pcl::io::loadPCDFile(data_dir + "/" + camera_name + "_points.pcd", *cloud)) {
            ROS_INFO_STREAM("failed to open the cloud file!!");
            return false;
        }

        ROS_INFO_STREAM("detecting tags...");
        auto detected_tags = tag_detector->detect(grayscale);

        std::shared_ptr<StaticCamera> camera(new StaticCamera());
        int begin = camera_name.find_first_not_of('/');
        int end = camera_name.find_first_of('/', begin);
        camera->camera_name = camera_name.substr(begin);
        ROS_INFO_STREAM("camera_name: " << camera->camera_name);

        camera->image = undistorted.clone();
        camera->camera_matrix = camera_matrix.clone();
        camera->cloud = cloud;

        cameras.push_back(camera);

        for(int i=0; i<zarray_size(detected_tags.get()); i++) {
            apriltag_detection* det;
            zarray_get(detected_tags.get(), i, &det);

            Eigen::Isometry3d pose = tag_detector->estimate_pose(camera_matrix, det);

            camera->detected_tag_ids.push_back(det->id);
            camera->detected_tag_poses.push_back(pose);

            auto found = tagmap.find(det->id);
            if(found == tagmap.end()) {
                std::shared_ptr<StaticTag> tag(new StaticTag());
                tag->id = det->id;
                tags.push_back(tag);
                tagmap[det->id] = tag;
            }

            cv::Mat obj_points(4, 1, CV_64FC3, cv::Scalar::all(0));
            obj_points.at<cv::Vec3d>(1)[0] = 1;
            obj_points.at<cv::Vec3d>(2)[1] = 1;
            obj_points.at<cv::Vec3d>(3)[2] = 1;

            cv::Mat r(3, 3, CV_64FC1);
            for(int i=0; i<3; i++) {
                for(int j=0; j<3; j++) {
                    r.at<double>(i, j) = pose.linear()(i, j);
                }
            }
            cv::Rodrigues(r, r);

            cv::Mat t(3, 1, CV_64FC1);
            for(int i=0; i<3; i++) {
                t.at<double>(i) = pose.translation()(i);
            }

            cv::Mat image_points;
            cv::projectPoints(obj_points, r, t, camera_matrix, cv::Mat(), image_points);

            cv::Vec2d org = image_points.at<cv::Vec2d>(0);
            cv::Vec2d x_axis = image_points.at<cv::Vec2d>(1);
            cv::Vec2d y_axis = image_points.at<cv::Vec2d>(2);
            cv::Vec2d z_axis = image_points.at<cv::Vec2d>(3);
            cv::line(undistorted, cv::Point(org[0], org[1]), cv::Point(x_axis[0], x_axis[1]), cv::Scalar(0, 0, 255), 2);
            cv::line(undistorted, cv::Point(org[0], org[1]), cv::Point(y_axis[0], y_axis[1]), cv::Scalar(0, 255, 0), 2);
            cv::line(undistorted, cv::Point(org[0], org[1]), cv::Point(z_axis[0], z_axis[1]), cv::Scalar(255, 0, 0), 2);
        }

        for(int i=0; i<zarray_size(detected_tags.get()); i++) {
            apriltag_detection* det;
            zarray_get(detected_tags.get(), i, &det);
            cv::line(undistorted, cv::Point(det->p[0][0], det->p[0][1]), cv::Point(det->p[1][0], det->p[1][1]), cv::Scalar(255, 0, 0), 2);
            cv::line(undistorted, cv::Point(det->p[1][0], det->p[1][1]), cv::Point(det->p[2][0], det->p[2][1]), cv::Scalar(255, 0, 0), 2);
            cv::line(undistorted, cv::Point(det->p[2][0], det->p[2][1]), cv::Point(det->p[3][0], det->p[3][1]), cv::Scalar(255, 0, 0), 2);
            cv::line(undistorted, cv::Point(det->p[3][0], det->p[3][1]), cv::Point(det->p[0][0], det->p[0][1]), cv::Scalar(255, 0, 0), 2);

            std::stringstream sst;
            sst << det->id;

            int baseline = 0;
            cv::Size textsize = cv::getTextSize(sst.str(), CV_FONT_HERSHEY_SCRIPT_SIMPLEX, 2.0,  2, &baseline);
            cv::putText(undistorted, sst.str(), cv::Point(det->c[0] - textsize.width/2, det->c[1] + textsize.height/2), CV_FONT_HERSHEY_SCRIPT_SIMPLEX, 2.0, cv::Scalar(255, 64, 64), 2);
        }

        cv::imwrite((boost::format("/tmp/%s.jpg") % camera->camera_name).str(), undistorted);
        cv::imshow("image", undistorted);
        cv::waitKey(1000);

        return true;
    }

private:
    tf::TransformListener tf_listener;
    std::unique_ptr<TagDetector> tag_detector;

public:
    std::vector<std::shared_ptr<StaticTag>> tags;
    std::vector<std::shared_ptr<StaticCamera>> cameras;
    std::unordered_map<long, std::shared_ptr<StaticTag>> tagmap;
};

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "generate_tag_camera_network_conf");
    std::string data_dir = ros::package::getPath("sparse_dynamic_calibration") + "/data";

    sparse_dynamic_calibration::CameraNeworkTagDetection tag_detection(data_dir + "/tags.yaml");
    tag_detection.detect();
    // tag_detection.read_from_file();

    if(tag_detection.cameras.empty()) {
        ROS_WARN_STREAM("no cameras!!");
        return 1;
    }

    cv::FileStorage fs(data_dir + "/cameras.yaml", cv::FileStorage::WRITE);
    for(const auto& camera : tag_detection.cameras) {
        cv::imwrite((boost::format("%s/%s_image.jpg") % data_dir % camera->camera_name).str(), camera->image);
        std::ofstream camera_matrix_ofs((boost::format("%s/%s_camera_matrix") % data_dir % camera->camera_name).str());
        for(int i=0; i<9; i++) {
            camera_matrix_ofs << camera->camera_matrix.at<double>(i) << " ";
        }

        std::ofstream depth2rgb_ofs((boost::format("%s/%s_depth2rgb") % data_dir % camera->camera_name).str());
        depth2rgb_ofs << camera->depth2rgb.translation().transpose() << " " << Eigen::Quaterniond(camera->depth2rgb.linear()).coeffs().transpose() << std::endl;

        if(camera->cloud) {
            pcl::io::savePCDFileBinary((boost::format("%s/%s_points.pcd") % data_dir % camera->camera_name).str(), *camera->cloud);
        }

        std::cout << camera->camera_name << ":" << std::endl;
        fs << camera->camera_name << "{";
        if(camera->prior_pose) {
            fs << "pose" << sparse_dynamic_calibration::pose2vec(*camera->prior_pose);
        }

        fs << "tag_detections" << "{";

        for(size_t i=0; i<camera->detected_tag_ids.size(); i++) {
            const auto& tag_pose = camera->detected_tag_poses[i];
            std::vector<double> pose = sparse_dynamic_calibration::pose2vec(tag_pose);

            fs << (boost::format("tag_%d") % camera->detected_tag_ids[i]).str() << pose;

            std::cout << "  - tag_" << camera->detected_tag_ids[i] << ": [ ";
            for(size_t j=0; j<pose.size(); j++) {
                std::cout << pose[j] << " ";
            }
            std::cout << "]" << std::endl;
        }
        fs << "}";
        fs << "}";
    }

    fs.release();

    return 0;
}
