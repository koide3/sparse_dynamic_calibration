#include <mutex>
#include <regex>
#include <memory>
#include <string>
#include <iostream>
#include <unordered_map>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sparse_dynamic_calibration/PoseSim3Stamped.h>

#include <sparse_dynamic_calibration/sim3_estimator.hpp>
#include <sparse_dynamic_calibration/graph_marker_publisher.hpp>


class TransformPublisher {
public:
    TransformPublisher(const std::string& world_frame_id)
        : world_frame_id(world_frame_id)
    {
    }

    void update_pose(const std::string& name, const g2o::Sim3& pose) {
        Eigen::Isometry3d pose_ = Eigen::Isometry3d::Identity();
        pose_.translation() = pose.translation() * pose.scale();
        pose_.linear() = pose.rotation().toRotationMatrix();
        update_pose(name, pose_);
    }

    void update_pose(const std::string& name, const Eigen::Isometry3d& pose) {
        posemap[name] = pose.inverse();
    }

    void publish() {
        ros::Time stamp = ros::Time::now();
        for(const auto& pose : posemap) {
            geometry_msgs::TransformStamped transform;
            transform.header.frame_id = "vodom";
            transform.header.stamp = stamp;
            transform.child_frame_id = pose.first;

            Eigen::Quaterniond quat(pose.second.linear());
            transform.transform.translation.x = pose.second.translation().x();
            transform.transform.translation.y = pose.second.translation().y();
            transform.transform.translation.z = pose.second.translation().z();
            transform.transform.rotation.w = quat.w();
            transform.transform.rotation.x = quat.x();
            transform.transform.rotation.y = quat.y();
            transform.transform.rotation.z = quat.z();
            tf_broadcaster.sendTransform(transform);
        }

        auto found = posemap.find(world_frame_id);
        if(found == posemap.end()) {
            return;
        }

        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = "world";
        transform.header.stamp = stamp;
        transform.child_frame_id = "vodom";

        Eigen::Isometry3d world_pose = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) * found->second.inverse();

        Eigen::Quaterniond quat(world_pose.linear());
        transform.transform.translation.x = world_pose.translation().x();
        transform.transform.translation.y = world_pose.translation().y();
        transform.transform.translation.z = world_pose.translation().z();
        transform.transform.rotation.w = quat.w();
        transform.transform.rotation.x = quat.x();
        transform.transform.rotation.y = quat.y();
        transform.transform.rotation.z = quat.z();
        tf_broadcaster.sendTransform(transform);
    }

    void spin() {
        ros::Rate rate(10);
        while(ros::ok()) {
            publish();
            rate.sleep();
        }
    }

private:
    std::string world_frame_id;
    tf::TransformBroadcaster tf_broadcaster;
    std::unordered_map<std::string, Eigen::Isometry3d> posemap;
};


class SparseDynamicCalibrationNode {
public:
    SparseDynamicCalibrationNode()
        : nh(),
          private_nh("~"),
          image_sub(nh, "image", 30),
          camera_info_sub(nh, "camera_info", 30),
          vodom_sub(nh, "/vodom", 30),
          save_sub(private_nh.subscribe<std_msgs::Empty>("save", 1, &SparseDynamicCalibrationNode::save_callback, this)),
          points_sub(nh.subscribe<sensor_msgs::PointCloud2>("/points", 30, &SparseDynamicCalibrationNode::points_callback, this)),
          sync(SyncPolicy(30), image_sub, camera_info_sub, vodom_sub),
          camera_pose_pub(private_nh.advertise<sparse_dynamic_calibration::PoseSim3Stamped>("camera_pose", 5)),
          vodom_points_pub(private_nh.advertise<sensor_msgs::PointCloud2>("points", 5)),
        marker_pub(new sparse_dynamic_calibration::GraphMarkerPublisher(private_nh))
    {
        transform_publisher.reset(new TransformPublisher("tag_0"));

        std::string data_dir = ros::package::getPath("sparse_dynamic_calibration") + "/data";
        estimator.reset(new sparse_dynamic_calibration::Sim3Estimator());
        estimator->initialize_graph(private_nh.param<std::string>("g2o_optimizer_name", "lm_var_cholmod"), private_nh.param<int>("g2o_max_iterations", 32));
        estimator->initialize_tag_camera_network(data_dir, data_dir + "/tags.yaml", data_dir + "/cameras.yaml");
        if(estimator->tag_camera_network->read_poses(nh, estimator->graph.get(), data_dir + "/tag_camera_poses_refined.yaml")) {
            ROS_WARN_STREAM("calibrated poses loaded!!");
        }

        points_pubs.resize(estimator->tag_camera_network->cameras.size());
        for(size_t i=0; i<estimator->tag_camera_network->cameras.size(); i++) {
            const auto& camera = estimator->tag_camera_network->cameras[i];
            points_pubs[i] = nh.advertise<sensor_msgs::PointCloud2>("/" + camera->camera_name + "/points", 1, true);
            points_pubs[i].publish(camera->cloud);
        }

        points_pub_timer = nh.createTimer(ros::Duration(3.0), &SparseDynamicCalibrationNode::points_pub_callback, this);
        sync.registerCallback(boost::bind(&SparseDynamicCalibrationNode::callback, this, _1, _2, _3));
    }

    void callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& camera_info_msg, const geometry_msgs::PoseStampedConstPtr& vodom_msg) {
        auto cv_image = cv_bridge::toCvCopy(image_msg, "bgr8");

        cv::Mat1d camera_matrix(3, 3, const_cast<double*>(camera_info_msg->K.data()));
        cv::Mat1d distortion(5, 1, const_cast<double*>(camera_info_msg->D.data()));

        cv::Mat undistorted = cv_image->image.clone();
        cv::undistort(cv_image->image, undistorted, camera_matrix, distortion);

        Eigen::Isometry3d vodom = Eigen::Isometry3d::Identity();
        vodom.translation() = Eigen::Vector3d(vodom_msg->pose.position.x, vodom_msg->pose.position.y, vodom_msg->pose.position.z);
        vodom.linear() = Eigen::Quaterniond(vodom_msg->pose.orientation.w, vodom_msg->pose.orientation.x, vodom_msg->pose.orientation.y, vodom_msg->pose.orientation.z).toRotationMatrix();
        vodom = vodom.inverse();

        std::lock_guard<std::mutex> lock(estimator_mutex);
        if(image_msg->header.stamp - last_update_time > ros::Duration(0.5)) {
            last_update_time = image_msg->header.stamp;
            estimator->update(private_nh, image_msg->header.stamp, camera_matrix, undistorted, vodom);
        } else {
            publish_camera_pose(vodom);
            return;
        }

        for(const auto& tag: estimator->tag_camera_network->tagmap) {
            if(!tag.second->vertex) {
                continue;
            }
            std::stringstream tag_name;
            tag_name << "tag_" << tag.second->id;
            // std::cout << tag_name.str() << std::endl;
            transform_publisher->update_pose(tag_name.str(), tag.second->vertex->estimate());
        }
        for(const auto& camera: estimator->tag_camera_network->cameras) {
            if(!camera->vertex) {
                continue;
            }
            // std::cout << camera->camera_name << std::endl;
            transform_publisher->update_pose(camera->camera_name, camera->vertex->estimate());
        }
        for(size_t i=0; i<estimator->keyframes.size(); i++) {
            continue;
            std::stringstream sst;
            sst << "keyframe_" << i;

            g2o::Sim3 sim3 = estimator->keyframes[i]->vertex->estimate();
            transform_publisher->update_pose(sst.str(), sim3);
            // std::cout << "keyframe[" << i << "]:" << sim3.scale() << std::endl;
            // std::cout << "keyframe[" << i << "]:" << sim3.translation().transpose() << std::endl;
        }

        if(camera_pose_pub.getNumSubscribers()) {
            const auto& last_keyframe = estimator->keyframes.back();
            sparse_dynamic_calibration::PoseSim3Stamped pose;
            pose.header.frame_id = "vodom";
            pose.header.stamp = last_keyframe->stamp;

            g2o::Sim3 sim3 = last_keyframe->vertex->estimate();
            g2o::SE3Quat se3 = g2o::SE3Quat(sim3.rotation(), sim3.translation()).inverse();
            pose.pose.position.x = sim3.translation().x();
            pose.pose.position.y = sim3.translation().y();
            pose.pose.position.z = sim3.translation().z();
            pose.pose.orientation.w = sim3.rotation().w();
            pose.pose.orientation.x = sim3.rotation().x();
            pose.pose.orientation.y = sim3.rotation().y();
            pose.pose.orientation.z = sim3.rotation().z();
            pose.pose.scale = sim3.scale();

            camera_pose_pub.publish(pose);
        }

        publish_camera_pose(vodom);

        transform_publisher->publish();
        marker_pub->publish(image_msg->header.stamp, estimator->keyframes, estimator->tag_camera_network->tagmap, estimator->tag_camera_network->cameras);
    }

    void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
        return;
        ROS_INFO("points received!!");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::fromROSMsg(*points_msg, *cloud);

        std::lock_guard<std::mutex> lock(estimator_mutex);
        auto accumulated = estimator->update(points_msg->header.stamp, cloud);
        accumulated->header = cloud->header;
        accumulated->header.frame_id = "world";

        sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*accumulated, *cloud_msg);

        vodom_points_pub.publish(cloud_msg);
    }

    void points_pub_callback(const ros::TimerEvent& e) {
        for(size_t i=0; i<estimator->tag_camera_network->cameras.size(); i++) {
            const auto& camera = estimator->tag_camera_network->cameras[i];
            points_pubs[i].publish(camera->cloud);
        }
    }

    void save_callback(const std_msgs::EmptyConstPtr& msg) {
        std::string package_path = ros::package::getPath("sparse_dynamic_calibration");

        std::lock_guard<std::mutex> lock(estimator_mutex);
        estimator->write(package_path + "/data");

        transform_publisher->publish();
        visualization_msgs::MarkerArrayPtr markers_msg(new visualization_msgs::MarkerArray());
        marker_pub->publish(ros::Time::now(), estimator->keyframes, estimator->tag_camera_network->tagmap, estimator->tag_camera_network->cameras, markers_msg);

        uint32_t serial_size = ros::serialization::serializationLength(*markers_msg);
        boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

        ros::serialization::OStream stream(buffer.get(), serial_size);
        ros::serialization::serialize(stream, *markers_msg);

        std::ofstream ofs(package_path + "/data/markers.msg", std::ios::binary);
        ofs.write(reinterpret_cast<char*>(buffer.get()), serial_size);
        ofs.close();
    }

    void publish_camera_pose(const Eigen::Isometry3d& vodom) {
        auto last_keyframe = estimator->keyframes.back();
        Eigen::Isometry3d relative = vodom * last_keyframe->vodom.inverse();

        g2o::Sim3 relative_sim3(relative.linear(), relative.translation(), 1.0);
        g2o::Sim3 prev_sim3 = last_keyframe->vertex->estimate();
        g2o::Sim3 current_sim3 = relative_sim3 * prev_sim3;

        transform_publisher->update_pose("camera", current_sim3);
    }

private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> vodom_sub;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, geometry_msgs::PoseStamped>;
    message_filters::Synchronizer<SyncPolicy> sync;

    ros::Subscriber save_sub;
    ros::Subscriber points_sub;

    ros::Publisher camera_pose_pub;
    ros::Publisher vodom_points_pub;
    std::vector<ros::Publisher> points_pubs;
    std::unique_ptr<sparse_dynamic_calibration::GraphMarkerPublisher> marker_pub;

    ros::Time last_update_time;
    std::unique_ptr<TransformPublisher> transform_publisher;

    ros::Timer points_pub_timer;

    std::mutex estimator_mutex;
    std::unique_ptr<sparse_dynamic_calibration::Sim3Estimator> estimator;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sparse_dynamic_calibration_node");

    std::cout << "create node" << std::endl;
    std::unique_ptr<SparseDynamicCalibrationNode> node(new SparseDynamicCalibrationNode());

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
