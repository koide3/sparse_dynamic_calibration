#ifndef TRANSFORM_PUBLISHER_HPP
#define TRANSFORM_PUBLISHER_HPP

#include <Eigen/Dense>
#include <g2o/types/sim3/sim3.h>
#include <g2o/types/sba/sbacam.h>
#include <tf/transform_broadcaster.h>

namespace sparse_dynamic_calibration {

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

 }

#endif // TRANSFORM_PUBLISHER_HPP
