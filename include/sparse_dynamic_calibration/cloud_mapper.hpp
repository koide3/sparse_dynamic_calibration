#ifndef CLOUD_MAPPER_HPP
#define CLOUD_MAPPER_HPP

#include <vector>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <g2o/types/sba/sbacam.h>

#include <sparse_dynamic_calibration/keyframe.hpp>
#include <sparse_dynamic_calibration/ros_time_hash.hpp>


namespace sparse_dynamic_calibration {


class CloudMapper {
public:
    void update(const ros::Time& stamp, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        clouds.push_back(StampCloud(stamp, cloud));

        std::sort(clouds.begin(), clouds.end(), [=](const StampCloud& lhs, const StampCloud& rhs) { return lhs.first < rhs.first; });
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr generate(std::vector<std::shared_ptr<Keyframe>> keyframes) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated(new pcl::PointCloud<pcl::PointXYZ>());

        auto key_search_begin = keyframes.begin();
        for(auto cloud_itr = clouds.begin(); cloud_itr != clouds.end(); cloud_itr++) {
            auto key2 = std::lower_bound(key_search_begin, keyframes.end(), cloud_itr->first, [=](const std::shared_ptr<Keyframe>& key, const ros::Time& stamp) { return key->stamp < stamp; });
            if(key2 == keyframes.begin() || key2 == keyframes.end() - 1 || key2 == keyframes.end()) {
                continue;
            }
            auto key1 = key2 - 1;
            double t = (cloud_itr->first - (*key1)->stamp).toSec() / ((*key2)->stamp - (*key1)->stamp).toSec();

            auto se3_1 = g2o::SE3Quat((*key1)->vodom.linear(), (*key1)->vodom.translation());
            auto se3_2 = g2o::SE3Quat((*key2)->vodom.linear(), (*key2)->vodom.translation());

            auto sim3_1 = (*key1)->vertex->estimate();
            auto sim3_2 = (*key2)->vertex->estimate();

            g2o::SE3Quat interpolated_se3 = g2o::SE3Quat(t * (se3_2 * se3_1.inverse()).log()) * se3_1;
            g2o::Sim3 interpolated_sim3 = g2o::Sim3(t * (sim3_2 * sim3_1.inverse()).log()) * sim3_1;

            Eigen::Matrix4d mat_se3 = Eigen::Matrix4d::Identity();
            mat_se3.block<3, 3>(0, 0) = interpolated_se3.rotation().toRotationMatrix();
            mat_se3.block<3, 1>(0, 3) = interpolated_se3.translation();

            Eigen::Matrix4d mat_sim3 = Eigen::Matrix4d::Identity();
            mat_sim3.block<3, 3>(0, 0) = interpolated_sim3.scale() * interpolated_sim3.rotation().toRotationMatrix();
            mat_sim3.block<3, 1>(0, 3) = interpolated_sim3.translation();

            Eigen::Matrix4f mat = (mat_sim3.inverse() * mat_se3).cast<float>();
            mat = mat_se3.cast<float>();
            for(const auto& pt: cloud_itr->second->points) {
                pcl::PointXYZ pt_;
                pt_.getVector4fMap() = mat * pt.getVector4fMap();
                accumulated->push_back(pt_);
            }
        }

        accumulated->width = accumulated->size();
        accumulated->height = 1;
        accumulated->is_dense = true;

        return accumulated;
    }

private:
    using StampCloud = std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr>;
    std::vector<StampCloud> clouds;

};

}

#endif // CLOUD_MAPPER_HPP
