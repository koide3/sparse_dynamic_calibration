#ifndef POSE_IO_HPP
#define POSE_IO_HPP

#include <vector>
#include <Eigen/Dense>
#include <g2o/types/sba/sbacam.h>
#include <g2o/types/sim3/sim3.h>

namespace sparse_dynamic_calibration {

inline std::vector<double> pose2vec(const Eigen::Isometry3d& pose) {
    Eigen::Quaterniond quat(pose.linear());
    Eigen::Vector3d trans = pose.translation();

    std::vector<double> vec(7);
    vec[0] = trans[0];
    vec[1] = trans[1];
    vec[2] = trans[2];
    vec[3] = quat.x();
    vec[4] = quat.y();
    vec[5] = quat.z();
    vec[6] = quat.w();

    return vec;
}

inline std::vector<double> pose2vec(const g2o::SE3Quat& pose) {
    Eigen::Quaterniond quat = pose.rotation();
    Eigen::Vector3d trans = pose.translation();

    std::vector<double> vec(7);
    vec[0] = trans[0];
    vec[1] = trans[1];
    vec[2] = trans[2];
    vec[3] = quat.x();
    vec[4] = quat.y();
    vec[5] = quat.z();
    vec[6] = quat.w();

    return vec;
}

inline std::vector<double> pose2vec(const g2o::Sim3& pose) {
    Eigen::Quaterniond quat = pose.rotation();
    Eigen::Vector3d trans = pose.scale() * pose.translation();

    std::vector<double> vec(7);
    vec[0] = trans[0];
    vec[1] = trans[1];
    vec[2] = trans[2];
    vec[3] = quat.x();
    vec[4] = quat.y();
    vec[5] = quat.z();
    vec[6] = quat.w();

    return vec;
}

inline g2o::SE3Quat vec2se3(const std::vector<double>& vec) {
    Eigen::Vector3d trans(vec[0], vec[1], vec[2]);
    Eigen::Quaterniond quat(vec[6], vec[3], vec[4], vec[5]);
    return g2o::SE3Quat(quat, trans);
}

}

#endif // POSE_UTILS_HPP
