#ifndef STATIC_CAMERA_REFINEMENT_HPP
#define STATIC_CAMERA_REFINEMENT_HPP

#include <memory>
#include <Eigen/Dense>
#include <boost/optional.hpp>
#include <pcl/kdtree/kdtree_flann.h>

#include <sparse_dynamic_calibration/static_camera.hpp>

namespace sparse_dynamic_calibration {

class StaticCameraRefinement : public StaticCamera {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<StaticCameraRefinement>;

    StaticCameraRefinement(const std::string& data_dir, const std::string& camera_name);
    virtual ~StaticCameraRefinement() {}

    void update_transformed();

    bool hit_test(const StaticCameraRefinement& rhs) const;

public:
    void detect_floor_plane();

public:
    pcl::PointCloud<pcl::PointNormal>::Ptr filtered;
    pcl::PointCloud<pcl::PointNormal>::Ptr transformed;

    Eigen::Array4f min_pt;
    Eigen::Array4f max_pt;

    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
    boost::optional<Eigen::Vector4f> floor_plane;
};

}

#endif // STATIC_CAMERA_REFINEMENT_HPP
