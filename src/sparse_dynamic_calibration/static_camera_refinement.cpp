#include <sparse_dynamic_calibration/static_camera_refinement.hpp>

#include <g2o/types/slam3d_addons/plane3d.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

namespace sparse_dynamic_calibration {


StaticCameraRefinement::StaticCameraRefinement(const std::string& data_dir, const std::string& camera_name)
    : StaticCamera(data_dir, camera_name)
{
    if(cloud) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        voxelgrid.setLeafSize(0.05f, 0.05f, 0.05f);
        voxelgrid.setInputCloud(cloud);
        voxelgrid.filter(*downsampled);

        pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_moved(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(downsampled);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*outlier_moved);

        filtered.reset(new pcl::PointCloud<pcl::PointNormal>());
        filtered->resize(outlier_moved->size());
        for(size_t i=0; i<outlier_moved->size(); i++) {
            filtered->at(i).getVector4fMap() = outlier_moved->at(i).getVector4fMap();
        }

        pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> ne;
        ne.setInputCloud(filtered);

        pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>());
        ne.setSearchMethod(tree);

        ne.setRadiusSearch (0.2f);
        ne.compute(*filtered);

        pcl::io::savePCDFile(data_dir + "/" + camera_name + "_filtered.pcd", *filtered);
    }
}


void StaticCameraRefinement::update_transformed() {
    if(!transformed) {
        transformed.reset(new pcl::PointCloud<pcl::PointNormal>());
    }

    g2o::SE3Quat se3 = vertex->estimate().inverse();

    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat.block<3, 3>(0, 0) = se3.rotation().toRotationMatrix();
    mat.block<3, 1>(0, 3) = se3.translation();

    pcl::transformPointCloudWithNormals(*filtered, *transformed, mat.cast<float>());
    kdtree.setInputCloud(transformed);

    Eigen::Vector4f min, max;
    pcl::getMinMax3D(*transformed, min, max);
    min_pt = min;
    max_pt = max;
}


bool StaticCameraRefinement::hit_test(const StaticCameraRefinement& rhs) const {
    return ((min_pt < rhs.max_pt) && (max_pt > rhs.min_pt)).head<3>().all();
}

void StaticCameraRefinement::detect_floor_plane() {
    std::cout << ">> detect_floor_plane" << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr clipped(new pcl::PointCloud<pcl::PointNormal>());
    std::copy_if(transformed->begin(), transformed->end(), std::back_inserter(clipped->points),
        [=](const pcl::PointNormal& pt) {
            float dot = pt.getNormalVector3fMap().dot(Eigen::Vector3f::UnitY());
            return std::abs(dot) > std::cos(30.0 * M_PI / 180.0);
        }
    );
    clipped->width = clipped->size();
    clipped->height = 1;
    clipped->is_dense = false;

    if(clipped->size() < 512) {
        ROS_WARN_STREAM("too few points for plane detection: " << clipped->size());
        return;
    }

    // RANSAC
    pcl::SampleConsensusModelPlane<pcl::PointNormal>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointNormal>(clipped));
    pcl::RandomSampleConsensus<pcl::PointNormal> ransac(model_p);
    ransac.setDistanceThreshold(0.1);
    ransac.computeModel();

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    ransac.getInliers(inliers->indices);

    // too few inliers
    if(inliers->indices.size() < 128) {
        ROS_WARN_STREAM("too few inliers for plane detection: " << inliers->indices.size());
        return;
    }

    // verticality check of the detected floor's normal
    Eigen::Vector4f reference = Eigen::Vector4f::UnitY();

    Eigen::VectorXf coeffs;
    ransac.getModelCoefficients(coeffs);

    double dot = coeffs.head<3>().dot(reference.head<3>());
    if(std::abs(dot) < std::cos(15.0 * M_PI / 180.0)) {
        // the normal is not vertical
        ROS_WARN_STREAM("detected normal is not vertical!!");
        return;
    }

    // make the normal upward
    if(coeffs.head<3>().dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
        coeffs *= -1.0f;
    }


    g2o::SE3Quat se3 = vertex->estimate().inverse();

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = se3.rotation().toRotationMatrix();
    pose.translation() = se3.translation();

    g2o::Plane3D local_plane = pose.inverse() * g2o::Plane3D(coeffs.cast<double>());

    floor_plane = local_plane.toVector().cast<float>();
    ROS_INFO_STREAM("floor_detected: " << floor_plane->transpose());
}


}
