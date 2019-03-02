#ifndef SIM3_ESTIMATION_TAG_DETECTOR_HPP
#define SIM3_ESTIMATION_TAG_DETECTOR_HPP

#include <regex>
#include <memory>
#include <unordered_map>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

extern "C" {
#include <apriltag/apriltag.h>
}

namespace sparse_dynamic_calibration {

class TagDetector {
public:
    TagDetector(const std::string& tag_settings);

    std::shared_ptr<zarray> detect(const cv::Mat& grayscale);

    Eigen::Isometry3d estimate_pose(const cv::Mat& camera_matrix, apriltag_detection* det) const;

    void draw(cv::Mat& canvas, const cv::Mat& camera_matrix, const std::shared_ptr<zarray>& dets) const;

private:
    double default_tag_size;
    std::unordered_map<long, double> tag_sizes;

    std::unique_ptr<apriltag_family, void(*)(apriltag_family*)> tag_family;
    std::unique_ptr<apriltag_detector, void(*)(apriltag_detector*)> tag_detector;
};

}

#endif
