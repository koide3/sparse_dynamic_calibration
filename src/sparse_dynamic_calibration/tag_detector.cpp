#include <sparse_dynamic_calibration/tag_detector.hpp>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/apriltag_pose.h>
}

#include <boost/format.hpp>

namespace sparse_dynamic_calibration {

TagDetector::TagDetector(const std::string& tag_settings)
    : tag_family(tag36h11_create(), &tag36h11_destroy),
      tag_detector(apriltag_detector_create(), &apriltag_detector_destroy)
{
    cv::FileStorage tag_fs(tag_settings, cv::FileStorage::READ);
    if(!tag_fs.isOpened()) {
        std::cerr << "failed to open the tag setting file!!" << std::endl;
        return;
    }

    default_tag_size = 0.0;
    for(cv::FileNodeIterator tag_itr=tag_fs.root().begin(); tag_itr!=tag_fs.root().end(); tag_itr++) {
        if((*tag_itr).name() == "default_tag_size") {
            default_tag_size = (*tag_itr).real();
            continue;
        }

        std::string tag_name = (*tag_itr).name();
        std::regex pattern("tag_([0-9]+)");
        std::smatch matched;
        std::regex_search(tag_name, matched, pattern);

        long tag_id = std::stol(matched.str(1));
        double tag_size = (*tag_itr)["size"].real();
        tag_sizes[tag_id] = tag_size;
    }

    tag_detector->quad_decimate = 1.0;
    tag_detector->quad_sigma = 0.0;
    tag_detector->refine_edges = 1;
    tag_detector->decode_sharpening = 0.25;
    apriltag_detector_add_family(tag_detector.get(), tag_family.get());
}

std::shared_ptr<zarray> TagDetector::detect(const cv::Mat& grayscale) {
    image_u8 im = {.width=grayscale.cols, .height=grayscale.rows, .stride=grayscale.cols, .buf=grayscale.data};
    std::shared_ptr<zarray> detections(apriltag_detector_detect(tag_detector.get(), &im), &zarray_destroy);

    return detections;
}

Eigen::Isometry3d TagDetector::estimate_pose(const cv::Mat& camera_matrix, apriltag_detection* det) const {
    double half_scale = default_tag_size / 2;
    auto found = tag_sizes.find(det->id);
    if(found != tag_sizes.end()) {
        half_scale = found->second / 2;
    }

    apriltag_detection_info_t info;
    info.det = det;
    info.tagsize = half_scale * 2;
    info.fx = camera_matrix.at<double>(0, 0);
    info.fy = camera_matrix.at<double>(1, 1);
    info.cx = camera_matrix.at<double>(0, 2);
    info.cy = camera_matrix.at<double>(1, 2);

    apriltag_pose_t pose;
    double err = estimate_tag_pose(&info, &pose);

    Eigen::Isometry3d tagpose = Eigen::Isometry3d::Identity();
    tagpose.linear() = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(pose.R->data);
    tagpose.translation() = Eigen::Map<Eigen::Vector3d>(pose.t->data);

    return tagpose;
}

void TagDetector::draw(cv::Mat& canvas, const cv::Mat& camera_matrix, const std::shared_ptr<zarray>& dets) const {
    for(int i=0; i<zarray_size(dets.get()); i++) {
        apriltag_detection* det;
        zarray_get(dets.get(), i, &det);

        Eigen::Isometry3d pose = estimate_pose(camera_matrix, det);
        if(det->id != -1) {
            Eigen::Vector3d center = pose.translation();
            Eigen::Vector3d x_axis = center + pose.linear().col(0);
            Eigen::Vector3d y_axis = center + pose.linear().col(1);
            Eigen::Vector3d z_axis = center + pose.linear().col(2);

            std::vector<cv::Point3f> axes_3d = {
                cv::Point3f(center[0], center[1], center[2]),
                cv::Point3f(x_axis[0], x_axis[1], x_axis[2]),
                cv::Point3f(y_axis[0], y_axis[1], y_axis[2]),
                cv::Point3f(z_axis[0], z_axis[1], z_axis[2])
            };
            std::vector<cv::Point2f> axes_2d;
            cv::Mat ivec(3, 1, CV_32FC1, cv::Scalar::all(0));
            cv::projectPoints(axes_3d, ivec, ivec, camera_matrix, cv::Mat(), axes_2d);

            cv::line(canvas, cv::Point(axes_2d[0].x, axes_2d[0].y), cv::Point(axes_2d[1].x, axes_2d[1].y), cv::Scalar(0, 0, 255), 2);
            cv::line(canvas, cv::Point(axes_2d[0].x, axes_2d[0].y), cv::Point(axes_2d[2].x, axes_2d[2].y), cv::Scalar(0, 255, 0), 2);
            cv::line(canvas, cv::Point(axes_2d[0].x, axes_2d[0].y), cv::Point(axes_2d[3].x, axes_2d[3].y), cv::Scalar(255, 0, 0), 2);
        }

        std::vector<cv::Point> points = {
            cv::Point(det->p[0][0], det->p[0][1]),
            cv::Point(det->p[1][0], det->p[1][1]),
            cv::Point(det->p[2][0], det->p[2][1]),
            cv::Point(det->p[3][0], det->p[3][1])
        };

        cv::Scalar color = det->id == -1 ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        cv::polylines(canvas, points, true, color, 2);

        std::stringstream sst;
        sst << det->id;

        int baseline = 0;
        cv::Size textsize = cv::getTextSize(sst.str(), CV_FONT_HERSHEY_SCRIPT_SIMPLEX, 2.0, 2, &baseline);
        cv::putText(canvas, sst.str(), cv::Point(det->c[0] - textsize.width/2, det->c[1] + textsize.height/2), CV_FONT_HERSHEY_SCRIPT_SIMPLEX, 2.0, cv::Scalar(0xff, 0x99, 0), 2);

        sst.str("");
        sst.clear();
        sst << "margin:" << boost::format("%.1f") % det->decision_margin << " dist:" << boost::format("%.1f") % pose.translation().norm();
        cv::Size textsize2 = cv::getTextSize(sst.str(), CV_FONT_HERSHEY_PLAIN, 1.0, 2, &baseline);
        cv::putText(canvas, sst.str(), cv::Point(det->c[0] - textsize.width/2, det->c[1] + textsize.height/2 + textsize2.height + 5), CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0xff, 0x99, 0), 2);
    }
}

}
