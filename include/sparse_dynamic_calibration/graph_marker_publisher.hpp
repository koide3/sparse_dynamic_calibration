#ifndef GRAPH_MARKER_PUBLISHER_HPP
#define GRAPH_MARKER_PUBLISHER_HPP

#include <unordered_map>
#include <g2o/core/sparse_optimizer.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <sparse_dynamic_calibration/keyframe.hpp>
#include <sparse_dynamic_calibration/static_tag.hpp>
#include <sparse_dynamic_calibration/static_camera.hpp>

namespace sparse_dynamic_calibration {


class GraphMarkerPublisher {
public:
    GraphMarkerPublisher(ros::NodeHandle& nh);

    void publish(const ros::Time& stamp, const std::vector<std::shared_ptr<Keyframe>>& keyframes, const std::unordered_map<long, StaticTag::Ptr>& tags, const std::vector<StaticCamera::Ptr>& cameras, visualization_msgs::MarkerArrayPtr markers=nullptr);
private:
    std_msgs::ColorRGBA create_color(float r, float g, float b, float a);
    visualization_msgs::Marker create_basic_marker(const ros::Time& stamp, const std::string& ns, int id, int marker_type, double scale);

    ros::Publisher markers_pub;
};

}

#endif // GRAPH_MARKER_GENERATOR_HPP
