#include <sparse_dynamic_calibration/graph_marker_publisher.hpp>



namespace sparse_dynamic_calibration {

GraphMarkerPublisher::GraphMarkerPublisher(ros::NodeHandle& nh)
    : markers_pub(nh.advertise<visualization_msgs::MarkerArray>("markers", 1, false))
{}


void GraphMarkerPublisher::publish(const ros::Time& stamp, const std::vector<std::shared_ptr<Keyframe>>& keyframes, const std::unordered_map<long, StaticTag::Ptr>& tags, const std::vector<StaticCamera::Ptr>& cameras, visualization_msgs::MarkerArrayPtr markers) {
    if(!markers && !markers_pub.getNumSubscribers()) {
        return;
    }

    if(!markers) {
        markers.reset(new visualization_msgs::MarkerArray());
    }
    markers->markers.resize(6);

    auto tag_color = create_color(0.0, 1.0, 0.0, 1.0);
    auto camera_color = create_color(0.2, 0.2, 1.0, 1.0);
    auto vodom_color = create_color(1.0, 0.0, 0.0, 1.0);

    visualization_msgs::Marker& tag_vertices = markers->markers[0];
    visualization_msgs::Marker& camera_vertices = markers->markers[1];
    visualization_msgs::Marker& camera_tag_edges = markers->markers[2];
    visualization_msgs::Marker& vodom_vertices = markers->markers[3];
    visualization_msgs::Marker& vodom_edges = markers->markers[4];
    visualization_msgs::Marker& vodom_tag_edges = markers->markers[5];

    // tag
    std::unordered_map<long, geometry_msgs::Point> tagmap;
    tag_vertices = create_basic_marker(stamp, "tags", 0, visualization_msgs::Marker::CUBE_LIST, 0.1);
    tag_vertices.color = tag_color;
    for(const auto& tag: tags) {
        if(!tag.second->vertex) {
            continue;
        }

        Eigen::Vector3d pos = tag.second->vertex->estimate().inverse().translation();
        geometry_msgs::Point tagpos;
        tagpos.x = pos.x();
        tagpos.y = pos.y();
        tagpos.z = pos.z();

        tag_vertices.points.push_back(tagpos);
        tagmap[tag.first] = tagpos;
    }

    // cameras
    camera_vertices = create_basic_marker(stamp, "cameras", 0, visualization_msgs::Marker::CUBE_LIST, 0.1);
    camera_vertices.color = camera_color;
    camera_tag_edges = create_basic_marker(stamp, "cameras_tags", 1, visualization_msgs::Marker::LINE_LIST, 0.025);
    camera_vertices.points.resize(cameras.size());
    camera_tag_edges.points.reserve(cameras.size() * 4);
    for(size_t i=0; i<cameras.size(); i++) {
        const auto& camera = cameras[i];
        if(!camera->vertex) {
            continue;
        }

        Eigen::Vector3d pos = camera->vertex->estimate().inverse().translation();
        geometry_msgs::Point camera_pos;
        camera_pos.x = pos.x();
        camera_pos.y = pos.y();
        camera_pos.z = pos.z();

        camera_vertices.points[i] = camera_pos;

        for(const auto& tag_id: camera->detected_tag_ids) {
            camera_tag_edges.points.push_back(camera_pos);
            camera_tag_edges.points.push_back(tagmap[tag_id]);

            camera_tag_edges.colors.push_back(camera_color);
            camera_tag_edges.colors.push_back(tag_color);
        }
    }


    // vodom
    vodom_vertices = create_basic_marker(stamp, "vodom", 0, visualization_msgs::Marker::SPHERE_LIST, 0.1);
    vodom_vertices.color = vodom_color;
    vodom_edges = create_basic_marker(stamp, "vodom", 1, visualization_msgs::Marker::LINE_STRIP, 0.025);
    vodom_edges.color = vodom_color;
    vodom_tag_edges = create_basic_marker(stamp, "vodom_tag", 2, visualization_msgs::Marker::LINE_LIST, 0.01);

    vodom_vertices.points.resize(keyframes.size());
    vodom_edges.points.resize(keyframes.size());
    vodom_tag_edges.points.reserve(keyframes.size() * 2);
    for(size_t i=0; i<keyframes.size(); i++) {
        g2o::Sim3 sim3 = keyframes[i]->vertex->estimate();
        g2o::SE3Quat se3(sim3.rotation(), sim3.translation() * sim3.scale());
        Eigen::Vector3d pos = se3.inverse().translation();
        geometry_msgs::Point vodom_pos;
        vodom_pos.x = pos.x();
        vodom_pos.y = pos.y();
        vodom_pos.z = pos.z();

        vodom_vertices.points[i] = vodom_edges.points[i] = vodom_pos;

        const auto& keyframe = keyframes[i];
        for(long tag_id: keyframe->detected_tag_ids) {
            vodom_tag_edges.points.push_back(vodom_pos);
            vodom_tag_edges.points.push_back(tagmap[tag_id]);

            vodom_tag_edges.colors.push_back(vodom_color);
            vodom_tag_edges.colors.push_back(tag_color);
        }
    }

    markers_pub.publish(markers);
}

std_msgs::ColorRGBA GraphMarkerPublisher::create_color(float r, float g, float b, float a) {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    return color;
}

visualization_msgs::Marker GraphMarkerPublisher::create_basic_marker(const ros::Time& stamp, const std::string& ns, int id, int marker_type, double scale) {
    visualization_msgs::Marker marker;

    marker.header.stamp = stamp;
    marker.header.frame_id = "vodom";
    marker.ns = ns;
    marker.id = id;
    marker.type = marker_type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = scale;

    return marker;
}

}
