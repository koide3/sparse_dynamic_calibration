#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <sparse_dynamic_calibration/graph_refinement.hpp>


int main(int argc, char** argv) {
    ros::init(argc, argv, "sparse_dynamic_refinement");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string package_path = ros::package::getPath("sparse_dynamic_calibration");

    std::unique_ptr<sparse_dynamic_calibration::GraphRefinement> refinement(new sparse_dynamic_calibration::GraphRefinement());
    refinement->refine(private_nh, package_path + "/data");

    return 0;
}
