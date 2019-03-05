cmake_minimum_required(VERSION 2.8.3)
project(sparse_dynamic_calibration)

add_compile_options(-std=c++11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(G2O REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    tf
    rospy
    roscpp
    pcl_ros
    cv_bridge
    tf_conversions
    image_transport
    )

########################
## message generation ##
########################
add_message_files(FILES
    PoseSim3.msg
    PoseSim3Stamped.msg
    )
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    #  INCLUDE_DIRS include
    #  CATKIN_DEPENDS other_catkin_pkg
    #  DEPENDS system_lib
    )

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )
link_directories(
    /usr/local/lib
    ${G2O_LIBARY_DIRS}
    )

add_library(sparse_dynamic_calibration_lib SHARED
    src/g2o/g2o_optimizers.cpp
    src/g2o/robust_kernel_io.cpp
    src/g2o/edge_icp.cpp
    src/g2o/edge_se3_plane.cpp
    src/g2o/edge_sim3_se3_expmap.cpp
    src/sparse_dynamic_calibration/keyframe.cpp
    src/sparse_dynamic_calibration/tag_detector.cpp
    src/sparse_dynamic_calibration/sim3_estimator.cpp
    src/sparse_dynamic_calibration/tag_camera_network.cpp
    src/sparse_dynamic_calibration/graph_marker_publisher.cpp
    src/sparse_dynamic_calibration/icp_constraints.cpp
    src/sparse_dynamic_calibration/static_camera.cpp
    src/sparse_dynamic_calibration/static_camera_refinement.cpp
    src/sparse_dynamic_calibration/graph_refinement.cpp
    )
target_link_libraries(sparse_dynamic_calibration_lib
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${G2O_TYPES_DATA}
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${G2O_SOLVER_PCG}
    ${G2O_SOLVER_CSPARSE}
    ${G2O_SOLVER_CHOLMOD}
    ${G2O_TYPES_SBA}
    ${G2O_TYPES_SIM3}
    ${G2O_TYPES_ICP}
    ${G2O_TYPES_SLAM3D_ADDONS}
    apriltag
    )
add_dependencies(sparse_dynamic_calibration_lib sparse_dynamic_calibration_gencpp)

add_executable(generate_tag_camera_network_conf src/generate_tag_camera_network_conf.cpp)
add_dependencies(generate_tag_camera_network_conf sparse_dynamic_calibration_lib)
target_link_libraries(generate_tag_camera_network_conf sparse_dynamic_calibration_lib)

add_executable(sparse_dynamic_calibration_node src/sparse_dynamic_calibration_node.cpp)
add_dependencies(sparse_dynamic_calibration_node sparse_dynamic_calibration_lib)
target_link_libraries(sparse_dynamic_calibration_node sparse_dynamic_calibration_lib)

add_executable(sparse_dynamic_refinement_node src/sparse_dynamic_refinement_node.cpp)
add_dependencies(sparse_dynamic_refinement_node sparse_dynamic_calibration_lib)
target_link_libraries(sparse_dynamic_refinement_node sparse_dynamic_calibration_lib)
