# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

# ======================================
# CMakeLists file to demonstrate how to use GPU Voxels as a collision checker in OMPL:
# ======================================

cmake_minimum_required (VERSION 2.8)
SET(CMAKE_CXX_STANDARD 11)

project (gvl_ompl_planning)

# First we have to find our dependencies:
FIND_PACKAGE(icl_core REQUIRED )
FIND_PACKAGE(gpu_voxels REQUIRED)
FIND_PACKAGE(Boost COMPONENTS system filesystem date_time thread REQUIRED)
FIND_PACKAGE(CUDA REQUIRED)
FIND_PACKAGE(ompl REQUIRED)
FIND_PACKAGE(PCL)
FIND_PACKAGE(ROS)
FIND_PACKAGE(pcl_ros)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

IF(ROS_FOUND)
  MESSAGE(STATUS "[OK]      Building GPU-Voxels with ROS connections. ROS was found.")
ELSE(ROS_FOUND)
  MESSAGE(STATUS "[WARNING] Building GPU-Voxels without ROS connections. ROS not found.")
ENDIF(ROS_FOUND)
IF(ROS_FOUND)
    MESSAGE(STATUS "!!! to fix this, source /opt/ros/YOUR_ROS_DISTRO/setup.bash before building !!!")
  ENDIF(ROS_FOUND)
IF(PCL_FOUND)
  # needed because of problems with libvtk6 and PCL on Jetson
  list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
  MESSAGE(STATUS "[OK]      Building GPU-Voxels with PCL interfaces. PCL found.")
ELSE(PCL_FOUND)
  MESSAGE(STATUS "[WARNING] Building GPU-Voxels without PCL interfaces. PCL not found.")
ENDIF(PCL_FOUND)


# This is a quirk and should be removed in upcoming versions
# If you built GPU Voxels without ROS support, remove this.
FIND_PACKAGE(orocos_kdl REQUIRED)

# A little debug info:
MESSAGE(STATUS "GVL include dirs are: ${gpu_voxels_INCLUDE_DIRS}")
MESSAGE(STATUS "OMPL include dirs are: ${OMPL_INCLUDE_DIRS}")

# Also we have to inherit some Environment definitions required for our base libs:
add_definitions(
  ${icl_core_DEFINITIONS}
  ${gpu_voxels_DEFINITIONS}
)


# Create a library that uses GPU Voxels:
add_library (gvl_ompl_planner_helper gvl_ompl_planner_helper.cpp 
)

target_link_libraries (gvl_ompl_planner_helper 
    LINK_PUBLIC gvl_ompl_planner_helper
    LINK_PUBLIC ${Boost_SYSTEM_LIBRARY}
    LINK_PUBLIC ${icl_core_LIBRARIES}
    LINK_PUBLIC ${gpu_voxels_LIBRARIES}
    LINK_PUBLIC ${OMPL_LIBRARIES}
    LINK_PUBLIC ${CUDA_LIBRARIES}
    LINK_PUBLIC ${PCL_LIBRARIES}
    LINK_PUBLIC ${ROS_LIBRARIES}
    LINK_PUBLIC ${catkin_LIBRARIES}
    LINK_PUBLIC ${orocos_kdl_LIBRARIES} 
    LINK_PUBLIC ${moveit_LIBRARIES} 
    LINK_PUBLIC ${PYTHON_LIBRARIES}

)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  visualization_msgs
  message_generation
  kdl_parser
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

target_include_directories (gvl_ompl_planner_helper
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${gpu_voxels_INCLUDE_DIRS}
    PUBLIC ${icl_core_INCLUDE_DIRS}
    PUBLIC ${CUDA_INCLUDE_DIRS}
    PUBLIC ${OMPL_INCLUDE_DIRS}
    PUBLIC ${libfranka_INCLUDE_DIRS} # this should be removed in upcoming versions.
    PUBLIC ${orocos_kdl_INCLUDE_DIRS} # this should be removed in upcoming versions.
    PUBLIC ${moveit_INCLUDE_DIRS} # this should be removed in upcoming versions.
    PUBLIC  ${catkin_INCLUDE_DIRS}
    PUBLIC ${Eigen_INCLUDE_DIRS}

)

# Add an executable that calls the lib:
add_executable (gvl_ompl_planner gvl_ompl_planner.cpp)
add_definitions(-DIC_PERFORMANCE_MONITOR)
# Link the executable to the library.
# We currently also have to link against Boost and icl_core...
target_link_libraries (gvl_ompl_planner 
    LINK_PUBLIC gvl_ompl_planner_helper
    LINK_PUBLIC ${Boost_SYSTEM_LIBRARY}
    LINK_PUBLIC ${icl_core_LIBRARIES}
    LINK_PUBLIC ${gpu_voxels_LIBRARIES}
    LINK_PUBLIC ${OMPL_LIBRARIES}
    LINK_PUBLIC ${CUDA_LIBRARIES}
    LINK_PUBLIC ${PCL_LIBRARIES}
    LINK_PUBLIC ${ROS_LIBRARIES}
    LINK_PUBLIC ${catkin_LIBRARIES}
    LINK_PUBLIC ${orocos_kdl_LIBRARIES} 
    LINK_PUBLIC ${Boost_LIBRARIES}


)
target_include_directories(gvl_ompl_planner
     PUBLIC ${orocos_kdl_INCLUDE_DIRS} # this should be removed in upcoming versions.
)
