// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the GPU Voxels Software Library.
//
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE.txt in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Andreas Hermann
 * \date    2018-01-07
 *
 */
//----------------------------------------------------------------------/*
#ifndef GVL_LINKAGE_TEST_LIB_H_INCLUDED
#define GVL_LINKAGE_TEST_LIB_H_INCLUDED
#define JOINTNUM 9
#include <gpu_voxels/GpuVoxels.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ompl/config.h>
#include <iostream>
#include <tuple>
#include <mutex>
#include <vector>



const std::string hostname = "127.0.0.1"; //localhost IP Address
//const std::string hostname = "192.168.0.39"; //STEP2 IP Address 
//const std::string hostname = "192.168.0.100"; //STEP2 IP Address Monkey
//const std::string hostname = "192.168.1.18"; //STEP2 IP Address Tensegrity
//const std::string hostname = "192.168.0.122"; //STEP2 IP Address Tensegrity

enum {
	SIZE_HEADER = 52,
	SIZE_COMMAND = 4,
	SIZE_HEADER_COMMAND = 56,
	SIZE_DATA_MAX = 37,
	SIZE_DATA_ASCII_MAX = 32
};
union Data

{
	unsigned char byte[1];
	float value[9];

};

 


std::vector<std::array<double,JOINTNUM>> joint_trajectory;


robot::JointValueMap myRobotJointValues;
robot::JointValueMap myRobotJointValues_mm;

gpu_voxels::GpuVoxelsSharedPtr gvl;
PointCloud my_point_cloud;
Matrix4f tf;
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <sensor_msgs/JointState.h>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>

KDL::Tree my_tree;
KDL::Chain my_chain;
KDL::JntArray q_min(JOINTNUM);
KDL::JntArray q_max(JOINTNUM);
namespace ob = ompl::base;
namespace og = ompl::geometric;
using boost::dynamic_pointer_cast;
using boost::shared_ptr;
using gpu_voxels::voxelmap::ProbVoxelMap;
using gpu_voxels::voxelmap::DistanceVoxelMap;
using gpu_voxels::voxellist::CountingVoxelList;
using gpu_voxels::voxellist::BitVectorVoxelList;

float voxel_side_length = 0.05f; // 1 cm voxel size
bool new_data_received;
bool new_pose_received;
bool isMoving = false;
boost::shared_ptr<ProbVoxelMap> myEnvironmentMap;
boost::shared_ptr<CountingVoxelList> countingVoxelList;

boost::shared_ptr<BitVectorVoxelList> myRobotCollisionMapBitVoxel;
void ctrlchandler(int)
{
  exit(EXIT_SUCCESS);
}

void killhandler(int)
{
  exit(EXIT_SUCCESS);
}

class GvlOmplPlannerHelper : public ompl::base::StateValidityChecker, public ompl::base::MotionValidator, public std::enable_shared_from_this<GvlOmplPlannerHelper>
{

public:
    GvlOmplPlannerHelper(const ompl::base::SpaceInformationPtr &si);
    ~GvlOmplPlannerHelper();

    virtual bool isValid(const ompl::base::State *state) const;
    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                             std::pair< ompl::base::State*, double > & lastValid) const;
    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const;

    void plan();
    void rosIter();
    void tcpIter();
    void rosPublishJointTrajectory(std::vector<std::array<double,7>>& q_list);
    void rosPublishJointStates(double *values);
    Eigen::Matrix4f loadBaseToCam(std::string filename);
    std::shared_ptr<GvlOmplPlannerHelper> getptr() {
        return shared_from_this();
    }
    std::vector<std::array<double,JOINTNUM>>  doTaskPlanning(double goal_values[7],double start_values[JOINTNUM], ob::PathPtr path);

    void insertStartAndGoal(const ompl::base::ScopedState<> &start, const ompl::base::ScopedState<> &goal) const;
    void visualizeSolution(ompl::base::PathPtr path);
    void visualizeSolution( std::vector<std::array<double,JOINTNUM>> solution);
    void doVis();
    void doVis2();
    void setTransformation(Eigen::Matrix<float, 4, 4> T);
    void isMove(int i);
    int getMoveDone();
    void setParams(float roll,float pitch,float yaw,float X,float Y,float Z);

    void getJointStates(double *values);

    void moveObstacle();
    void visualizeRobot(const double *values);
private:

    ompl::base::StateSpace *stateSpace_;
    ompl::base::SpaceInformationPtr si_;

    mutable std::mutex g_i_mutex;
    mutable std::mutex g_j_mutex;
};



#endif
