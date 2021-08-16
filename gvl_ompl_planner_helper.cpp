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
#include "gvl_ompl_planner_helper.h"
#include <Eigen/Dense>
#include <signal.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <sstream>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/helpers/MetaPointCloud.h>
#include <gpu_voxels/robot/urdf_robot/urdf_robot.h>
//#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <thread>

#include <stdio.h>
#include <iostream>
#define IC_PERFORMANCE_MONITOR
#include <icl_core_config/Config.h>
#include <icl_core_performance_monitor/PerformanceMonitor.h>
#include <vector>
namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace gpu_voxels;
namespace bfs = boost::filesystem;
#define PI 3.141592
#define D2R 3.141592/180.0
#define R2D 180.0/3.141592
#define RED BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (10 % 249) )
#define PURPLE BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (150 % 249) )
#define BLUE BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (200 % 249))
#define YELLOW BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (1 % 249))

double joint_states[7] = {0,0,0,0,0,0,0};
Vector3ui map_dimensions(700,500,400);
void rosjointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    gvl->clearMap("myRobotMap");
    gvl->clearMap("myRobotMapBitVoxel");
    
    gvl->clearMap("myRobotCollisionMap");
    gvl->clearMap("myRobotCollisionMapBitVoxel");
    
    
    for(size_t i = 0; i < msg->name.size(); i++)
    {
        myRobotJointValues[msg->name[i]] = msg->position[i];
        joint_states[i] = msg->position[i];
    }
    gvl->setRobotConfiguration("myUrdfRobot",myRobotJointValues);
    gvl->setRobotConfiguration("myUrdfCollisionRobot",myRobotJointValues);
    gvl->insertRobotIntoMap("myUrdfRobot","myRobotMap",eBVM_OCCUPIED);
    gvl->insertRobotIntoMap("myUrdfRobot","myRobotMapBitVoxel",YELLOW);
    LOGGING_INFO(Gpu_voxels, "ROS JointState " << endl);
    gvl->insertRobotIntoMap("myUrdfCollisionRobot","myRobotCollisionMap",eBVM_OCCUPIED);
    gvl->insertRobotIntoMap("myUrdfCollisionRobot", "myRobotCollisionMapBitVoxel", BLUE);

    
}
void roscallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
    std::vector<Vector3f> point_data;
    point_data.resize(msg->points.size());
    for (uint32_t i = 0; i < msg->points.size(); i++)
    {
    	point_data[i].x = msg->points[i].x;
    	point_data[i].y = msg->points[i].y;
    	point_data[i].z = msg->points[i].z;
    }
    my_point_cloud.update(point_data);
    my_point_cloud.transformSelf(&tf);
    new_data_received=true;
}

Eigen::Matrix4f GvlOmplPlannerHelper::loadBaseToCam(std::string filename){

    // Image Coordinate -> Cam Coordinate -> World Coordinate
    std::string testline;
    std::string word[4][4];
    Eigen::Matrix4f TBaseToCamera = Eigen::Matrix4f::Identity();
    std::ifstream Test (filename);

    if (!Test)
    {
        std::cout << "There was an error opening the file.\n"<<std::endl;
        return TBaseToCamera;
    }
    int x=0,y=0;
    while( Test>>testline ){
        word[y][x]=testline;
        x++;
        if (testline=="")
        y++;
    }
        for (int y=0;y<4;y++)
        {
            for (int x=0;x<4;x++)
                 TBaseToCamera(y,x)= std::stod(word[y][x]);
        }
    Eigen::Matrix3f Rx  = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f Ry  = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f Rz  = Eigen::Matrix3f::Identity();
    float roll = -PI/2.0;
    float pitch = 0.0;
    float yaw = -PI/2.0;

    Rx(1,1) = cos(roll);
    Rx(1,2) = -sin(roll);
    Rx(2,1) = sin(roll);
    Rx(2,2) = cos(roll);
    Ry(0,0) = cos(pitch);
    Ry(0,2) = sin(pitch);
    Ry(2,0) = -sin(pitch);
    Ry(2,2) = cos(pitch);
    Rz(0,0) = cos(yaw);
    Rz(0,1) = -sin(yaw);
    Rz(1,0) = sin(yaw);
    Rz(1,1) = cos(yaw);

    Eigen::Matrix3f R = Rz*Ry*Rx;
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f TBaseToCameraTemp = TBaseToCamera;
    T(0,0)=R(0,0);
    T(0,1)=R(0,1);
    T(0,2)=R(0,2);
    T(1,0)=R(1,0);
    T(1,1)=R(1,1);
    T(1,2)=R(1,2);
    T(2,0)=R(2,0);
    T(2,1)=R(2,1);
    T(2,2)=R(2,2);
    TBaseToCamera(0,3)=0;
    TBaseToCamera(1,3)=0;
    TBaseToCamera(2,3)=0;
    
    TBaseToCamera = TBaseToCamera*T;
    TBaseToCamera(0,3) = TBaseToCameraTemp(0,3);
    TBaseToCamera(1,3) = TBaseToCameraTemp(1,3);
    TBaseToCamera(2,3) = TBaseToCameraTemp(2,3);
    
    return TBaseToCamera;
}




void GvlOmplPlannerHelper::rosIter(){
    int argc;
    char **argv;
    ros::init(argc,argv,"gpu_voxel_temp");
    signal(SIGINT, ctrlchandler);
    signal(SIGTERM, killhandler);

    const Vector3f camera_offsets(0.0f,
                                 0.0f, 
                                 0.0f); 
    Eigen::Matrix4f TBaseToCamera = GvlOmplPlannerHelper::loadBaseToCam("TBaseToCamera.txt");
    
    tf = Matrix4f(TBaseToCamera(0,0),TBaseToCamera(0,1),TBaseToCamera(0,2),TBaseToCamera(0,3)
        ,TBaseToCamera(1,0),TBaseToCamera(1,1),TBaseToCamera(1,2),TBaseToCamera(1,3)
        ,TBaseToCamera(2,0),TBaseToCamera(2,1),TBaseToCamera(2,2),TBaseToCamera(2,3)
        ,TBaseToCamera(3,0),TBaseToCamera(3,1),TBaseToCamera(3,2),TBaseToCamera(3,3));
        
    //tf = Matrix4f(1,0,0,2.0, 0,1,0,2.0, 0,0,1,1.0, 0,0,0,1);
    std::cout<<"==========TBaseToCmaera.txt==========\n"<<std::endl;
    std::cout<<tf<<std::endl;
    std::cin.ignore();


    countingVoxelList = dynamic_pointer_cast<CountingVoxelList>(gvl->getMap("countingVoxelList"));
    myRobotCollisionMapBitVoxel = dynamic_pointer_cast<BitVectorVoxelList>(gvl->getMap("myRobotCollisionMapBitVoxel"));
    
    ros::NodeHandle nh;
    ros::Subscriber joint_sub = nh.subscribe("/joint_states", 1, rosjointStateCallback); 
    ros::Subscriber point_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >("/camera/depth/color/points", 1,roscallback);

    ros::Rate r(100);
    new_data_received = true; // call visualize on the first iteration

    while (ros::ok())
    {
        ros::spinOnce();
        //LOGGING_INFO(Gpu_voxels, "ROSITER " << endl);

        //init
        countingVoxelList->clearMap();
        myEnvironmentMap->clearMap();



        countingVoxelList->insertPointCloud(my_point_cloud,eBVM_OCCUPIED);
        countingVoxelList->as<gpu_voxels::voxellist::CountingVoxelList>()->subtractFromCountingVoxelList(
        myRobotCollisionMapBitVoxel->as<gpu_voxels::voxellist::BitVectorVoxelList>(),
        Vector3f());
        myEnvironmentMap->merge(countingVoxelList);



        GvlOmplPlannerHelper::doVis();
        r.sleep();
    }
    exit(EXIT_SUCCESS);
}




GvlOmplPlannerHelper::GvlOmplPlannerHelper(const ob::SpaceInformationPtr &si)
    : ob::StateValidityChecker(si)
    , ob::MotionValidator(si)
{

    si_ = si;
    stateSpace_ = si_->getStateSpace().get();

    assert(stateSpace_ != nullptr);

    gvl = gpu_voxels::GpuVoxels::getInstance();
    gvl->initialize(map_dimensions.x,map_dimensions.y, map_dimensions.y, voxel_side_length);

    // We add maps with objects, to collide them
    gvl->addMap(MT_PROBAB_VOXELMAP,"myRobotMap");
    gvl->addMap(MT_BITVECTOR_VOXELLIST, "myRobotMapBitVoxel");


    gvl->addMap(MT_PROBAB_VOXELMAP,"myRobotCollisionMap");
    gvl->addMap(MT_PROBAB_VOXELMAP,"myEnvironmentAllMap");

    gvl->insertPointCloudFromFile("myEnvironmentAllMap", "./binvox/environment_all.binvox", true,
                                      gpu_voxels::eBVM_OCCUPIED, true, gpu_voxels::Vector3f(0.0, 0.0, 0.0),1);
    gvl->addMap(MT_PROBAB_VOXELMAP,"myEnvironmentMap");
    gvl->addMap(MT_BITVECTOR_VOXELLIST, "myRobotCollisionMapBitVoxel");
    myEnvironmentMap = dynamic_pointer_cast<ProbVoxelMap>(gvl->getMap("myEnvironmentMap"));

    gvl->addMap(MT_BITVECTOR_VOXELLIST,"mySolutionMap");
    gvl->addMap(MT_PROBAB_VOXELMAP,"myQueryMap");
    gvl->addMap(MT_COUNTING_VOXELLIST,"countingVoxelList");
    gvl->addRobot("myUrdfCollisionRobot", "./mm_hyu_coarse_collision/right_sim.urdf", true);
    gvl->addRobot("myUrdfRobot", "./mm_hyu_coarse/right_sim.urdf", true);
    
    PERF_MON_ENABLE("pose_check");
    PERF_MON_ENABLE("motion_check");
    PERF_MON_ENABLE("motion_check_lv");
}



GvlOmplPlannerHelper::~GvlOmplPlannerHelper()
{
    gvl.reset(); // Not even required, as we use smart pointers.
}

void GvlOmplPlannerHelper::moveObstacle()
{

}

void GvlOmplPlannerHelper::doVis()
{
     //LOGGING_INFO(Gpu_voxels, "Dovis " << endl);
     gvl->visualizeMap("myEnvironmentMap");
     gvl->visualizeMap("myEnvironmentAllMap");
     
 
    
    gvl->visualizeMap("myRobotMap");    
    gvl->visualizeMap("myRobotMapBitVoxel");    
    
    gvl->visualizeMap("myRobotCollisionMapBitVoxel");
    //gvl->visualizeMap("myRobotCollisionMap");


    gvl->visualizeMap("countingVoxelList");

}


void GvlOmplPlannerHelper::visualizeSolution(ob::PathPtr path)
{
    gvl->clearMap("mySolutionMap");

    PERF_MON_SUMMARY_PREFIX_INFO("pose_check");
    PERF_MON_SUMMARY_PREFIX_INFO("motion_check");
    PERF_MON_SUMMARY_PREFIX_INFO("motion_check_lv");

    //std::cout << "Robot consists of " << gvl->getRobot("myUrdfRobot")->getTransformedClouds()->getAccumulatedPointcloudSize() << " points" << std::endl;

    og::PathGeometric* solution = path->as<og::PathGeometric>();
    solution->interpolate();


    for(size_t step = 0; step < solution->getStateCount(); ++step)
    {

        const double *values = solution->getState(step)->as<ob::RealVectorStateSpace::StateType>()->values;

        robot::JointValueMap state_joint_values;
        state_joint_values["lin_x_joint"] = values[0];
        state_joint_values["lin_y_joint"] = values[1];
        state_joint_values["rot_z_joint"] = values[2];
        state_joint_values["Arm_Joint_1"] = values[3];
        state_joint_values["Arm_Joint_2"] = values[4];
        state_joint_values["Arm_Joint_3"] = values[5];
        state_joint_values["Arm_Joint_4"] = values[6];
        state_joint_values["Arm_Joint_5"] = values[7];
        state_joint_values["Arm_Joint_6"] = values[8];
        // update the robot joints:
        gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
        // insert the robot into the map:
        gvl->insertRobotIntoMap("myUrdfRobot", "mySolutionMap", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (step % 249) ));
    }

    gvl->visualizeMap("mySolutionMap");

}

void GvlOmplPlannerHelper::insertStartAndGoal(const ompl::base::ScopedState<> &start, const ompl::base::ScopedState<> &goal) const
{
    gvl->clearMap("myQueryMap");

    robot::JointValueMap state_joint_values;

    state_joint_values["lin_x_joint"] = start[0];
    state_joint_values["lin_y_joint"] = start[1];
    state_joint_values["rot_z_joint"] = start[2];
    state_joint_values["Arm_Joint_1"] = start[3];
    state_joint_values["Arm_Joint_2"] = start[4];
    state_joint_values["Arm_Joint_3"] = start[5];
    state_joint_values["Arm_Joint_4"] = start[6];
    state_joint_values["Arm_Joint_5"] = start[7];
    state_joint_values["Arm_Joint_6"] = start[8];

    // update the robot joints:
    gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
    gvl->insertRobotIntoMap("myUrdfRobot", "myQueryMap", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START));

    state_joint_values["lin_x_joint"] = goal[0];
    state_joint_values["lin_y_joint"] = goal[1];
    state_joint_values["rot_z_joint"] = goal[2];
    state_joint_values["Arm_Joint_1"] = goal[3];
    state_joint_values["Arm_Joint_2"] = goal[4];
    state_joint_values["Arm_Joint_3"] = goal[5];
    state_joint_values["Arm_Joint_4"] = goal[6];
    state_joint_values["Arm_Joint_5"] = goal[7];
    state_joint_values["Arm_Joint_6"] = goal[8];
    // update the robot joints:
    gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
    gvl->insertRobotIntoMap("myUrdfRobot", "myQueryMap", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START+1));
}

bool GvlOmplPlannerHelper::isValid(const ompl::base::State *state) const
{
    PERF_MON_START("inserting");

    std::lock_guard<std::mutex> lock(g_i_mutex);

    gvl->clearMap("myRobotMap");

    const double *values = state->as<ob::RealVectorStateSpace::StateType>()->values;

    robot::JointValueMap state_joint_values;
    state_joint_values["lin_x_joint"] = values[0];
    state_joint_values["lin_y_joint"] = values[1];
    state_joint_values["rot_z_joint"] = values[2];
    state_joint_values["Arm_Joint_1"] = values[3];
    state_joint_values["Arm_Joint_2"] = values[4];
    state_joint_values["Arm_Joint_3"] = values[5];
    state_joint_values["Arm_Joint_4"] = values[6];
    state_joint_values["Arm_Joint_5"] = values[7];
    state_joint_values["Arm_Joint_6"] = values[8];
    // update the robot joints:
    gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
    // insert the robot into the map:
    gvl->insertRobotIntoMap("myUrdfRobot", "myRobotMap", eBVM_OCCUPIED);

    PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("insert", "Pose Insertion", "pose_check");

    PERF_MON_START("coll_test");
    size_t num_colls_pc = gvl->getMap("myRobotMap")->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap("myEnvironmentMap")->as<voxelmap::ProbVoxelMap>(), 0.7f);
    gvl->insertRobotIntoMap("myUrdfRobot", "myRobotMap", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START+30));
    PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("coll_test", "Pose Collsion", "pose_check");

    //std::cout << "Validity check on state ["  << values[0] << ", " << values[1] << ", " << values[2] << ", " << values[3] << ", " << values[4] << ", " << values[5] << "] resulting in " <<  num_colls_pc << " colls." << std::endl;

    return num_colls_pc == 0;
}

bool GvlOmplPlannerHelper::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                                       std::pair< ompl::base::State*, double > & lastValid) const
{

    //    std::cout << "LongestValidSegmentFraction = " << stateSpace_->getLongestValidSegmentFraction() << std::endl;
    //    std::cout << "getLongestValidSegmentLength = " << stateSpace_->getLongestValidSegmentLength() << std::endl;
    //    std::cout << "getMaximumExtent = " << stateSpace_->getMaximumExtent() << std::endl;


    std::lock_guard<std::mutex> lock(g_j_mutex);
    gvl->clearMap("myRobotMap");

    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    //std::cout << "Called interpolating motion_check_lv to evaluate " << nd << " segments" << std::endl;

    PERF_MON_ADD_DATA_NONTIME_P("Num poses in motion", float(nd), "motion_check_lv");
    if (nd > 1)
    {
        /* temporary storage for the checked state */
        ob::State *test = si_->allocState();

        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
            if (!si_->isValid(test))

            {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first != nullptr)
                    stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
                result = false;
                break;

            }
        }

        si_->freeState(test);

    }

    if (result)
        if (!si_->isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first != nullptr)
                stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
            result = false;
        }


    if (result)
        valid_++;
    else
        invalid_++;


    return result;
}



bool GvlOmplPlannerHelper::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    std::lock_guard<std::mutex> lock(g_i_mutex);
    gvl->clearMap("myRobotMap");


    //        std::cout << "LongestValidSegmentFraction = " << stateSpace_->getLongestValidSegmentFraction() << std::endl;
    //        std::cout << "getLongestValidSegmentLength = " << stateSpace_->getLongestValidSegmentLength() << std::endl;
    //        std::cout << "getMaximumExtent = " << stateSpace_->getMaximumExtent() << std::endl;



    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    // not required with ProbabVoxels:
    //    if(nd > 249)
    //    {
    //        std::cout << "Too many intermediate states for BitVoxels" << std::endl;
    //        exit(1);
    //    }

    if (nd > 1)
    {
        PERF_MON_START("inserting");

        /* temporary storage for the checked state */
        ob::State *test = si_->allocState();

        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);


            const double *values = test->as<ob::RealVectorStateSpace::StateType>()->values;

            robot::JointValueMap state_joint_values;
            state_joint_values["lin_x_joint"] = values[0];
            state_joint_values["lin_y_joint"] = values[1];
            state_joint_values["rot_z_joint"] = values[2];
            state_joint_values["Arm_Joint_1"] = values[3];
            state_joint_values["Arm_Joint_2"] = values[4];
            state_joint_values["Arm_Joint_3"] = values[5];
            state_joint_values["Arm_Joint_4"] = values[6];
            state_joint_values["Arm_Joint_5"] = values[7];
            state_joint_values["Arm_Joint_6"] = values[8];

            // update the robot joints:
            gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
            // insert the robot into the map:
            gvl->insertRobotIntoMap("myUrdfRobot", "myRobotMap", eBVM_OCCUPIED);

        }
        PERF_MON_ADD_DATA_NONTIME_P("Num poses in motion", float(nd), "motion_check");

        si_->freeState(test);

        PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("insert", "Motion Insertion", "motion_check");

        //gvl->visualizeMap("myRobotMap");
        PERF_MON_START("coll_test");
        size_t num_colls_pc = gvl->getMap("myRobotMap")->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap("myEnvironmentMap")->as<voxelmap::ProbVoxelMap>(), 0.7f);

        std::cout << "CheckMotion1 for " << nd << " segments. Resulting in " << num_colls_pc << " colls." << std::endl;
        PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("coll_test", "Pose Collsion", "motion_check");

        result = (num_colls_pc == 0);

    }


    if (result)
        valid_++;
    else
        invalid_++;


    return result;


}
