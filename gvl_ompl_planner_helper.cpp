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
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
//#include "Poco/Net/Net.h"





#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
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

using namespace gpu_voxels;
namespace bfs = boost::filesystem;
#define PI 3.141592
#define D2R 3.141592/180.0
#define R2D 180.0/3.141592
#define RED BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (10 % 249) )
#define PURPLE BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (150 % 249) )
#define BLUE BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (200 % 249))
#define YELLOW BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (1 % 249))

double joint_states[JOINTNUM] = {0,0,0,0,0,0,0,0,0};
double task_goal_values[7] = {0,0,0,1,3.0,2.0,1.35}; 
Vector3ui map_dimensions(700,500,400);
std::array<double,JOINTNUM> send_q = {0};

void printCatesianKDLFrame(KDL::Frame frame,char* str ){
    std::cout<<"======="<<str<<"=======\n\n"<<endl;
    for(int i =0;i<4;i++){
        for(int j=0;j<4;j++)
            std::cout<<frame(i,j)<<"\t";
        std::cout<<"\n"<<std::endl;
    }
}


std::vector<std::array<double,JOINTNUM>>  GvlOmplPlannerHelper::doTaskPlanning(double goal_values[7],double start_values[JOINTNUM],ob::PathPtr path){
    PERF_MON_INITIALIZE(100, 1000);
    PERF_MON_ENABLE("planning");
    auto space(std::make_shared<ob::RealVectorStateSpace>(JOINTNUM));
    ob::RealVectorBounds bounds(JOINTNUM);
    bounds.setLow(-10);
    bounds.setHigh(10);

    std::vector<std::array<double,JOINTNUM>> q_list;
    q_list.clear();
    space->setBounds(bounds);

      this->si_->setStateValidityChecker(this->getptr());
      this->si_->setMotionValidator(this->getptr());
      this->si_->setup();

    og::PathSimplifier simp(this->si_);

    KDL::JntArray q_start(JOINTNUM);  
    KDL::JntArray q_result(JOINTNUM);  
    
    KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(my_chain);
    KDL::Frame cartesian_pos;
    KDL::Frame cartesian_pos_result;
    
    KDL::Frame goal_pose( KDL::Rotation::Quaternion(goal_values[0],goal_values[1],goal_values[2],goal_values[3]),KDL::Vector(goal_values[4],goal_values[5],goal_values[6]));

    fk_solver.JntToCart(q_start, cartesian_pos);

    KDL::ChainIkSolverVel_pinv iksolver1v(my_chain);
    KDL::ChainIkSolverPos_NR_JL iksolver1(my_chain,q_min,q_max,fk_solver,iksolver1v,2000,0.01);

    for(int i=0;i<JOINTNUM;i++){
        q_start(i) = start_values[i];
    }

    bool ret = iksolver1.CartToJnt(q_start,goal_pose,q_result);
    //std::cout<<"ik ret : "<<ret<<std::endl;
    //std::cout<<"ik q : "<<q_result(0)<<","<<q_result(1)<<","<<q_result(2)
    //                    <<","<<q_result(3)<<","<<q_result(4)<<","
    //                    <<q_result(5)<<","<<q_result(6)<<","
    //                    <<q_result(7)<<","<<q_result(8)<<std::endl;

    fk_solver.JntToCart(q_result, cartesian_pos_result);

    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    
    for(int i = 0;i<JOINTNUM;i++){
        start[i] = q_start(i);
        goal[i] = q_result(i);
    }
    std::system("clear");
    LOGGING_INFO(Gpu_voxels, "PDEF \n" << endl);
    auto pdef(std::make_shared<ob::ProblemDefinition>(this->si_));
    pdef->setStartAndGoalStates(start, goal);
    LOGGING_INFO(Gpu_voxels, "PLANNER \n" << endl);

    //auto planner(std::make_shared<og::KPIECE1>(this->si_));

    auto planner(std::make_shared<og::ABITstar>(this->si_));

    
    planner->setProblemDefinition(pdef);
    planner->setup();
    int succs = 0;
    
    LOGGING_INFO(Gpu_voxels, "WHILE \n" << endl);
    float solveTime = 0.1;
    int no_succs_count = 0;
     while(succs<1)
    {
        double sum = 0.0;
        for(int k = 0;k<JOINTNUM;k++){
            sum += sqrt((start[k]-goal[k])*(start[k]-goal[k]));
        }
        if(sum< 0.01){
            LOGGING_INFO(Gpu_voxels, "COMPLETE MOTION PLANNING \n" << endl);
            break;
        }
        try{
            planner->clear();
            LOGGING_INFO(Gpu_voxels,"SUCCESS : " <<succs<<", START PLANNING \n" << endl);

            const std::function< bool()> ptc;

            ob::PlannerStatus  solved = planner->ob::Planner::solve(solveTime);
            LOGGING_INFO(Gpu_voxels, "end PLANNING \n" << endl);

            PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("planner", "Planning time", "planning");


            if (solved)
            {
                ++succs;
                path = pdef->getSolutionPath();
                std::cout << "Found solution:" << std::endl;
                path->print(std::cout);
                simp.simplifyMax(*(path->as<og::PathGeometric>()));


            }else{
                std::cout << "No solution could be found" << std::endl;
                no_succs_count++;
                solveTime +=0.1; 
                if(no_succs_count>5)return q_list;
            }

            PERF_MON_SUMMARY_PREFIX_INFO("planning");
            std::cout << "END OMPL" << std::endl;
           }
        catch(int expn){
            std::cout << "ERRORROROROROR" << std::endl;
             return q_list;
        }

    }



    og::PathGeometric* solution= path->as<og::PathGeometric>();
    solution->interpolate(100);
    int step_count = solution->getStateCount();

    for(int i=0;i<step_count;i++){
        const double *values = solution->getState(i)->as<ob::RealVectorStateSpace::StateType>()->values;
        double *temp_values = (double*)values;
        std::array<double,JOINTNUM> temp_joints_value={{temp_values[0],temp_values[1],temp_values[2],temp_values[3],temp_values[4],temp_values[5],temp_values[6]
            ,temp_values[7],temp_values[8]}};
        q_list.push_back(temp_joints_value);
    
     }
    return q_list;


}




std::array<double,JOINTNUM> pop_front(std::vector<std::array<double,JOINTNUM>> joint_trajectory);






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
    //LOGGING_INFO(Gpu_voxels, "ROS JointState " << endl);
    gvl->insertRobotIntoMap("myUrdfCollisionRobot","myRobotCollisionMap",eBVM_OCCUPIED);
    gvl->insertRobotIntoMap("myUrdfCollisionRobot", "myRobotCollisionMapBitVoxel", BLUE);

}

void rosDesiredPoseCallback(const geometry_msgs::Pose::ConstPtr& msg){
    
    LOGGING_INFO(Gpu_voxels,msg->position.x<< endl);
     task_goal_values[0] = msg->orientation.x;
     task_goal_values[1] = msg->orientation.y;
     task_goal_values[2] = msg->orientation.z;
     task_goal_values[3] = msg->orientation.w;
     task_goal_values[4] = msg->position.x;
     task_goal_values[5] = msg->position.y;
     task_goal_values[6] = msg->position.z;
    new_pose_received=true;

}
void rosMovingFlagCallback(const std_msgs::Bool::ConstPtr& msg){
    isMoving = msg->data;
    LOGGING_INFO(Gpu_voxels, "ISMOVING :  "<<isMoving << endl);
    try{
        if(joint_trajectory.size()>0){
            reverse(joint_trajectory.begin(), joint_trajectory.end());
            joint_trajectory.pop_back();
            reverse(joint_trajectory.begin(), joint_trajectory.end());
        }
    }
    catch(int e){


    }
    

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


void pubJointState(double *jointValue,ros::Publisher *pub_joint ){
    sensor_msgs::JointState jointState;
    jointState.name.push_back("lin_x_joint");
    jointState.name.push_back("lin_y_joint");
    jointState.name.push_back("rot_z_joint");
    jointState.name.push_back("Arm_Joint_1");
    jointState.name.push_back("Arm_Joint_2");
    jointState.name.push_back("Arm_Joint_3");
    jointState.name.push_back("Arm_Joint_4");
    jointState.name.push_back("Arm_Joint_5");
    jointState.name.push_back("Arm_Joint_6");
    
    for(int i = 0;i<JOINTNUM;i++)
        jointState.position.push_back(jointValue[i]);
    
    jointState.header.stamp=ros::Time::now();
    pub_joint->publish(jointState);

}

void GvlOmplPlannerHelper::tcpIter(){
        /*
	StreamSocket ss;
	Data data_rev, data;
	unsigned char writeBuff[SIZE_DATA_MAX];
	ss.connect(SocketAddress(hostname, PORT));
        Data data_send;
	int count = 0;
	while(true){
		for(int i =0;i<6;i++)
			data.value[i+3] = send_q.at(i);
		memcpy(writeBuff, data.byte, SIZE_DATA_MAX);
		ss.sendBytes(writeBuff, SIZE_DATA_MAX);
		usleep(1000);
	}
	ss.close();
        */

}

void GvlOmplPlannerHelper::rosIter(){
          std::cout<<"ddd"<<std::endl;

    int argc;
    char **argv;
          std::cout<<"ddd"<<std::endl;

    ros::init(argc,argv,"gpu_voxel");

      std::cout<<"ddd"<<std::endl;

    const Vector3f camera_offsets(0.0f,
                                 0.0f, 
                                 0.0f); 
      std::cout<<"ddd"<<std::endl;

    Eigen::Matrix4f TBaseToCamera = GvlOmplPlannerHelper::loadBaseToCam("TBaseToCamera.txt");
      std::cout<<"ddd"<<std::endl;

    tf = Matrix4f(TBaseToCamera(0,0),TBaseToCamera(0,1),TBaseToCamera(0,2),TBaseToCamera(0,3)
        ,TBaseToCamera(1,0),TBaseToCamera(1,1),TBaseToCamera(1,2),TBaseToCamera(1,3)
        ,TBaseToCamera(2,0),TBaseToCamera(2,1),TBaseToCamera(2,2),TBaseToCamera(2,3)
        ,TBaseToCamera(3,0),TBaseToCamera(3,1),TBaseToCamera(3,2),TBaseToCamera(3,3));
        
    //tf = Matrix4f(1,0,0,2.0, 0,1,0,2.0, 0,0,1,1.0, 0,0,0,1);
    std::cout<<"==========TBaseToCmaera.txt==========\n"<<std::endl;
    std::cout<<tf<<std::endl;


    countingVoxelList = dynamic_pointer_cast<CountingVoxelList>(gvl->getMap("countingVoxelList"));
    myRobotCollisionMapBitVoxel = dynamic_pointer_cast<BitVectorVoxelList>(gvl->getMap("myRobotCollisionMapBitVoxel"));
    
    ros::NodeHandle nh;
    ros::Subscriber joint_sub = nh.subscribe("/joint_states", 1, rosjointStateCallback); 
    ros::Subscriber desiredPose_sub = nh.subscribe("/desired_pose", 1, rosDesiredPoseCallback); 
    ros::Subscriber moving_flag = nh.subscribe("/ismoving", 1, rosMovingFlagCallback); 
    
    
    //ros::Subscriber point_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >("/camera/depth/color/points", 1,roscallback);
    ros::Subscriber point_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >("/merged", 1,roscallback);

    //ros::Subscriber point_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >("/cam_0_zf", 1,roscallback);
    ros::Publisher pub_joint =  nh.advertise<sensor_msgs::JointState>("/joint_states_desired", 1000);

    ros::Rate r(100);
    new_data_received = true; // call visualize on the first iteration
    new_pose_received=false;
      std::cout<<"ddd"<<std::endl;
    size_t num_colls = 0;
  

    while (ros::ok())
    {
        ros::spinOnce();
       // LOGGING_INFO(Gpu_voxels, "ROSITER " << endl);

        //init
        if(new_data_received){
          //  LOGGING_INFO(Gpu_voxels, "Recived Pointcloud " << endl);

            countingVoxelList->clearMap();
            myEnvironmentMap->clearMap();



            countingVoxelList->insertPointCloud(my_point_cloud,eBVM_OCCUPIED);
            countingVoxelList->as<gpu_voxels::voxellist::CountingVoxelList>()->subtractFromCountingVoxelList(
            myRobotCollisionMapBitVoxel->as<gpu_voxels::voxellist::BitVectorVoxelList>(),
            Vector3f());
            myEnvironmentMap->merge(countingVoxelList);
            num_colls = gvl->getMap("countingVoxelList")->as<gpu_voxels::voxellist::CountingVoxelList>()->collideWith(gvl->getMap("mySolutionMap")->as<gpu_voxels::voxellist::BitVectorVoxelList>(), 1.0f);
            if(num_colls>50){
                std::cout << "!!!!!!!!!!!!!!!Detected Collision!!!!!!!!! " << num_colls << " collisions " << std::endl;
                new_pose_received = true;
            }


        }
        if(new_pose_received){
           // LOGGING_INFO(Gpu_voxels, "Recived Target Position " << endl);
            ob::PathPtr path;
            joint_trajectory=GvlOmplPlannerHelper::doTaskPlanning(task_goal_values,joint_states,path);
                GvlOmplPlannerHelper::visualizeSolution(joint_trajectory);    
                    
        }
        else{
          std::cout << "!!!!!!!!!!!!!!!isMoving!!!!!!!!! " << isMoving << std::endl;
            if(joint_trajectory.size()>0 && isMoving){
                GvlOmplPlannerHelper::visualizeSolution(joint_trajectory);  
                std::array<double,JOINTNUM> temp_q = joint_trajectory.at(0);
                send_q = temp_q;
                sensor_msgs::JointState jointState;
                jointState.name.push_back("lin_x_joint");
                jointState.name.push_back("lin_y_joint");
                jointState.name.push_back("rot_z_joint");
                jointState.name.push_back("Arm_Joint_1");
                jointState.name.push_back("Arm_Joint_2");
                jointState.name.push_back("Arm_Joint_3");
                jointState.name.push_back("Arm_Joint_4");
                jointState.name.push_back("Arm_Joint_5");
                jointState.name.push_back("Arm_Joint_6");
                
                for(int i = 0;i<JOINTNUM;i++)
                    jointState.position.push_back(temp_q.at(i));
                
                jointState.header.stamp=ros::Time::now();
                pub_joint.publish(jointState);
                std::cout << "!!!!!!!!!!!!!!!PUBLISH!!!!!!!!! " << isMoving << std::endl;
                isMoving=false;
            }

        }


        GvlOmplPlannerHelper::doVis();

        new_data_received = false;
        new_pose_received=false;

        r.sleep();
    }

    exit(EXIT_SUCCESS);
}



GvlOmplPlannerHelper::GvlOmplPlannerHelper(const ob::SpaceInformationPtr &si)
    : ob::StateValidityChecker(si)
    , ob::MotionValidator(si)
{


    const char* urdf_name = "./mm_hyu_coarse/right_sim.urdf";
    const char* colilsion_urdf_name = "./mm_hyu_coarse_collision/right_sim.urdf";    

    si_ = si;
    stateSpace_ = si_->getStateSpace().get();

    assert(stateSpace_ != nullptr);

    gvl = gpu_voxels::GpuVoxels::getInstance();
    gvl->initialize(map_dimensions.x,map_dimensions.y, map_dimensions.z, voxel_side_length);

    // We add maps with objects, to collide them
    gvl->addMap(MT_PROBAB_VOXELMAP,"myRobotMap");
    gvl->addMap(MT_BITVECTOR_VOXELLIST, "myRobotMapBitVoxel");


    gvl->addMap(MT_PROBAB_VOXELMAP,"myRobotCollisionMap");
    gvl->addMap(MT_PROBAB_VOXELMAP,"myEnvironmentAllMap");

    gvl->insertPointCloudFromFile("myEnvironmentAllMap", "./binvox/environment_all.binvox", true,
                                      gpu_voxels::eBVM_OCCUPIED, true, gpu_voxels::Vector3f(0.0, 0.0, -0.01),1);
    gvl->addMap(MT_PROBAB_VOXELMAP,"myEnvironmentMap");
    gvl->addMap(MT_BITVECTOR_VOXELLIST, "myRobotCollisionMapBitVoxel");
    myEnvironmentMap = dynamic_pointer_cast<ProbVoxelMap>(gvl->getMap("myEnvironmentMap"));

    gvl->addMap(MT_BITVECTOR_VOXELLIST,"mySolutionMap");
    gvl->addMap(MT_PROBAB_VOXELMAP,"myQueryMap");
    gvl->addMap(MT_COUNTING_VOXELLIST,"countingVoxelList");
    gvl->addRobot("myUrdfCollisionRobot",colilsion_urdf_name , true);
    gvl->addRobot("myUrdfRobot",urdf_name , true);
    

     if (!kdl_parser::treeFromFile(urdf_name, my_tree)){
             LOGGING_INFO(Gpu_voxels,"Failed to construct kdl tree");
    }

    LOGGING_INFO(Gpu_voxels, "\n\nKDL Number of Joints : "<<my_tree.getNrOfJoints() <<"\n"<< endl);

    LOGGING_INFO(Gpu_voxels, "\n\nKDL Chain load : "<<my_tree.getChain("world","SPA_Link_06",my_chain) <<"\n"<< endl);

    q_min(0) = 0;
    q_min(1) = 0;
    q_min(2) = -3.141592;

    q_min(3) = -2.63;
    q_min(4) = -0.6981;
    q_min(5) = -1.34;
    q_min(6) = -0.156;
    q_min(7) = -3.141592;
    q_min(8) = -1.41;


    q_max(0) = 7;
    q_max(1) = 5;
    q_max(2) = 3.141592;

    q_max(3) = 1.57;
    q_max(4) = 1.9;
    q_max(5) = 1.09;
    q_max(6) = 2.48;
    q_max(7) = 3.141592;
    q_max(8) = 1.392;


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
        gvl->visualizeMap("mySolutionMap");

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
void GvlOmplPlannerHelper::visualizeSolution(std::vector<std::array<double,JOINTNUM>> solution)
{
    gvl->clearMap("mySolutionMap");

   for(int j = 0;j<solution.size();j++)
    {

        std::array<double,JOINTNUM> temp_q = solution.at(j);

        robot::JointValueMap state_joint_values;
        state_joint_values["lin_x_joint"] = temp_q[0];
        state_joint_values["lin_y_joint"] = temp_q[1];
        state_joint_values["rot_z_joint"] = temp_q[2];
        state_joint_values["Arm_Joint_1"] = temp_q[3];
        state_joint_values["Arm_Joint_2"] = temp_q[4];
        state_joint_values["Arm_Joint_3"] = temp_q[5];
        state_joint_values["Arm_Joint_4"] = temp_q[6];
        state_joint_values["Arm_Joint_5"] = temp_q[7];
        state_joint_values["Arm_Joint_6"] = temp_q[8];
        // update the robot joints:
        gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
        // insert the robot into the map:
        gvl->insertRobotIntoMap("myUrdfRobot", "mySolutionMap", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (j % 249) ));
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
