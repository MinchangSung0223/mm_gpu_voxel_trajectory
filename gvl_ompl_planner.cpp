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
#include <iostream>
using namespace std;
#include <signal.h>

#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#define IC_PERFORMANCE_MONITOR
#include <icl_core_performance_monitor/PerformanceMonitor.h>
#include <unistd.h> 


#include "gvl_ompl_planner_helper.h"
#include <stdlib.h>
#include <ros/ros.h>
#include <thread>
#include <memory>

#include <Python.h>
#include <stdlib.h>
#include <vector>

#define PI 3.141592
#define D2R 3.141592/180.0
#define R2D 180.0/3.141592
using namespace std;


// initial quaternion 0.49996,0.86605,0.00010683,0

std::shared_ptr<GvlOmplPlannerHelper> my_class_ptr;

int main(int argc, char **argv)
{ 


  signal(SIGINT, ctrlchandler);
  signal(SIGTERM, killhandler);

  icl_core::logging::initialize(argc, argv);

  PERF_MON_INITIALIZE(100, 1000);
  PERF_MON_ENABLE("planning");

  // construct the state space we are planning in
  auto space(std::make_shared<ob::RealVectorStateSpace>(JOINTNUM));
  //We then set the bounds for the R3 component of this state space:
  ob::RealVectorBounds bounds(JOINTNUM);
    bounds.setLow(0,-7);
    bounds.setHigh(0,7);
    bounds.setLow(1,-5);
    bounds.setHigh(1,5);
    bounds.setLow(2,-PI);
    bounds.setHigh(2,PI);



    bounds.setLow(3,-2.63);
    bounds.setHigh(3,1.57);
    bounds.setLow(4,-0.6981);
    bounds.setHigh(4,1.9);
    bounds.setLow(5,-1.34);
    bounds.setHigh(5,1.09);
    bounds.setLow(6,-0.156);
    bounds.setHigh(6,2.48);
    bounds.setLow(7,-PI);
    bounds.setHigh(7,PI);
    bounds.setLow(8,-1.41);
    bounds.setHigh(8,392);


  space->setBounds(bounds);
  //Create an instance of ompl::base::SpaceInformation for the state space
  auto si(std::make_shared<ob::SpaceInformation>(space));
  //Set the state validity checker
  std::shared_ptr<GvlOmplPlannerHelper> my_class_ptr(std::make_shared<GvlOmplPlannerHelper>(si));
  si->setStateValidityChecker(my_class_ptr->getptr());
  si->setMotionValidator(my_class_ptr->getptr());
  si->setup();




  my_class_ptr->doVis();
  std::cout << "Press Enter Key if ready!" << std::endl;
  std::cin.ignore();


  thread t1{&GvlOmplPlannerHelper::rosIter ,my_class_ptr};  
  //thread t2{&GvlOmplPlannerHelper::tcpIter ,my_class_ptr};  


  while(1){
       // double task_goal_values[7] ={0,0,0,1,3.0,2.5,1.325};
      //  double start_values[JOINTNUM]={3.0,2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};      
      //  my_class_ptr->doTaskPlanning(task_goal_values,start_values);
      //  std::cout<<"===========Task Planning End=========="<<std::endl;
        usleep(3000000);
  }
//----------------------------------------------------//
    t1.join();
    //t2.join();
    return 1;
}
