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
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/fmt/FMT.h>

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

#include <ompl/geometric/PathSimplifier.h>

#include "gvl_ompl_planner_helper.h"
#include <stdlib.h>
#include <ros/ros.h>
#include <thread>
#include <memory>

#include <Python.h>
#include <stdlib.h>
#include <vector>
namespace ob = ompl::base;
namespace og = ompl::geometric;

#define PI 3.141592
#define D2R 3.141592/180.0
#define R2D 180.0/3.141592
using namespace KDL;
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
  auto space(std::make_shared<ob::RealVectorStateSpace>(7));
  //We then set the bounds for the R3 component of this state space:
  ob::RealVectorBounds bounds(7);
  bounds.setLow(-3.14159265);
  bounds.setHigh(3.14159265);
	  bounds.setLow(0,-2.8973);
  bounds.setHigh(0,2.9671);

  bounds.setLow(1,-1.7628);
  bounds.setHigh(1,1.7628);

  bounds.setLow(2,-2.8973);
  bounds.setHigh(2,2.8973);

  bounds.setLow(3,-3.0718);
  bounds.setHigh(3,-0.0698);

  bounds.setLow(4,-2.8973);
  bounds.setHigh(4,2.8973);

  bounds.setLow(5,-0.0175);
  bounds.setHigh(5,3.7525);

  bounds.setLow(6,-2.8973);
  bounds.setHigh(6,2.8973);


  space->setBounds(bounds);
  //Create an instance of ompl::base::SpaceInformation for the state space
  auto si(std::make_shared<ob::SpaceInformation>(space));
  //Set the state validity checker
  std::shared_ptr<GvlOmplPlannerHelper> my_class_ptr(std::make_shared<GvlOmplPlannerHelper>(si));

  thread t1{&GvlOmplPlannerHelper::rosIter ,my_class_ptr};    


  std::cin.ignore();

  while(1){
        usleep(300000);
  }
//----------------------------------------------------//
    t1.join();

    return 1;
}
