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
#include <gpu_voxels/logging/logging_gpu_voxels.h>

#define IC_PERFORMANCE_MONITOR
#include <icl_core_performance_monitor/PerformanceMonitor.h>

#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>

#include <ompl/geometric/PathSimplifier.h>

#include "gvl_ompl_planner_helper.h"
#include <stdlib.h>

#include <memory>
#include <thread>
//---------------------------SMC-------------------------------
#include <signal.h>
//--------------------------------------------------------------
namespace ob = ompl::base;
namespace og = ompl::geometric;
void ctrlchandler(int)
{
  exit(EXIT_SUCCESS);
}

void killhandler(int)
{
  exit(EXIT_SUCCESS);
}



int main(int argc, char **argv)
{
signal(SIGINT, ctrlchandler);
signal(SIGTERM, killhandler);

    icl_core::logging::initialize(argc, argv);
    PERF_MON_INITIALIZE(100, 1000);
    PERF_MON_ENABLE("planning");
    auto space(std::make_shared<ob::RealVectorStateSpace>(7));
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
    auto si(std::make_shared<ob::SpaceInformation>(space));
    std::shared_ptr<GvlOmplPlannerHelper> my_class_ptr(std::make_shared<GvlOmplPlannerHelper>(si));
    //og::PathSimplifier simp(si);
    //si->setStateValidityChecker(my_class_ptr->getptr());
    //si->setMotionValidator(my_class_ptr->getptr());
    //si->setup();
    std::cout << "Press Enter Key if ready!" << std::endl;
    std::cin.ignore();

    thread t1{&GvlOmplPlannerHelper::rosIter ,my_class_ptr};    
    while(true)
    {
        my_class_ptr->doVis();
        usleep(30000);
    }
    t1.join();
    return 1;
}
