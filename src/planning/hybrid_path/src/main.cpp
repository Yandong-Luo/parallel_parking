/**
   \file main.cpp
   \brief Main entry point of the program, starts an instance of Planner
   这是ROS的主函数入口程序：即启动规划器入口
*/

//###################################################
//                      HYBRID A* ALGORITHM
//  AUTHOR:   Karl Kurzer
//  WRITTEN:  2015-03-02
//###################################################

#include <cstring>
#include <iostream>
#include <ros/ros.h>

#include "constants.h"
#include "planner.h"



int main(int argc, char** argv) {

  ros::init(argc, argv, "hybrid_astar");

  HybridAStar::Planner m_planner;
  m_planner.plan(); 

  ros::spin();
  return 0;
}
