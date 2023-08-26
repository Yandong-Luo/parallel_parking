#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include "constants.h"
#include "helper.h"
#include "collisiondetection.h"
#include "dynamicvoronoi.h"
#include "algorithm.h"
#include "node3d.h"
#include "path.h"
#include "smoother.h"
#include "visualize.h"
#include "lookup.h"

namespace HybridAStar {
/*!
   \brief A class that creates the interface for the hybrid A* algorithm.

    It inherits from `ros::nav_core::BaseGlobalPlanner` so that it can easily be used with the ROS navigation stack
   \todo make it actually inherit from nav_core::BaseGlobalPlanner
*/
    class Planner {
    public:
    /// The default constructor
    Planner();

    /*!
        \brief Initializes the collision as well as heuristic lookup table
        \todo probably removed
    */
    void initializeLookups();

    /*!
        \brief Sets the map e.g. through a callback from a subscriber listening to map updates.
        \param map the map or occupancy grid
    */
    void setMap(const nav_msgs::OccupancyGrid::Ptr map);

    /*!
        \brief setStart
        \param start the start pose
    */
    void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);

    /*!
        \brief setGoal
        \param goal the goal pose
    */
    void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);

    void setRePlan(const std_msgs::Bool replan);

    void getPath_callBack(const std_msgs::Bool getPath);

    /*!
        \brief The central function entry point making the necessary preparations to start the planning.
    */
    void plan();

    private:
    /// The node handle
    ros::NodeHandle n;
    /// A publisher publishing the start position for RViz
    ros::Publisher pubStart;
    /// A subscriber for receiving map updates
    ros::Subscriber subMap;
    /// A subscriber for receiving goal updates
    ros::Subscriber subGoal;
    /// A subscriber for receiving start updates
    ros::Subscriber subStart;

    /// A subscriber for receiving replan command
    ros::Subscriber subReplan;

    /// A subscriber for receiving get final path command
    ros::Subscriber subGetFinal;

    /// A listener that awaits transforms
    tf::TransformListener listener;
    /// A transform for moving start positions
    tf::StampedTransform transform;
    /// The path produced by the hybrid A* algorithm
    Path path;
    /// The smoother used for optimizing the path
    Smoother smoother;
    /// The path smoothed and ready for the controller
    Path smoothedPath = Path(true);
    /// The visualization used for search visualization
    Visualize visualization;
    /// The collission detection for testing specific configurations
    CollisionDetection configurationSpace;
    /// The voronoi diagram
    DynamicVoronoi voronoiDiagram;
    /// A pointer to the grid the planner runs on
    nav_msgs::OccupancyGrid::Ptr grid;
    /// The start pose set through RViz
    geometry_msgs::PoseWithCovarianceStamped start;
    /// The goal pose set through RViz
    geometry_msgs::PoseStamped goal;
    /// Flags for allowing the planner to plan
    bool validStart = false;
    /// Flags for allowing the planner to plan
    bool validGoal = false;
    /// A lookup table for configurations of the vehicle and their spatial occupancy enumeration
    Constants::config collisionLookup[Constants::headings * Constants::positions];
    /// A lookup of analytical solutions (Dubin's paths)
    float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];

    bool replan = true;

    float start_x = 0.0;
    float start_y = 0.0;
    float start_theta = 0.0;

    float goal_x = 0.0;
    float goal_y = 0.0;
    float goal_theta = 0.0;

    // 每个栅格代表多少m, 初始默认为1
    float cellSize = 1;

    // 由于map的origin可能不为0，也就是map的origin移动了，这个数值将作为补偿值帮助找到data和origin之间的关系
    // 由于map信息以数组方式存储，所以我在这里用了int类型而不是float
    int origin_x = 0;
    int origin_y = 0;

    int width = 0;
    int height = 0;
};
}
#endif // PLANNER_H
