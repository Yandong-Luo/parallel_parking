/*********************************************************************************
  *Copyright(C),1996-2021, Company
  *FileName:  planner.cpp
  *Author:  Yandong Luo
  *Version:  V1.0
  *Date:
  *Description:


  *History:
     1.Date:
       Author:
       Modification:
     2.
**********************************************************************************/
#include "planner.h"

using namespace HybridAStar;
using namespace std;
/**
 * @Brief : 节点订阅
 */
Planner::Planner()
{
    // Map
    // subMap = nh.subscribe("/map", 1, &Planner::setMap, this);
    // // Subscribe start pose
    // subStart = nh.subscribe("/start_pose", 1, &Planner::setStart, this);
    // // Subscribe goal pose
    // subGoal = nh.subscribe("/goal_pose", 1, &Planner::setGoal, this);


    /**
     * 用于测试的
    */
    // Map
    subMap = nh.subscribe("/map", 1, &Planner::setMap, this);
    // Subscribe start pose
    subStart = nh.subscribe("/initialpose", 1, &Planner::setStart, this);
    // Subscribe goal pose
    subGoal = nh.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);
};

/**
 * @Brief : 地图信息的订阅回调函数
 * @param  mapData          
 */
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr mapData){
    grid = mapData;
    configurationSpace.updateGrid(mapData);

    // height = mapData->info.height;
    // width = mapData->info.width;

    // // 转为int
    // origin_x = int(mapData->info.origin.position.x);
    // origin_y = int(mapData->info.origin.position.y);

    // // cell size
    // cellSize = mapData->info.resolution;

    // 初始化 mapInfo的内容
    m_mapInfo.cellSize = mapData->info.resolution;
    m_mapInfo.origin_x = mapData->info.origin.position.x;
    m_mapInfo.origin_y = mapData->info.origin.position.y;
    m_mapInfo.height = mapData->info.height;
    m_mapInfo.width = mapData->info.width;

    

    cout<<"origin_x:"<<m_mapInfo.origin_x<<"origin_y:"<<m_mapInfo.origin_y<<endl;

    bool** binMap;//二维数组，
    binMap = new bool*[m_mapInfo.width];

    for (int x = 0; x < m_mapInfo.width; x++) { binMap[x] = new bool[height]; }//这里可简化为一次申请

    for (int x = 0; x < m_mapInfo.width; ++x) {
      for (int y = 0; y < height; ++y) {
        binMap[x][y] = mapData->data[y * m_mapInfo.width + x] ? true : false;
      }
    }//转化为二值地图

    voronoiDiagram.initializeMap(m_mapInfo.width, m_mapInfo.height, binMap);//注意这里传入到DynamicVoronoi里并进行保存，所以没有delete
    voronoiDiagram.update();
    voronoiDiagram.visualize();//将Voronoi Diagram初始化、更新并显示
}

/**
 * @Brief : 起始点信息的订阅回调函数
 * @param  startData        
 */
void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& startData){

    // 变换为在data中的索引
    float x = startData->pose.pose.position.x / m_mapInfo.cellSize - m_mapInfo.origin_x;
    float y = startData->pose.pose.position.y / m_mapInfo.cellSize - m_mapInfo.origin_y;
    float theta = tf::getYaw(startData->pose.pose.orientation);
    
    // 验证是否有效(不越界)
    if(x >= 0 && y >= 0 && x < m_mapInfo.width && y < m_mapInfo.height){
        validStart = true;
        start_x = x;
        start_y = y;
        start_theta = Helper::normalizeHeadingRad(theta);
        
        cout<<"I got the valid start position"<<endl;

        plan();
    }
    else{
        cout<<"Invalid start position"<<endl;
    }
}

/**
 * @Brief : 目标点信息的订阅回调函数
 * @param  goalData         
 */
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& goalData){
    // position
    float x = goalData->pose.position.x / m_mapInfo.cellSize - m_mapInfo.origin_x;
    float y = goalData->pose.position.y / m_mapInfo.cellSize - m_mapInfo.origin_y;
    float theta = tf::getYaw(goalData->pose.orientation);

    if(x>=0 && y>=0 && x < m_mapInfo.width && y < m_mapInfo.height){
        validGoal = true;
        goal_x = x;
        goal_y = y;
        goal_theta = Helper::normalizeHeadingRad(theta);
        cout<<"I got the valid goal position"<<endl;

        plan();
    }
    else{
        cout<<"Invalid goal position"<<endl;
    }
}

//###################################################
//                                       LOOKUPTABLES
//###################################################
//初始化 查找表，主要有两个：Dubins Looup Table及Collision Looup Table
void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}


//###################################################
//                                      PLAN THE PATH
//###################################################
// !!! 核心函数，规划过程函数
void Planner::plan() {
  // if a start as well as goal are defined go ahead and plan
  if (validStart && validGoal) {
    int depth = Constants::headings;
    Node3D* nodes3D = new Node3D[m_mapInfo.width * m_mapInfo.height * depth]();
    Node2D* nodes2D = new Node2D[m_mapInfo.width * m_mapInfo.height]();

    Node3D nStart(start_x,start_y,start_theta,0,0,nullptr);
    const Node3D nGoal(goal_x,goal_y,goal_theta,0,0,nullptr);

    smoothedPath.updateMap(m_mapInfo);
    path.updateMap(m_mapInfo);
    visualization.updateMap(m_mapInfo);

    // ___________________________
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization.clear();
    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear();

    //核心步骤：
    // 1) 调用hybridAStar()函数获取一条路径
    // 2) 获取路径点(3D Node) -> 原始路径
    // 3) 对路径依据Voronoi图进行平滑->平滑路径
    // FIND THE PATH
    Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, m_mapInfo, 
    configurationSpace, dubinsLookup, visualization);
    // TRACE THE PATH
    smoother.tracePath(nSolution);
    // CREATE THE UPDATED PATH
    path.updatePath(smoother.getPath());
    // SMOOTH THE PATH
    smoother.smoothPath(voronoiDiagram);
    // CREATE THE UPDATED PATH
    smoothedPath.updatePath(smoother.getPath());
    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;

    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    //将结果在相应的topic进行发布
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();
    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();
    visualization.publishNode3DCosts(nodes3D, width, height, depth);
    visualization.publishNode2DCosts(nodes2D, width, height);

    delete [] nodes3D;
    delete [] nodes2D;

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}
