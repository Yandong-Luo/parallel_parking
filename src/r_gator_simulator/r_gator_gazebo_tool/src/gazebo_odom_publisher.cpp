#include <ros/ros.h>
#include <string.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>

using namespace std;
using namespace ros;

class Gazebo_Odometry
{
public:
    Gazebo_Odometry();
    bool check_robot_name();
    void ModelStatecallback(const gazebo_msgs::ModelStates& box_state_current);
    void pub_odom();

    // tf广播
    // static tf::TransformBroadcaster odom_broadcaster;
private:
    NodeHandle node;

    // 参数
    string robot_name;  // 机器人在gazebo里的名称
    string odom_topic;  // 发布的话题
    // 里程计的下一级
    string robot_frame_id;
    string hz; // 话题发布的频率

    bool robot_exist;

    ServiceClient states_client;
    gazebo_msgs::GetModelState model_states;
    nav_msgs::Odometry odom;

    Publisher odometry_pub;

    tf::TransformBroadcaster odom_broadcaster;

    tf::Transform transform;
    tf::Quaternion quaternion;
};

// 订阅"/gazebo/model_states"的回调
void Gazebo_Odometry::ModelStatecallback(const gazebo_msgs::ModelStates& model_state_current)
{
    // 一个模型都没有
    if(model_state_current.name.size()==0)
    {
        return;
    }
    // 更新从系统中获取模型的名字
    for(int i=0;i<model_state_current.name.size();i++)
    {
        // 存在模型的名称和机器人名称相同（已经加载出了机器人）
        if(model_state_current.name[i].compare(robot_name) == 0)
        {
            robot_exist = true;
        }
    }
}

Gazebo_Odometry::Gazebo_Odometry()
{
    NodeHandle nh("~");

    // 获取要读取的机器人名称，从而获得对应的信息
    nh.param<string>("robot_name",robot_name,"r_gator");
    nh.param<string>("odom_topic",odom_topic,"/odom");
    // nh.param<string>("global_frame_id",global_frame_id,"/map");
    nh.param<string>("robot_frame_id",robot_frame_id,"/base_footprint");
    nh.param<string>("Frequency_of_odom_publisher",hz,"10");

    // 初始赋值，默认机器人尚未加载出来
    robot_exist = false;

    // 读取机器人名称对应的机器人状态
    states_client = node.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    // 发布
    odometry_pub = node.advertise<nav_msgs::Odometry>(odom_topic,atoi(hz.c_str()));
    // cout<<"发布的频率："<<atoi(hz.c_str())<<endl;
}


// 返回机器人模型是否已经加载的阈值
bool Gazebo_Odometry::check_robot_name()
{
    return robot_exist;
}

// 模型加载完成后，才对其服务器进行调用
void Gazebo_Odometry::pub_odom()
{   
    // cout<<"发布发布发布"<<endl;
    // 传入机器人模型名称
    model_states.request.model_name = robot_name;

    // 参考坐标系
    model_states.request.relative_entity_name = "world";

    // 客户端发送请求，读取gazebo世界中的
    states_client.call(model_states);

    odom.pose.pose = model_states.response.pose;    // 位置
    // cout<<"robot_odom_pose:"<<model_states.response.pose<<endl;
    odom.twist.twist = model_states.response.twist; // twist
    bool success = model_states.response.success;   // 是否接收到服务器的回应

    odometry_pub.publish(odom);

    tf::quaternionMsgToTF(odom.pose.pose.orientation,quaternion);   // quaternion msg转tf
    transform.setOrigin(tf::Vector3(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z));
    transform.setRotation(quaternion);
    // 广播odom到机器人坐标系
    odom_broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time::now(),odom_topic.c_str(),robot_frame_id.c_str()));
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_odom_publisher");

    Gazebo_Odometry m_gazebo_odom;

    ros::NodeHandle n;

    // 订阅
    ros::Subscriber state_listener = n.subscribe("/gazebo/model_states",20,&Gazebo_Odometry::ModelStatecallback,&m_gazebo_odom);

    ros::Rate loop_rate(10);

    bool first_time = true;

    while(ros::ok() )
    {   
        // 模型已经加载好了
        if(m_gazebo_odom.check_robot_name())
        {
            m_gazebo_odom.pub_odom();
        }
        else
        {
            if(first_time){
                first_time = true;
                cout<<"机器人未彻底加载，请稍等，若过长时间仍然无法连上，则请检查robot_name在gazebo中是否存在"<<endl;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    // ros::spin();
    return 0;
}