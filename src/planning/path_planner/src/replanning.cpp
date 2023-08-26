#include <ros/ros.h>
#include <std_msgs/Bool.h>

int main(int argc, char** argv) {
    // initiate the broadcaster
    ros::init(argc, argv, "replanning");

    ros::NodeHandle nh;

    ros::Publisher pubReplan = nh.advertise<std_msgs::Bool>("/replanning",10,true);

    std_msgs::Bool replan_data;

    replan_data.data = true;

    pubReplan.publish(replan_data);

    while(pubReplan.getNumSubscribers()<1);

    ros::spin();
}