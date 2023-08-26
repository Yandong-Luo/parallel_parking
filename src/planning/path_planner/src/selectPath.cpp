#include <ros/ros.h>
#include <std_msgs/Bool.h>

int main(int argc, char** argv) {
    // initiate the broadcaster
    ros::init(argc, argv, "selectpath");

    ros::NodeHandle nh;

    ros::Publisher pubGetPath = nh.advertise<std_msgs::Bool>("/getFinalPath",10,true);

    std_msgs::Bool getFinalData;

    getFinalData.data = true;

    pubGetPath.publish(getFinalData);

    while(pubGetPath.getNumSubscribers()<1);

    ros::spin();
}