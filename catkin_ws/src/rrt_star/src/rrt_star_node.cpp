#include <ros/ros.h>
#include <rrt_star.hpp>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "rrt_star_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    RRT::RRTStar rrtStar(nh, pnh);
    while(ros::ok)
    {
        ROS_INFO("Spinning");
        ros::spin();
    }

    return 0;
}
