#pragma once
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <rrt_star_types.hpp>

namespace RRT {

    class RRTStar
    {
    public:
        RRTStar(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        ~RRTStar();
    private:

        ros::Subscriber m_map_sub;
        ros::Subscriber m_goal_sub;
        ros::Publisher m_map_pub;

        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void GoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void getStartParam();
        void planPath();
        void generateRandomPoint();
        void drawStartAndGoal();

        int i = 0;

        bool m_have_map;
        bool m_have_current_pose;
        
        int max_x_dist;
        int max_y_dist;
        int min_x_dist;
        int min_y_dist;

        int start_grid_x;
        int start_grid_y;
        int goal_grid_x;
        int goal_grid_y;
        double Randomdouble(double a, double b);


        std::vector<TreeNode> Tree_nodes;


        nav_msgs::OccupancyGrid::ConstPtr m_map;
        nav_msgs::OccupancyGrid m_map_new;
        geometry_msgs::PoseStamped m_goal_pose;
        geometry_msgs::PoseStamped m_current_pose;
    };


}
