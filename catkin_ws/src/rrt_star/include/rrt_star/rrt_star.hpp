#pragma once
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <rrt_star_types.hpp>
#include <nav_msgs/Path.h>
#include <algorithm>

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
        ros::Publisher m_path_pub;

        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void GoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void getStartParam();
        void planPath();
        void generateRandomPoint();
        void drawStartAndGoal();
        void findNearestNode();
        void expandNearestNode();
        void printCurrentStatus();
        bool checkIfItsGoal();
        void backTraceThePath();
        void printFinalPath();
        void publishPath();
        bool checkIfThereIsObstacle();
        bool checkIFGoalIsVisible();
        void getNeighboursInKRadius();
        void getBestParent();
        double distanceBetween(const TreeNode &a, const TreeNode &b);
        void reWireTheTree();
        void initiateStartGoalNode();
        bool isInFreeSpace(TreeNode node);
        void addPointToMap(TreeNode node);
        void createNearestNode(TreeNode check_node);
        double angleBetween(TreeNode a, TreeNode b);
        void createNewNode();
        void removePointFromMap(TreeNode node);
        bool isConnectionPossible(TreeNode a, TreeNode b);
        void reCalculateTheCost();







        std::vector<TreeNode> path_of_nodes;

        int i = 0;

        bool m_have_map;
        bool m_have_current_pose;
        bool once_flag=true;
        bool first_time= false;
        bool end_planning = false;
        
        int max_x_dist;
        int max_y_dist;
        int min_x_dist;
        int min_y_dist;

        int start_grid_x;
        int start_grid_y;
        int goal_grid_x;
        int goal_grid_y;
        double Randomdouble(double a, double b);
        double dist_to_random_point;
        double euclidean_distance;
        double distance_to_neighbour;

        double step_distance = 0.05;
        double rand_pose_x;
        double rand_pose_y;
        double new_node_x;
        double new_node_y;
        int dimention_of_spcae = 2;
        double radius;

        int rand_grid_pose_x;
        int rand_grid_pose_y;

        int new_node_grid_x;
        int new_node_grid_y;

        double node_num;
        double angle;
        double goal_buffer = 0.1;



//      std::vector<TreeNode> Tree_nodes;
        std::vector<TreeNode> open_nodes;
        std::vector<TreeNode> close_nodes;
        std::map<int, TreeNode> Tree_nodes;

        std::vector<TreeNode> node_in_radius;


        TreeNode Nearest_node;
        TreeNode start;
        TreeNode goal;
        TreeNode newNode;
        TreeNode tempNode;
        TreeNode randomNode;

        TreeNode best_parent_node;

        double dist_to_goal;


        nav_msgs::OccupancyGrid::ConstPtr m_map;
        nav_msgs::OccupancyGrid m_map_new;
        geometry_msgs::PoseStamped m_goal_pose;
        geometry_msgs::PoseStamped m_current_pose;
    };


}
