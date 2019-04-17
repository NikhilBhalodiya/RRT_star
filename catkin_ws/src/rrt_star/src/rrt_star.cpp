#include <rrt_star.hpp>

namespace RRT {

RRTStar::RRTStar(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &RRTStar::mapCallback, this);
    m_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &RRTStar::GoalPoseCallback, this);
    m_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map_new",10);


    getStartParam();
    std::srand(ros::Time::now().toSec());

}

RRTStar::~RRTStar(){}

void RRTStar::planPath()
{
 ROS_INFO("Path planning has been started");


 drawStartAndGoal();
 generateRandomPoint();

}

void RRTStar::drawStartAndGoal()
{
    m_map_new.data[(m_map->info.width*start_grid_y + start_grid_x)] = 100;
    m_map_new.data[(m_map->info.width*goal_grid_y + goal_grid_x)] = 100;
    m_map_pub.publish(m_map_new);
    ROS_INFO("Start and Goal published");
}

void RRTStar::generateRandomPoint()
{
//    max_x_dist = (m_map->info.width*m_map->info.resolution) - 6;
//    min_x_dist = (m_map->info.width*m_map->info.resolution) - 10;
//    max_y_dist = (m_map->info.height*m_map->info.resolution) - 6;
//    min_y_dist = (m_map->info.height*m_map->info.resolution) - 11;
//    int rand_grid_pose_x = std::rand() % 94 + 140;
//    int rand_grid_pose_y = std::rand() % 95 + 121;
//    double rand_pose_x = double(std::rand() % 4.0) + 7.0;
//    double rand_pose_y = double(std::rand() % 3.0) + 7.0;

    double rand_pose_x = Randomdouble(7,11.7);   // values Found by try and error from the map
    double rand_pose_y = Randomdouble(6.05,10.8);

//    ROS_INFO("X Y %f %f meters",rand_pose_x,rand_pose_y);

    int rand_grid_pose_x = int(rand_pose_x/m_map->info.resolution);
    int rand_grid_pose_y = int(rand_pose_y/m_map->info.resolution);

    ROS_INFO("Random Num generated X Y %d %d",rand_grid_pose_x,rand_grid_pose_y);

    if(m_map_new.data[(m_map->info.width*rand_grid_pose_y + rand_grid_pose_x)] == 0)
    {
        ROS_INFO("Is In free space %d %d",rand_grid_pose_x,rand_grid_pose_y);
        m_map_new.data[(m_map->info.width*rand_grid_pose_y + rand_grid_pose_x)] = 100;
        m_map_pub.publish(m_map_new);

//        if(i<=100)
//        {
//           generateRandomPoint();
//           i++;
//        }
    }
    else
    {
        ROS_INFO("Not in free space");
        generateRandomPoint();
    }

}

double RRTStar::Randomdouble(double a, double b)
{
    double random = ((double) rand()) / (double) RAND_MAX;
    double diff = b - a;
    double r = random * diff;
    return a + r;
}

void RRTStar::getStartParam()
{
  ROS_INFO("Start is taken");
  tf::Quaternion q;
  m_current_pose.pose.position.x = 10;
  m_current_pose.pose.position.y = 10;
  m_current_pose.pose.position.z = 0;
  m_have_current_pose = true;
   ROS_INFO("3");
}

void RRTStar::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if(!m_have_map)
    {
        m_have_map = true;
        ROS_INFO("Map has been taken");
    }
    m_map = msg;
    m_map_new = *m_map;

    start_grid_x = int(m_current_pose.pose.position.x/m_map->info.resolution);
    start_grid_y = int(m_current_pose.pose.position.y/m_map->info.resolution);
}

void RRTStar::GoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
   if(m_have_map && m_have_current_pose)
    {
        ROS_INFO("Global Goal Pose Updated Manually");
        m_goal_pose = *msg;
        goal_grid_x = int(m_goal_pose.pose.position.x/m_map->info.resolution) + 200;
        goal_grid_y = int(m_goal_pose.pose.position.y/m_map->info.resolution) + 200;
        ROS_INFO("G X G Y meters %d %d",int(m_goal_pose.pose.position.x)+10,int(m_goal_pose.pose.position.y)+10);
        ROS_INFO("G X G Y %d %d",goal_grid_x,goal_grid_y);
        planPath();
    }
}



















}
