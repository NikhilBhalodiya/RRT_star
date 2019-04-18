#include <rrt_star.hpp>

namespace RRT {

RRTStar::RRTStar(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &RRTStar::mapCallback, this);
    m_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &RRTStar::GoalPoseCallback, this);
    m_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map_new",10);
    m_path_pub = nh.advertise<nav_msgs::Path>("/global_path", 10);

    getStartParam();
    std::srand(ros::Time::now().toSec());
}

RRTStar::~RRTStar(){}

void RRTStar::planPath()
{
//ROS_INFO("Path planning has been started");
 drawStartAndGoal();
 while(!end_planning)
 {
 generateRandomPoint();
 }
}

void RRTStar::drawStartAndGoal()
{
    node_num = 0;
    m_map_new.data[(m_map->info.width*start_grid_y + start_grid_x)] = 100;
    TreeNode start(m_current_pose.pose.position.x,m_current_pose.pose.position.y,0);
    start.parent_node = &start;
    start.node_id = 1;
    open_nodes.push_back(start);
    node_num++;
    Tree_nodes.insert({int(node_num),start});
    m_map_new.data[(m_map->info.width*goal_grid_y + goal_grid_x)] = 100;
    goal.x_coordinate = m_goal_pose.pose.position.x+10;
    goal.y_coordinate = m_goal_pose.pose.position.y+10;
    m_map_pub.publish(m_map_new);
//  ROS_INFO("Start and Goal published");
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

    rand_pose_x = Randomdouble(7,11.7);   // values Found by try and error from the map
    rand_pose_y = Randomdouble(6.05,10.8);

    rand_grid_pose_x = int(rand_pose_x/m_map->info.resolution);
    rand_grid_pose_y = int(rand_pose_y/m_map->info.resolution);

    if(m_map_new.data[(m_map->info.width*rand_grid_pose_y + rand_grid_pose_x)] == 0)
    {
//      ROS_INFO("Is In free space %d %d",rand_grid_pose_x,rand_grid_pose_y);
        m_map_new.data[(m_map->info.width*rand_grid_pose_y + rand_grid_pose_x)] = 100;
        m_map_pub.publish(m_map_new);

        expandtree();

    }
    else
    {
        generateRandomPoint();
    }

}

void RRTStar::expandtree()
{
//   ROS_INFO("expandtree initiated");
   findNearestNode();
   expandNearestNode();
}


void RRTStar::findNearestNode()
{
//    ROS_INFO("Open_nodes size  %d",open_nodes.size());

dist_to_random_point = INFINITY;
euclidean_distance = 0;

ROS_INFO("started finding Nearest node");

  for(const auto &check_node : open_nodes)
  {
     euclidean_distance = std::sqrt(std::pow(rand_pose_x-check_node.x_coordinate,2) + std::pow(rand_pose_y-check_node.y_coordinate,2));
     if(dist_to_random_point > euclidean_distance)
     {
         dist_to_random_point = euclidean_distance;
         Nearest_node.x_coordinate = check_node.x_coordinate;
         Nearest_node.y_coordinate = check_node.y_coordinate;
         Nearest_node.parent_node = check_node.parent_node;
         Nearest_node.node_id = check_node.node_id;
         Nearest_node.parent_id = check_node.parent_id;
//         ROS_INFO("Nearest Node is %IF %IF %IF",Nearest_node.x_coordinate,Nearest_node.y_coordinate,dist_to_random_point);
     }
  }
}

void RRTStar::expandNearestNode()
{
// ROS_INFO("expanding Nearest Node");
   angle = atan2(rand_pose_y - Nearest_node.y_coordinate ,rand_pose_x - Nearest_node.x_coordinate);
   new_node_x = Nearest_node.x_coordinate + step_distance*cos(angle);
   new_node_y = Nearest_node.y_coordinate + step_distance*sin(angle);

   new_node_grid_x = int(new_node_x/m_map->info.resolution);
   new_node_grid_y = int(new_node_y/m_map->info.resolution);

//   ROS_INFO("%d %d",new_node_grid_x,new_node_grid_y);

   //   TreeNode newNode(new_node_x,new_node_y);
   node_num++;
   newNode.node_id = int(node_num);
   newNode.parent_id = Nearest_node.node_id;
   newNode.x_coordinate = new_node_x;
   newNode.y_coordinate = new_node_y;
   newNode.parent_node = Nearest_node.parent_node;
   if(checkIfItsGoal())
   {
       return;
   }
   if(checkIfThereIsObstacle())
   {
       generateRandomPoint();
   }

   if(m_map_new.data[(m_map->info.width*new_node_grid_y + new_node_grid_x)] == 0)
   {

//       ROS_INFO("New_node In free space x y are %d %d",new_node_grid_x,new_node_grid_y);
       m_map_new.data[(m_map->info.width*new_node_grid_y + new_node_grid_x)] = 100;
       m_map_pub.publish(m_map_new);
       ros::Duration(0.05).sleep();
       m_map_new.data[(m_map->info.width*rand_grid_pose_y + rand_grid_pose_x)] = 0;
       m_map_pub.publish(m_map_new);

//      ROS_INFO("new node x y %IF %IF generated with id and node Num %d %IF ",new_node_x,new_node_y, newNode.node_id,node_num);

//      close_nodes.push_back(Nearest_node);

      open_nodes.push_back(newNode);
      Tree_nodes.insert({int(node_num),newNode});
   }
   else
   {
       m_map_new.data[(m_map->info.width*rand_grid_pose_y + rand_grid_pose_x)] = 0;
       m_map_pub.publish(m_map_new);
//     ROS_INFO("New node in obstacle");
       generateRandomPoint();
   }

}

bool RRTStar::checkIfThereIsObstacle()
{
//  ROS_INFO("checking before making");
    double ray_dist = step_distance;   // find the minimum ray_dist and put it instead of zero

    for(double inc = 0.05; inc<=ray_dist ; inc+=0.05)
    {
//        ROS_INFO("inc %IF",inc);
        double check_x = new_node_x + inc* cos(angle);
        double check_y = new_node_y + inc* sin(angle);

        int check_grid_x = int(check_x/m_map_new.info.resolution);
        int check_grid_y = int(check_y/m_map_new.info.resolution);
        if(check_grid_x != new_node_grid_x && check_grid_y != new_node_grid_y)
        {
            if(m_map_new.data[check_grid_y*m_map->info.width + check_grid_x] == 100)
            {
//                ROS_INFO("obstacles");
                return 1;
            }
        }
    }
    return 0;

}
bool RRTStar::checkIFGoalIsVisible()
{
      // find the minimum ray_dist and put it instead of zero
    /////////////////////////////////////////////////////////////////////////////////////////
    double angleToGoal = atan2(goal.y_coordinate - new_node_y, goal.x_coordinate - new_node_x);

    for(double inc = 0.05; inc<=goal_buffer ; inc+=0.05)
    {
        double check_x = new_node_x + inc* cos(angleToGoal);
        double check_y = new_node_y + inc* sin(angleToGoal);

        int check_grid_x = int(check_x/m_map_new.info.resolution);
        int check_grid_y = int(check_y/m_map_new.info.resolution);
        if(check_grid_x != new_node_grid_x && check_grid_y != new_node_grid_y)
        {
//            ROS_INFO("new node to goal one step clear");
            if(m_map_new.data[check_grid_y*m_map->info.width + check_grid_x] == 100)
            {
                ROS_INFO("obstacles inbetween");
                return 1;
            }
        }
    }
    return 0;

}

bool RRTStar::checkIfItsGoal()
{
    if(checkIFGoalIsVisible())
    {
        ROS_INFO("goal is not visible");
        return false;
    }
//    ROS_INFO("new node x y %IF %IF goal x y %IF %IF ",new_node_x,new_node_y,goal.x_coordinate,goal.y_coordinate);
    dist_to_goal = std::sqrt(std::pow(new_node_x - goal.x_coordinate,2) + std::pow(new_node_y - goal.y_coordinate,2));



    if(dist_to_goal <= goal_buffer)
    {
        ROS_INFO("Last_node near goal x y %IF %IF",newNode.x_coordinate,newNode.y_coordinate);

        ROS_INFO("GGGGGGGOOOOOOOOOOOOOOOOOOOAAAAAAAAAAAAAAALLLLLLLLLLLLLLLLLl");
        goal.node_id = int(node_num)+1;
        goal.parent_id = int(node_num);
        goal.parent_node = &newNode;
        ROS_INFO("1");
        backTraceThePath();
        end_planning = true;
        return true;
    }
    return false;
}

void RRTStar::backTraceThePath()
{

     tempNode = *goal.parent_node;
     path_of_nodes.push_back(goal);
//     ROS_INFO("goal is now temp node and its id %d ",goal.node_id);
     while(tempNode.parent_id != 0)
     {
//         ROS_INFO("temp_node parent id %d ",tempNode.parent_id);
         path_of_nodes.push_back(tempNode);
         tempNode = Tree_nodes[tempNode.parent_id];  //    *tempNode.parent_node;
     }
//     path_of_nodes.push_back(start);
     printFinalPath();
}


void RRTStar::printFinalPath()
{
//    ROS_INFO("Final Path");
    for (auto const& i: path_of_nodes) {
//                    ROS_INFO("node id is %d and its parent is %d,",i.node_id,i.parent_id);
            }
    publishPath();
}

void RRTStar::publishPath()
{
    nav_msgs::Path path;
//    path.header.stamp = m_goal_pose.header.stamp;
    path.header.frame_id = "/map";
    for(int traj_it = 0; traj_it < path_of_nodes.size(); traj_it++)
    {
//        const double &x = traj[traj_it].child_point.x;
//        const double &y = traj[traj_it].child_point.y;
        double x = path_of_nodes[traj_it].x_coordinate;
        double y = path_of_nodes[traj_it].y_coordinate;

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x - 10.0;
        pose.pose.position.y = y - 10.0;
        path.poses.push_back(pose);
    }
//    path.poses.back().pose.orientation = tf::createQuaternionMsgFromYaw(m_goal_pose.heading);
    m_path_pub.publish(path);
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

void RRTStar::printCurrentStatus()
{
    ROS_INFO("OPEN NODES");
    for (auto const& i: open_nodes) {
                    ROS_INFO("%IF %IF, ",i.x_coordinate,i.y_coordinate);
            }
//    ROS_INFO("Close NODes");
//    for (auto const& i: close_nodes) {
//                    ROS_INFO("%IF %IF,",i.x_coordinate,i.y_coordinate);
//            }
//    ROS_INFO("Tree Nodes");
//    for (auto const& i: Tree_nodes) {
//                    ROS_INFO("%IF %IF,",i.x_coordinate,i.y_coordinate);
//            }

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
       if(!first_time)
       {
        ROS_INFO("Global Goal Pose Updated Manually");
        m_goal_pose = *msg;
        goal_grid_x = int(m_goal_pose.pose.position.x/m_map->info.resolution) + 200;
        goal_grid_y = int(m_goal_pose.pose.position.y/m_map->info.resolution) + 200;
        ROS_INFO("G X G Y meters %d %d",int(m_goal_pose.pose.position.x)+10,int(m_goal_pose.pose.position.y)+10);
        ROS_INFO("G X G Y %d %d",goal_grid_x,goal_grid_y);
        planPath();
        first_time = true;
       }
       else
       {
         for(int k = 0; k<1000; k++)
         {
        generateRandomPoint();
         }
       }
    }

}



















}
