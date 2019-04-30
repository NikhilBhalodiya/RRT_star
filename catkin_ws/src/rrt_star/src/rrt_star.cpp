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
    node_num = 1;
    m_map_new.data[(m_map->info.width*start_grid_y + start_grid_x)] = 100;
    initiateStartGoalNode();

    node_num++;
    open_nodes.push_back(start);
    Tree_nodes.insert({int(node_num),start});

    m_map_new.data[(m_map->info.width*start_grid_y + start_grid_x)] = 100;
    m_map_new.data[(m_map->info.width*goal_grid_y + goal_grid_x)] = 100;
    m_map_pub.publish(m_map_new);

                                                                                                    //  ROS_INFO("Start and Goal published");
}

void RRTStar::initiateStartGoalNode()
{
    start.x_coordinate = m_current_pose.pose.position.x;
    start.y_coordinate = m_current_pose.pose.position.y;
    start.parent_id = 1;
    start.parent_node = &start;
    start.node_id = node_num;
    start.cost = 0.0;

    goal.x_coordinate = m_goal_pose.pose.position.x+10;
    goal.y_coordinate = m_goal_pose.pose.position.y+10;
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

    randomNode.x_coordinate = rand_pose_x;
    randomNode.y_coordinate = rand_pose_y;

    if(isInFreeSpace(randomNode))
    {
        addPointToMap(randomNode);
        findNearestNode();
        expandNearestNode();
    }
    else
    {
        generateRandomPoint();
    }
}

void RRTStar::addPointToMap(TreeNode node)
{
    int node_grid_x = int(node.x_coordinate/m_map->info.resolution);
    int node_grid_y = int(node.y_coordinate/m_map->info.resolution);
    m_map_new.data[(m_map->info.width*node_grid_y + node_grid_x)] = 100;
    m_map_pub.publish(m_map_new);
}

void RRTStar::removePointFromMap(TreeNode node)
{
    int node_grid_x = int(node.x_coordinate/m_map->info.resolution);
    int node_grid_y = int(node.y_coordinate/m_map->info.resolution);
    m_map_new.data[(m_map->info.width*node_grid_y + node_grid_x)] = 0;
    m_map_pub.publish(m_map_new);
}

bool RRTStar::isInFreeSpace(TreeNode node)
{
    int node_grid_x = int(node.x_coordinate/m_map->info.resolution);
    int node_grid_y = int(node.y_coordinate/m_map->info.resolution);
    return (m_map_new.data[(m_map->info.width*node_grid_y + node_grid_x)] == 0);
}

void RRTStar::findNearestNode()
{                                                                                          //ROS_INFO("started finding Nearest node");
    dist_to_random_point = INFINITY;
    euclidean_distance = 0;

    for(const auto &check_node : open_nodes)
    {
     euclidean_distance = distanceBetween(randomNode,check_node);
     if(dist_to_random_point > euclidean_distance)
     {
         dist_to_random_point = euclidean_distance;
         Nearest_node.x_coordinate = check_node.x_coordinate;
         Nearest_node.y_coordinate = check_node.y_coordinate;
         Nearest_node.parent_node = check_node.parent_node;
         Nearest_node.node_id = check_node.node_id;
         Nearest_node.parent_id = check_node.parent_id;
         Nearest_node.cost = check_node.cost;
                                                                                                //         ROS_INFO("Nearest Node is %IF %IF %IF",Nearest_node.x_coordinate,Nearest_node.y_coordinate,dist_to_random_point);
     }
  }
}

void RRTStar::expandNearestNode()
{
                                                                                                // ROS_INFO("expanding Nearest Node");
   angle = angleBetween(randomNode,Nearest_node);
   createNewNode();

   if(!end_planning)
   {
      if(checkIfItsGoal())
      {
          return;
      }
   }
   if(checkIfThereIsObstacle())
   {
       generateRandomPoint();
   }

   if(isInFreeSpace(newNode))     // m_map_new.data[(m_map->info.width*new_node_grid_y + new_node_grid_x)] == 0)
   {
       removePointFromMap(randomNode);
       addPointToMap(newNode);
       getNeighboursInKRadius();
       getBestParent();

        int count = 0;
        int size = node_in_radius.size();
       while (!(isConnectionPossible(best_parent_node,newNode)))
       {
           std::vector<TreeNode>::iterator position = std::find(node_in_radius.begin(), node_in_radius.end(),best_parent_node);
           if (position != node_in_radius.end())        // == myVector.end() means the element was not found
           {
               node_in_radius.erase(position);
           }
          getBestParent();
          count++;
          if(count>=size)
          {
              break;
          }
       }

       newNode.parent_node = &best_parent_node;
       newNode.parent_id = best_parent_node.node_id;
       newNode.cost = best_parent_node.cost + distanceBetween(newNode,best_parent_node);
       reWireTheTree();
//       reCalculateTheCost();
       open_nodes.push_back(newNode);

       Tree_nodes.insert({int(node_num),newNode});
   }
   else
   {
       removePointFromMap(randomNode);
       generateRandomPoint();
   }
}

void RRTStar::reCalculateTheCost()
{
    for(auto &node : open_nodes)
    {
        double new_cost = node.parent_node->cost + distanceBetween(*node.parent_node,node);
        ROS_INFO("node Id %d old cost %IF new Cost %IF",node.node_id, node.cost,new_cost);
        node.cost = new_cost;
    }
}


bool RRTStar::isConnectionPossible(TreeNode a, TreeNode b)
{
    double distance = distanceBetween(a,b);                                                                 // find the minimum ray_dist and put it instead of zero

    for(double inc = 0.05; inc<=distance ; inc+=0.05)
    {
        double check_x = a.x_coordinate + inc* cos(angle);
        double check_y = a.y_coordinate + inc* sin(angle);

        int check_grid_x = int(check_x/m_map_new.info.resolution);
        int check_grid_y = int(check_y/m_map_new.info.resolution);
        if(check_grid_x != new_node_grid_x && check_grid_y != new_node_grid_y)
        {
            if(m_map_new.data[check_grid_y*m_map->info.width + check_grid_x] == 100)
            {
                return false;
            }
        }
    }
    return true;
}

void RRTStar::createNewNode()
{
    new_node_x = Nearest_node.x_coordinate + step_distance*cos(angle);
    new_node_y = Nearest_node.y_coordinate + step_distance*sin(angle);

    new_node_grid_x = int(new_node_x/m_map->info.resolution);
    new_node_grid_y = int(new_node_y/m_map->info.resolution);

    node_num++;
    newNode.node_id = int(node_num);
    newNode.parent_id = Nearest_node.node_id;
    newNode.x_coordinate = new_node_x;
    newNode.y_coordinate = new_node_y;
    newNode.parent_node = &Nearest_node;
    newNode.cost =  Nearest_node.cost + distanceBetween(newNode,Nearest_node);
}


double RRTStar::angleBetween(TreeNode a, TreeNode b)
{
    return atan2(rand_pose_y - Nearest_node.y_coordinate ,rand_pose_x - Nearest_node.x_coordinate);
}

void RRTStar::reWireTheTree()
{
    for(auto &check_node : node_in_radius)
    {
        double newCost = newNode.cost + distanceBetween(newNode,check_node);
        if(newCost < check_node.cost)
        {
            check_node.parent_node = &newNode;
            check_node.parent_id = newNode.node_id;
        }
    }

}

double RRTStar::distanceBetween(const TreeNode &a,const TreeNode &b)
{
  return std::sqrt(std::pow(a.x_coordinate - b.x_coordinate,2) + std::pow(a.y_coordinate - b.y_coordinate,2));
}

void RRTStar::getBestParent()
{
     best_parent_node = Nearest_node;
    for(const auto &check_node : node_in_radius)
      {
        double min_cost = check_node.cost + distanceBetween(newNode,check_node);
        if(min_cost < newNode.cost)
        {
            best_parent_node = check_node;
            newNode.cost  = min_cost ;
        }

    }
}

void RRTStar::getNeighboursInKRadius()
{
    node_in_radius.clear();
    radius =std::pow((std::log(node_num)/node_num),(1.0/dimention_of_spcae));

    for(const auto &check_node : open_nodes)
    {
        distance_to_neighbour =distanceBetween(newNode, check_node);
        if(distance_to_neighbour < radius)
        {
          node_in_radius.push_back(check_node);
        }
    }
}

bool RRTStar::checkIfThereIsObstacle()
{
                                                                                                        //  ROS_INFO("checking before making");
    double ray_dist = step_distance;                                                                 // find the minimum ray_dist and put it instead of zero

    for(double inc = 0.05; inc<=ray_dist ; inc+=0.05)
    {
        double check_x = new_node_x + inc* cos(angle);
        double check_y = new_node_y + inc* sin(angle);

        int check_grid_x = int(check_x/m_map_new.info.resolution);
        int check_grid_y = int(check_y/m_map_new.info.resolution);
        if(check_grid_x != new_node_grid_x && check_grid_y != new_node_grid_y)
        {
            if(m_map_new.data[check_grid_y*m_map->info.width + check_grid_x] == 100)
            {
                return true;
            }
        }
    }
    return false;

}
bool RRTStar::checkIFGoalIsVisible()
{
    double angleToGoal = angleBetween(goal,newNode);
    for(double inc = 0.05; inc<=goal_buffer ; inc+=0.05)
    {
        double check_x = new_node_x + inc* cos(angleToGoal);
        double check_y = new_node_y + inc* sin(angleToGoal);

        int check_grid_x = int(check_x/m_map_new.info.resolution);
        int check_grid_y = int(check_y/m_map_new.info.resolution);
        if(check_grid_x != new_node_grid_x && check_grid_y != new_node_grid_y)
        {
            if(m_map_new.data[check_grid_y*m_map->info.width + check_grid_x] == 100)
            {
                                                                                                    //                ROS_INFO("obstacles inbetween");
                return true;
            }
        }
    }
    return false;

}

bool RRTStar::checkIfItsGoal()
{
    if(checkIFGoalIsVisible())
    {
                                                                                                //        ROS_INFO("goal is not visible");
        return false;
    }
    dist_to_goal = distanceBetween(newNode,goal);

    if(dist_to_goal <= goal_buffer)
    {
        ROS_INFO("GGGGGGGOOOOOOOOOOOOOOOOOOOAAAAAAAAAAAAAAALLLLLLLLLLLLLLLLLl");
        goal.node_id = int(node_num) + 1;
        goal.parent_id = int(node_num);
        goal.parent_node = &newNode;
        goal.cost = newNode.cost;
        ROS_INFO("goal cost is %IF",goal.cost);
        backTraceThePath();
        end_planning = true;
        return true;
    }
    return false;
}

void RRTStar::backTraceThePath()
{
    path_of_nodes.clear();
     tempNode = *goal.parent_node;
     path_of_nodes.push_back(goal);
                                                                                        //     ROS_INFO("goal is now temp node and its id %d ",goal.node_id);
     while(tempNode.parent_id != 1)
     {
                                                                                        //         ROS_INFO("temp_node parent id %d ",tempNode.parent_id);
         path_of_nodes.push_back(tempNode);
         tempNode = Tree_nodes[tempNode.parent_id];
     }
                                                                                         //     path_of_nodes.push_back(start);
     printFinalPath();
     publishPath();
}


void RRTStar::printFinalPath()
{
    ROS_INFO("Final Path");
    for (auto const& i: path_of_nodes) {
                    ROS_INFO("node id is %d and its parent is %d,",i.node_id,i.parent_id);
            }
}

void RRTStar::publishPath()
{
    nav_msgs::Path path;
    path.header.frame_id = "/map";
    for(int traj_it = 0; traj_it < path_of_nodes.size(); traj_it++)
    {
        double x = path_of_nodes[traj_it].x_coordinate;
        double y = path_of_nodes[traj_it].y_coordinate;
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x - 10.0;
        pose.pose.position.y = y - 10.0;
        path.poses.push_back(pose);
    }
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
}

void RRTStar::printCurrentStatus()
{
    ROS_INFO("OPEN NODES");
    for (auto const& i: open_nodes) {
                    ROS_INFO("%d, ",i.node_id);
            }
    ROS_INFO("Node in Radius");
    for (auto const& i: node_in_radius) {
                    ROS_INFO("%d,",i.node_id);
            }
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
        planPath();
        first_time = true;
       }
       else
       {
         for(int num = 0; num<200; num++)
         {
          generateRandomPoint();
         }
         backTraceThePath();


       }
    }

}

}
