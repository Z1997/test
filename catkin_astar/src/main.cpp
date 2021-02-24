#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <iostream>
#include"catkin_astar/Astar.h"
#include"catkin_astar/blockallocator.h"

std::vector<std::vector<bool>> grid;
float resolution = 0.0;
int cols,rows;
struct Point_{
    int x;
    int y;
};
Point_ init_p_to_map;
Point_ goal_p_to_map;
ros::Publisher path_pub_;


void iterationMapCallback(nav_msgs::OccupancyGrid msg)
{
    resolution = msg.info.resolution;
//    float origin_x  = msg.info.origin.position.x;
//    float origin_y = msg.info.origin.position.y;
    cols = msg.info.width;
    rows =msg.info.height;

    //resize
    grid.resize(rows);
    for (int i = 0; i < rows; i++) {
        grid[i].resize(cols);
    }

    int currCell = 0;
    for (int i = 0; i < rows; i++)  {
        for (int j = 0; j < cols; j++)      {
            if (msg.data[currCell] == 100 || msg.data[currCell] == -1) // occupied cell
                grid[i][j] = 1;
            else
                grid[i][j] = 0; // unoccupied-free (0) or unknown cell (-1)
            currCell++;
        }
    }
    std::cout<<"init map ok"<<std::endl;
}

void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    std::cout<<"init_pose:"<<msg->pose.pose.position.x<<","<<msg->pose.pose.position.y<<std::endl;
    if(resolution != 0.0)
    {
        init_p_to_map.x = ceil(msg->pose.pose.position.x/resolution);

        init_p_to_map.y = ceil(msg->pose.pose.position.y/resolution);
    }
}

void goal_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
     std::cout<<"end point:"<<msg->pose.position.x<<","<<msg->pose.position.y<<std::endl;
    if(resolution != 0.0&& grid.size() > 0)
    {

        goal_p_to_map.x = ceil(msg->pose.position.x/resolution);
        goal_p_to_map.y = ceil(msg->pose.position.y/resolution);

        // 搜索参数
        AStar::Params param;
        param.width = cols;
        param.height = rows;
        param.corner = true;
        param.start = AStar::Vec2(init_p_to_map.x, init_p_to_map.y);
        param.end = AStar::Vec2(goal_p_to_map.x, goal_p_to_map.y);
        param.can_pass = [&](const AStar::Vec2 &pos)->bool
        {
            return grid[pos.y][pos.x] == 0;//若x==y，在矩阵中赋值为0
        };

        // 执行搜索
        BlockAllocator allocator;
        AStar algorithm(&allocator);
        auto path_astar = algorithm.find(param);

        nav_msgs::Path path;
        ros::Time current_time, last_time;
        current_time = ros::Time::now();
        last_time = ros::Time::now();
        path.header.stamp=current_time;
        path.header.frame_id="odom";
        for(int i = 0;i < path_astar.size();i++)
        {
            current_time = ros::Time::now();
            //std::cout<<"i am in"<<std::endl;

            geometry_msgs::PoseStamped path_point;
            path_point.pose.position.x = path_astar[i].x*0.04;
            path_point.pose.position.y = path_astar[i].y*0.04;
            path_point.pose.orientation.x = 0;
            path_point.pose.orientation.y = 0;
            path_point.pose.orientation.z = 0;
            path_point.pose.orientation.w = 1.0;
            path_point.header.stamp=current_time;
            path_point.header.frame_id="odom";
            last_time = current_time;
            path.poses.push_back(path_point);
            std::cout<<path_point.pose.position.x <<","<<path_point.pose.position.y<<std::endl;
        }
        path_pub_.publish(path);


    }
    else
    {
        std::cout<<"No image or resolution invaliad:"<<grid.size()<<","<<resolution<<std::endl;
    }

}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "path_pub");
    ros::NodeHandle nh;
    //nh = new ros::NodeHandle();

    ros::Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid>("map",1,iterationMapCallback);
    ros::Subscriber init_pose_sub = nh.subscribe("initialpose", 10, initPoseCallback);
    ros::Subscriber goal_pose_sub = nh.subscribe("move_base_simple/goal", 10, goal_pose_callback);
    path_pub_ = nh.advertise<nav_msgs::Path>("/AStar_path_pub",10);

    //std::cout<<"initial funish!!!"<<std::endl;
    ros::spin();
    return 0;

}

