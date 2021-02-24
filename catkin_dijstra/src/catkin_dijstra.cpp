#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <iostream>
#include<catkin_dijstra/dijstra.h>
#include<catkin_dijstra/blockallocator.h>
#include <fstream>

std::ofstream debug_data("/home/robot/catkin_test/debug_data.txt",std::ios::out);

std::vector<std::vector<bool> > grid;
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
    cols = msg.info.width;
    rows =msg.info.height;
    int matrix_ite = rows * cols;
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

/*std::vector<std::vector<float> >edges;
    edges.resize(matrix_ite);
    std::vector<float> edges_b(matrix_ite);
    for(int i = 0;i < rows;i++)
    {
        edges[i].resize(matrix_ite);
        for(int j = 0;j<cols;j++)
        {

            //邻接矩阵填充
            int q = 0;
            if(i == j && msg.data[i*rows+j] == 0)
            {
                edges[i][j] = 100.0;
                //edges_b.insert(edges_b.begin()+j,100.0);
            }

            if(j!=cols&& msg.data[i*rows + j + 1] == 0)
            {
                q = i*rows + j + 1;
                edges[i][q] = 0.04;
                //edges_b.insert(edges_b.begin()+q,0.04);
            }
            if(j!=0&& msg.data[i*rows + j - 1] == 0)
            {
                q = i*rows + j - 1;
                edges[i][q] = 0.04;
                //edges_b.insert(edges_b.begin()+q,0.04);
            }
            if(i!=rows && msg.data[(i+1)*rows + j] == 0)
            {
                q = (i+1)*rows + j;
                edges[i][q] = 0.04;
                //edges_b.insert(edges_b.begin()+q,0.04);
            }
            if(i!=0 && msg.data[(i-1)*rows + j] == 0)
            {
                q = (i-1)*rows + j;
                edges[i][q] = 0.04;
                //edges_b.insert(edges_b.begin()+q,0.04);
            }

            std::cout<<"j"<<j<<std::endl;
            std::cout<<"i"<<i<<std::endl;

            debug_data<<"edges:";
            for(int p=0;p<matrix_ite;p++)
            {
                debug_data<< edges[i][p]<<" ";
                //edges_b[p] = 0;
            }
            debug_data<<std::endl;

            //vector<float>::iterator tier=edges_b.begin();
            //edges[i][j] = *tier;
            //edges[i].push_back(edges_b);//此时edges_b中0为不可到达点，0.04为可到达点
            //edges_b.clear();//清空vector数据，数据重置为0
            //std::cout<<"edges.size_j()"<<edges.size()<<std::endl;
        }
        //std::cout<<"edges.size_i()"<<edges.size()<<std::endl;
    }


    //把之前矩阵中的0置为inf表示障碍物或非相邻的像素格,incomplete
    for(int i =0;i<matrix_ite; i++)
    {
        for(int j = 0; j < matrix_ite;j++)
        {
            debug_data<<edges[i][j]<<" ";
        }
        debug_data<<std::endl;
    }
*/


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

        Dijkstra::Params param;
        param.width = cols;
        param.height = rows;
        param.corner = true;
        param.start = Dijkstra::Vec2(init_p_to_map.x, init_p_to_map.y);
        param.end = Dijkstra::Vec2(goal_p_to_map.x, goal_p_to_map.y);
        param.can_pass = [&](const Dijkstra::Vec2 &pos)->bool
        {
            return grid[pos.y][pos.x] == 0;//若x==y，在矩阵中赋值为0
        };

        // 执行搜索
        BlockAllocator allocator;
        Dijkstra algorithm(&allocator);
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
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "D_path_pub");
    ros::NodeHandle nh;
    //nh = new ros::NodeHandle();

    ros::Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid>("map",1,iterationMapCallback);



    ros::Subscriber init_pose_sub = nh.subscribe("initialpose", 10, initPoseCallback);
    ros::Subscriber goal_pose_sub = nh.subscribe("move_base_simple/goal", 10, goal_pose_callback);
    path_pub_ = nh.advertise<nav_msgs::Path>("/Dijstra_path_pub",10);

    //std::cout<<"initial funish!!!"<<std::endl;
    ros::spin();
    return 0;

}
