#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <iostream>
#include <fstream>   
using namespace std;
std::vector<std::vector<bool> > grid;

bool start_map_server_node(std::string map_file)
{
	std::string stop_cmd = "pkill -9 map_server";
	std::system(stop_cmd.c_str());

    map_file = map_file + ".pgm";
    std::string cmd = "rosrun map_server map_server " + map_file+" 0.04";
    std::string start_cmd = std::string("gnome-terminal --geometry=50x20+10+10 -x  bash -c \" ") + cmd + std::string(" \"");
	std::system(start_cmd.c_str());
	return true;
}

void iterationMapCallback(nav_msgs::OccupancyGrid msg)
{
    float resolution = msg.info.resolution;
    int cols = msg.info.width;
    int rows =msg.info.height;
    std::cout<<"width:"<<cols<<std::endl;
    std::cout<<"height:"<<rows<<std::endl;

    //resize
    grid.resize(rows);
    for (int i = 0; i < rows; i++) {
        grid[i].resize(cols);
    }

    int currCell = 0;
    for (int i = 0; i < rows; i++)  {
        for (int j = 0; j < cols; j++)      {
            if (msg.data[currCell] == 100) // occupied cell
                grid[i][j] = true;
            else
                grid[i][j] = false; // unoccupied-free (0) or unknown cell (-1)
            currCell++;
        }
    }
 /*
    for(int i= 0; i<rows;i++)
    {
        for(int j = 0;j<cols;j++)
        {
            std::cout<<grid[i][j]<<" ";
        }
       std::cout<<std::endl;
    }
*/
    std::vector<bool>pointInEx;
    std::vector<bool>::iterator it;
    float x,y;
    float area;
    printf("请输入点坐标： ");
    scanf("%f,%f", &x,&y);
    printf("请输入膨胀距离/m： ");
    scanf("%f", &area);
    int map_x = ceil(x/resolution);
    int map_y = ceil(y/resolution);
    std::cout<<"map_x = "<<map_x<<",map_y = "<<map_y<<endl;
    int map_inflation = ceil(area/resolution);
    int r_2 = map_inflation*map_inflation;
    int i,j;
    for(i = -map_inflation;i< map_inflation;i++)
    {
        for(j = -map_inflation;j<map_inflation;j++)
        {

            int a = map_x+i;
            int b = map_y+j;
            int circle_ = i*i+j*j;
            if(circle_ < r_2 ||circle_ == r_2)
            {
                if(grid[b][a] == true)
                {
                    //std::cout<<"in*************************"<<std::endl;
                    pointInEx.push_back(grid[b][a]);
                    break;
                }
                else
                    continue;
            }

        }
    }
//    if(i==map_inflation && j==map_inflation)
//    {
//        std::cout<<"out****************************"<<std::endl;
//    }
    it = find(pointInEx.begin(),pointInEx.end(),true);
    if(it != pointInEx.end())
    {
        std::cout<<"in*************************"<<std::endl;
    }
    else
    {
        std::cout<<"out****************************"<<std::endl;
    }


}


int main(int argc, char **argv)
{
    std::string str;
    ros::init(argc, argv, "getPoint");
    ros::NodeHandle n_param("~");
    //n_param.param<std::string>("map_name",str,"map1");

    //bool start_map_server = start_map_server_node("map1");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid>("map",1,iterationMapCallback);

    ros::spin();
	return 0;
}


