#include "ros/ros.h"
#include <catkin_wheel/SH_Circle.h> 
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <time.h>
#include <string>
#include <unistd.h>


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
double radius;	 		//半径
double y_need;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_listener");
        ros::NodeHandle n;
        ros::NodeHandle nh("~");
        nh.param<double>("radius",radius,0.05);
        nh.param<double>("y_need",y_need,0.2);
        ros::Subscriber sub = n.subscribe("/scan", 1, laserCallback);
    	ros::spin();
	return 0;
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    vector<SH_Point> circles;
    std::vector<float> laser;
    std::vector<float> deep;
    std::vector<float> angle_;
    std::vector<float> range_ = msg->ranges;

	double deep_threshold  = 0.05;   //0.05
    double angle_step  = msg->angle_increment;//角分辨率 0.00581718236208
    double angle_start = msg->angle_min ;//最小角-2.35572338104
    double angle_end   = msg->angle_max;//最大角 2.35572338104
    float angle_pre = angle_start;
    int datasize  = floor(angle_end/angle_step);

    while(angle_pre<angle_end)
    {

        angle_pre += angle_step;
        angle_.push_back(angle_pre);

    }

    for(int i=0;i<angle_.size();i++)
    {
        if(i<datasize)
        {
            //laser.push_back(angle_[i]);
            //deep.push_back(range_[i]);
            laser = angle_;
            deep = range_;

        }
        else
        {
            angle_.clear();
            range_.clear();
            continue;
        }
    }
    long start, end;
    start = clock();
    circles = SH_Circle::get_circles(laser,deep , radius, deep_threshold, angle_step, angle_start,y_need);
    end = clock();
    cout << "time  cost: " << end - start << endl;


    for (int i = 0; i < circles.size(); i++)

    {
        circles[i].b = circles[i].b*circles[i].c;

        cout << "x: " << circles[i].x << ", y: " << circles[i].y <<  ", believe: " << circles[i].b<< ", circle count: "<<circles[i].c<<endl;
        myfile_circle <<"x: " << circles[i].x << ", y: " << circles[i].y <<  ", believe: " << circles[i].b<< ", circle count: "<<circles[i].c<<endl;
        myfile_py_circle <<circles[i].x<<","<<circles[i].y<<endl;
    }

    system("pause");
    //pause();
    deep.clear();
    laser.clear();



}


