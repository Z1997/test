#pragma once
#include <iostream>
#include <vector>
#include <fstream>
using namespace std;

const float ZERO_EPSILON = 0.01;
std::ofstream myfile_circle("/home/robot/catkin_wheel/myfile_circle.txt",std::ios::out);
std::ofstream myfile_py_circle("/home/robot/catkin_wheel/myfile_py_circle.txt",std::ios::out);
struct SH_Point
{
	double x = 0.0;   //y
	double y = 0.0;   //x
	double b = 0.0;   //believe
	int c = 0;      //count
	double rr = 0.0;//radius
};


class SH_Circle
{
public:
	SH_Circle();
	~SH_Circle();

public:
	//static SH_Point get_circle(SH_Point point1, SH_Point point2, SH_Point point3);
	//static int if_exist(vector<SH_Point> circles, SH_Point point, double threshold);
	//static bool if_valid(SH_Point center, SH_Point point1, SH_Point point2, SH_Point point3, double radius);
	//static vector<SH_Point> find_circle(vector<SH_Point> points, double radius, double threshold);
    //static std::vector<SH_Point> get_circles(double *laser, double* deep, double radius, double deep_threshold, double laser_threshold, int datasize, double angle_step, double angle_start, double laser_thr);
    static vector<SH_Point> get_circles(vector<float> laser, vector<float> deep, double radius, double deep_threshold, double angle_step, double angle_start,double y_need);

};

