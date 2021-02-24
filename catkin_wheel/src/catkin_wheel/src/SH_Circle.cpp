#include <catkin_wheel/SH_Circle.h>
#include <math.h>


SH_Circle::SH_Circle()
{
}


SH_Circle::~SH_Circle()
{
}

double point_distance(SH_Point point1, SH_Point point2)
{
    return sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));
}
bool point_dis_far(SH_Point point1, SH_Point point2,double dis)
{
    if(fabs(point1.x-point2.x)>dis||fabs(point1.y-point2.y)>dis)
    {
        return true;
    }
    else
    {
        return false;
    }

}

bool point_dis_near(SH_Point point1, SH_Point point2,double dis)
{
    if(fabs(point1.x-point2.x)<dis&&fabs(point1.y-point2.y)<dis)
    {
        return true;
    }
    return false;
}

//center is away from the view point (0,0) 
bool if_valid(SH_Point center, SH_Point point1, SH_Point point2, SH_Point point3, SH_Point point4, SH_Point point5,double radius)
{
	double value0 = center.x*center.x + center.y*center.y - radius * radius;
	double value1 = point1.x*point1.x + point1.y*point1.y;  //point1.d*point1.d
	double value2 = point2.x*point2.x + point2.y*point2.y;
	double value3 = point3.x*point3.x + point3.y*point3.y;
    double value4 = point4.x*point4.x + point4.y*point4.y;
    double value5 = point5.x*point5.x + point5.y*point5.y;
    //cout <<"value0:"<<value0<<" value1:"<<value1<<"value2:"<<value2<<" value3:"<<value3<<"value4:"<<value4<<endl;
    if (value0 >=  value1 && value0 >= value2 && value0 >= value3&& value0 >= value4&&value0 >= value5)
	{
		return true;
	}
	else
	{
		return false;
	}
}

SH_Point get_circle(vector<SH_Point> m_point)
{
	SH_Point center;
	//double k1 = (point2.y - point1.y) / double(point2.x - point1.x);
	//double k2 = (point2.y - point3.y) / double(point2.x - point3.x);
	//k1 = -1 / k1;
	//k2 = -1 / k2;

	////three points should not in one line
	//if (k1 != k2)
	//{
	//	center.x = (k2 * (point2.x + point3.x) / 2 - k1*(point1.x + point2.x) / 2 + (point1.y - point3.y) / 2) / (k2 - k1);
	//	center.y = k2 * (center.x - (point3.x + point2.x) / 2) + (point3.y + point2.y) / 2;
	//}
	//leastsqr
	vector<SH_Point> m_points = m_point;
	double X1 = 0,Y1 = 0, X2 = 0, Y2 = 0, X3 = 0, Y3 = 0, X1Y1 = 0, X1Y2 = 0,X2Y1 = 0;
	for (int i = 0; i < m_points.size(); i++)
	{
		X1 = X1 + m_points[i].x;
		Y1 = Y1 + m_points[i].y;
		X2 = X2 + m_points[i].x*m_points[i].x;
		Y2 = Y2 + m_points[i].y*m_points[i].y;
		X3 = X3 + m_points[i].x*m_points[i].x*m_points[i].x;
		Y3 = Y3 + m_points[i].y*m_points[i].y*m_points[i].y;
		X1Y1 = X1Y1 + m_points[i].x*m_points[i].y;
		X1Y2 = X1Y2 + m_points[i].x*m_points[i].y*m_points[i].y;
		X2Y1 = X2Y1 + m_points[i].x*m_points[i].x*m_points[i].y;
	}
	double C, D, E, G, H, N;
	double a, b, c;
	N = m_points.size();
	C = N*X2 - X1*X1;
	D = N*X1Y1 - X1*Y1;
	E = N*X3 + N*X1Y2 - (X2 + Y2)*X1;
	G = N*Y2 - Y1*Y1;
	H = N*X2Y1 + N*Y3 - (X2 + Y2)*Y1;
	a = (H*D - E*G) / (C*G - D*D);
	b = (H*C - E*D) / (D*D - G*C);
	c = -(a*X1 + b*Y1 + X2 + Y2) / N;

    //double RR;
	center.x = a / (-2);
	center.y = b / (-2);
	//center.rr = (a*a + b*b - 4 * c) / 2;
    //std::cout << "center.x:" << center.x << std::endl;
	return center;
}


int /*SH_Circle::*/if_exist(vector<SH_Point> circles, SH_Point point, double threshold)
{
	for (int i = 0; i < circles.size(); i++)
	{
		if ((circles[i].x != 0 || circles[i].y != 0)&&fabs(circles[i].x - point.x) < threshold && fabs(circles[i].y - point.y) < threshold)
		{
			return i;
		}
	}

	return -1;
}

//merge overlapping circle center,
std::vector<SH_Point> merge(vector<SH_Point> circles, double threshold)
{
	vector<SH_Point> result;
	if (circles.size() == 1)
	{
		result.push_back(circles[0]);
	}
	else
	{
		for (int i = 0; i < circles.size() - 1; i++)
		{
			if (circles[i].x == 0 && circles[i].y == 0)
			{
				continue;
			}
			for (int j = i + 1; j < circles.size(); j++)
			{

				if ((circles[i].x != 0 || circles[i].y != 0) && fabs(circles[i].x - circles[j].x) < threshold && fabs(circles[i].y - circles[j].y) < threshold)
				{
					if (circles[i].b <= circles[j].b)
					{
						circles[i].x = (circles[i].x + circles[j].x) / 2;
						circles[i].y = (circles[i].y + circles[j].y) / 2;
						circles[i].b = (circles[i].b + circles[j].b) / 2;
					}

					circles[i].c++;
					circles[j].x = 0;
					circles[j].y = 0;
				}
			}
			result.push_back(circles[i]);
		}
	}

	return result;

}

// find circles by its radius, return circles center and its believe
std::vector<SH_Point> find_circle(vector<SH_Point> points, double radius, double threshold)
{
	// find circles' center with a known radius,
	// take three points, the distance(of each two points) < radius*2
    double wheel_d=radius*2;
    vector<SH_Point> circles;
    //int step = 100;
    for (int i = 5; i < points.size(); i++)
	{
        SH_Point point1 = points[i];
        SH_Point point2 = points[i-1];
        SH_Point point3 = points[i-2];
        SH_Point point4 = points[i-3];
        SH_Point point5 = points[i-4];

        if (point_dis_far(point1,point2,wheel_d))continue;
        if (point_dis_far(point1,point3,wheel_d))continue;
        if (point_dis_far(point1,point4,wheel_d))continue;
        if (point_dis_far(point1,point5,wheel_d))continue;
        if (point_dis_far(point2,point3,wheel_d))continue;
        if (point_dis_far(point2,point4,wheel_d))continue;
        if (point_dis_far(point2,point5,wheel_d))continue;
        if (point_dis_far(point3,point4,wheel_d))continue;
        if (point_dis_far(point3,point5,wheel_d))continue;
        if (point_dis_far(point4,point5,wheel_d))continue;

        vector<SH_Point> m_points;
        m_points.push_back(point1);
        m_points.push_back(point2);
        m_points.push_back(point3);
        m_points.push_back(point4);
        m_points.push_back(point5);
       //m_points.push_back(point6);
        //m_points.push_back(point7);

        SH_Point center = get_circle(m_points);
        double diff = fabs(point_distance(point1, center) - radius);
        //detect if it is valid
        if (diff < threshold && if_valid(center, point1, point2, point3, point4,point5, radius))
        {

            if (diff < 0.01)
            {
                diff = 0.01;;
            }
            center.b = 0.01 / diff;
            center.c++;
            circles.push_back(center);
        }
	}
    if(circles.size() == 0)
    {
        return circles;
    }
	return merge(circles, threshold);

}


std::vector<SH_Point> filter_circles(vector<SH_Point>circles, std::vector<float> ranges, double radius, double angle_start, double angle_step, double threshold)
{
	vector<SH_Point> result = circles;
	for (int i = 0; i <result.size(); i++)
	{
		//if (fabs(result[i].x) < 0.000001&& fabs(result[i].y) < 0.000001)continue;
		double deep = sqrt(result[i].x*result[i].x + result[i].y*result[i].y);
		double thelta = asin(radius / deep);
		double alpha = atan(result[i].x / result[i].y);
		double alpha1 = alpha - thelta;
		int index1 = (alpha1 - angle_start) / angle_step;
		int index2 = index1 + 2 * thelta / angle_step;
		int count = 0;
		for (int j = index1; j < index2; j++)
		{
			if (ranges[j]>(deep - threshold))
			{
				count++;
			}
		}
		if (count > 3)
		{
			result[i].b = 0;
		}

	}
	return result;
}

vector<SH_Point> get_points(vector<float> laser, vector<float> deep,double y_need)
{
    vector<SH_Point> points_;
    points_.clear();
    SH_Point point;
    for (int i = 0; i < laser.size(); i++)
    {
        if(deep[i]==0 ||  deep[i]> 2.0)
        {
            continue;
        }

        point.x = deep[i] * cos(laser[i]);
        point.y = deep[i] * sin(laser[i]);
        if(fabs(point.y) >y_need)
        {
            continue;
        }
        points_.push_back(point);

    }
    return points_;
}


vector<SH_Point> SH_Circle::get_circles(vector<float> laser, vector<float> deep, double radius, double deep_threshold,  double angle_step, double angle_start,double y_need)
{
	vector<SH_Point> points;
    vector<SH_Point> circles;
    points.clear();
    circles.clear();
    //SH_Point point;
    points = get_points(laser,deep,y_need);
	cout << "point number : " << points.size() << endl;
    myfile_circle<<"point number : " << points.size() << endl;
	circles = find_circle(points, radius, deep_threshold);
	circles = filter_circles(circles, deep, radius, angle_start, angle_step, deep_threshold);


	return circles;
}
