#ifndef ASTAR_H
#define ASTAR_H
#include <iostream>
#include <queue>
#include <vector>
#include <stack>
#include<algorithm>
#include <list>
#include <vector>
using namespace std;


const int kCost1 = 10; //直移一格消耗
const int kCost2 = 14; //斜移一格消耗

typedef struct Node
{
    int x,y;
    int g; //起始点到当前点实际代价
    int h;//当前节点到目标节点最佳路径的估计代价
    int f;//估计值
    Node* father;
    Node(int x,int y)
    {
        this->x = x;
        this->y = y ;
        this->g = 0;
        this->h = 0;
        this->f = 0;
        this->father = NULL;
    }
    Node(int x,int y,Node* father)
    {
        this->x = x;
        this->y = y ;
        this->g = 0;
        this->h = 0;
        this->f = 0;
        this->father = father;
    }
}Node;

struct Point
{
    int x, y; //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
    int F, G, H; //F=G+H
    Point *parent; //parent的坐标，这里没有用指针，从而简化代码
    Point(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL)  //变量初始化
    {
    }
};

class Astar
{
public:
    void InitAstar(std::vector<std::vector<int>> &_maze);
    std::list<Point *> GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);

private:
    Point *findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
    std::vector<Point *> getSurroundPoints(const Point *point, bool isIgnoreCorner) const;
    bool isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const; //判断某点是否可以用于下一步判断
    Point *isInList(const std::list<Point *> &list, const Point *point) const; //判断开启/关闭列表中是否包含某点
    Point *getLeastFpoint(); //从开启列表中返回F值最小的节点
    //计算FGH值
    int calcG(Point *temp_start, Point *point);
    int calcH(Point *point, Point *end);
    int calcF(Point *point);
private:
    std::vector<std::vector<int>> maze;
    std::list<Point *> openList;  //开启列表
    std::list<Point *> closeList; //关闭列表
};

/*
class Astar{
    private:
       //int map_flag[2000][2000];
    public:
        Astar(){}
        ~Astar(){}
        void search(Node* startPos,Node* endPos);

        void checkPoit(int x,int y,Node* father,int g);
        void NextStep(Node* currentPoint);
        int isContains(vector<Node*>* Nodelist ,int x,int y);
        void countGHF(Node* sNode,Node* eNode,int g);
        static bool compare(Node* n1,Node* n2);
        bool unWalk(int x,int y);
        void printPath( Node* current);
       /* void setMapFlag(int cell_flag[2000][2000])
        {
            cout<<"setMapFlag"<<endl;
            for(int i=0;i<2000;i++)
            {
                for(int j=0;j<2000;j++)
                {
                    map_flag[i][j]=cell_flag[i][j];
                }
            }
        }*//*
        vector<Node*> openList;
        vector<Node*> closeList;
        vector<Node*> path;
        vector<Node*> filter_node(Node* node1,Node* node2,int map_cell[500][500]);
        Node *startPos;
        Node *endPos;
        static const int WeightW = 10;// 正方向消耗
        static const int WeightWH = 14;//打斜方向的消耗
        static const int row = 500;
        static const int col = 500;

};

*/




/*

#ifndef ASTAR_H
#define ASTAR_H
#include <iostream>
#include <queue>
#include <vector>
#include <stack>
#include<algorithm>
#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <math.h>
#include "std_msgs/String.h"

using namespace std;


typedef struct Node
{
    int x,y;
    int g; //起始点到当前点实际代价
    int h;//当前节点到目标节点最佳路径的估计代价
    int f;//估计值
    Node* father;
    Node(int x,int y)
    {
        this->x = x;
        this->y = y ;
        this->g = 0;
        this->h = 0;
        this->f = 0;
        this->father = NULL;
    }
    Node(int x,int y,Node* father)
    {
        this->x = x;
        this->y = y ;
        this->g = 0;
        this->h = 0;
        this->f = 0;
        this->father = father;
    }
}Node;

class Astar{
    private:
       int map_flag[2000][2000];
    public:
       vector<Node*> openList;
       vector<Node*> closeList;
       vector<Node*> path;
       static Node *startPos_;
       static Node *endPos_;
       static const int WeightW = 10;// 正方向消耗
       static const int WeightWH = 14;//打斜方向的消耗
       static const int row = 2000;
       static const int col = 2000;
       bool unWalk( int x,int y)
       {

           if (map_flag[x][y] == 1||map_flag[x][y]==-1)
               return true;
           return false;
       }
       int isContains(vector<Node*>* Nodelist, int x,int y )
       {
           for (int i = 0;i < Nodelist->size();i++)
           {
               if (Nodelist->at(i)->x  == x && Nodelist->at(i)->y == y)
               {
                   return i;
               }
           }
           return -1;
       }
       void countGHF( Node* sNode,Node* eNode,int g)
       {
           int h = abs(sNode->x - eNode->x) + abs(sNode->y - eNode->y) * WeightW;
           int currentg = sNode->father->g + g;
           int f = currentg + h;
           sNode->f = f;
           sNode->h = h;
           sNode->g = currentg;
       }
       void checkPoit( int x,int y,Node* father,int g)
       {
           if (x < 0 || x > row || y < 0 || y > col)
               return;
           if (this->unWalk(x,y))
               return;
           if (isContains(&closeList,x,y) != -1)
               return;
           int index;
           if ((index = isContains(&openList,x,y)) != -1)
           {
               Node *point = openList[index];
               if (point->g > father->g + g)
               {
                   point->father = father;
                   point->g = father->g + g;
                   point->f = point->g + point->h;
               }
           }
           else
           {
               Node * point = new Node(x,y,father);
               countGHF(point,endPos_,g);
               openList.push_back(point);
           }
       }

       void NextStep( Node* current )
       {
           checkPoit(current->x - 1,current->y,current,WeightW);//左
           checkPoit(current->x + 1,current->y,current,WeightW);//右
           checkPoit(current->x,current->y + 1,current,WeightW);//上
           checkPoit(current->x,current->y - 1,current,WeightW);//下
       }

       bool compare( Node* n1,Node* n2 );

       void search( Node* startPos,Node* endPos );


       void printPath( Node* current )
       {
           if (current->father != NULL)
               printPath(current->father);
           path.push_back(current);
           //printf("(%d,%d)",current->x,current->y);
           //path_flag[current->x][current->y]=1;
       }

       void setMapFlag(int cell_flag[2000][2000])
        {
            for(int i=0;i<2000;i++)
            {
                for(int j=0;j<2000;j++)
                {
                    map_flag[i][j]=cell_flag[i][j];
                }
            }
        }
       vector<Node*> filter_node(Node* node1,Node* node2,int cell_map[2000][2000])
          {
            setMapFlag(cell_map);
            search(node1,node2);
            vector<Node*> result_path;
            if(path.size()<3)
                return path;
            else
           {
            //s为每条线上第一个点，f为待考虑是否删除的点，t为比较s,f,t是否共线的中间点,t指向f的下一个。
               double sfx,stx,sfy,sty;
               int i=2;
               int node_num = path.size();
               Node* s = path[0];
               Node* f = path[1];
               Node* t = path[2];
               result_path.push_back(s);
               while(i<node_num-1)
                   {
                        sfx = f->x - s->x;
                        sfy = f->y - s->y;
                        stx = t->x - s->x;
                        sty = t->y - s->y;

                        if((sfx==0 && stx==0)|| sfy/sfx == sty/stx )
                        {  //s t f三点共线

                           f = t;

                        }
                        else
                        {
                           result_path.push_back(f);
                           s = f;
                           f = t;
                        }
                        t = path[++i];
                   }
                result_path.push_back(path.back());
            return result_path;
           }
          }


};



#endif
*/

#endif
