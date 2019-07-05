#pragma once
#include<iostream>
#include<string>
#include<vector>
#include<queue>
#include<set>
using namespace std;
//每个顶点的数据结构
struct Vertex
{
	int name;//顶点号
	int g;//顶点到终点的估计距离
	//bool g_if_max;//g是否是MAX
	int rhs;//顶点到终点的实际距离
	//bool rhs_if_max;//rhs是否是max
	int h;//顶点到起点的启发式距离，采用曼哈顿距离
	int key1;//key2+h+km,比对时优先比对
	int key2;//min (g rhs)
	int x;//顶点的横坐标
	int y;//顶点的纵坐标
	bool isinopen;//是否在优先队列
	Vertex* next;//指向下一个顶点的指针
	string Dpath;//用于记录Dijkstra算法求出的路径
	int distance;//用于记录Dijkstra算法的路径长度
	bool visit;//用于记录Dijkstra算法是否搜索过
};
class Graph
{
public:
	Graph();
	~Graph();
	void CreatGraph();//创建图
	vector<int> GetNeighbor(int name);//返回点name的邻居
	bool IsNeighbor(int name1,int name2);//
	int Manhattan(int start, int target);//计算Manhattan距离
	void Caculatekey(int name);//计算key1&key2
	void Init(int end);
	void Computepath();
	bool Compare_key(int name1,int name2);//name1.key<name2.key?
	void Setgequal(int name);//g=rhs
	void Setginfinite(int name);//g=无穷
	void UpdateVertex(int name);//更新点
	bool Compare_g_rhs(int name);//比较g>rhs
	bool Compare_g_equal_rhs(int name);//比较g！=rhs
	bool Requirevehicle();//请求停车场内车辆变化情况
	bool RequirevehicleFromWindowsFile(int& lastfilename);
	void Setstartandend(int start,int end);//设置目标点和起始点
	void Mainmethod();//程序入口
	void Showvertex(int name);//展示点的信息
	void Showvertex(Vertex* name);
	int Add(int left, int right);
	void Showopenlist();
	void Showcloselist();
	void Dijkstra();
	void Tempcreatfile();//临时创造停车场情况变化文件
	void Showtotalweight();
	void Updateopenlist();
	//使priority_queue最小优先
	struct cmp
	{
		bool operator() (Vertex *v1, Vertex *v2)
		{
			if (v1->key1 == v2->key1)return v1->key2 > v2->key2;
			return v1->key1 > v2->key1;
		}

	};
	struct cmp2
	{
		bool operator() (Vertex *v1, Vertex *v2)
		{
			return  v1->rhs > v2->rhs;
		}

	};
private:
	Vertex* vertex;//全部顶点
	int vernum;//顶点数目
	int** weight;//初始权值邻接矩阵
	int** totalweight;//最终权值邻接矩阵
	int** finish;//记录走过路径次数
	int edgenum;//边的数目
	int start;//起始点
	int goal;//目标点
	int openlistcountglessrhs;//open list中g《rhs的数量
	priority_queue<Vertex*, vector<Vertex*>, cmp> openlist;
	//点更新序列
	priority_queue<Vertex*, vector<Vertex*>, cmp2> updatelist;
	queue<Vertex> path;//最短路径
	set<Vertex*> closelist;
};

