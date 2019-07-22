#pragma once
#include<iostream>
#include<string>
#include<vector>
#include<queue>
#include<set>
#include"windows.h"
#include<time.h>
#include <fstream>
#include<thread>
#include <mutex>
#include"Vertex.h"
#include"ParkGraph.h"
using namespace std;
class Car {
private:
	int carname;
	Graph *graph;
	int speed = 4;
	Vertex *vertex;//全部顶点
	int start;//起始点
	int goal;//目标点
	int openlistcountglessrhs;//open list中g《rhs的数量
	fstream gfstream;//向文件中输入控制台信息
	int totaltime=0;
	int lastfilename;
	int **weight;//初始权值邻接矩阵
	int **totalweight;//最终权值邻接矩阵
	int **finish;//记录走过路径次数
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
	priority_queue<Vertex*, vector<Vertex*>, cmp> openlist;
	//点更新序列
	priority_queue<Vertex*, vector<Vertex*>, cmp2> updatelist;
	queue<Vertex> path;//最短路径
	set<Vertex*> closelist;

public:
	Car() {};
	Car(Graph* graph1, int carname);
	~Car();
	//初始化vertex,gfstream
	//初始化终点
	void Init();
	bool Requirelastfilename();
	//重新设置起点和终点
	void SetStartandGoal(int start, int goal);
	
	//计算Manhattan距离
	int Manhattan(int start, int target);

	//计算key1&key2
	void Caculatekey(int name);

	//name1.key<name2.key?
	bool Compare_key(int name1, int name2);
	//name1.key<name2.key?
	bool Compare_key(Vertex* one, Vertex* two);

	//更新节点
	//如果点在openlist中则除去
	//将rhs！=g的点加入openlist
	//只有updatevertex能将点加入openlist
	void UpdateVertex(int name);
	//程序入口
	void DstarLite();
	void ComputeDijkstra();
	void Computepath();

	//展示点的信息
	void Showvertex(int name);
	void Showvertex(Vertex* pointer);
	int Add(int left, int right);
	void Showopenlist();
	void Showcloselist();
	void Dijkstra();
	void Showtotalweight();
	void Updateopenlist();
	

};