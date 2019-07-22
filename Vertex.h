#pragma once
#include<string>
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
	Vertex *next;//指向下一个顶点的指针
	int distance;//用于记录Dijkstra算法的路径长度
	bool visit;//用于记录Dijkstra算法是否搜索过
};