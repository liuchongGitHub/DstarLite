#pragma once
#include"Vertex.h"
#include<iostream>
#include<string>
#include<vector>
#include<queue>
#include<set>
#include <fstream>
#include<thread>
#include <mutex>
using namespace std;

class Graph
{
public:
	Graph();
	~Graph();
	void CreatGraph();//创建图
	vector<int> GetNeighbor(int name, int** weight);//返回点name的邻居
	vector<int> GetNeighbor(int name); 
	bool IsNeighbor(int name1, int name2, int** weight);//
	bool RequirevehicleFromWindowsFile(int& lastfilename);
	void Mainmethod();//程序入口
	//临时创建停车场内环境变化的文件
	void Tempcreatfile();
	int Add(int left, int right);
	//设置car的初始weight
	void SetWeight(int** targetweight);
	//设置car的totalweight
	void SetTotalWeight(int** tragetweight,int start,int** weight,int lastfilename);
	int Returnlastfilename();
	int vernum;//顶点数目
private:
	int lastfilename;//目前车辆情况
	Vertex *vertex;//全部顶点
	string filepath;
	int **weight;//初始权值邻接矩阵
	//int** totalweight;//最终权值邻接矩阵
	int edgenum;//边的数目
	fstream gfstream;//向文件中输入控制台信息
	mutex graphmutex;


};
