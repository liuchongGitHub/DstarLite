#include"ParkGraph.h"
#include"Car.h"
#include<time.h>
#include<iostream>
#include <fstream>
#include<string.h>
#include<thread>
#include <mutex>
#include"windows.h"
using namespace std;


//创建Graph
void Graph::CreatGraph()
{
	vernum = 10000;
	weight = new int*[vernum];//初始化基本权值矩阵weight
	for (size_t i = 0; i < vernum; i++)
	{
		weight[i] = new int[vernum];
		for (size_t j = 0; j < vernum; j++)
		{
			weight[i][j] = INT_MAX;
			if (i == j) { weight[i][j] = 0; }
			if (i == j + 20 || j == i + 20) { weight[i][j] = 8; }
			if (i == j + 1 || j == i + 1) { weight[i][j] = 20; }
			if ((i % 20 == 19 && j % 20 == 0) || (j % 20 == 19 && i % 20 == 0)) {
				weight[i][j] = INT_MAX;
			}
		}
	}
	Tempcreatfile();
}
//构造函数
Graph::Graph() { }
//析构函数
Graph::~Graph()
{
	for (size_t i = 0; i < vernum; i++)
	{
		delete[] weight[i];
	}
	delete[] weight;
	gfstream.close();
}
int Graph::Add(int left, int right) {
	if (left == INT_MAX || right == INT_MAX)
	{
		return INT_MAX;
	}
	else {
		return left + right;
	}
}
//返回name点的邻居节点
vector<int> Graph::GetNeighbor(int name, int** weight) {
	vector<int> neighbor;
	for (int i = 0; i < vernum; i++) {
		if (i == name) { continue; }
		if (weight[name][i] != INT_MAX) { neighbor.push_back(i); }
	}
	return neighbor;
}
vector<int> Graph::GetNeighbor(int name) {
	vector<int> neighbor;
	for (int i = 0; i < vernum; i++) {
		if (i == name) { continue; }
		if (weight[name][i] != INT_MAX) { neighbor.push_back(i); }
	}
	return neighbor;
}
bool Graph::IsNeighbor(int name1, int name2, int** weight) {
	vector<int> v = GetNeighbor(name1,weight);
	for (auto beg = v.begin(), end = v.end(); beg != end; beg++) {
		if ( *beg == name2) { return true; }
	}
	return false;
}
int Graph::Returnlastfilename() {
	return lastfilename;
}
//模拟接受广播
bool Graph::RequirevehicleFromWindowsFile(int &lastfilename) {
	graphmutex.lock();
	int currentfilename;
	//如果第一次接受变化
	if (lastfilename == 10)
	{
		currentfilename = rand();
		if (currentfilename >= 10) { currentfilename = currentfilename % 10; }
		lastfilename = currentfilename;
		graphmutex.unlock();
		return true;
	}//if (lastfilename==10)
	else
	{
		currentfilename = rand();
		if (currentfilename >= 10) { currentfilename = currentfilename % 10; }
		if (lastfilename == currentfilename)
		{
			graphmutex.unlock(); return false;
		}
		else
		{
			lastfilename = currentfilename;
			graphmutex.unlock();
			return true;
		}//else
	}//else
	
}
//临时建立车辆变化文件
void Graph::Tempcreatfile() {
	for (int i = 0; i < 10; i++)
	{
		fstream f;
		string filename = "C:\\Users\\chongliu\\Desktop\\华人运通\\轨迹规划\\VisualStudiocode\\vehiclechangecondition"
			+ to_string(i) + ".txt";
		f.open(filename, ios::out);
		if (f.bad())
		{
			cout << "打开文件出错" << endl;
			return;
		}
		for (size_t i = 0; i < 100; i++)
		{
			int random = rand();
			random = random % vernum;
			vector<int> neighbor = GetNeighbor(random);
			for (auto beg = neighbor.begin(); beg != neighbor.end(); beg++)
			{
				int ifcreat = rand();
				if (ifcreat % 2 == 0)
				{
					f << 1 << " " << random << " " << *beg << "\n";
					cout << 1 << " " << random << " " << *beg << "\n";
				}
			}
		}//for (size_t i = 0; i < 200; i++)

		f.close();
	}//for (int i = 0; i < 10; i++)
}
void Graph::SetWeight(int** targetweight) {
	graphmutex.lock();
	for (int i = 0; i < vernum; i++)
	{
		for (int j = 0; j < vernum;j++) {
			targetweight[i][j] = weight[i][j];
		}

	}
	graphmutex.unlock();
}
void Graph::SetTotalWeight(int** targetweight,int start,int** weight,int lastfilename) {
	// 防止在更新时car访问weight, totalweight, finish
	graphmutex.lock();
	string filename = "C:\\Users\\chongliu\\Desktop\\华人运通\\轨迹规划\\VisualStudiocode\\vehiclechangecondition"
		+ to_string(lastfilename) + ".txt";
	ifstream infile;
	infile.open(filename);
	bool park; int vertex1, vertex2;
	while (!infile.eof())
	{
		infile >> park >> vertex1 >> vertex2;
		//cout <<"车辆状况  "<< park << "  " << vertex1 << "   " << vertex2 << endl;
		if (IsNeighbor(vertex1, vertex2,weight)) {
			if (park == 1) {
				targetweight[vertex1][vertex2] = Add(10, targetweight[vertex1][vertex2]);
				targetweight[vertex2][vertex1] = Add(10, targetweight[vertex2][vertex1]);
			}
			else {
				targetweight[vertex1][vertex2] = Add(5, targetweight[vertex1][vertex2]);
				targetweight[vertex2][vertex1] = Add(5, targetweight[vertex2][vertex1]);
			}
		}
		//与start不相邻
		else {
			if (park == 1) {
				targetweight[vertex1][vertex2] = Add(7, targetweight[vertex1][vertex2]);
				targetweight[vertex2][vertex1] = Add(7, targetweight[vertex2][vertex1]);
			}
			else {
				targetweight[vertex1][vertex2] = Add(3, targetweight[vertex1][vertex2]);
				targetweight[vertex2][vertex1] = Add(3, targetweight[vertex2][vertex1]);
			}
		}
	}//while (!infile.eof())			 
	infile.close();	
	graphmutex.unlock();
}
void Graph::Mainmethod() {
	lastfilename = 10;
	while (1) {	
		RequirevehicleFromWindowsFile(lastfilename);
		//cout << "lastfilename= " << lastfilename << endl;
		this_thread::sleep_for(std::chrono::seconds(4));//4秒
		
	}
}

