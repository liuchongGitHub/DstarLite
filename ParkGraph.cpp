#include"ParkGraph.h"
#include"Car.h"
#include<time.h>
#include<iostream>
#include <fstream>
#include<string.h>
#include<thread>
#include <mutex>

using namespace std;


//����Graph
void Graph::CreatGraph()
{
	vernum = 8000;
	weight = new int*[vernum];//��ʼ������Ȩֵ����weight
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
//���캯��
Graph::Graph() { }
//��������
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
//����name����ھӽڵ�
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
//ģ����ܹ㲥
bool Graph::RequirevehicleFromWindowsFile(int &lastfilename) {
	graphmutex.lock();
	int currentfilename;
	//�����һ�ν��ܱ仯
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
//��ʱ���������仯�ļ�
void Graph::Tempcreatfile() {
	for (int i = 0; i < 10; i++)
	{
		fstream f;
		string filename = "C:\\Users\\chongliu\\Desktop\\������ͨ\\�켣�滮\\VisualStudiocode\\vehiclechangecondition"
			+ to_string(i) + ".txt";
		f.open(filename, ios::out);
		if (f.bad())
		{
			cout << "���ļ�����" << endl;
			return;
		}
		for (size_t i = 0; i < 50; i++)
		{
			int random = rand();
			random = random % vernum;
			for (int i = 0; i < vernum; i++)
			{
				if (weight[random][i] != INT_MAX) {
					f << 1 << " " << random << " " << i << "\n";
					cout << 1 << " " << random << " " << i << "\n";
					break;
				}
			}
			/*
			vector<int> neighbor = GetNeighbor(random);
			for (auto beg = neighbor.begin(); beg != neighbor.end(); beg++)
			{
				int ifcreat = rand();
				if (ifcreat % 2 == 0)
				{
					f << 1 << " " << random << " " << *beg << "\n";
					cout << 1 << " " << random << " " << *beg << "\n";
				}
			}*/
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
	// ��ֹ�ڸ���ʱcar����weight, totalweight, finish
	graphmutex.lock();
	string filename = "C:\\Users\\chongliu\\Desktop\\������ͨ\\�켣�滮\\VisualStudiocode\\vehiclechangecondition"
		+ to_string(lastfilename) + ".txt";
	ifstream infile;
	infile.open(filename);
	bool park; int vertex1, vertex2;
	while (!infile.eof())
	{
		infile >> park >> vertex1 >> vertex2;
		//cout <<"����״��  "<< park << "  " << vertex1 << "   " << vertex2 << endl;
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
		//��start������
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
		this_thread::sleep_for(std::chrono::seconds(4));//4��
		
	}
}

