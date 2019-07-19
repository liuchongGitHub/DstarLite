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
	void CreatGraph();//����ͼ
	vector<int> GetNeighbor(int name, int** weight);//���ص�name���ھ�
	vector<int> GetNeighbor(int name); 
	bool IsNeighbor(int name1, int name2, int** weight);//
	bool RequirevehicleFromWindowsFile(int& lastfilename);
	void Mainmethod();//�������
	//��ʱ����ͣ�����ڻ����仯���ļ�
	void Tempcreatfile();
	int Add(int left, int right);
	//����car�ĳ�ʼweight
	void SetWeight(int** targetweight);
	//����car��totalweight
	void SetTotalWeight(int** tragetweight,int start,int** weight,int lastfilename);
	int Returnlastfilename();
	int vernum;//������Ŀ
private:
	int lastfilename;//Ŀǰ�������
	Vertex *vertex;//ȫ������
	int **weight;//��ʼȨֵ�ڽӾ���
	//int** totalweight;//����Ȩֵ�ڽӾ���
	int edgenum;//�ߵ���Ŀ
	fstream gfstream;//���ļ����������̨��Ϣ
	mutex graphmutex;


};