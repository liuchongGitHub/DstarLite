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
	Vertex *vertex;//ȫ������
	int start;//��ʼ��
	int goal;//Ŀ���
	int openlistcountglessrhs;//open list��g��rhs������
	fstream gfstream;//���ļ����������̨��Ϣ
	int totaltime=0;
	int lastfilename;
	int **weight;//��ʼȨֵ�ڽӾ���
	int **totalweight;//����Ȩֵ�ڽӾ���
	int **finish;//��¼�߹�·������
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
	//���������
	priority_queue<Vertex*, vector<Vertex*>, cmp2> updatelist;
	queue<Vertex> path;//���·��
	set<Vertex*> closelist;

public:
	Car() {};
	Car(Graph* graph1, int carname);
	~Car();
	//��ʼ��vertex,gfstream
	//��ʼ���յ�
	void Init();
	bool Requirelastfilename();
	//�������������յ�
	void SetStartandGoal(int start, int goal);
	
	//����Manhattan����
	int Manhattan(int start, int target);

	//����key1&key2
	void Caculatekey(int name);

	//name1.key<name2.key?
	bool Compare_key(int name1, int name2);
	//name1.key<name2.key?
	bool Compare_key(Vertex* one, Vertex* two);

	//���½ڵ�
	//�������openlist�����ȥ
	//��rhs��=g�ĵ����openlist
	//ֻ��updatevertex�ܽ������openlist
	void UpdateVertex(int name);
	//�������
	void DstarLite();
	void ComputeDijkstra();
	void Computepath();

	//չʾ�����Ϣ
	void Showvertex(int name);
	void Showvertex(Vertex* pointer);
	int Add(int left, int right);
	void Showopenlist();
	void Showcloselist();
	void Dijkstra();
	void Showtotalweight();
	void Updateopenlist();
	

};