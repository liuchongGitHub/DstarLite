#pragma once
#include<string>
//ÿ����������ݽṹ
struct Vertex
{
	int name;//�����
	int g;//���㵽�յ�Ĺ��ƾ���
	//bool g_if_max;//g�Ƿ���MAX
	int rhs;//���㵽�յ��ʵ�ʾ���
	//bool rhs_if_max;//rhs�Ƿ���max
	int h;//���㵽��������ʽ���룬���������پ���
	int key1;//key2+h+km,�ȶ�ʱ���ȱȶ�
	int key2;//min (g rhs)
	int x;//����ĺ�����
	int y;//�����������
	bool isinopen;//�Ƿ������ȶ���
	Vertex *next;//ָ����һ�������ָ��
	int distance;//���ڼ�¼Dijkstra�㷨��·������
	bool visit;//���ڼ�¼Dijkstra�㷨�Ƿ�������
};