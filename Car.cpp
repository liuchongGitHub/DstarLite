#include<iostream>
#include<string>
#include<vector>
#include<queue>
#include<set>
#include<time.h>
#include <fstream>
#include<stack>
#include<thread>
#include <mutex>
#include"Vertex.h"
#include"ParkGraph.h"
#include"Car.h"
using namespace std;
Car::Car(Graph *graph,int carname) 
{
	this->graph = graph;
	this->carname = carname;
	vertex = new Vertex[graph->vernum];
	//cout << "����ÿ�����������x&y"<<endl;
	for (int i = 0; i < graph->vernum; i++) {
		//cout << "����" << i << ":";
		vertex[i].name = i;//Ϊÿ��������
		vertex[i].key1 = 0;
		vertex[i].key2 = 0;
		vertex[i].isinopen = false;
		//cin >> (vertex)[i].x >> (vertex)[i].y;
		int row = i / 20;
		int column = i % 20;
		(vertex)[i].x = row * 20;
		(vertex)[i].y = column * 8;
	}
	for (size_t i = 0; i < graph->vernum; i++)
	{
		vertex[i].g = INT_MAX;
		vertex[i].rhs = INT_MAX;
		vertex[i].next = NULL;
		vertex[i].isinopen = false;

	}
	//��ʼ��weight
	//����
	weight= new int*[graph->vernum];
	for (size_t i = 0; i < graph->vernum; i++)
	{
		weight[i] = new int[graph->vernum];
	}
	graph->SetWeight(weight);
	totalweight = new int*[graph->vernum];//��ʼ��Ȩֵ��������totalweight
	for (size_t i = 0; i < graph->vernum; i++)
	{
		totalweight[i] = new int[graph->vernum];
		for (size_t j = 0; j < graph->vernum; j++)
		{
			totalweight[i][j] = weight[i][j];
		}
	}
	finish = new int*[graph->vernum];//��ʼ������Ȩֵ����finish
	for (size_t i = 0; i < graph->vernum; i++)
	{
		finish[i] = new int[graph->vernum];
		for (size_t j = 0; j < graph->vernum; j++)
		{
			finish[i][j] = 0;

		}
	}

}
Car::~Car() { 
	delete[] vertex;
	for (int i = 0; i < graph->vernum; i++)
	{
		delete[] weight[i];
	}
	delete[] weight;
	for (int i = 0; i < graph->vernum; i++)
	{
		delete[] totalweight[i];
	}
	delete[] totalweight;
	for (int i = 0; i < graph->vernum; i++)
	{
		delete[] finish[i];
	}
	delete[] finish;
}
//��ʼ��vertex,gfstream
//��ʼ���յ�
void Car::Init() {
	string filename = "C:\\Users\\chongliu\\Desktop\\������ͨ\\�켣�滮\\VisualStudiocode\\carmessage" + to_string(carname) + ".txt";
	gfstream.open(filename, ios::out);
	if (gfstream.bad())
	{
		cout << "���ļ�����" + filename << endl;
		return;
	}
	//��ʼ���յ�
	openlistcountglessrhs = 0;
	vertex[goal].rhs = 0;
	Caculatekey(goal);
	Vertex *v = &(vertex[goal]);
	openlist.push(v);
	vertex[goal].isinopen = true;

	Showvertex(openlist.top());

}
bool Car::Requirelastfilename() {
	return true;
}
//�������������յ�
void Car::SetStartandGoal(int start, int goal) {
	this->start = start;
	this->goal = goal;
}
//����Manhattan����
int Car::Manhattan(int start, int target) {
	return abs(vertex[target].x - vertex[start].x) + abs(vertex[target].y - vertex[start].y);
}

//����key1&key2
void Car::Caculatekey(int name) {
	Vertex &v = vertex[name];
	v.key2 = v.g > v.rhs ? v.rhs : v.g;
	//if (v.g > v.rhs) { v.g = v.rhs; }
	v.key1 =  Add(v.key2, Manhattan(start, name));
}

//name1.key<name2.key?
bool Car::Compare_key(int name1, int name2) {
	Caculatekey(name1);
	Caculatekey(name2);
	if (vertex[name1].key1 < vertex[name2].key1) { return true; }
	if (vertex[name1].key1 == vertex[name2].key1)
	{
		return vertex[name1].key2 < vertex[name2].key2;
	}
	return false;
}
//name1.key<name2.key?
bool Car::Compare_key(Vertex* one, Vertex* two) {
	int name1 = one->name;
	int name2 = two->name;
	Caculatekey(name1);
	Caculatekey(name2);
	if (vertex[name1].key1 < vertex[name2].key1) { return true; }
	if (vertex[name1].key1 == vertex[name2].key1)
	{
		return vertex[name1].key2 < vertex[name2].key2;
	}
	return false;
}

//���½ڵ�
//�������openlist�����ȥ
//��rhs��=g�ĵ����openlist
//ֻ��updatevertex�ܽ������openlist
void Car::UpdateVertex(int name) {
	Vertex& ver = vertex[name];
	//��¼Դ����
	Vertex oldver;
	oldver.g = ver.g;
	oldver.h = ver.h;
	oldver.rhs = ver.rhs;
	oldver.isinopen = ver.isinopen;
	oldver.key1 = ver.key1;
	oldver.key2 = ver.key1;
	oldver.next = ver.next;
	oldver.name = ver.name;
	if (name != goal) {

		vector<int> neighbor = graph->GetNeighbor(name);
		//rhs= min(g��(s)+c(s,s��))

		ver.rhs = INT_MAX;
		Vertex* point = ver.next;
		for (auto beg = neighbor.begin(); beg != neighbor.end(); beg++) {

			if (vertex[*beg].next != &vertex[name])//��ֹ��Ӻ����ѡnext
			{
				int tem = Add(vertex[*beg].g, totalweight[*beg][name]);
				if (ver.rhs >= tem)
				{
					ver.rhs = tem;
					ver.next = &(vertex[*beg]);

				}
			}


		}
		Caculatekey(name);
		//����㷢���ı�����ʾ����
		/*
		if (oldver.g != ver.g || oldver.rhs != ver.rhs || oldver.next != ver.next) {
			Vertex* s = &oldver;
			gfstream << "*****************************" << endl;
			Showvertex(s);
			gfstream << "�ı�Ϊ�� " << endl;
			Showvertex(ver.name);
			gfstream << "*****************************" << endl;
			gfstream << "\n";
		}*/
		/*
		if (point!=NULL && point->name!= ver.next->name) {
			cout << name << "��ԭnext=" << point->name << " ��next=" << ver.next->name << "    " << name
				<< ".rhs=" << vertex[name].rhs << " next.g=" << vertex[name].next->g << endl;
		}*/
	}
	//�������openlist�����ȥ
	if (ver.isinopen)
	{
		vector<Vertex*> vec;
		Vertex* v = openlist.top();
		//��ǰ��Ķ�������
		while (v != &(vertex[name]))
		{
			vec.push_back(v);
			openlist.pop();
			if (v->g < v->rhs) { openlistcountglessrhs--; }
			v = openlist.top();
		}
		//ɾȥĿ��
		v->isinopen = false;
		openlist.pop();
		if (v->g < v->rhs) { openlistcountglessrhs--; }
		//��ԭ�ȵ������Ķ��ٲ��ȥ
		for (auto beg = vec.begin(); beg != vec.end(); beg++)
		{
			openlist.push(*beg);
			if ((*beg)->g < (*beg)->rhs) { openlistcountglessrhs++; }
		}
	}
	//��rhs��=g�ĵ����openlist��closelistɾ��
	if (ver.g != ver.rhs)
	{
		Vertex* s = &vertex[name];
		//Caculatekey(name);
		openlist.push(s);
		if (s->g < s->rhs) { openlistcountglessrhs++; }
		s->isinopen = true;
		//gfstream << name << " �����openlist" << endl;
		//Showvertex(name);

		auto ifincloselist = closelist.find(s);
		if (ifincloselist != closelist.end())
		{
			closelist.erase(ifincloselist);
			//gfstream << "delete closelist point:" << s->name << endl;
		}
	}
	else {
		if (ver.g != INT_MAX) {
			Vertex* s = &vertex[name];
			closelist.insert(s);
		}
	}
	//cout << "updatevertex done";
}
//�������
void Car::DstarLite() {
	//cout << "���������յ㣺";
	//cin >> start >> goal;
	//cout << endl;
	start =188;
	goal = 2930;
	SetStartandGoal(start, goal);
	//��ʼ��h
	for (int i = 0; i < graph->vernum; i++)
	{
		vertex[i].h = Manhattan(start, i);
	}
	//��ʼ���㷨
	Init();
	clock_t starttime = clock();
	//cout << "inti done" << endl;
	Computepath();
	lastfilename = graph->Returnlastfilename();
	while (start != goal)//������û������ȫ��ʱ
	{
		int time = 0;
		while (time < 4 && start != goal)
		{
			path.pop();
			Vertex next = path.front();
			time = time +( totalweight[start][next.name] / speed);
			finish[start][next.name]++;
			finish[next.name][start]++;
			//���·�������߹�3����������
			if (finish[start][next.name] >= 3)
			{
				weight[start][next.name] = INT_MAX;
				weight[next.name][start] = INT_MAX;

			}
			//cout << "car" << carname << "��ʻ��" <<start<<"��ʻ������"<< 
				//totalweight[start][next.name] / speed <<"s"<< endl;
			this_thread::sleep_for(std::chrono::seconds(totalweight[start][next.name] / speed));
			//��totalweight[start][next.name] / speed��
			gfstream << " �� " << start << "���߹�" << endl;
			SetStartandGoal(next.name, goal);
			//cout << time <<"s"<< endl;
			gfstream << " �� " << start << " ��Ϊ����ʼ��" << endl;
		}//while (time<15 && start != goal)
		//��������յ�����while (start!=goal)
		if (start == goal) { //cout << "break" << endl; 
			clock_t endtime = clock();
			gfstream << "totaltime" << endtime - starttime << endl;
			break; }
		//��¼ԭ�ȵ�Ȩֵ
		int** oldweight;
		oldweight = new int*[graph->vernum];
		for (size_t i = 0; i < graph->vernum; i++)
		{
			oldweight[i] = new int[graph->vernum];
			for (size_t j = 0; j < graph->vernum; j++)
			{
				oldweight[i][j] = totalweight[i][j];
			}
		}
		//��ͣ��������·��仯
		if (lastfilename != graph->Returnlastfilename() ) {

			graph->SetTotalWeight(totalweight, start, weight,lastfilename);
			int different = 0;
			gfstream << " ·��Ȩ�����仯" << endl;
			//�Ա仯Ȩ�ߵ������˵㼰���ھӼ���update list
			for (size_t i = 0; i < graph->vernum; i++)
			{
				for (size_t j = 0; j < graph->vernum; j++)
				{
					if (oldweight[i][j] != totalweight[i][j])
					{
						different++;
						Vertex* s1 = &vertex[i];
						updatelist.push(s1);
						Vertex* s3 = &vertex[j];
						updatelist.push(s3);
					}
				}
			}
			//cout << " different has " << different << endl;
			//����updatelist��Ԫ��
			while (!updatelist.empty())
			{
				Vertex* s = updatelist.top();
				updatelist.pop();
				UpdateVertex(s->name);
			}
			gfstream << " ��Ȩ�˵���µ����" << endl;
			//���� openlist
			Updateopenlist();
			//����closelist
			for (auto beg = closelist.begin(); beg != closelist.end(); beg++)
			{
				UpdateVertex((*beg)->name);
			}
			//Showcloselist();
			gfstream << "���¼���·��" << endl;
			Computepath();
		}//if(lastfilename=graph->Returnlastfilename())
		//else { cout << "·��û�� lastfilename="<<lastfilename << endl; }
		//�ͷ�oldweight
		for (size_t i = 0; i < graph->vernum; i++)
		{
			delete oldweight[i];

		}
		delete oldweight;
	}//while (start!=goal)
	
	//cout << "totaltime" << endtime - starttime;
	cout << "car"<<carname<<" complete" << endl;
}
void Car::Computepath() {
	gfstream << "�����" << start << "��" << goal << "�����·��" << endl;
	//Showopenlist();
	clock_t time1 = clock();
	while (Compare_key(openlist.top()->name, start) || vertex[start].g != vertex[start].rhs
		|| openlistcountglessrhs != 0)
	{

		//gfstream << "compute path openlist------------------------------------------" << endl;
		//������ͷ
		Vertex* s = openlist.top();
		openlist.pop();
		if (s->g < s->rhs)openlistcountglessrhs--;
		s->isinopen = false;
		if ((s->g) > (s->rhs))
		{

			s->g = s->rhs;
			Caculatekey(s->name);
			closelist.insert(s);
			vector<int> neighbor = graph->GetNeighbor(s->name);
			for (auto beg = neighbor.begin(), end = neighbor.end(); beg != end; beg++)
			{
				UpdateVertex(*beg);
			}

		}
		else
		{

			s->g = INT_MAX;
			closelist.erase(s);
			//gfstream << " delete point " << s->name << endl;
			UpdateVertex(s->name);
			vector<int> neighbor = graph->GetNeighbor(s->name);
			for (auto beg = neighbor.begin(), end = neighbor.end(); beg != end; beg++)
			{
				UpdateVertex(*beg);
			}

		}
		//if (openlist.empty()) { cout << "openlist empty" << endl; }
		//���openlist�еĵ���Ϣ
		//cout << "���һ��compute--------------------------------------------"<<endl;
		//cout << "���һ��compute--------------------------------------------"<<endl;
		//Showopenlist();
		//Showcloselist();
		if (openlist.empty())break;
	}//while (Compare_key(openlist.top()->name,start) || vertex[start].g!=vertex[start].rhs)
	//gfstream << "compute while finish" << endl;
	//�鿴����next��Ϊnull�ĵ��nextָ��
	//cout << "�鿴����next��Ϊnull�ĵ��nextָ��--------------------------------------------" << endl;
	/*for (size_t i = 0; i < vernum; i++)
	{
		if (vertex[i].next!=NULL) { cout <<"["<< i << "].next->" << vertex[i].next->name << endl; }
	}*/
	//�����·������path��
	Vertex* vstart = &vertex[start];
	while (!path.empty())
	{
		path.pop();
	}
	while (vstart != NULL)
	{
		path.push(*vstart);
		vstart = vstart->next;
	}
	clock_t time2 = clock();
	//��ӡ������·��

	if (!path.empty()) {
		Vertex *vpath = &path.front();
		gfstream << "-------------------------------------------------------------------" << endl;
		//cout << "���·����         ";
		gfstream << "���·����         ";
		int distance = 0;

		while (vpath != NULL)
		{
			//cout << vpath->name;
			gfstream << vpath->name;
			int front = vpath->name;
			int real = vpath->name;

			if (vpath->next != NULL)
			{
				//cout << "->";
				gfstream << "->";
				real = vpath->next->name;
			}
			vpath = vpath->next;
			distance = distance + totalweight[front][real];

		}

		//cout << "total:" << distance;
		//cout << endl;
		gfstream << "total:" << distance;
		gfstream << endl;
	
		//cout << "��ʱ��" << time2 - time1 << " ms" << endl;
		gfstream << "��ʱ��" << time2 - time1 << " ms" << endl;
	}
	gfstream << "-------------------------------------------------------------------" << endl;
}

//չʾ�����Ϣ
void Car::Showvertex(int name) {
	Vertex v = vertex[name];
	gfstream << "name=" << v.name << "  " << "g=";
	if (v.g == INT_MAX) { gfstream << "��"; }
	else { gfstream << v.g; }
	gfstream << "  " << "rhs=";
	if (v.rhs == INT_MAX) { gfstream << "��"; }
	else { gfstream << v.rhs; }
	gfstream << "  " << "isinopen=" << v.isinopen << "  key1=";
	if (v.key1 == INT_MAX) { gfstream << "��"; }
	else { gfstream << v.key1; }
	gfstream << "  key2=";
	if (v.key2 == INT_MAX) { gfstream << "��"; }
	else { gfstream << v.key2; }
	if (v.next != NULL) {
		gfstream << "  next=" << v.next->name << endl;
	}
	else
	{
		gfstream << "  next=null" << endl;
	}
}
void Car::Showvertex(Vertex* pointer) {
	Vertex v = *pointer;
	gfstream << "name=" << v.name << "  " << "g=";
	if (v.g == INT_MAX) { gfstream << "��"; }
	else { gfstream << v.g; }
	gfstream << "  " << "rhs=";
	if (v.rhs == INT_MAX) { gfstream << "��"; }
	else { gfstream << v.rhs; }
	gfstream << "  " << "isinopen=" << v.isinopen << "  key1=";
	if (v.key1 == INT_MAX) { gfstream << "��"; }
	else { gfstream << v.key1; }
	gfstream << "  key2=";
	if (v.key2 == INT_MAX) { gfstream << "��"; }
	else { gfstream << v.key2; }
	if (v.next != NULL) {
		gfstream << "  next=" << v.next->name << endl;
	}
	else
	{
		gfstream << "  next=null" << endl;
	}
}
int Car::Add(int left, int right) {
	if (left == INT_MAX || right == INT_MAX)
	{
		return INT_MAX;
	}
	else {
		return left + right;
	}
}
void Car::Showopenlist() {
	//���openlist�еĵ���Ϣ
	priority_queue<Vertex*, vector<Vertex*>, cmp> openlistcopy = openlist;
	gfstream << "openlist:" << endl;
	while (!openlistcopy.empty()) {
		Showvertex(openlistcopy.top());
		openlistcopy.pop();
	}
}
void Car::Showcloselist() {
	gfstream << " closelist: ---------------------------------------------" << endl;
	for (auto beg = closelist.begin(); beg != closelist.end(); beg++)
	{
		Showvertex(*beg);
	}
}
void Car::ComputeDijkstra() {
	//��ʼ��
	for (size_t i = 0; i < graph->vernum; i++)
	{
		vertex[i].distance = totalweight[start][i];
		vertex[i].visit = false;
		vertex[i].next = NULL;
	}
	vertex[start].distance = 0;
	int count = 1;
	clock_t time1 = clock();
	while (count != graph->vernum)
	{
		int temp = 0;
		int min = INT_MAX;
		for (int i = 0; i < graph->vernum; i++) {//�ҵ�һ����Сֵ��
			if (!vertex[i].visit && vertex[i].distance < min) {
				min = vertex[i].distance;
				temp = i;
			}
		}
		if (temp == goal) { break; }
		//��temp����Ϊ�µķ����
		vertex[temp].visit = true;
		++count;
		for (int i = 0; i < graph->vernum; i++) {
			//ע�����������arc[temp][i]!=INT_MAX����ӣ���Ȼ�����������Ӷ���ɳ����쳣
			if (!vertex[i].visit && totalweight[temp][i] != INT_MAX && (Add(vertex[temp].distance, totalweight[temp][i])) <= vertex[i].distance) {
				//����µõ��ı߿���Ӱ������Ϊ���ʵĶ��㣬�Ǿ;͸����������·���ͳ���
				vertex[i].distance = vertex[temp].distance + totalweight[temp][i];
			    vertex[i].next=&vertex[temp];
			}
		}
		//gfstream << temp << " next " << vertex[temp].next->name << endl;
	}//while (count!=vernum)
	/*for (int i = 0; i < graph->vernum; i++)
	{
		Showvertex(i);
	}*/
	stack<Vertex *> stack0;
	Vertex *point = &vertex[goal];
	while (point != NULL) {
		stack0.push(point);
		point = point->next;
	}
	clock_t time2 = clock();
	gfstream << "Dijkstra���·��Ϊ:";
	//cout << "Dijkstra���·��Ϊ:";
	
	while (!stack0.empty())
	{
		point = stack0.top();
		path.push(*point);
		stack0.pop();
		gfstream << point->name;
		//cout << point->name;
		if (!stack0.empty()) { gfstream << "->"; //cout << "->";
		}
	}
	gfstream << "total:" << vertex[goal].distance << endl;
	gfstream << "��ʱ��" << time2 - time1 << " ms" << endl;
	//cout << "total:" << vertex[goal].distance << endl;
	//cout << "��ʱ��" << time2 - time1 << " ms" << endl;
	//cout << start <<"   "<<goal << endl;
}
void Car::Dijkstra() {
	//cout << "���������յ㣺";
	//cin >> start >> goal;
	//cout << endl;
	start = 188;
	goal = 2930;
	SetStartandGoal(start, goal);
	//��ʼ���㷨
	Init();
	clock_t starttime = clock();
	//cout << "inti done" << endl;
	ComputeDijkstra();
	while (start != goal)//������û������ȫ��ʱ
	{
		int time = 0;
		while (time < 4 && start != goal)
		{
			Vertex *next =&path.front() ;
			path.pop();
			time = time + (totalweight[start][next->name] / speed);
			finish[start][next->name]++;
			finish[next->name][start]++;
			//���·�������߹�3����������
			if (finish[start][next->name] >= 3)
			{
				weight[start][next->name] = INT_MAX;
				weight[next->name][start] = INT_MAX;

			}
			gfstream << "car" << carname << "��ʻ��" << start << "��ʻ������" <<
				totalweight[start][next->name] / speed << "s" << endl;
			//cout << "car" << carname << "��ʻ��" << start << "��ʻ������" <<
				//totalweight[start][next->name] / speed << "s" << endl;
			this_thread::sleep_for(std::chrono::seconds(totalweight[start][next->name] / speed));
			//��totalweight[start][next.name] / speed��
			gfstream << " �� " << start << "���߹�" << endl;
			//cout<< " �� " << start << "���߹�" << endl;
			SetStartandGoal(next->name, goal);
			gfstream << " �� " << start << " ��Ϊ����ʼ��" << endl;
			//cout << " �� " << start << " ��Ϊ����ʼ��" << endl;
		}//while (time<15 && start != goal)
		//��������յ�����while (start!=goal)
		if (start == goal) { //cout << "break" << endl; 
			break; }
		//��ͣ��������·��仯
		if (lastfilename != graph->Returnlastfilename()) {

			graph->SetTotalWeight(totalweight, start, weight, lastfilename);
			gfstream << " ·��Ȩ�����仯" << endl;		
			gfstream << "���¼���·��" << endl;
			ComputeDijkstra();
		}//if(lastfilename=graph->Returnlastfilename())		
	}//while (start != goal)
	clock_t endtime = clock();
	gfstream << "totaltime" << endtime - starttime;
	cout << "car" << carname << " complete" << endl;
	//cout << "Dijkstra done" << endl;
}
void Car::Showtotalweight() {
	fstream f;
	string filename = "C:\\Users\\chongliu\\Desktop\\������ͨ\\�켣�滮\\VisualStudiocode\\totalweight.txt";
	f.open(filename, ios::out);
	if (f.bad())
	{
		cout << "���ļ�����" << endl;
		return;
	}

	for (int i = 0; i < graph->vernum; i++)
	{
		vector<int> v = graph->GetNeighbor(i);
		f << "p" << i;
		for (auto beg = v.begin(); beg != v.end(); beg++) {
			f << "-->p" << *beg << "[ " << totalweight[i][*beg] << " ]  ";
		}
		f << "\n";
	}
	f.close();

}
void Car::Updateopenlist() {
	queue<Vertex*>  tmp2;
	Vertex* v;
	//��openlist��Ԫ��ȫ������װ��tmp2��
	while (!openlist.empty())
	{
		v = openlist.top();
		tmp2.push(v);
		openlist.pop();
		if (v->g < v->rhs) { openlistcountglessrhs--; }
	}
	//cout << " open list pop done" << endl;
	//��tmp2��Ԫ��һ��һ������-����-����open list��close list
	while (!tmp2.empty())
	{

		v = tmp2.front();

		tmp2.pop();
		//��Ϊopen list����Ϊ����Ҫ���� if (ver.isinopen)��Ϊ�Ѿ���open list�г�ȥ��
		v->isinopen = false;
		UpdateVertex(v->name);
	}
	gfstream << "����openlist finish��----------------------------------------------------" << endl;
	//Showopenlist();
}
