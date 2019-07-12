#include"ParkGraph.h"
#include<time.h>
#include<iostream>
#include <fstream>
#include<string.h>
#include"windows.h"
using namespace std;
//�������ͱ����Ĺ�ϵ�ǣ�((Vexnum*(Vexnum - 1)) / 2) < edge
bool check(int vexnum, int edgenum) {
	if (vexnum <= 0 || edgenum <= 0 || ((vexnum*(vexnum - 1)) / 2) < edgenum)
		return false;
	return true;
}
//����Graph
void Graph::CreatGraph()
{
	//cout << "����ȫ���������Ŀ�ߵ���Ŀ"<<endl;
	//cin >> vernum>>edgenum;
	//edgenum = 370;
	vernum = 20000;
	/*while (!check(vernum, edgenum)) {
		cout << "�������ֵ���Ϸ�������������" << endl;
		cin >> vernum >> edgenum;
	}*/
	vertex = new Vertex[vernum];//��ʼ��vertex
	//cout << "����ÿ�����������x&y"<<endl;
	for (int i = 0; i < vernum; i++) {
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
	//����������Ȩ
	/*for (size_t i = 0; i < vernum; i++)
	{
		for (size_t j = 0; j < vernum; j++)
		{
			if (weight[i][j]!=INT_MAX) { cout << i << "  to  " << j <<" = "<< weight[i][j] << endl;
			}
		}
	}*/
	totalweight = new int*[vernum];//��ʼ��Ȩֵ��������totalweight
	for (size_t i = 0; i < vernum; i++)
	{
		totalweight[i] = new int[vernum];
		for (size_t j = 0; j < vernum; j++)
		{
			totalweight[i][j] = weight[i][j];
		}
	}
	finish = new int*[vernum];//��ʼ������Ȩֵ����finish
	for (size_t i = 0; i < vernum; i++)
	{
		finish[i] = new int[vernum];
		for (size_t j = 0; j < vernum; j++)
		{
			finish[i][j] = 0;

		}
	}

	string filename = "C:\\Users\\chongliu\\Desktop\\������ͨ\\�켣�滮\\VisualStudiocode\\message.txt";
	gfstream.open(filename, ios::out);
	if (gfstream.bad())
	{
		cout << "���ļ�����" << endl;
		return;
	}
	/*cout << "���ÿ���ߵĻ���Ȩֵ�������0��" << vernum-1 << "�� ��0 1 10 "<<endl;
	for (size_t i = 0; i < edgenum; i++)
	{
		int oneside; int otherside; int value;
		cin >> oneside >> otherside>>value;
		cout << endl;
		weight[oneside][otherside] = value;
		weight[otherside][oneside] = value;
	}*/
}

//���캯��
Graph::Graph() { }
//��������
Graph::~Graph()
{
	for (size_t i = 0; i < vernum; i++)
	{
		delete weight[i];
	}
	delete weight;
	for (size_t i = 0; i < vernum; i++)
	{
		delete totalweight[i];
	}
	delete totalweight;
	for (size_t i = 0; i < vernum; i++)
	{
		delete finish[i];
	}
	delete finish;
	gfstream.close();
}
//����start��target�������پ��뼴h
int Graph::Manhattan(int start, int target)
{
	return abs(vertex[target].x - vertex[start].x) + abs(vertex[target].y - vertex[start].y);
}
void Graph::Init(int goal)
{
	for (size_t i = 0; i < vernum; i++)
	{
		vertex[i].g = INT_MAX;
		vertex[i].rhs = INT_MAX;
		vertex[i].next = NULL;
		vertex[i].isinopen = false;

	}
	openlistcountglessrhs = 0;
	vertex[goal].rhs = 0;
	Caculatekey(goal);
	Vertex *v = &(vertex[goal]);
	openlist.push(v);
	vertex[goal].isinopen = true;
	//�����һ��openlist�еĵ���Ϣ
	Showvertex(openlist.top());

}
//����name����ھӽڵ�
vector<int> Graph::GetNeighbor(int name) {
	vector<int> neighbor;
	for (int i = 0; i < vernum; i++) {
		if (i == name) { continue; }
		if (weight[name][i] != INT_MAX) { neighbor.push_back(i); }
	}
	return neighbor;
}
//������ʼ���Ŀ���
void Graph::Setstartandend(int start, int goal) {
	this->start = start; this->goal = goal;
	//cout << "���&�յ��������"<<endl;
}
bool Graph::IsNeighbor(int name1, int name2) {
	vector<int> v = GetNeighbor(start);
	for (auto beg = v.begin(), end = v.end(); beg != end; beg++) {
		if (*beg == name1 || *beg == name2) { return true; }
	}
	return false;
}
//��������״̬����total weight����
bool Graph::Requirevehicle() {
	std::cout << "�Ƿ�������³���״��Y/N" << endl;
	char YN;
	cin >> YN;
	if (YN == 'N')
	{
		return false;
	}
	//��ʼ������Ȩֵ����
	for (size_t i = 0; i < vernum; i++)
	{
		for (size_t j = 0; j < vernum; j++)
		{
			totalweight[i][j] = weight[i][j];
		}
	}

	while (YN == 'Y') {
		cout << "���복����״̬0��ʻ1���� �Լ����ڵıߵ������˵�" << endl;
		bool park; int vertex1, vertex2;
		cin >> park >> vertex1 >> vertex2;
		//��start����
		if (IsNeighbor(vertex1, vertex2)) {
			if (park == 1) {
				totalweight[vertex1][vertex2] = Add(10, totalweight[vertex1][vertex2]);
				totalweight[vertex2][vertex1] = Add(10, totalweight[vertex2][vertex1]);
			}
			else {
				totalweight[vertex1][vertex2] = Add(5, totalweight[vertex1][vertex2]);
				totalweight[vertex2][vertex1] = Add(5, totalweight[vertex2][vertex1]);
			}
		}
		//��start������
		else {
			if (park == 1) {
				totalweight[vertex1][vertex2] = Add(7, totalweight[vertex1][vertex2]);
				totalweight[vertex2][vertex1] = Add(7, totalweight[vertex2][vertex1]);
			}
			else {
				totalweight[vertex1][vertex2] = Add(3, totalweight[vertex1][vertex2]);
				totalweight[vertex2][vertex1] = Add(3, totalweight[vertex2][vertex1]);
			}
		}
		cout << "�Ƿ�������³���״��Y/N" << endl;
		cin >> YN;
	}
	return true;
}
//����name���keyֵ
void Graph::Caculatekey(int name) {
	Vertex& v = vertex[name];
	v.key2 = v.g > v.rhs ? v.rhs : v.g;
	//if (v.g > v.rhs) { v.g = v.rhs; }
	v.key1 = Add(v.key2, Manhattan(start, name));
}
//���½ڵ�
//�������openlist�����ȥ
//��rhs��=g�ĵ����openlist
//ֻ��updatevertex�ܽ������openlist
void Graph::UpdateVertex(int name) {
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

		vector<int> neighbor = GetNeighbor(name);
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

		if (oldver.g != ver.g || oldver.rhs != ver.rhs || oldver.next != ver.next) {
			Vertex* s = &oldver;
			gfstream << "*****************************" << endl;
			Showvertex(s);
			gfstream << "�ı�Ϊ�� " << endl;
			Showvertex(ver.name);
			gfstream << "*****************************" << endl;
			gfstream << "\n";
		}
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
		gfstream << name << " �����openlist" << endl;
		//Showvertex(name);

		auto ifincloselist = closelist.find(s);
		if (ifincloselist != closelist.end())
		{
			closelist.erase(ifincloselist);
			gfstream << "delete closelist point:" << s->name << endl;
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
//�������·��
void Graph::Computepath() {
	gfstream << "�����" << start << "��" << goal << "�����·��" << endl;
	//Showopenlist();
	clock_t time1 = clock();
	while (Compare_key(openlist.top()->name, start) || vertex[start].g != vertex[start].rhs
		|| openlistcountglessrhs != 0)
	{

		gfstream << "compute path openlist------------------------------------------" << endl;
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
			vector<int> neighbor = GetNeighbor(s->name);
			for (auto beg = neighbor.begin(), end = neighbor.end(); beg != end; beg++)
			{
				UpdateVertex(*beg);
			}

		}
		else
		{

			s->g = INT_MAX;
			closelist.erase(s);
			gfstream << " delete point " << s->name << endl;
			UpdateVertex(s->name);
			vector<int> neighbor = GetNeighbor(s->name);
			for (auto beg = neighbor.begin(), end = neighbor.end(); beg != end; beg++)
			{
				UpdateVertex(*beg);
			}

		}
		if (openlist.empty()) { cout << "openlist empty" << endl; }
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
		cout << "-------------------------------------------------------------------" << endl;
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_GREEN);
		cout << "���·����         ";
		gfstream << "���·����         ";
		int distance = 0;

		while (vpath != NULL)
		{
			cout << vpath->name;
			gfstream << vpath->name;
			int front = vpath->name;
			int real = vpath->name;

			if (vpath->next != NULL)
			{
				cout << "->";
				gfstream << "->";
				real = vpath->next->name;
			}
			vpath = vpath->next;
			distance = distance + totalweight[front][real];

		}

		cout << "total:" << distance;
		cout << endl;
		gfstream << "total:" << distance;
		gfstream << endl;
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_INTENSITY | FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
		cout << "��ʱ��" << time2 - time1 << " ms" << endl;
		gfstream << "��ʱ��" << time2 - time1 << " ms" << endl;
	}
	Dijkstra();
	cout << "-------------------------------------------------------------------" << endl;
}
//name1.key<name2.key?
bool Graph::Compare_key(int name1, int name2) {
	Caculatekey(name1);
	Caculatekey(name2);
	if (vertex[name1].key1 < vertex[name2].key1) { return true; }
	if (vertex[name1].key1 == vertex[name2].key1)
	{
		return vertex[name1].key2 < vertex[name2].key2;
	}
	return false;
}

//D*Lite��Ҫ�㷨
void Graph::Mainmethod() {
	cout << "���������յ㣺";
	cin >> start >> goal;
	cout << endl;
	Setstartandend(start, goal);
	//��ʼ��h
	for (size_t i = 0; i < vernum; i++)
	{
		vertex[i].h = Manhattan(start, i);
	}
	//��ʼ���㷨
	Init(goal);
	cout << "inti done" << endl;
	Computepath();
	float speed;
	cout << "���복��" << endl;
	cin >> speed;
	int lastfilename = 10;
	while (start != goal)//������û������ȫ��ʱ
	{
		float time = 0;
		while (time < 7.5 && start != goal)
		{
			path.pop();

			Vertex next = path.front();

			time = time + totalweight[start][next.name] / speed;
			finish[start][next.name]++;
			finish[next.name][start]++;
			if (finish[start][next.name] >= 3)//���·�������߹�3����������
			{
				weight[start][next.name] = INT_MAX;
				weight[next.name][start] = INT_MAX;

			}
			gfstream << " �� " << start << "���߹�" << endl;
			Setstartandend(next.name, goal);
			gfstream << " �� " << start << " ��Ϊ����ʼ��" << endl;
		}//while (time<7.5 && start != goal)
		//��¼ԭ�ȵ�Ȩֵ
		int** oldweight;
		oldweight = new int*[vernum];
		for (size_t i = 0; i < vernum; i++)
		{
			oldweight[i] = new int[vernum];
			for (size_t j = 0; j < vernum; j++)
			{
				oldweight[i][j] = totalweight[i][j];
			}
		}
		//��ͣ��������·��仯
		bool ifchange = RequirevehicleFromWindowsFile(lastfilename);
		if (ifchange) {
			int different = 0;
			gfstream << " ·��Ȩ�����仯" << endl;
			//�Ա仯Ȩ�ߵ������˵㼰���ھӼ���update list
			for (size_t i = 0; i < vernum; i++)
			{
				for (size_t j = 0; j < vernum; j++)
				{
					if (oldweight[i][j] != totalweight[i][j])
					{
						different++;
						Vertex* s1 = &vertex[i];
						updatelist.push(s1);
						//UpdateVertex(i);
						//vector<int> v1 = GetNeighbor(i);
						/*for (auto beg=v1.begin(); beg!=v1.end(); beg++)
						{
							Vertex* s2 = &vertex[*beg];
							updatelist.push(s2);
							//UpdateVertex(*beg);
						}*/
						Vertex* s3 = &vertex[j];
						updatelist.push(s3);
						//UpdateVertex(j);
						//vector<int> v2 = GetNeighbor(j);
						/*for (auto beg = v2.begin(); beg != v2.end(); beg++)
						{
							Vertex* s4 = &vertex[*beg];
							updatelist.push(s4);
							//UpdateVertex(*beg);
						}*/
					}
				}
			}
			cout << " different has " << different << endl;
			//����updatelist��Ԫ��
			while (!updatelist.empty())
			{
				Vertex* s = updatelist.top();
				updatelist.pop();
				UpdateVertex(s->name);
			}
			gfstream << " ��Ȩ�˵���µ����" << endl;
			//�ͷ�oldweight
			for (size_t i = 0; i < vernum; i++)
			{
				delete oldweight[i];

			}
			delete oldweight;
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
		}//if (ifchange)
	}//while (start!=goal)
}
//չʾ�����Ϣ
void Graph::Showvertex(int name) {
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
void Graph::Showvertex(Vertex* pointer) {
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
//��Դ�������ķǸ������ļӷ�
int Graph::Add(int left, int right) {
	if (left == INT_MAX || right == INT_MAX)
	{
		return INT_MAX;
	}
	else {
		return left + right;
	}
}
void Graph::Showopenlist() {
	//���openlist�еĵ���Ϣ
	priority_queue<Vertex*, vector<Vertex*>, cmp> openlistcopy = openlist;
	gfstream << "openlist:" << endl;
	while (!openlistcopy.empty()) {
		Showvertex(openlistcopy.top());
		openlistcopy.pop();
	}
}
void Graph::Showcloselist() {
	gfstream << " closelist: ---------------------------------------------" << endl;
	for (auto beg = closelist.begin(); beg != closelist.end(); beg++)
	{
		Showvertex(*beg);
	}
}
//Dijkstra�㷨������·��
void Graph::Dijkstra() {
	//��ʼ��
	for (size_t i = 0; i < vernum; i++)
	{
		vertex[i].distance = totalweight[start][i];
		vertex[i].visit = false;
		vertex[i].Dpath = "";
	}

	vertex[start].distance = 0;
	int count = 1;
	clock_t time1 = clock();
	while (count != vernum)
	{
		int temp = 0;
		int min = INT_MAX;
		for (int i = 0; i < vernum; i++) {//�ҵ�һ����Сֵ��
			if (!vertex[i].visit && vertex[i].distance < min) {
				min = vertex[i].distance;
				temp = i;
			}
		}
		vertex[temp].visit = true;
		++count;
		for (int i = 0; i < vernum; i++) {
			//ע�����������arc[temp][i]!=INT_MAX����ӣ���Ȼ�����������Ӷ���ɳ����쳣
			if (!vertex[i].visit && totalweight[temp][i] != INT_MAX && (Add(vertex[temp].distance, totalweight[temp][i])) <= vertex[i].distance) {
				//����µõ��ı߿���Ӱ������Ϊ���ʵĶ��㣬�Ǿ;͸����������·���ͳ���
				vertex[i].distance = vertex[temp].distance + totalweight[temp][i];
				vertex[i].Dpath = vertex[temp].Dpath + "->" + to_string(i);
			}
		}

	}//while (count!=vernum)
	clock_t time2 = clock();
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_GREEN);
	cout << "Dijkstra���·��Ϊ:" << start << vertex[goal].Dpath << "total:" << vertex[goal].distance << endl;
	gfstream << "Dijkstra���·��Ϊ:" << start << vertex[goal].Dpath << "total:" << vertex[goal].distance << endl;
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_INTENSITY | FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
	cout << "��ʱ��" << time2 - time1 << " ms" << endl;
	gfstream << "��ʱ��" << time2 - time1 << " ms" << endl;

}
//ģ����ܹ㲥
bool Graph::RequirevehicleFromWindowsFile(int &lastfilename) {
	int currentfilename;
	//�����һ�ν��ܱ仯
	if (lastfilename == 10)
	{
		currentfilename = rand();
		if (currentfilename >= 10) { currentfilename = currentfilename % 10; }
		string filename = "C:\\Users\\chongliu\\Desktop\\������ͨ\\�켣�滮\\VisualStudiocode\\vehiclechangecondition"
			+ to_string(currentfilename) + ".txt";
		ifstream infile;
		gfstream << "document name :" << to_string(currentfilename) << endl;
		infile.open(filename);
		bool park; int vertex1, vertex2;
		while (!infile.eof() )
		{
			infile >> park >> vertex1 >> vertex2;
			//cout <<"����״��  "<< park << "  " << vertex1 << "   " << vertex2 << endl;
			if (IsNeighbor(vertex1, vertex2)) {
				if (park == 1) {
					totalweight[vertex1][vertex2] = Add(10, totalweight[vertex1][vertex2]);
					totalweight[vertex2][vertex1] = Add(10, totalweight[vertex2][vertex1]);
				}
				else {
					totalweight[vertex1][vertex2] = Add(5, totalweight[vertex1][vertex2]);
					totalweight[vertex2][vertex1] = Add(5, totalweight[vertex2][vertex1]);
				}
			}
			//��start������
			else {
				if (park == 1) {
					totalweight[vertex1][vertex2] = Add(7, totalweight[vertex1][vertex2]);
					totalweight[vertex2][vertex1] = Add(7, totalweight[vertex2][vertex1]);
				}
				else {
					totalweight[vertex1][vertex2] = Add(3, totalweight[vertex1][vertex2]);
					totalweight[vertex2][vertex1] = Add(3, totalweight[vertex2][vertex1]);
				}
			}
		}//while (!infile.eof())			 
		infile.close();
		Showtotalweight();
		lastfilename = currentfilename;
		return true;
	}//if (lastfilename==10)
	else
	{
		currentfilename = rand();
		if (currentfilename >= 10) { currentfilename = currentfilename % 10; }
		if (lastfilename == currentfilename)
		{
			return false;
		}
		else
		{

			string filename = "C:\\Users\\chongliu\\Desktop\\������ͨ\\�켣�滮\\VisualStudiocode\\vehiclechangecondition"
				+ to_string(currentfilename) + ".txt";
			ifstream infile;
			infile.open(filename);
			bool park; int vertex1, vertex2;
			while (!infile.eof())
			{
				infile >> park >> vertex1 >> vertex2;
				//cout << "����״��  " << park << "  " << vertex1 << "   " << vertex2 << endl;
				if (IsNeighbor(vertex1, vertex2)) {
					if (park == 1) {
						totalweight[vertex1][vertex2] = Add(10, totalweight[vertex1][vertex2]);
						totalweight[vertex2][vertex1] = Add(10, totalweight[vertex2][vertex1]);
					}
					else {
						totalweight[vertex1][vertex2] = Add(5, totalweight[vertex1][vertex2]);
						totalweight[vertex2][vertex1] = Add(5, totalweight[vertex2][vertex1]);
					}
				}
				//��start������
				else {
					if (park == 1) {
						totalweight[vertex1][vertex2] = Add(7, totalweight[vertex1][vertex2]);
						totalweight[vertex2][vertex1] = Add(7, totalweight[vertex2][vertex1]);
					}
					else {
						totalweight[vertex1][vertex2] = Add(3, totalweight[vertex1][vertex2]);
						totalweight[vertex2][vertex1] = Add(3, totalweight[vertex2][vertex1]);
					}
				}
			}//while (!infile.eof())			 
			infile.close();
			lastfilename = currentfilename;
			Showtotalweight();
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
void Graph::Showtotalweight() {
	fstream f;
	string filename = "C:\\Users\\chongliu\\Desktop\\������ͨ\\�켣�滮\\VisualStudiocode\\totalweight.txt";
	f.open(filename, ios::out);
	if (f.bad())
	{
		cout << "���ļ�����" << endl;
		return;
	}

	for (int i = 0; i < vernum; i++)
	{
		vector<int> v = GetNeighbor(i);
		f << "p" << i;
		for (auto beg = v.begin(); beg != v.end(); beg++) {
			f << "-->p" << *beg << "[ " << totalweight[i][*beg] << " ]  ";
		}
		f << "\n";
	}
	f.close();

}
void Graph::Updateopenlist() {
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