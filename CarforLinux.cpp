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
#define INT_MAX 2147483647
using namespace std;
Car::Car(Graph *graph,int carname)
{
	this->graph = graph;
	this->carname = carname;
	vertex = new Vertex[graph->vernum];
	//cout<<"graph->vernum"<<graph->vernum<<endl;
	//cout << "输入每个顶点的坐标x&y"<<endl;
	for (int i = 0; i < graph->vernum; i++) {
		//cout << "顶点" << i << ":";
		vertex[i].name = i;//为每个顶点编号
		vertex[i].key1 = 0;
		vertex[i].key2 = 0;
		vertex[i].isinopen = false;
		//cin >> (vertex)[i].x >> (vertex)[i].y;
		int row = i / 20;
		int column = i % 20;
		(vertex)[i].x = row * 20;
		(vertex)[i].y = column * 8;
	}
	for (int i = 0; i < graph->vernum; i++)
	{
		vertex[i].g = INT_MAX;
		vertex[i].rhs = INT_MAX;
		vertex[i].next = NULL;
		vertex[i].isinopen = false;

	}
	//初始化weight
	//用锁
	weight= new int*[graph->vernum];
	for (int i = 0; i < graph->vernum; i++)
	{
		weight[i] = new int[graph->vernum];
	}
	graph->SetWeight(weight);
	totalweight = new int*[graph->vernum];//初始化权值矩阵最终totalweight
	for (int i = 0; i < graph->vernum; i++)
	{
		totalweight[i] = new int[graph->vernum];
		for (int j = 0; j < graph->vernum; j++)
		{
			totalweight[i][j] = weight[i][j];
		}
	}
	finish = new int*[graph->vernum];//初始化基本权值矩阵finish
	for (int i = 0; i < graph->vernum; i++)
	{
		finish[i] = new int[graph->vernum];
		for (int j = 0; j < graph->vernum; j++)
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
//初始化vertex,gfstream
//初始化终点
void Car::Init() {
	ifstream f;
	f.open("configuration");
	string str;
	int tempstart,tempgoal;
	f>>str;
	if(str=="filepath"){ f>>filepath;}
	f>>str;
	if(str=="start"){f>>tempstart;}
	f>>str;
	if(str=="goal"){f>>tempgoal;}
	f.close();
	SetStartandGoal(tempstart,tempgoal);
	string filename = filepath+"/carmessage" + to_string(carname) + ".txt";
	gfstream.open(filename, ios::out);
	if (gfstream.bad())
	{
		cout << "打开文件出错" + filename << endl;
		return;
	}
	//初始化终点
	//cout<<"openfile"<<endl;
	openlistcountglessrhs = 0;
	vertex[goal].rhs = 0;
	Caculatekey(goal);
	Vertex *v = &(vertex[goal]);
	openlist.push(v);
	vertex[goal].isinopen = true;
	//cout<<"vertex[goal].next: "<<vertex[goal].next<<endl;
	//cout<<"v->next: "<<v->next<<endl;
	//cout<<"(vertex[goal].next==NULL)"<<(vertex[goal].next==NULL)<<endl;
	//cout<<"(v->next == NULL)" <<(v->next == NULL)<<endl;
	//cout<<"until showvertex"<<endl;
	Showvertex(openlist.top());
	//cout<<"complete showvertex"<<endl;
}
bool Car::Requirelastfilename() {
	return true;
}
//重新设置起点和终点
void Car::SetStartandGoal(int start, int goal) {
	this->start = start;
	this->goal = goal;
}
//计算Manhattan距离
int Car::Manhattan(int start, int target) {
	return abs(vertex[target].x - vertex[start].x) + abs(vertex[target].y - vertex[start].y);
}

//计算key1&key2
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

//更新节点
//如果点在openlist中则除去
//将rhs！=g的点加入openlist
//只有updatevertex能将点加入openlist
void Car::UpdateVertex(int name) {
	Vertex& ver = vertex[name];
	//记录源数据
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
		//rhs= min(g’(s)+c(s,s’))

		ver.rhs = INT_MAX;
		Vertex* point = ver.next;
		for (auto beg = neighbor.begin(); beg != neighbor.end(); beg++) {

			if (vertex[*beg].next != &vertex[name])//防止点从后继中选next
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
		//如果点发生改变则显示出来
		/*
		if (oldver.g != ver.g || oldver.rhs != ver.rhs || oldver.next != ver.next) {
			Vertex* s = &oldver;
			gfstream << "*****************************" << endl;
			Showvertex(s);
			gfstream << "改变为： " << endl;
			Showvertex(ver.name);
			gfstream << "*****************************" << endl;
			gfstream << "\n";
		}*/
		/*
		if (point!=NULL && point->name!= ver.next->name) {
			cout << name << "点原next=" << point->name << " 现next=" << ver.next->name << "    " << name
				<< ".rhs=" << vertex[name].rhs << " next.g=" << vertex[name].next->g << endl;
		}*/
	}
	//如果点在openlist中则除去
	if (ver.isinopen)
	{
		vector<Vertex*> vec;
		Vertex* v = openlist.top();
		//将前面的都调出来
		while (v != &(vertex[name]))
		{
			vec.push_back(v);
			openlist.pop();
			if (v->g < v->rhs) { openlistcountglessrhs--; }
			v = openlist.top();
		}
		//删去目标
		v->isinopen = false;
		openlist.pop();
		if (v->g < v->rhs) { openlistcountglessrhs--; }
		//将原先调出来的都再插进去
		for (auto beg = vec.begin(); beg != vec.end(); beg++)
		{
			openlist.push(*beg);
			if ((*beg)->g < (*beg)->rhs) { openlistcountglessrhs++; }
		}
	}
	//将rhs！=g的点加入openlist从closelist删除
	if (ver.g != ver.rhs)
	{
		Vertex* s = &vertex[name];
		//Caculatekey(name);
		openlist.push(s);
		if (s->g < s->rhs) { openlistcountglessrhs++; }
		s->isinopen = true;
		//gfstream << name << " 点加入openlist" << endl;
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
//程序入口
void Car::DstarLite() {
	//初始化h
	for (int i = 0; i < graph->vernum; i++)
	{
		vertex[i].h = Manhattan(start, i);
	}
	//初始化算法
	Init();
	clock_t starttime = clock();
	//cout << "inti done" << endl;
	Computepath();
	lastfilename = graph->Returnlastfilename();
	while (start != goal)//当车辆没有走完全程时
	{
		int time = 0;
		while (time < 4 && start != goal)
		{
			path.pop();
			Vertex next = path.front();
			time = time +( totalweight[start][next.name] / speed);
			finish[start][next.name]++;
			finish[next.name][start]++;
			//如果路径反复走过3遍则不能再走
			if (finish[start][next.name] >= 3)
			{
				weight[start][next.name] = INT_MAX;
				weight[next.name][start] = INT_MAX;

			}
			//cout << "car" << carname << "行驶中" <<start<<"行驶过花费"<<
				//totalweight[start][next.name] / speed <<"s"<< endl;
			this_thread::sleep_for(std::chrono::seconds(totalweight[start][next.name] / speed));
			//走totalweight[start][next.name] / speed秒
			gfstream << " 点 " << start << "点走过" << endl;
			SetStartandGoal(next.name, goal);
			//cout << time <<"s"<< endl;
			gfstream << " 点 " << start << " 设为新起始点" << endl;
		}//while (time<15 && start != goal)
		//如果到达终点跳出while (start!=goal)
		if (start == goal) { //cout << "break" << endl;
			clock_t endtime = clock();
			gfstream << "totaltime" << endtime - starttime << endl;
			break; }
		//记录原先的权值
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
		//从停车场接受路面变化
		if (lastfilename != graph->Returnlastfilename() ) {

			graph->SetTotalWeight(totalweight, start, weight,lastfilename);
			int different = 0;
			gfstream << " 路径权发生变化" << endl;
			//对变化权边的两个端点及其邻居加入update list
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
			//更新updatelist的元素
			while (!updatelist.empty())
			{
				Vertex* s = updatelist.top();
				updatelist.pop();
				UpdateVertex(s->name);
			}
			gfstream << " 变权端点更新点完成" << endl;
			//更新 openlist
			Updateopenlist();
			//更新closelist
			for (auto beg = closelist.begin(); beg != closelist.end(); beg++)
			{
				UpdateVertex((*beg)->name);
			}
			//Showcloselist();
			gfstream << "重新计算路径" << endl;
			Computepath();
		}//if(lastfilename=graph->Returnlastfilename())
		//else { cout << "路径没变 lastfilename="<<lastfilename << endl; }
		//释放oldweight
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
	gfstream << "计算从" << start << "到" << goal << "的最短路径" << endl;
	//Showopenlist();
	clock_t time1 = clock();
	while (Compare_key(openlist.top()->name, start) || vertex[start].g != vertex[start].rhs
		|| openlistcountglessrhs != 0)
	{

		//gfstream << "compute path openlist------------------------------------------" << endl;
		//弹出队头
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
		//输出openlist中的点信息
		//cout << "完成一次compute--------------------------------------------"<<endl;
		//cout << "完成一次compute--------------------------------------------"<<endl;
		//Showopenlist();
		//Showcloselist();
		if (openlist.empty())break;
	}//while (Compare_key(openlist.top()->name,start) || vertex[start].g!=vertex[start].rhs)
	//gfstream << "compute while finish" << endl;
	//查看所有next不为null的点的next指向
	//cout << "查看所有next不为null的点的next指向--------------------------------------------" << endl;
	/*for (size_t i = 0; i < vernum; i++)
	{
		if (vertex[i].next!=NULL) { cout <<"["<< i << "].next->" << vertex[i].next->name << endl; }
	}*/
	//将最短路径存入path中
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
	//打印输出最短路径

	if (!path.empty()) {
		Vertex *vpath = &path.front();
		gfstream << "-------------------------------------------------------------------" << endl;
		//cout << "最短路径：         ";
		gfstream << "最短路径：         ";
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

		//cout << "耗时：" << time2 - time1 << " ms" << endl;
		gfstream << "耗时：" << time2 - time1 << " ms" << endl;
	}
	gfstream << "-------------------------------------------------------------------" << endl;
}

//展示点的信息
void Car::Showvertex(int name) {
	Vertex v = vertex[name];
	gfstream << "name=" << v.name << "  " << "g=";
	if (v.g == INT_MAX) { gfstream << "∞"; }
	else { gfstream << v.g; }
	gfstream << "  " << "rhs=";
	if (v.rhs == INT_MAX) { gfstream << "∞"; }
	else { gfstream << v.rhs; }
	gfstream << "  " << "isinopen=" << v.isinopen << "  key1=";
	if (v.key1 == INT_MAX) { gfstream << "∞"; }
	else { gfstream << v.key1; }
	gfstream << "  key2=";
	if (v.key2 == INT_MAX) { gfstream << "∞"; }
	else { gfstream << v.key2; }
	if (v.next != NULL) {
		gfstream << "  next=" << v.next->name << endl;
	}
	else
	{
		gfstream << "  next=null" << endl;
	}
}
void Car::Showvertex(Vertex* v) {
	gfstream << "name=" << v->name << "  " << "g=";
	if (v->g == INT_MAX) { gfstream << "∞"; }
	else { gfstream << v->g; }
	gfstream << "  " << "rhs=";
	if (v->rhs == INT_MAX) { gfstream << "∞"; }
	else { gfstream << v->rhs; }
	gfstream << "  " << "isinopen=" << v->isinopen << "  key1=";
	if (v->key1 == INT_MAX) { gfstream << "∞"; }
	else { gfstream << v->key1; }
	gfstream << "  key2=";
	if (v->key2 == INT_MAX) { gfstream << "∞"; }
	else { gfstream << v->key2; }
	//cout<<"if (v->next != nullptr) "<<endl;
	//cout<<(v->next)->name<<endl;
	if ((v->next)!=NULL) {
		//cout << "  (v->next)->name= " ;
		//cout<< (v->next)->name << endl;
		gfstream << "  next=" << (v->next)->name << endl;
	}
	else
	{
		
		gfstream << "  next=null" << endl;
	}
	//cout<<"complete 100"<<endl;
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
	//输出openlist中的点信息
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
	//初始化
	for (int i = 0; i < graph->vernum; i++)
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
		for (int i = 0; i < graph->vernum; i++) {//找到一个最小值点
			if (!vertex[i].visit && vertex[i].distance < min) {
				min = vertex[i].distance;
				temp = i;
			}
		}
		if (temp == goal) { break; }
		//将temp设置为新的发起点
		vertex[temp].visit = true;
		++count;
		for (int i = 0; i < graph->vernum; i++) {
			//注意这里的条件arc[temp][i]!=INT_MAX必须加，不然会出现溢出，从而造成程序异常
			if (!vertex[i].visit && totalweight[temp][i] != INT_MAX && (Add(vertex[temp].distance, totalweight[temp][i])) <= vertex[i].distance) {
				//如果新得到的边可以影响其他为访问的顶点，那就就更新它的最短路径和长度
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
	gfstream << "Dijkstra最短路径为:";
	//cout << "Dijkstra最短路径为:";

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
	gfstream << "耗时：" << time2 - time1 << " ms" << endl;
	//cout << "total:" << vertex[goal].distance << endl;
	//cout << "耗时：" << time2 - time1 << " ms" << endl;
	//cout << start <<"   "<<goal << endl;
}
void Car::Dijkstra() {
	//初始化算法
	//cout<<"(vertex[goal].next==NULL)"<<(vertex[goal].next==NULL)<<"goal"<<goal<<endl;
	Init();
	//cout<<"in dijkstra628"<<endl;
	clock_t starttime = clock();
	//cout<<"in dijkstra630"<<endl;
	//cout << "inti done" << endl;
	ComputeDijkstra();
	//cout<<"in dijkstra before while (start != goal)"<<endl;
	while (start != goal)//当车辆没有走完全程时
	{
		int time = 0;
		//cout<<"in dijkstra before while (time < 4 && start != goal)"<<endl;
		while (time < 4 && start != goal)
		{
			Vertex *next =&path.front() ;
			path.pop();
			time = time + (totalweight[start][next->name] / speed);
			finish[start][next->name]++;
			finish[next->name][start]++;
			//如果路径反复走过3遍则不能再走
			if (finish[start][next->name] >= 3)
			{
				weight[start][next->name] = INT_MAX;
				weight[next->name][start] = INT_MAX;

			}
			gfstream << "car" << carname << "行驶中" << start << "行驶过花费" <<
				totalweight[start][next->name] / speed << "s" << endl;
			//cout << "car" << carname << "行驶中" << start << "行驶过花费" <<
				//totalweight[start][next->name] / speed << "s" << endl;
			this_thread::sleep_for(std::chrono::seconds(totalweight[start][next->name] / speed));
			//走totalweight[start][next.name] / speed秒
			gfstream << " 点 " << start << "点走过" << endl;
			//cout<< " 点 " << start << "点走过" << endl;
			SetStartandGoal(next->name, goal);
			gfstream << " 点 " << start << " 设为新起始点" << endl;
			//cout << " 点 " << start << " 设为新起始点" << endl;
		}//while (time<15 && start != goal)
		//如果到达终点跳出while (start!=goal)
		//cout<<"in dijkstra after while (time < 4 && start != goal)"<<endl;
		if (start == goal) { //cout << "break" << endl;
			break; }
		//从停车场接受路面变化
		//cout<<"in dijkstra before if (lastfilename != graph->Returnlastfilename())"<<endl;
		if (lastfilename != graph->Returnlastfilename()) {

			graph->SetTotalWeight(totalweight, start, weight, lastfilename);
			//cout<<"after setweight"<<endl;
			gfstream << " 路径权发生变化" << endl;
			gfstream << "重新计算路径" << endl;
			//cout<<"in dijkstra before ComputeDijkstra()"<<endl;
			ComputeDijkstra();
			//cout<<"in dijkstra after ComputeDijkstra()"<<endl;
		}//if(lastfilename=graph->Returnlastfilename())
		//cout<<"in dijkstra after if (lastfilename != graph->Returnlastfilename())"<<endl;
	}//while (start != goal)
	//cout<<"in dijkstra after while (start != goal)"<<endl;
	clock_t endtime = clock();
	gfstream << "totaltime" << endtime - starttime;
	cout << "car" << carname << " complete" << endl;
	//cout << "Dijkstra done" << endl;
}
void Car::Showtotalweight() {
	fstream f;
	string filename = "/home/nvidia/Desktop/totalweight.txt";
	f.open(filename, ios::out);
	if (f.bad())
	{
		cout << "打开文件出错" << endl;
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
	//将openlist的元素全部弹出装入tmp2中
	while (!openlist.empty())
	{
		v = openlist.top();
		tmp2.push(v);
		openlist.pop();
		if (v->g < v->rhs) { openlistcountglessrhs--; }
	}
	//cout << " open list pop done" << endl;
	//将tmp2的元素一个一个弹出-更新-加入open list或close list
	while (!tmp2.empty())
	{

		v = tmp2.front();

		tmp2.pop();
		//因为open list现在为空需要跳过 if (ver.isinopen)因为已经从open list中除去了
		v->isinopen = false;
		UpdateVertex(v->name);
	}
	gfstream << "更新openlist finish：----------------------------------------------------" << endl;
	//Showopenlist();
}
