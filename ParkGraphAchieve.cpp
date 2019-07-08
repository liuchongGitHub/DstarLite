#include"ParkGraph.h"
#include<time.h>
#include<iostream>
#include <fstream>
#include<string.h>
#include"windows.h"
using namespace std;
//顶点数和边数的关系是：((Vexnum*(Vexnum - 1)) / 2) < edge
bool check(int vexnum, int edgenum) {
	if (vexnum <= 0 || edgenum <= 0 || ((vexnum*(vexnum - 1)) / 2) < edgenum)
		return false;
	return true;
}
//创建Graph
void Graph::CreatGraph()
{
	//cout << "输入全部顶点的数目边的数目"<<endl;
	//cin >> vernum>>edgenum;
	edgenum = 370;
	vernum = 200;
	/*while (!check(vernum, edgenum)) {
		cout << "输入的数值不合法，请重新输入" << endl;
		cin >> vernum >> edgenum;
	}*/
	vertex = new Vertex[vernum];//初始化vertex
	//cout << "输入每个顶点的坐标x&y"<<endl;
	for (int i = 0; i < vernum; i++) {
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
	weight =new int*[vernum];//初始化基本权值矩阵weight
	for (size_t i = 0; i < vernum; i++)
	{
		weight[i] = new int[vernum];
		for (size_t j = 0; j < vernum; j++)
		{
			weight[i][j] = INT_MAX;
			if (i == j) { weight[i][j] = 0; }
			if (i == j + 20 || j == i + 20) { weight[i][j] = 8;  }
			if (i == j + 1 || j == i + 1) { weight[i][j] = 20; }
			if ((i % 20 == 19 && j % 20 == 0) || (j % 20 == 19 && i % 20 == 0)) {
				weight[i][j] = INT_MAX;
			}
		}
	}
	//输出非无穷边权
	/*for (size_t i = 0; i < vernum; i++)
	{	
		for (size_t j = 0; j < vernum; j++)
		{
			if (weight[i][j]!=INT_MAX) { cout << i << "  to  " << j <<" = "<< weight[i][j] << endl;
			}
		}
	}*/
	totalweight = new int*[vernum];//初始化权值矩阵最终totalweight
	for (size_t i = 0; i < vernum; i++)
	{
		totalweight[i] = new int[vernum];
		for (size_t j = 0; j < vernum; j++)
		{
			totalweight[i][j] = weight[i][j];
		}
	}
	finish = new int*[vernum];//初始化基本权值矩阵finish
	for (size_t i = 0; i < vernum; i++)
	{
		finish[i] = new int[vernum];
		for (size_t j = 0; j < vernum; j++)
		{
			finish[i][j] = 0;
			
		}
	}
	
	string filename = "C:\\Users\\chongliu\\Desktop\\华人运通\\轨迹规划\\VisualStudiocode\\message.txt";
	gfstream.open(filename, ios::out);
	if (gfstream.bad())
	{
		cout << "打开文件出错" << endl;
		return;
	}
	/*cout << "输出每条边的基础权值（顶点从0到" << vernum-1 << "） 例0 1 10 "<<endl;
	for (size_t i = 0; i < edgenum; i++)
	{
		int oneside; int otherside; int value;
		cin >> oneside >> otherside>>value;
		cout << endl;
		weight[oneside][otherside] = value;
		weight[otherside][oneside] = value;
	}*/
}

//构造函数
Graph::Graph(){ }
//析构函数
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
//计算start与target的曼哈顿距离即h
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
	//输出第一个openlist中的点信息
	Showvertex(openlist.top());

}
//返回name点的邻居节点
vector<int> Graph::GetNeighbor(int name) {
	vector<int> neighbor;
	for (int i = 0; i < vernum;i++) {
		if (i == name) { continue; }
		if (weight[name][i] != INT_MAX) { neighbor.push_back(i); }
    }
	return neighbor;
}
//设置起始点和目标点
void Graph::Setstartandend(int start, int goal) {
	this->start = start; this->goal = goal; 
	//cout << "起点&终点设置完成"<<endl;
}
bool Graph::IsNeighbor(int name1,int name2) {
	vector<int> v = GetNeighbor(start);
	for (auto beg = v.begin(), end = v.end(); beg != end; beg++) {
		if (*beg == name1 || *beg == name2) { return true; }
	}
	return false;
}
//请求车辆的状态更新total weight矩阵
bool Graph::Requirevehicle() {
	std::cout << "是否继续更新车辆状况Y/N" << endl;
	char YN;
	cin >> YN;
	if (YN == 'N')
	{
		return false;
	}
	//初始化最终权值矩阵
	for (size_t i = 0; i < vernum; i++)
	{
		for (size_t j = 0; j < vernum; j++)
		{
			totalweight[i][j] = weight[i][j];
		}
	}
	
	while(YN=='Y'){
		cout << "输入车辆的状态0行驶1泊车 以及所在的边的两个端点" << endl;
		bool park; int vertex1, vertex2;
		cin >> park >> vertex1 >> vertex2;
		//与start相邻
		if (IsNeighbor(vertex1, vertex2)) {
			if (park == 1) { totalweight[vertex1][vertex2] = Add(10, totalweight[vertex1][vertex2]); 
			                totalweight[vertex2][vertex1] = Add(10, totalweight[vertex2][vertex1]);}
			else { totalweight[vertex1][vertex2] = Add(5, totalweight[vertex1][vertex2]); 
			totalweight[vertex2][vertex1] = Add(5, totalweight[vertex2][vertex1]);
			}
		}
		//与start不相邻
		else {
			if (park == 1) { totalweight[vertex1][vertex2] = Add(7, totalweight[vertex1][vertex2]);
			totalweight[vertex2][vertex1] = Add(7, totalweight[vertex2][vertex1]);
			}
			else { totalweight[vertex1][vertex2] = Add(3, totalweight[vertex1][vertex2]);
			totalweight[vertex2][vertex1] = Add(3, totalweight[vertex2][vertex1]);
			}
		}
		cout << "是否继续更新车辆状况Y/N" << endl;
		cin >> YN;
	} 
	return true;
}
//计算name点的key值
void Graph::Caculatekey(int name) {
	Vertex& v = vertex[name];
	 v.key2 = v.g > v.rhs ? v.rhs : v.g;
	 //if (v.g > v.rhs) { v.g = v.rhs; }
	 v.key1 = Add(v.key2 , Manhattan(start, name));
}
//更新节点
//如果点在openlist中则除去
//将rhs！=g的点加入openlist
//只有updatevertex能将点加入openlist
void Graph::UpdateVertex(int name) {
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

		vector<int> neighbor = GetNeighbor(name);
		//rhs= min(g’(s)+c(s,s’))
	   
		ver.rhs=INT_MAX;
		Vertex* point = ver.next;
		for (auto beg = neighbor.begin(); beg != neighbor.end(); beg++) {
			
			if (vertex[*beg].next!=&vertex[name])//防止点从后继中选next
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
		
		if (oldver.g!= ver.g || oldver.rhs != ver.rhs || oldver.next!=ver.next ) {
			Vertex* s = &oldver;
			gfstream << "*****************************"<<endl;
			Showvertex(s);
			gfstream << "改变为： " << endl;
			Showvertex(ver.name);
			gfstream << "*****************************"<<endl;
			gfstream << "\n";
		}
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
		Vertex* v=openlist.top();
		//将前面的都调出来
		while(v!=&(vertex[name]) )
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
		gfstream <<name<< " 点加入openlist"<<endl;
		//Showvertex(name);

		auto ifincloselist = closelist.find(s);
		if (ifincloselist!=closelist.end())
		{
			closelist.erase(ifincloselist);
			gfstream << "delete closelist point:" << s->name << endl;
		}
	}
	else {
		if (ver.g!=INT_MAX) {
		Vertex* s = &vertex[name];
		closelist.insert(s);
		}
	}
	//cout << "updatevertex done";
}
//计算最短路径
void Graph::Computepath() {
	gfstream << "计算从" << start << "到" << goal << "的最短路径" << endl;
	//Showopenlist();
	clock_t time1 = clock();
	while (Compare_key(openlist.top()->name,start) || vertex[start].g!=vertex[start].rhs 
		|| openlistcountglessrhs!=0)
	{
		
		gfstream << "compute path openlist------------------------------------------"<<endl;
		Vertex* s = openlist.top();
		openlist.pop();
		if (s->g < s->rhs)openlistcountglessrhs--;
		s->isinopen = false;
		if ((s->g) > (s->rhs) )
		{
			
			s->g = s->rhs;
			Caculatekey(s->name);
			closelist.insert(s);
			vector<int> neighbor=GetNeighbor(s->name);
			for (auto beg=neighbor.begin(),end=neighbor.end();beg!=end;beg++)
			{
				UpdateVertex(*beg);
			}
		
		}		
		else
		{
			
			s->g = INT_MAX;
			closelist.erase(s);
			gfstream << " delete point "<<s->name <<endl;
			UpdateVertex(s->name);
			vector<int> neighbor = GetNeighbor(s->name);
			for (auto beg = neighbor.begin(), end = neighbor.end(); beg != end; beg++)
			{
				UpdateVertex(*beg);
			}
			
		}
		if (openlist.empty()) { cout << "openlist empty" << endl; }
		//输出openlist中的点信息
		//cout << "完成一次compute--------------------------------------------"<<endl;
		//cout << "完成一次compute--------------------------------------------"<<endl;
		Showopenlist();
		Showcloselist();
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

		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_GREEN);
		cout << "最短路径：         ";
		gfstream << "最短路径：         ";
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
		cout << "耗时：" << time2 - time1 << " ms" << endl;
		gfstream << "耗时：" << time2 - time1 << " ms" << endl;
	}
	Dijkstra();
}
//name1.key<name2.key?
bool Graph::Compare_key(int name1, int name2) {
	Caculatekey(name1);
	Caculatekey(name2);
	if (vertex[name1].key1 < vertex[name2].key1) { return true; }
	if (vertex[name1].key1==vertex[name2].key1)
	{
		return vertex[name1].key2 < vertex[name2].key2;
	}
	return false;
}

//D*Lite主要算法
void Graph::Mainmethod() {
	cout << "输入起点和终点：";
	cin >> start >> goal;
	cout << endl;
	Setstartandend(start, goal);
	//初始化h
	for (size_t i = 0; i < vernum; i++)
	{
		vertex[i].h = Manhattan(start, i);
	}
	//初始化算法
	Init(goal);
	cout << "inti done" << endl;
	Computepath();
	float speed;
	cout << "输入车速"<<endl;
	cin >> speed;
	int lastfilename = 10;
	while (start!=goal)//当车辆没有走完全程时
	{
		float time = 0;
		while (time<7.5 && start != goal)
		{
			path.pop();
			
			Vertex next = path.front();
			
			time =time+ totalweight[start][next.name] / speed;
			finish[start][next.name]++;
			finish[next.name][start]++;
			if (finish[start][next.name]>=3)//如果路径反复走过3遍则不能再走
			{
				weight[start][next.name] = INT_MAX;
				weight[next.name][start] = INT_MAX;

			}
			gfstream << " 点 "<<start<<"点走过"<<endl;
			Setstartandend( next.name,goal);
			gfstream <<" 点 "<< start <<" 设为新起始点" <<endl;
		}//while (time<7.5 && start != goal)
		//记录原先的权值
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
		//从停车场接受路面变化
		bool ifchange= RequirevehicleFromWindowsFile(lastfilename);
		if (ifchange){
			gfstream << " 路径权发生变化" << endl;
			//对变化权边的两个端点及其邻居加入update list
			for (size_t i = 0; i < vernum; i++)
			{
				for (size_t j = 0; j < vernum; j++)
				{
					if (oldweight[i][j]!=totalweight[i][j])
					{
						Vertex* s1 = &vertex[i];
						updatelist.push(s1);
						//UpdateVertex(i);
						vector<int> v1 = GetNeighbor(i);
						/*for (auto beg=v1.begin(); beg!=v1.end(); beg++)
						{
							Vertex* s2 = &vertex[*beg];
							updatelist.push(s2);
							//UpdateVertex(*beg);
						}*/
						Vertex* s3 = &vertex[j];
						updatelist.push(s3);
						//UpdateVertex(j);
						vector<int> v2= GetNeighbor(j);
						/*for (auto beg = v2.begin(); beg != v2.end(); beg++)
						{
							Vertex* s4 = &vertex[*beg];
							updatelist.push(s4);
							//UpdateVertex(*beg);
						}*/					
					}
				}
			}
			//更新updatelist的元素
			while (!updatelist.empty())
			{
				Vertex* s = updatelist.top();
				updatelist.pop();
				UpdateVertex(s->name);
			}
			gfstream << " 变权端点更新点完成"<<endl;
			//释放oldweight
			for (size_t i = 0; i < vernum; i++)
			{
				delete oldweight[i];
			
			}
			delete oldweight;
			//更新 openlist
			Updateopenlist();
			//更新closelist
			for (auto beg=closelist.begin();beg!=closelist.end();beg++)
			{
				UpdateVertex((*beg)->name);
			}
			//Showcloselist();
			gfstream << "重新计算路径" << endl;
			Computepath();
		}//if (ifchange)
	}//while (start!=goal)
}
//展示点的信息
void Graph::Showvertex(int name) {
	Vertex v=vertex[name];
	gfstream << "name=" << v.name << "  " << "g=";
	if (v.g == INT_MAX) { gfstream << "∞"; }
	else{ gfstream << v.g;}
	gfstream << "  " << "rhs=";
	if (v.rhs == INT_MAX) { gfstream << "∞"; }
	else { gfstream << v.rhs;}
	gfstream << "  " << "isinopen=" << v.isinopen << "  key1=";
	if (v.key1==INT_MAX) { gfstream << "∞"; }
	else { gfstream << v.key1; }
	gfstream << "  key2=";
	if (v.key2==INT_MAX) { gfstream << "∞"; }
	else { gfstream << v.key2; }
	if (v.next!=NULL){
		gfstream << "  next=" << v.next->name << endl;
	}
	else
	{
		gfstream << "  next=null" << endl;
	}
}
void Graph::Showvertex(Vertex* pointer ) {
	Vertex v = *pointer;
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
//针对带有无穷的非负数数的加法
int Graph::Add(int left, int right) {
	if (left==INT_MAX || right==INT_MAX)
	{
		return INT_MAX;
	}
	else {
		return left + right;
	}
}
void Graph::Showopenlist() {
	//输出openlist中的点信息
	priority_queue<Vertex*, vector<Vertex*>, cmp> openlistcopy = openlist;
	gfstream << "openlist:" << endl;
	while (!openlistcopy.empty()) {
		Showvertex(openlistcopy.top());
		openlistcopy.pop();
	}
}
void Graph::Showcloselist() {
	gfstream << " closelist: ---------------------------------------------" << endl;
	for (auto beg=closelist.begin();beg!=closelist.end();beg++)
	{
		Showvertex(*beg);
	}
}
//Dijkstra算法求出最短路径
void Graph::Dijkstra() {
	//初始化
	for (size_t i = 0; i < vernum; i++)
	{
		vertex[i].distance = totalweight[start][i];
		vertex[i].visit = false;
		vertex[i].Dpath = "";
	}
	
	vertex[start].distance = 0;
	int count = 1;
	clock_t time1 = clock();
	while (count!=vernum)
	{
		int temp = 0;
		int min = INT_MAX;
		for (int i = 0; i < vernum; i++) {//找到一个最小值点
			if (!vertex[i].visit && vertex[i].distance < min) {
				min = vertex[i].distance;
				temp = i;
			}
		}
		vertex[temp].visit = true;
		++count;
		for (int i = 0; i < vernum; i++) {
			//注意这里的条件arc[temp][i]!=INT_MAX必须加，不然会出现溢出，从而造成程序异常
			if (!vertex[i].visit && totalweight[temp][i] != INT_MAX && ( Add(vertex[temp].distance ,totalweight[temp][i])) <= vertex[i].distance) {
				//如果新得到的边可以影响其他为访问的顶点，那就就更新它的最短路径和长度
				vertex[i].distance = vertex[temp].distance + totalweight[temp][i];
				vertex[i].Dpath = vertex[temp].Dpath + "->" + to_string(i);
			}
		}
		
	}//while (count!=vernum)
	clock_t time2 = clock();
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_GREEN);
	cout << "Dijkstra最短路径为:" <<start<< vertex[goal].Dpath <<"total:"<<vertex[goal].distance<< endl;
    gfstream << "Dijkstra最短路径为:" << start << vertex[goal].Dpath << "total:" << vertex[goal].distance << endl;
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_INTENSITY | FOREGROUND_RED |FOREGROUND_GREEN | FOREGROUND_BLUE);
	cout << "耗时：" << time2 - time1 << " ms" << endl;
	gfstream << "耗时：" << time2 - time1 << " ms" << endl;

}
//模拟接受广播
bool Graph::RequirevehicleFromWindowsFile(int &lastfilename) {
	int currentfilename;
	//如果第一次接受变化
	if (lastfilename==10)
	{
		currentfilename = rand();
		if(currentfilename>=10){currentfilename= currentfilename % 10;}
		string filename = "C:\\Users\\chongliu\\Desktop\\华人运通\\轨迹规划\\VisualStudiocode\\vehiclechangecondition"
			+to_string(currentfilename)+".txt";
		ifstream infile;
		gfstream <<"document name :"<< to_string(currentfilename)<<endl;
		infile.open(filename);
		bool park; int vertex1, vertex2;
		while (!infile.eof())
		{
			infile>> park >> vertex1 >> vertex2;
			//cout <<"车辆状况  "<< park << "  " << vertex1 << "   " << vertex2 << endl;
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
			//与start不相邻
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

			string filename = "C:\\Users\\chongliu\\Desktop\\华人运通\\轨迹规划\\VisualStudiocode\\vehiclechangecondition"
				+ to_string(currentfilename) + ".txt";
			ifstream infile;
			infile.open(filename);
			bool park; int vertex1, vertex2;
			while (!infile.eof())
			{
				infile >> park >> vertex1 >> vertex2;
				//cout << "车辆状况  " << park << "  " << vertex1 << "   " << vertex2 << endl;
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
				//与start不相邻
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
//临时建立车辆变化文件
void Graph::Tempcreatfile() {
	for (int i = 0; i < 10; i++)
	{
		fstream f;
		string filename="C:\\Users\\chongliu\\Desktop\\华人运通\\轨迹规划\\VisualStudiocode\\vehiclechangecondition"
			+ to_string(i) + ".txt";
		f.open(filename, ios::app);
		if (f.bad())
		{
			cout << "打开文件出错" << endl;
			return;
		}
		for (size_t i = 0; i < 200; i++)
		{
			int random = rand();
			random = random % 200;
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
	string filename = "C:\\Users\\chongliu\\Desktop\\华人运通\\轨迹规划\\VisualStudiocode\\totalweight.txt";
	f.open(filename,ios::out);
	if (f.bad())
	{
		cout << "打开文件出错" << endl;
		return;
	}

	for (int i = 0; i < 200; i++)
	{
		vector<int> v = GetNeighbor(i);
		f << "p" << i ;
		for (auto beg = v.begin(); beg != v.end();beg++) {
			f << "-->p" << *beg << "[ " << totalweight[i][*beg] << " ]  ";
		}
		f << "\n";
	}
	f.close();
	
}
void Graph::Updateopenlist() {
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
	while ( !tmp2.empty() )
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

