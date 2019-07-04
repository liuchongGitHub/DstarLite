#include"ParkGraph.h"
#include<functional>
#include<queue>
#include<vector>
#include<queue>
using namespace std;
int main() {	
	
	Graph graph;
	graph.CreatGraph();
	//graph.Tempcreatfile();	
	int start; int goal;
	graph.Mainmethod();
	system("pause");//防止命令行自动退出
	return 0;
}



