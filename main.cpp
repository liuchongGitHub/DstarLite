#include"ParkGraph.h"
#include"Vertex.h"
#include"Car.h"
#include<functional>
#include<queue>
#include<vector>
#include<bitset>
using namespace std;
///*
void addcarDijkstra(Graph *graph, int carname) {
	Car car( graph, carname);
	car.Dijkstra();

}
void addcarDstarLite(Graph *graph, int carname) {
	Car car(graph, carname);
	car.DstarLite();

}
int main() {	
	//int start; int goal;
	//cin >> start >> goal;
	Graph graph;
	graph.CreatGraph();
	thread t1(&Graph::Mainmethod,&graph);
	t1.detach();
	//地图创建完毕模拟停车场
	//this_thread::sleep_for(std::chrono::seconds(8));//10秒
	thread t2(&addcarDijkstra, &graph, 0);
	t2.detach();
	thread t3(&addcarDstarLite, &graph, 1);
	t3.detach();
	while (1) {

	}

	return 0;
	
}

//*/


