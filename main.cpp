#include"ParkGraph.h"
#include"Vertex.h"
#include"Car.h"
#include<functional>
#include<queue>
#include<vector>
#include<bitset>
using namespace std;
///*
void addcar(Graph *graph, int start, int goal, int carname) {
	Car car(graph, start, goal, carname);
	car.Mainmethod();
	cout << "complete addcar" << endl;

}
int main() {	
	//int start; int goal;
	//cin >> start >> goal;
	Graph graph;
	graph.CreatGraph();
	thread t1(&Graph::Mainmethod,&graph);
	t1.detach();
	//this_thread::sleep_for(std::chrono::seconds(8));//10√Î
	thread t2(&addcar, &graph, 0, 255, 0);
	t2.detach();
	while (1) {

	}

	return 0;
	
}

//*/


