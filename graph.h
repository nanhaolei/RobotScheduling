#ifndef GRAPH_H_
#define GRAPH_H_

#include "geometry.h"
#include "constant.h"
#include <vector>
#include <unordered_map>
#include <map>
#include <unordered_set>
#include <iostream>
#include <string>
using namespace std;
using namespace geometry;

struct Node
{
	int id;
	array<int, 2> map_index;
	Vec2 coordinate;
	bool is_obstacle;
	bool is_workbench;
	int workbench_id;
	vector<Node*> neighbors;
	//unordered_set<Node*> neighbors;
	//unordered_map<int, Node*> neighbors;
	
	void print() {
		cerr << map_index[0] << ',' << map_index[1] << endl;
	}
};

class Graph
{
public:
	vector<Node*> nodes;
	Graph(char map[MAP_SIZE][MAP_SIZE]);
	Graph() {};
	void initNodes(char map[MAP_SIZE][MAP_SIZE]);
	void initNeighbors();
	Node* workbenchToNode(int workbench_id);
	Node* coordinateToNode(Vec2 coordinate);
	Node* indexToNode(array<int, 2> index);
	void init(char map[MAP_SIZE][MAP_SIZE]);
	
private:
	map<array<int, 2>, Node*> index_to_node;
};


#endif