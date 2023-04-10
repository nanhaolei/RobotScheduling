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
#include <climits>
#include "robot.h"
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
	bool operator==(const Node& other) const {
		return map_index[0] == other.map_index[0] && map_index[1] == other.map_index[1];
	}
	bool operator!=(const Node& other) const {
		return !(*this == other);
	}
	void print() {
		cerr << map_index[0] << ',' << map_index[1] << endl;
	}
};

class Graph
{
public:
	vector<Node*> nodes;
	vector<Node*> robotNodes;
	Graph(char map[MAP_SIZE][MAP_SIZE]);
	Graph() {};
	void initNodes(char map[MAP_SIZE][MAP_SIZE]);
	void initNeighbors();
	Node* workbenchToNode(int workbench_id);
	Node* coordinateToNode(Vec2 coordinate);
	Node* indexToNode(array<int, 2> index);
	Node* robotToNode(Robot* robot);
	void init(char map[MAP_SIZE][MAP_SIZE]);
	void updateObstacle(vector<Vec2> robots_coor);
	
private:
	map<array<int, 2>, Node*> index_to_node;
};


#endif