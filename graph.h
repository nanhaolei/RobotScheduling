#ifndef GRAPH_H_
#define GRAPH_H_

#include "geometry.h"
#include "constant.h"
#include <vector>
#include <unordered_map>
#include <map>
#include <unordered_set>
using namespace std;
using namespace geometry;

struct Node
{
	Vec2 coordinate;
	bool is_obstacle;
	bool is_workbench;
	int workbench_id;
	//vector<Node*> neighbors;
	unordered_set<Node*> neighbors;
	//array<int, 2> map_index;
};

class Graph
{
public:
	void init(char map[MAP_SIZE][MAP_SIZE]);
	void initNeighbors();
private:
	vector<Node> nodes;
	std::map<array<int, 2>, Node*> index_to_node;
	std::map<Node*, array<int, 2>> node_to_index;
};

#endif