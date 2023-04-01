#ifndef GRAPH_H_
#define GRAPH_H_

#include "geometry.h"
#include "constant.h"
#include <vector>
using namespace std;
using namespace geometry;

struct Node
{
	Vec2 coordinate;
	bool is_obstacle;
	bool is_workbench;
	int workbench_id;
	vector<Node*> neighbors;
};

class Graph
{
public:
	vector<Node> nodes;
	void init(char map[MAP_SIZE][MAP_SIZE]);
private:
	//vector<Node> nodes;

};

#endif