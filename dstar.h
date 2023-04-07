#ifndef DSTAR_H_
#define DSTAR_H_

#include "graph.h"
#include "geometry.h"
#include <queue>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
using namespace std;
using namespace geometry;

class DStar
{
private:
	Node* start;
	Node* goal;
	unordered_map<Node*, Node*> parent;
	unordered_map<Node*, double> g;
	unordered_map<Node*, double> rhs;
	unordered_map<Node*, vector<double>> U;
	Graph* graph;

public:
	DStar(Node* start_, Node* goal_, Graph* graph);
	vector<Node*> searching();
	vector<Node*> extractPath();
	double cost(Node* cur_node, Node* neigh_node);
	double h(Node* start, Node* goal);
	int isBesideObstacle(Node* node);
	static vector<Vec2> getCoorPath(const vector<Node*>& path);
	void smoothPath(vector<Node*>& path);

	vector<double> CalculateKey(Node* s);
	pair<Node*, vector<double>> TopKey();
	void UpdateVertex(Node* s);
};

#endif