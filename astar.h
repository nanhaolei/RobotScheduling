#ifndef ASTAR_H_
#define ASTAR_H_

#include "graph.h"
#include <queue>
#include <vector>
#include <algorithm>
using namespace std;
using namespace geometry;

class AStar
{
private:
	struct NodeHash {
		size_t operator()(Node* node) const {
			return hash<int>()(node->coordinate[0]) ^ hash<int>()(node->coordinate[1]);
		}
	};
	Node* start;
	Node* goal;
	unordered_map<Node*, Node*, NodeHash> parent;
	unordered_map<Node*, double> g;
	struct f_compare {
		AStar* astar;
		bool operator()(Node* a, Node* b) {
			return astar->f(a) > astar->f(b);
		}
	};
	priority_queue <Node*, vector<Node*>, f_compare> open;
	double e = 2.5;
	
public:
	AStar(Node* start_, Node* goal_) : start(start_), goal(goal_) {};
	vector<Node*> searching();
	vector<Node*> extractPath();
	void smoothPath(vector<Node*>& path);
	double cost(Node* cur_node, Node* neigh_node);
	double heuristic(Node* cur_node);
	double f(Node* cur_node);
	int isBesideObstacle(Node* node);
	static vector<Vec2> getCoorPath(const vector<Node*>& path);

	unordered_set<Node*> obstacle_robot_nodes;
	Robot* cur_robot = nullptr;
};

#endif