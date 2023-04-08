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
	
public:
	AStar(Node* start_, Node* goal_) : start(start_), goal(goal_) {};
	AStar(Node* start_, Node* goal_, unordered_set<Node*> robot_nodes_, Robot* cur_robot_) : 
		start(start_), goal(goal_), robot_nodes(robot_nodes_),cur_robot(cur_robot_) {};
	double cost(Node* cur_node, Node* neigh_node);
	double heuristic(Node* cur_node);
	double f(Node* cur_node);
	vector<Node*> searching();
	vector<Node*> extractPath();
	void smoothPath(vector<Node*>& path);
	int isBesideObstacle(Node* node);
	static vector<Vec2> getCoorPath(const vector<Node*>& path);
	unordered_set<Node*> robot_nodes;
	Robot* cur_robot;
};

#endif