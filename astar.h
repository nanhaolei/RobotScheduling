#ifndef ASTAR_H_
#define ASTAR_H_

#include "graph.h"
#include <queue>
#include <vector>

class AStar
{
private:
	Node* start;
	Node* goal;
	unordered_map<Node*, Node*> parent;
	unordered_map<Node*, double> g;
	//vector<Node*> closed;
	struct f_compare {
		AStar* astar;
		bool operator()(Node* a, Node* b) {
			return astar->f(a) > astar->f(b);
		}
	};
	priority_queue <Node*, vector<Node*>, f_compare> open;
	
public:
	AStar(Node* start_, Node* goal_) : start(start_), goal(goal_) {};
	double cost(Node* cur_node, Node* neigh_node);
	double heuristic(Node* cur_node);
	double f(Node* cur_node);
	vector<Node*> searching();
	vector<Node*> extractPath();
	bool isBesideObstacle(Node* node);
};

#endif