#ifndef ASTAR_H_
#define ASTAR_H_

#include "graph.h"
#include <queue>
#include <vector>
#include <map>

class AStar
{
public:
	void searching();
	double cost(const Node* a, const Node* b);
	double heuristic(const Node* cur_node);
	double f(const Node* cur_node);
	vector<Node*> extractPath();

private:
	Node* start;
	Node* goal;
	struct f_compare {
		bool operator()(const Node* a, const Node* b) {
			return f(a) > f(b);
		}
	};
	priority_queue < Node*, vector<Node*>, f_compare> open;
	std::map<Node*, Node*> parent;
	std::map<Node*, double> g;
	//vector<Node*> closed;
};


void AStar::searching() {
	parent[start] = start;
	g[start] = 0;
	g[goal] = INT_MAX;
	open.push(start);

	while (!open.empty()) {
		Node* cur_node = open.top();
		open.pop();
		//closed.emplace_back(cur_node);
		if (cur_node == goal) {
			break;
		}
		for (auto neighbor : cur_node->neighbors) {
			double new_cost = g[cur_node] + cost(cur_node, neighbor);
			if (g.find(neighbor) == g.end()) {
				g[neighbor] = INT_MAX;
			}
			// if (g.find(neighbor) == g.end() || new_cost < g[neighbor])
			if (new_cost < g[neighbor]) {
				g[neighbor] = new_cost;
				parent[neighbor] = cur_node;
				open.push(neighbor);
			}
		}

	}
}

double AStar::cost(const Node* a, const Node* b) {
	return length(a->coordinate, b->coordinate);
}

double AStar::heuristic(const Node* cur_node) {
	/*double dx = abs(goal->coordinate[0] - cur_node->coordinate[0]);
	double dy = abs(goal->coordinate[1] - cur_node->coordinate[1]);
	return dx + dy;*/
	return length(cur_node->coordinate, goal->coordinate);
}

double AStar::f(const Node* cur_node) {
	return g[cur_node] + heuristic(cur_node);
}

vector<Node*> AStar::extractPath() {
	vector<Node*> path;
	path.emplace_back(goal);
	Node* cur_node = goal;
	while (true) {
		cur_node = parent[cur_node];
		path.emplace_back(cur_node);
		if (cur_node == start) {
			break;
		}
	}
	return path;
}

#endif