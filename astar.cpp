#include "astar.h"
#include <iostream>

vector<Node*> AStar::searching() {
	f_compare compare = { this };
	open = priority_queue<Node*, vector<Node*>, f_compare>(compare);
	parent[start] = start;
	g[start] = 0;
	g[goal] = INT_MAX;
	open.push(start);
	while (!open.empty()) {
		Node* cur_node = open.top();
		open.pop();
		if (cur_node == goal) {
			break;
		}
		for (auto neighbor : cur_node->neighbors) {
			double new_cost = g[cur_node] + cost(cur_node, neighbor);
			/*if (g.find(neighbor) == g.end()) {
				g[neighbor] = INT_MAX;
			}
			if (new_cost < g[neighbor]) {*/
			if (g.find(neighbor) == g.end() || new_cost < g[neighbor]) {
				g[neighbor] = new_cost;
				parent[neighbor] = cur_node;
				open.push(neighbor);
			}
		}
	}
	return extractPath();
}

vector<Node*> AStar::extractPath() {
	vector<Node*> path;
	Node* cur_node = goal;
	while (cur_node != start) {
		path.push_back(cur_node);
		cur_node = parent[cur_node];
	}
	path.push_back(start);
	reverse(path.begin(), path.end());
	return path;
}
//vector<Node*> AStar::extractPath() {
//	vector<Node*> path;
//	path.emplace_back(goal);
//	Node* cur_node = goal;
//	while (true) {
//		cur_node = parent[cur_node];
//		path.emplace_back(cur_node);
//		if (cur_node == start) {
//			break;
//		}
//	}
//	reverse(path.begin(), path.end());
//	return path;
//}

double AStar::cost(Node* a, Node* b) {
	return distance(a->coordinate, b->coordinate);
}

double AStar::heuristic(Node* cur_node) {
	/*double dx = abs(goal->coordinate[0] - cur_node->coordinate[0]);
	double dy = abs(goal->coordinate[1] - cur_node->coordinate[1]);
	return dx + dy;*/
	return distance(cur_node->coordinate, goal->coordinate);
}

double AStar::f(Node* cur_node) {
	return g[cur_node] + heuristic(cur_node);
}


