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
			return extractPath();
			//break;
		}
		for (auto neighbor : cur_node->neighbors) {
			if (neighbor->is_obstacle) continue;
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
	// 目标不可达
	return {};
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

double AStar::cost(Node* cur_node, Node* neigh_node) {
	if (isBesideObstacle(neigh_node)) {
		return INT_MAX / 2;
	}
	return 1.0;
	//return distance(cur_node->coordinate, neigh_node->coordinate);
}

double AStar::heuristic(Node* cur_node) {
	// 曼哈顿距离 四方向移动
	/*double dx = abs(goal->coordinate[0] - cur_node->coordinate[0]);
	double dy = abs(goal->coordinate[1] - cur_node->coordinate[1]);
	return dx + dy;*/

	// 对角距离 八方向移动
	/*double dx = abs(goal->coordinate[0] - cur_node->coordinate[0]);
	double dy = abs(goal->coordinate[1] - cur_node->coordinate[1]);
	double D = 1, D2 = 1;
	return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)*/

	// 欧几里得距离 任意方向移动
	return distance(cur_node->coordinate, goal->coordinate);
}

double AStar::f(Node* cur_node) {
	return g[cur_node] + heuristic(cur_node);
}

bool AStar::isBesideObstacle(Node* node) {
	for (auto neighbor : node->neighbors) {
		if (neighbor->is_obstacle) {
			return true;
		}
	}
	return false;
}

