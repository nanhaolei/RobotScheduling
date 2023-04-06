#include "astar.h"
#include <iostream>
#include <cassert>

vector<Node*> AStar::searching() {
	if (start->id == goal->id)
		return {};
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

void AStar::smoothPath(vector<Node*>& path) {
	if (path.size() < 100)
		return;
	int maxIter = 100;
	int iter = 0;
	bool changed = true;
	while (changed && iter < maxIter) {
		changed = false;
		for (int i = 1; i < path.size() - 1; i++) {
			Node* prev = path[i - 1];
			Node* cur = path[i];
			Node* next = path[i + 1];
			if (!prev->is_obstacle && !next->is_obstacle && cost(prev, next) <= cost(prev, cur) + cost(cur, next)) {
				path.erase(path.begin() + i);
				path.insert(path.begin() + i, parent[next]);
				changed = true;
			}
		}
		iter++;
	}
}

double AStar::cost(Node* cur_node, Node* neigh_node) {
	if (isBesideObstacle(neigh_node) == 2) {
		return INT_MAX;
	}
	if (isBesideObstacle(neigh_node) == 1) {
		return 1000;
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

int AStar::isBesideObstacle(Node* node) {
	auto neighbors = node->neighbors;
	// 节点在角落里
	if (neighbors.size() == 3) {
		return 2;
	}
	
	if (neighbors.size() == 8) {
		// 节点上下左右有障碍
		if (neighbors[0]->is_obstacle || neighbors[1]->is_obstacle || neighbors[2]->is_obstacle || neighbors[3]->is_obstacle) {
			return 2;
		}
		// 节点斜向有障碍
		else if (neighbors[4]->is_obstacle || neighbors[5]->is_obstacle || neighbors[6]->is_obstacle || neighbors[7]->is_obstacle) {
			return 1;
		}

		//for (auto nei : neighbors) {
		//	for (auto nei_nei : nei->neighbors) {
		//		// 节点上下左右有障碍
		//		if (neighbors[0]->is_obstacle || neighbors[1]->is_obstacle || neighbors[2]->is_obstacle || neighbors[3]->is_obstacle) {
		//			return 2;
		//		}
		//		// 节点斜向有障碍
		//		else if (neighbors[4]->is_obstacle || neighbors[5]->is_obstacle || neighbors[6]->is_obstacle || neighbors[7]->is_obstacle) {
		//			return 1;
		//		}
		//	}
		//}

		//else if (neighbors[4]->is_obstacle || neighbors[5]->is_obstacle || neighbors[6]->is_obstacle || neighbors[7]->is_obstacle) {
		//	for (auto nei : neighbors) {
		//		for (auto nei_nei : nei->neighbors) {
		//			// 节点上下左右有障碍
		//			if (neighbors[0]->is_obstacle || neighbors[1]->is_obstacle || neighbors[2]->is_obstacle || neighbors[3]->is_obstacle) {
		//				return 2;
		//			}
		//			// 节点斜向有障碍
		//			else if (neighbors[4]->is_obstacle || neighbors[5]->is_obstacle || neighbors[6]->is_obstacle || neighbors[7]->is_obstacle) {
		//				return 1;
		//			}
		//		}
		//	}
		//}
		
	}
	
	return 0;
}

vector<Vec2> AStar::getCoorPath(const vector<Node*>& path)
{
	vector<Vec2> coor_path;
	for (auto node : path) {
		coor_path.emplace_back(node->coordinate);
	}
	return coor_path;
}

