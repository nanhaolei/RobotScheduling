#include "astar.h"
#include <iostream>
#include <cassert>

vector<Node*> AStar::searching() {
	int iter_max = 9000;
	int iter = 0;
	if (start->id == goal->id)
		return { goal };
	f_compare compare = { this };
	open = priority_queue<Node*, vector<Node*>, f_compare>(compare);
	parent[start] = start;
	g[start] = 0;
	g[goal] = INT_MAX;
	open.push(start);
	while (!open.empty() /*&& iter < iter_max*/) {
		++iter;
		Node* cur_node = open.top();
		open.pop();
		if (cur_node == goal) {
			return extractPath();
		}
		for (auto neighbor : cur_node->neighbors) {
			/*if (closed.find(neighbor) != closed.end()) 
				continue;
			if (neighbor->is_obstacle && *neighbor!=*start)
				continue;*/
			if (neighbor->is_obstacle || obstacle_robot_nodes.find(neighbor)!=obstacle_robot_nodes.end()) 
				continue;
			//if (neighbor->is_obstacle) continue;
			double new_cost = g[cur_node] + cost(cur_node, neighbor);
			if (g.find(neighbor) == g.end() || new_cost < g[neighbor]) {
				g[neighbor] = new_cost;
				parent[neighbor] = cur_node;
				open.push(neighbor);
			}
		}
	}
	cerr << "err:AStar::searching()" << endl;
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
	if (path.size() < 50)
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
	double coff = isBesideObstacle(neigh_node);
	return coff;
	//return 1.0;
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
	return g[cur_node] + e * heuristic(cur_node);
}

int AStar::isBesideObstacle(Node* node) {
	auto neighbors = node->neighbors;
	// 节点在角落里或墙边
	if (neighbors.size() == 3 || neighbors.size() == 5) {
		return 10000;
	}
	// 携带物品
	if ((cur_robot != nullptr && cur_robot->getGoodsType() > 0) || cur_robot == nullptr) {
		// 节点上下左右有障碍
		for (int i = 0; i < 4; ++i) {
			if (neighbors[i]->is_obstacle)
				return 10000;
		}
		// 节点斜向有障碍
		for (int i = 4; i < 8; ++i) {
			if (neighbors[i]->is_obstacle)
				return 1000;
		}
		// 节点周围障碍物越多 惩罚越大
		int coeffcient = 1;
		for (auto neigh : neighbors) {
			if (neigh->is_obstacle) {
				++coeffcient;
			}
		}
		return coeffcient;
	}
	// 不携带物品
	else {
		int count = 0;
		for (int i = 0; i < 4; ++i) {
			if (neighbors[i]->is_obstacle)
				++count;
		}
		// 上下左右的障碍大于1个
		if (count > 1)
			return 10000;
		else if (count == 1)
			return 4;
		// 节点周围障碍物越多 惩罚越大
		int coeffcient = 1;
		for (auto neigh : neighbors) {
			if (neigh->is_obstacle) {
				++coeffcient;
			}
		}
		return coeffcient;
	}
	return 1;
}

vector<Vec2> AStar::getCoorPath(const vector<Node*>& path)
{
	vector<Vec2> coor_path;
	for (auto node : path) {
		coor_path.emplace_back(node->coordinate);
	}
	return coor_path;
}

