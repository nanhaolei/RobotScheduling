#include "dstar.h"

DStar::DStar(Node* start_, Node* goal_, Graph* graph_) : start(start_), goal(goal_),graph(graph_) {
	rhs[goal] = 0.0;
	U[goal] = calculateKey(goal);
	for (auto node : graph->nodes) {
		rhs[node] = INT_MAX;
		g[node] = INT_MAX;
	}
};

vector<Node*> DStar::searching() {
	while (true) {
		auto pair = topKey();
		auto s = pair.first;
		auto v = pair.second;
		if (v >= calculateKey(start) && rhs[start] == g[start]) {
			return extractPath();
			//break;
		}
		auto k_old = v;
		U.erase(start);
		if (k_old < calculateKey(s)) {
			U[s] = calculateKey(s);
		}
		else if (g[s] > rhs[s]) {
			g[s] = rhs[s];
			for (auto neigh : s->neighbors) {
				updateVertex(neigh);
			}
		}
		else {
			g[s] = INT_MAX;
			updateVertex(s);
			for (auto neigh : s->neighbors) {
				updateVertex(neigh);
			}
		}
	}
	return {};
}

void DStar::updateVertex(Node* s) {
	if (s != goal) {
		rhs[s] = INT_MAX;
		for (auto nei : s->neighbors) {
			rhs[s] = min(rhs[s], g[nei] + cost(s, nei));
		}
	}
	if (U.find(s) != U.end()) {
		U.erase(s);
	}
	if (g[s] != rhs[s]) {
		U[s] = calculateKey(s);
	}
}

vector<double> DStar::calculateKey(Node* s) {
	return { min(g[s], rhs[s]) + h(start, s), min(g[s], rhs[s]) };
}

pair<Node*, vector<double>> DStar::topKey() {
	auto cmp = [](auto const& lhs, auto const& rhs) {
		return lhs.second < rhs.second;
	};
	auto s = min_element(U.begin(), U.end(), cmp)->first;
	return make_pair(s, U[s]);
}

vector<Node*> DStar::extractPath() {
	vector<Node*> path;
	path.emplace_back(start);
	auto s = start;
	for (int i = 0; i < 1000; i++) {
		unordered_map<Node*, double> g_list;
		for (auto neigh : s->neighbors) {
			if (!neigh->is_obstacle) {
				g_list[neigh] = g[neigh];
			}
		}
		auto cmp = [](auto const& lhs, auto const& rhs) {
			return lhs.second < rhs.second;
		};
		auto s = min_element(g_list.begin(), g_list.end(), cmp)->first;
		path.emplace_back(s);
		if (s == goal) {
			break;
		}
	}
	return path;
}

double DStar::cost(Node* cur_node, Node* neigh_node) {
	if (isBesideObstacle(neigh_node) == 2) {
		return INT_MAX;
	}
	if (isBesideObstacle(neigh_node) == 1) {
		return 1000;
	}
	return 1.0;
}

double DStar::h(Node* start, Node* goal) {
	// 欧几里得距离 任意方向移动
	return distance(start->coordinate, goal->coordinate);
}

int DStar::isBesideObstacle(Node* node) {
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

	}

	return 0;
}

vector<Vec2> DStar::getCoorPath(const vector<Node*>& path) {
	vector<Vec2> coor_path;
	for (auto node : path) {
		coor_path.emplace_back(node->coordinate);
	}
	return coor_path;
}

void DStar::smoothPath(vector<Node*>& path) {
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