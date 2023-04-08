#include "graph.h"
#include <iostream>

Graph::Graph(char map[MAP_SIZE][MAP_SIZE]) {
	initNodes(map);
	initNeighbors();
}

void Graph::init(char map[MAP_SIZE][MAP_SIZE]) {
	initNodes(map);
	initNeighbors();
}

void Graph::initNodes(char map[MAP_SIZE][MAP_SIZE]) {
	int workbench_id = 0;
	int id = 0;
	// 从左上角开始
	for (int i = 0; i < MAP_SIZE; i++) {
		for (int j = 0; j < MAP_SIZE; j++) {
			Node* node = new Node();
			node->id = id;
			++id;
			node->coordinate[0] = 0.25 + j * 0.5; // x坐标
			node->coordinate[1] = 0.25 + (MAP_SIZE - 1 - i) * 0.5; // y坐标
			// 障碍物
			if (map[i][j] == '#') {
				node->is_obstacle = true;
				node->is_workbench = false;
				node->workbench_id = -1;
			}
			// 工作台
			else if ('1' <= map[i][j] && map[i][j] <= '9') {
				node->is_obstacle = false;
				node->is_workbench = true;
				node->workbench_id = workbench_id;
				workbench_id++;
			}
			else {
				node->is_obstacle = false;
				node->is_workbench = false;
				node->workbench_id = -1;
			}
			node->map_index = { i, j };
			nodes.emplace_back(node);
			index_to_node[{i, j}] = node;
		}
	}
}

void Graph::initNeighbors() {
	size_t size = nodes.size();
	for (int i = 0; i < size; i++) {
		Node* cur_node = nodes[i];
		array<int, 2> cur_index = cur_node->map_index;

		if (index_to_node.find({ cur_index[0], cur_index[1] - 1 }) != index_to_node.end()){
			Node* left = index_to_node[{cur_index[0], cur_index[1] - 1}];
			//cur_node->neighbors.insert(left);
			//cur_node->neighbors["left"] = left;
			//cur_node->neighbors[0] = left;
			cur_node->neighbors.emplace_back(left);
		}

		if (index_to_node.find({ cur_index[0], cur_index[1] + 1 }) != index_to_node.end()) {
			Node* right = index_to_node[{cur_index[0], cur_index[1] + 1}];
			//cur_node->neighbors.insert(right);
			//cur_node->neighbors["right"] = right;
			//cur_node->neighbors[2] = right;
			cur_node->neighbors.emplace_back(right);
		}

		if (index_to_node.find({ cur_index[0] - 1, cur_index[1] }) != index_to_node.end()) {
			Node* top = index_to_node[{cur_index[0] - 1, cur_index[1]}];
			//cur_node->neighbors.insert(top);
			//cur_node->neighbors["top"] = top;
			//cur_node->neighbors[1] = top;
			cur_node->neighbors.emplace_back(top);
		}

		if (index_to_node.find({ cur_index[0] + 1, cur_index[1] }) != index_to_node.end()) {
			Node* bottom = index_to_node[{cur_index[0] + 1, cur_index[1]}];
			//cur_node->neighbors.insert(bottom);
			//cur_node->neighbors["bottom"] = bottom;
			//cur_node->neighbors[3] = bottom;
			cur_node->neighbors.emplace_back(bottom);
		}

		if (index_to_node.find({ cur_index[0] - 1, cur_index[1] - 1 }) != index_to_node.end()) {
			Node* neighbor = index_to_node[{cur_index[0] - 1, cur_index[1] - 1}];
			//cur_node->neighbors.insert(neighbor);
			//cur_node->neighbors["left_top"] = neighbor;
			//cur_node->neighbors[7] = neighbor;
			cur_node->neighbors.emplace_back(neighbor);
		}

		if (index_to_node.find({ cur_index[0] - 1, cur_index[1] + 1 }) != index_to_node.end()) {
			Node* neighbor = index_to_node[{cur_index[0] - 1, cur_index[1] + 1}];
			//cur_node->neighbors.insert(neighbor);
			//cur_node->neighbors["right_top"] = neighbor;
			//cur_node->neighbors[7] = neighbor;
			cur_node->neighbors.emplace_back(neighbor);
		}

		if (index_to_node.find({ cur_index[0] + 1, cur_index[1] - 1 }) != index_to_node.end()) {
			Node* neighbor = index_to_node[{cur_index[0] + 1, cur_index[1] - 1}];
			//cur_node->neighbors.insert(neighbor);
			//cur_node->neighbors["left_bottom"] = neighbor;
			//cur_node->neighbors[7] = neighbor;
			cur_node->neighbors.emplace_back(neighbor);
		}

		if (index_to_node.find({ cur_index[0] + 1, cur_index[1] + 1 }) != index_to_node.end()) {
			Node* neighbor = index_to_node[{cur_index[0] + 1, cur_index[1] + 1}];
			//cur_node->neighbors.insert(neighbor);
			//cur_node->neighbors["right_bottom"] = neighbor;
			//cur_node->neighbors[7] = neighbor;
			cur_node->neighbors.emplace_back(neighbor);
		}
		
	}
}

Node* Graph::workbenchToNode(int workbench_id) {
	for (auto node : nodes) {
		if (node->workbench_id == workbench_id) {
			return node;
		}
	}
	cerr << "err:workbenchToNode" << endl;
	return nullptr;
}

Node* Graph::coordinateToNode(Vec2 coordinate) {
	double min_dist = DBL_MAX;
	Node* cur_node = nullptr;
	for (auto node : nodes) {
		double dx = node->coordinate[0] - coordinate[0];
		double dy = node->coordinate[1] - coordinate[1];
		double dist = sqrt(dx * dx + dy * dy);
		if (dist < min_dist) {
			min_dist = dist;
			cur_node = node;
		}
		if (abs(dx) < GRID_LENGTH / 2 + EPSILON  && abs(dy) < GRID_LENGTH / 2 + EPSILON) {
			return node;
		}
	}
	cerr << "err:coordinateToNode" << endl;
	return cur_node;
	//return nullptr;
}

Node* Graph::indexToNode(array<int, 2> index) {
	return index_to_node[index];
}

Node* Graph::robotToNode(Robot* robot) {
	Vec2 coor = robot->getCoordinate();
	return coordinateToNode(coor);
}

void Graph::updateObstacle(vector<Vec2> robots_coor) {
	if(robotNodes.size() > 0) {
		for (auto node : robotNodes) {
			node->is_obstacle = false;
		}
		robotNodes.clear();
	}
	for (auto coor : robots_coor) {
		Node* cur_node = coordinateToNode(coor);
		cur_node->is_obstacle = true;
		robotNodes.emplace_back(cur_node);
		/*for (auto neigh : node->neighbors) {
			neigh->is_obstacle = true;
			robotNodes.emplace_back(neigh);
		}*/
	}
}