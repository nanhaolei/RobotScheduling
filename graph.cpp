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
	// 从左上角开始
	for (int i = 0; i < MAP_SIZE; i++) {
		for (int j = 0; j < MAP_SIZE; j++) {
			Node* node = new Node();
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
			node->map_index = { i,j };
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
			if(!left->is_obstacle)
				cur_node->neighbors.insert(left);
		}
		if (index_to_node.find({ cur_index[0], cur_index[1] + 1 }) != index_to_node.end()) {
			Node* right = index_to_node[{cur_index[0], cur_index[1] + 1}];
			if (!right->is_obstacle)
				cur_node->neighbors.insert(right);
		}
		if (index_to_node.find({ cur_index[0] - 1, cur_index[1] }) != index_to_node.end()) {
			Node* top = index_to_node[{cur_index[0] - 1, cur_index[1]}];
			if (!top->is_obstacle)
				cur_node->neighbors.insert(top);
		}
		if (index_to_node.find({ cur_index[0] + 1, cur_index[1] }) != index_to_node.end()) {
			Node* bottom = index_to_node[{cur_index[0] + 1, cur_index[1]}];
			if (!bottom->is_obstacle)
				cur_node->neighbors.insert(bottom);
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
	for (auto node : nodes) {
		if (abs(node->coordinate[0] - coordinate[0]) < JUDGE_DISTANCE && abs(node->coordinate[1] - coordinate[1]) < JUDGE_DISTANCE) {
			return node;
		}
	}
	cerr << "err:coordinateToNode" << endl;
	return nullptr;
}

Node* Graph::indexToNode(array<int, 2> index) {
	return index_to_node[index];
}