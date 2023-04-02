#include "graph.h"

void Graph::init(char map[MAP_SIZE][MAP_SIZE]) {
	int workbench_id = 0;
	// 从左上角开始
	for (int i = 0; i < MAP_SIZE; i++) {
		for (int j = 0; j < MAP_SIZE; j++) {
			Node node;
			node.coordinate[0] = 0.25 + j * 0.5; // x坐标
			node.coordinate[1] = 0.25 + (MAP_SIZE - 1 - i) * 0.5; // y坐标
			// 障碍物
			if (map[i][j] == '#') {
				node.is_obstacle = true;
				node.is_workbench = false;
				node.workbench_id = -1;
			}
			// 工作台
			else if ('1' <= map[i][j] && map[i][j] <= '9') {
				node.is_obstacle = false;
				node.is_workbench = true;
				node.workbench_id = workbench_id;
				workbench_id++;
			}
			else {
				node.is_obstacle = false;
				node.is_workbench = false;
				node.workbench_id = -1;
			}
			nodes.emplace_back(node);
			index_to_node[{i, j}] = &node;
		}
	}
}

void Graph::initNeighbors() {
	size_t size = nodes.size();
	for (int i = 0; i < size; i++) {
		Node* cur_node = &nodes[i];
		array<int, 2> cur_index = node_to_index[cur_node];

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

