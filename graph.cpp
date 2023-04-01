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
		}
	}
}

