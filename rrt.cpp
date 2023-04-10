#include "rrt.h"

Rrt::Rrt(Node* start_, Node* goal_, Graph* graph_) :start(start_), goal(goal_), graph(graph_) {
    node_list.insert(start);
    threshold = 5;
    iter_max = 1000;
    parent[start] = start;
    step_len = 0.3;
    range = 8;
    model = 0;
    obstacle_check_step = 0.5;
}

Rrt::Rrt(Node* start_, Node* goal_, Graph* graph_, unordered_set<Node* > robot_nodes_) :start(start_), goal(goal_), graph(graph_), obstacle_robot_nodes(robot_nodes_) {
    node_list.insert(start);
    threshold = 5;
    iter_max = 1000;
    parent[start] = start;
    step_len = 0.3;
    range = 8;
    model = 0;
    obstacle_check_step = 0.5;
}

// 进行路径规划
vector<Node*> Rrt::searching() {
    // 迭代iter_max次
    for (int i = 0; i < iter_max; i++) {
        // 随机采样节点
        Node* node_rand = generateRandomNode();
        // 找到最近的节点
        Node* node_near = findNearestNode(node_list, node_rand);
        // 生成节点
        Node* node_new = generateNewNode(node_near, node_rand);
        // 节点已经在搜索树中
        if (node_list.find(node_new) != node_list.end()) 
            continue;
        // 添加新节点到搜索树中
        if (!isPassObstacle(node_near, node_new)) {
            parent[node_new] = node_near;
            node_list.insert(node_new);
            double dist = distance(node_new->coordinate, goal->coordinate);
            // 新节点与终点距离大于阈值
            if(model == 0) {
                if (dist > threshold) {
                    return extractPath(node_new);
                }
            }
            
            // 新节点与给定终点距离小于步长
            if (model == 1) {
                if (dist <= step_len && !isPassObstacle(node_new, goal)) {
                    parent[goal] = node_new;
                    node_list.insert(goal);
                    return extractPath(goal);
                }
            }
            
        }
    }
    cerr << "err:Rrt::searching" << endl;
    return {};
}

Node* Rrt::generateRandomNode() {
    double max_x = start->coordinate[0] + range < 49 ? start->coordinate[0] + range : 49;
    double min_x = start->coordinate[0] - range > 1 ? start->coordinate[0] - range : 1;
    double max_y = start->coordinate[1] + range < 49 ? start->coordinate[1] + range : 49;
    double min_y = start->coordinate[1] - range > 1 ? start->coordinate[1] - range : 1;
    /*double max_x = 50, max_y = 50;
    double min_x = 0, min_y = 0;*/
    
    double x = (double)rand() / RAND_MAX * (max_x - min_x) + min_x;
    double y = (double)rand() / RAND_MAX * (max_y - min_y) + min_y;
    Node* random_node = graph->coordinateToNode({ x,y });
    assert(random_node != nullptr);
    return random_node;
}

Node* Rrt::findNearestNode(unordered_set<Node*> node_list, Node* n) {
    double min_distance = INT_MAX;
    Node* near_node = nullptr;
    for (auto node : node_list) {
        double dist = distance(node->coordinate, n->coordinate);
        if (dist < min_distance) {
            near_node = node;
            min_distance = dist;
        }
    }
    return near_node;
}

Node* Rrt::generateNewNode(Node* start, Node* end) {
    double dx = end->coordinate[0] - start->coordinate[0];
    double dy = end->coordinate[1] - start->coordinate[1];
    double dist = distance(start->coordinate, end->coordinate);
    double theta = atan2(dy, dx);
    dist = min(step_len, dist);
    //dist = step_len;
    Node* node_new = graph->coordinateToNode({ start->coordinate[0] + dist * cos(theta), start->coordinate[1] + dist * sin(theta) });
    //parent[node_new] = start;
    assert(node_new != nullptr);
    return node_new;
}

bool Rrt::isPassObstacle(Node* start, Node* end) {
    assert(!start->is_obstacle);
    if (end->is_obstacle || obstacle_robot_nodes.find(end) != obstacle_robot_nodes.end()) {
        return true;
    }
    Vec2 start_coor = start->coordinate;
    Vec2 end_coor = end->coordinate;
    if (end_coor[0] - start_coor[0] != 0) {
        double k = (end_coor[1] - start_coor[1]) / (end_coor[0] - start_coor[0]);
        for (double x = start_coor[0]; x <= end_coor[0]; x += obstacle_check_step) {
            double y = k * (x - start_coor[0]) + start_coor[1];
            Node* node = graph->coordinateToNode({ x,y });
            assert(node != nullptr);
            if (node->is_obstacle || obstacle_robot_nodes.find(node) != obstacle_robot_nodes.end()) {
                return true;
            }
            // 节点上下左右有障碍
            for (int i = 0; i < 4; ++i) {
                if (node->neighbors[i]->is_obstacle)
                    return true;
            }
            /*for (auto neigh : node->neighbors) {
                if (neigh->is_obstacle || obstacle_robot_nodes.find(node) != obstacle_robot_nodes.end()) {
                    return true;
                }
            }*/
            
        }
    }
    else {
        for (double y = start_coor[1]; y <= end_coor[1]; y += obstacle_check_step) {
            Node* node = graph->coordinateToNode({ start_coor[0], y });
            assert(node != nullptr);
            if (node->is_obstacle || obstacle_robot_nodes.find(node) != obstacle_robot_nodes.end()) {
                return true;
            }
            // 节点上下左右有障碍
            for (int i = 0; i < 4; ++i) {
                if (node->neighbors[i]->is_obstacle)
                    return true;
            }
            /*for (auto neigh : node->neighbors) {
                if (neigh->is_obstacle || obstacle_robot_nodes.find(node) != obstacle_robot_nodes.end()) {
                    return true;
                }
            }*/
            
        }
    }
    
    return false;
}

vector<Node*> Rrt::extractPath(Node* goal) {
    Node* cur_node = goal;
    vector<Node*> path;
    while (cur_node != start) {
        path.emplace_back(cur_node);
        cur_node = parent[cur_node];
    }
    path.emplace_back(start);
    reverse(path.begin(), path.end());
    return path;
}