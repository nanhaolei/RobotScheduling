#ifndef RRT_H_
#define RRT_H_

#include "geometry.h"
#include "graph.h"
#include "robot.h"
#include <vector>
#include <random>
#include <limits>
#include <climits>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
using namespace std;
using namespace geometry;

class Rrt
{
public:
	 Rrt(Node* start_, Node* goal_, Graph* graph_);
     Rrt(Node* start_, Node* goal_, Graph* graph_, unordered_set<Node*> obstacle_robot_nodes);
	 vector<Node*> searching();
     Node* generateRandomNode();
     Node* findNearestNode(unordered_set<Node*> vertex, Node* n);
     Node* generateNewNode(Node* start, Node* end);
     bool isPassObstacle(Node* start, Node* end);
     vector<Node*> extractPath(Node* goal);
     void setModel(int model) { this->model = model; }
private:
	Node* start;
	Node* goal;
    Graph* graph;
	int iter_max;
    unordered_set<Node*> node_list;
    unordered_map<Node*, Node*> parent;
    double threshold;
    double step_len;
    double range;
    unordered_set<Node*> obstacle_robot_nodes;
    int model;
    double obstacle_check_step;
};


#endif