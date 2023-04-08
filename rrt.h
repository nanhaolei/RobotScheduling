#ifndef RRT_H_
#define RRT_H_

#include "geometry.h"
#include "graph.h"
#include "robot.h"
#include <vector>
#include <random>
#include <limits>
#include <unordered_map>
#include <unordered_set>
using namespace std;
using namespace geometry;

class Rrt
{
public:
	 Rrt(Node* start_, Node* goal_, Graph* graph_);
     Rrt(Node* start_, Node* goal_, Graph* graph_, unordered_set<Node*> robot_nodes);
	 vector<Node*> planning();
     Node* generate_random_node();
     Node* nearest_neighbor(unordered_set<Node*> vertex, Node* n);
     Node* new_state(Node* start, Node* end);
     bool is_collision(Node* start, Node* end);
     vector<Node*> extract_path(Node* goal);
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
    unordered_set<Node*> robot_nodes;
    int model;
    double collision_step;
};


#endif