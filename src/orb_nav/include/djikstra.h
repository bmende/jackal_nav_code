#ifndef DJIKSTRA_H
#define DJIKSTRA_H

#include <unordered_map>
#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>

#include "helpers.h"

using namespace std;

class GraphMap {
    vector< Pose > vertices, plan;
    Pose start, goal;
    vector< vector<int> > adjacency_mat;
    unordered_map< int_pair, LineSegment, int_pair_hash> edges;
    int num_vertices = 2; // start and goal are not vertices in the map.

public:

    GraphMap() {};
    GraphMap(string file_name);

    void set_start(double x, double y);
    void set_goal(double x, double y);
    void add_vertex(double x, double y);
    void initialize_adj_mat(vector< vector<int> > adj_mat);
    void set_edges();

    double dist_i_j(int i, int j);

    int min_dist_index(vector<double> dist, vector<bool> sptSet);
    vector<Pose> shortest_path();
    inline vector<Pose> getPlan() { return plan; }

    void print_adj_mat();
    void print_graph();
    void print_edges();
    void print_plan();
};

#endif // DJIKSTRA_H
