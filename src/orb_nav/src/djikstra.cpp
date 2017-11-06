#include <unordered_map>
#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>

#include "helpers.h"

using namespace std;

void print_pair(pair<double, double> p) {
    cout << "(" << p.first << ", " << p.second << ")";
}

void print_vertices(vector< Pose > v) {
    for (auto& n : v) {
        cout << n << ", ";
    }
    cout << endl;
}

class Graph
{
    vector< Pose > vertices, plan;
    Pose start, goal;
    vector< vector<double> > adjacency_mat;

public:
    int size = 2;

    Graph() {
        set_start(0, 0);
        set_goal(0, 0);
    }

    void set_start(double x, double y) {
        start.x = x; start.y = y;
    }

    void set_goal(double x, double y) {
        goal.x = x; goal.y = y;
    }

    void add_vertex(double x, double y) {
        Pose new_vertex;
        new_vertex.x = x; new_vertex.y = y;
        vertices.push_back(new_vertex);
        size++;
    }

    double dist_i_j(int i, int j) {
        if (adjacency_mat[i][j] > 0) {
            return vertices[i].dist(vertices[j]);
        } else {
            return 0.0;
        }
    }

    void initialize_adj_mat(vector< vector<double> > a_m) {
        for (int i = 0; i < size; i++) {
            adjacency_mat.push_back(vector<double>(size));
            if (i >= size - 2) continue;
            for (int j = 0; j < size - 2; j++) {
                adjacency_mat[i][j] = a_m[i][j];
            }
        }
    }

    void print_adj_mat() {
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                cout << adjacency_mat[i][j] << " ";
            }
            cout << endl;
        }
    }

    void print_graph() {
        cout << "size: " << size << endl;
        cout << "start: " << start << endl;
        cout << "goal: "  << goal  << endl;
        print_vertices(vertices); cout << endl;
        print_adj_mat(); cout << endl;
    }
};

int main()
{
    Graph g;
    g.add_vertex(31.5, 0);
    g.add_vertex(31.0, -10);
    g.add_vertex(-27, -8.36);
    g.add_vertex(-26, 0.7);

    g.initialize_adj_mat({
            {0, 1, 0, 1},
            {1, 0, 1, 0},
            {0, 1, 0, 1},
            {1, 0, 1, 0} });

    g.print_graph();

    for (int i = 0; i < g.size; i++) {
        for (int j = 0; j < g.size; j++) {
            cout << g.dist_i_j(i, j) << " ";
        }
        cout << endl;
    }

    return  0;
}

