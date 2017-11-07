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
    unordered_map< int_pair, LineSegment, int_pair_hash> edges;

public:
    int size = 2;

    void set_start(double x, double y) {
        start = Pose(x, y, 0);
        vertices[size-2] = start;

        double min_dist = std::numeric_limits<double>::max();
        int min_e1, min_e2;
        for (auto l : edges) {
            if (l.second.dist_to_point_sq(start) <= min_dist) {
                min_dist = l.second.dist_to_point_sq(start);
                min_e1 = l.first.first;
                min_e2 = l.first.second;
            }
        }
        for (int i = 0; i < size; i++) {
            if (i == min_e1 || i == min_e2) {
                adjacency_mat[size-2][i] = 1;
                adjacency_mat[i][size-2] = 1;
            }
            else {
                adjacency_mat[i][size-2] = 0;
                adjacency_mat[size-2][i] = 0;
            }
        }
        set_edges();
    }

    void set_goal(double x, double y) {
        goal = Pose(x, y, 0);
        vertices[size-1] = goal;

        double min_dist = std::numeric_limits<double>::max();
        int min_e1, min_e2;
        for (auto l : edges) {
            if (l.second.dist_to_point_sq(goal) <= min_dist) {
                min_dist = l.second.dist_to_point_sq(goal);
                min_e1 = l.first.first;
                min_e2 = l.first.second;
            }
        }
        for (int i = 0; i < size; i++) {
            if (i == min_e1 || i == min_e2) {
                adjacency_mat[size-1][i] = 1;
                adjacency_mat[i][size-1] = 1;
            }
            else {
                adjacency_mat[i][size-1] = 0;
                adjacency_mat[size-1][i] = 0;
            }
        }
        set_edges();
    }

    void add_vertex(double x, double y) {
        Pose new_vertex(x, y, 0);
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

        vertices.push_back(start);
        vertices.push_back(goal);

        set_edges();

    }

    void set_edges() {
        edges.clear();
        for (int i = 0; i < size; i++) {
            for (int j = i; j < size; j++) {
                if (adjacency_mat[i][j] != 0) {
                    edges[make_pair(i, j)] = LineSegment(vertices[i], vertices[j]);
                }
            }
        }
    }

    int min_dist_index(vector<double> dist, vector<bool> sptSet) {
        int min_index;
        double min_dist = std::numeric_limits<double>::max();

        for (int i = 0; i < size; i++) {
            if (sptSet[i] == false && dist[i] <= min_dist) {
                min_index = i;
                min_dist = dist[i];
            }
        }
        return min_index;
    }

    vector< Pose > shortest_path() {
        vector<double> dist(size, std::numeric_limits<double>::max());
        vector<int> pred(size, -1);

        vector<bool> shortest_path_tree_set(size, false); // true if node i is included in shortest path tree
                                                          // or shortest distance from start to i is finalized

        dist[size-2] = 0; // the start is zero away from itself :p

        for (int count = 0; count < size - 1; count++) {
            int u = min_dist_index(dist, shortest_path_tree_set);

            shortest_path_tree_set[u] = true;

            for (int v = 0; v < size; v++) {
                if (!shortest_path_tree_set[v] && adjacency_mat[u][v] &&
                    (dist[u] + dist_i_j(u, v)) < dist[v])
                {
                    pred[v] = u;
                    dist[v] = dist[u] + dist_i_j(u, v);
                }
            }
            if (u == size-1) break;

        }
        vector< Pose > path;
        vector<int> index_path;
        index_path.push_back(size-1);
        int parent = pred[size-1];

        while (parent != (size-2) || parent == -1) {
            index_path.push_back(parent);
            parent = pred[parent];
        }
        for (auto rit = index_path.rbegin(); rit != index_path.rend(); rit++) {
            path.push_back(vertices[*rit]);
        }

        return path;
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

    void print_edges() {
        for (auto& l : edges) {
            cout << "(" << l.first.first << ", " << l.first.second << "): " << l.second << endl;
        }
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

    g.print_edges();
    g.set_start(0, 0);
    cout << endl;
    g.print_edges();
    g.set_goal(31, -7);
    cout << endl;
    g.print_edges(); cout << endl;
    g.print_adj_mat(); cout << endl;

    print_vertices(g.shortest_path());

    return  0;
}

