#include <unordered_map>
#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>

#include "helpers.h"
#include "djikstra.h"

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

GraphMap::GraphMap(string file_name) {

    cout << "Reading from: " << file_name << endl;
    std::ifstream map_file(file_name);
    int count = 0, size = 0;

    vector< vector<int> > adj_mat;

    for (string line; getline(map_file, line); count++) {
        if (count == 0) {
            if (line.substr(0, 4).compare("size") != 0) {
                cout << "Need to specify number of vertices of graph (size)" << endl;
                exit(2);
            }
            stringstream(line.substr(5)) >> size;
        }
        else if (count <= size) {
            double x, y;
            string temp;
            stringstream l(line);
            getline(l, temp, ','); x = stod(temp);
            getline(l, temp); y = stod(temp);
            add_vertex(x, y);
        }
        else { // now we read the adjacency list.
            int cur_index = count - size - 1;
            vector<int> adj_vector(size);
            stringstream l(line);
            for (string temp; getline(l, temp, ',');) {
                int index = stoi(temp);
                adj_vector[index] = 1;
            }
            adj_mat.push_back(adj_vector);
        }
    }
    initialize_adj_mat(adj_mat);

}

void GraphMap::set_start(double x, double y) {
    start = Pose(x, y, 0);
    int s_ind = num_vertices - 2;
    vertices[s_ind] = start;

    double min_dist = std::numeric_limits<double>::max();
    int min_e1, min_e2;
    for (auto l : edges) {
        if (l.second.dist_to_point_sq(start) <= min_dist &&
            l.first.first != s_ind && l.first.second != s_ind) {
            min_dist = l.second.dist_to_point_sq(start);
            min_e1 = l.first.first;
            min_e2 = l.first.second;
        }
    }
    for (int i = 0; i < num_vertices; i++) {
        if (i == min_e1 || i == min_e2) {
            adjacency_mat[num_vertices-2][i] = 1;
            adjacency_mat[i][num_vertices-2] = 1;
        }
        else {
            adjacency_mat[i][num_vertices-2] = 0;
            adjacency_mat[num_vertices-2][i] = 0;
        }
    }
    set_edges();
}

void GraphMap::set_goal(double x, double y) {
    goal = Pose(x, y, 0);
    vertices[num_vertices-1] = goal;

    double min_dist = std::numeric_limits<double>::max();
    int min_e1, min_e2;
    for (auto l : edges) {
        if (l.second.dist_to_point_sq(goal) <= min_dist) {
            min_dist = l.second.dist_to_point_sq(goal);
            min_e1 = l.first.first;
            min_e2 = l.first.second;
        }
    }
    for (int i = 0; i < num_vertices; i++) {
        if (i == min_e1 || i == min_e2) {
            adjacency_mat[num_vertices-1][i] = 1;
            adjacency_mat[i][num_vertices-1] = 1;
        }
        else {
            adjacency_mat[i][num_vertices-1] = 0;
            adjacency_mat[num_vertices-1][i] = 0;
        }
    }
    set_edges();
}

void GraphMap::add_vertex(double x, double y) {
    Pose new_vertex(x, y, 0);
    vertices.push_back(new_vertex);
    num_vertices++;
}

double GraphMap::dist_i_j(int i, int j) {
    if (adjacency_mat[i][j] > 0) {
        return vertices[i].dist(vertices[j]);
    } else {
        return 0.0;
    }
}

void GraphMap::initialize_adj_mat(vector< vector<int> > a_m) {
    for (int i = 0; i < num_vertices; i++) {
        adjacency_mat.push_back(vector<int>(num_vertices));
        if (i >= num_vertices - 2) continue;
        for (int j = 0; j < num_vertices - 2; j++) {
            adjacency_mat[i][j] = a_m[i][j];
        }
    }

    vertices.push_back(start);
    vertices.push_back(goal);

    set_edges();

}

void GraphMap::set_edges() {
    edges.clear();
    for (int i = 0; i < num_vertices; i++) {
        for (int j = i; j < num_vertices; j++) {
            if (adjacency_mat[i][j] != 0) {
                edges[make_pair(i, j)] = LineSegment(vertices[i], vertices[j]);
            }
        }
    }
}

int GraphMap::min_dist_index(vector<double> dist, vector<bool> sptSet) {
    int min_index;
    double min_dist = std::numeric_limits<double>::max();

    for (int i = 0; i < num_vertices; i++) {
        if (sptSet[i] == false && dist[i] <= min_dist) {
            min_index = i;
            min_dist = dist[i];
        }
    }
    return min_index;
}

vector< Pose > GraphMap::shortest_path() {
    plan.clear();
    vector<double> dist(num_vertices, std::numeric_limits<double>::max());
    vector<int> pred(num_vertices, -1);

    vector<bool> shortest_path_tree_set(num_vertices, false); // true if node i is included in shortest path tree
    // or shortest distance from start to i is finalized

    dist[num_vertices-2] = 0; // the start is zero away from itself :p

    for (int count = 0; count < num_vertices - 1; count++) {
        int u = min_dist_index(dist, shortest_path_tree_set);

        shortest_path_tree_set[u] = true;

        for (int v = 0; v < num_vertices; v++) {
            if (!shortest_path_tree_set[v] && adjacency_mat[u][v] &&
                (dist[u] + dist_i_j(u, v)) < dist[v])
            {
                pred[v] = u;
                dist[v] = dist[u] + dist_i_j(u, v);
            }
        }
        if (u == num_vertices-1) break;

    }

    vector<int> index_path;
    index_path.push_back(num_vertices-1);
    int parent = pred[num_vertices-1];

    while (parent != (num_vertices-2) || parent == -1) {
        index_path.push_back(parent);
        parent = pred[parent];
    }
    for (auto rit = index_path.rbegin(); rit != index_path.rend(); rit++) {
        plan.push_back(vertices[*rit]);
    }

    return plan;
}


void GraphMap::print_adj_mat() {
    for (int i = 0; i < num_vertices; i++) {
        for (int j = 0; j < num_vertices; j++) {
            cout << adjacency_mat[i][j] << " ";
        }
        cout << endl;
    }
}

void GraphMap::print_graph() {
    cout << "num_vertices: " << num_vertices << endl;
    cout << "start: " << start << endl;
    cout << "goal: "  << goal  << endl;
    cout << "vertices: "; print_vertices(vertices); cout << endl;
    cout << "adjacency matrix:\n";
    print_adj_mat(); cout << endl;
    cout << "edges:\n";
    print_edges();
}

void GraphMap::print_edges() {
    for (auto& l : edges) {
        cout << "(" << l.first.first << ", " << l.first.second << "): " << l.second << endl;
    }
}

void GraphMap::print_plan() {
    print_vertices(plan);
}


int test_main()
{
    GraphMap g("src/orb_nav/src/map_test.txt");

    g.set_start(-1, 0);
    g.set_goal(0, -10);
    g.print_graph();

    cout << "path from start to goal:\n";
    g.shortest_path();
    g.print_plan();

    return  0;
}

