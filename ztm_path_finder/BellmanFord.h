#pragma once
#include "Algorithm.h"

template <typename T>
class BellmanFord :
    public Algorithm
{
public:
    virtual void find_path(Graph<T> G, std::shared_ptr<Vertex<T>> start, std::shared_ptr<Vertex<T>> end);
};

template <typename T>
void BellmanFord<T>::find_path(Graph<T> G, std::shared_ptr<Vertex<T>> start, std::shared_ptr<Vertex<T>> end) {
    std::cout << "BellmanFord" << std::endl;

    std::vector <std::shared_ptr<Vertex<T>>> vertices = G.get_vertices();
    Edges<T> edges = G.get_edges();

    //Set all vertices distances = infinity
    double inf = std::numeric_limits<double>::infinity();

    //map representing previous vertex in the path
    std::map<std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>> prev;

    //map representing distances from source vertex
    std::map<std::shared_ptr<Vertex<T>>, double> dist;

    //set do przechowywania posortowanych wierzcho³ków
    typedef std::pair<std::shared_ptr<Vertex<T>>, double> pair_sharedptr_double;
    std::multiset<pair_sharedptr_double, myComp> Q;

    //helper set to store visited vertcies
    std::map<std::shared_ptr<Vertex<T>>, bool> visited;

    dist[start] = 0;

    for (auto& vertex_iterator : vertices) {
        if (vertex_iterator != start) {
            dist[vertex_iterator] = inf;
            prev[vertex_iterator] = nullptr;
            visited[vertex_iterator] = false;
        }

        pair_sharedptr_double p(vertex_iterator, dist[vertex_iterator]);
        Q.insert(p);
    }

    int vertices_count = G.get_vertices_count();

    for (int i = 0; i < vertices_count - 1; i++) {
        for (auto& edge : G.get_edges()) {
            std::shared_ptr<Vertex<T>> v = edge.first.first;
            std::shared_ptr<Vertex<T>> u = edge.first.second;
            double w = edge.second;

            if ((dist[u] + w) < dist[v]) {
                dist[v] = dist[u] + w;
                prev[v] = u;
            }

        }  
    }

    //finding a path to end vertex
    std::vector<std::shared_ptr<Vertex<T>>> path;
    std::shared_ptr<Vertex<T>> current = end;

    do {
        Vertex<T>* v = current.get();
        path.push_back(current);
        current = prev[current];
    } while (current != start);
    path.push_back(current);


    //print path
    for (auto& v : path) {
        Vertex<T>* el = v.get();
        std::cout << el->get_data() << "->";
    }

    //print cost
    std::cout << std::endl << "Cost:" << dist[end] << std::endl;
}