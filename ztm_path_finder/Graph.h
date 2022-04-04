#pragma once
#include <stdio.h>
#include <set>
#include <vector>
#include <unordered_map>
#include <map>
#include <queue>


#include <cstdlib>
#include <time.h>
#include <stdio.h>

#include <limits>

#include "Vertex.h"

struct pair_hash
{
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2>& pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

// Structure of the operator
// overloading for comparison
struct myComp {
    template <typename T>
    constexpr bool operator()(
        std::pair<std::shared_ptr<Vertex<T>>, double> const& a,
        std::pair<std::shared_ptr<Vertex<T>>, double> const& b)
        const noexcept
    {
        return a.second < b.second;
    }
};

template <typename T>
using EdgeKey = std::pair<std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>>;

template <typename T>
using Edges = std::unordered_map<EdgeKey<T>, double, pair_hash>;

template <typename T>
using Vertices = std::vector <std::shared_ptr<Vertex<T>>>;

template <typename T>
class Graph
{
private:
    std::vector <std::shared_ptr<Vertex<T>>> vertices;
    Edges<T> edges;

public:
    Graph(const Vertices<T> & vertices_, const Edges<T> & edges_)
        : vertices(vertices_), edges(edges_) {}

public:
    std::set<std::shared_ptr<Vertex<T>>> get_neighbours(std::shared_ptr<Vertex<T>> v) const;
    std::vector <std::shared_ptr<Vertex<T>>> get_vertices(){ return vertices; }
    Edges<T> get_edges() { return edges; }

public:    
    int get_vertices_count() const { return vertices.size(); }
    int get_edges_count() const { return edges.size(); }
    void print_graph() const;
    double get_edge_weight(std::shared_ptr<Vertex<T>> start,std::shared_ptr<Vertex<T>> end) const;

};

template <typename T>
void Graph<T>::print_graph() const {
    std::cout << "Graph: Print graph: " << std::endl;
    for (auto& vertex_iterator : vertices) {
        Vertex<T> *v = vertex_iterator.get();
        std::cout << "V " << v->get_data() << ": ";
        get_neighbours(vertex_iterator);
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

template <typename T>
double Graph<T>::get_edge_weight(std::shared_ptr<Vertex<T>> start,std::shared_ptr<Vertex<T>> end) const {
    double weight = -1;
    for (auto& edge : edges) {

        std::shared_ptr<Vertex<T>> s = edge.first.first;
        std::shared_ptr<Vertex<T>> e = edge.first.second;

        if (s.get() == start.get() && e.get() == end.get()) {
            //std::cout << "Takie same" << edge.second << std::endl;
            weight = edge.second;
        }
    }
    return weight;
}


template <typename T>
std::set<std::shared_ptr<Vertex<T>>> Graph<T>::get_neighbours(std::shared_ptr<Vertex<T>> v) const {
       
    std::set<std::shared_ptr<Vertex<T>>> neighbours;

    for (auto& edge : edges) {
        if (edge.first.first == v) {
            neighbours.insert(edge.first.second);
        }
    }
    return neighbours;
}

