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

private:
    
    std::set<std::shared_ptr<Vertex<T>>> get_neighbours(std::shared_ptr<Vertex<T>> v) const;

public:    
    int get_vertices_count() const { return vertices.size(); }
    int get_edges_count() const { return edges.size(); }
    void print_graph() const;
    void a_star(const std::shared_ptr<Vertex<T>>& start, const std::shared_ptr<Vertex<T>>& end);
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

template <typename T>
void Graph<T>::a_star(const std::shared_ptr<Vertex<T>>& start, const std::shared_ptr<Vertex<T>>& end) {
    std::cout << "astar" << std::endl;

    std::shared_ptr<Vertex<T>> c = end;
    Vertex<T>* vy = c.get();
    //std::cout << vy->get_data() << ", ";
          
    //Set all vertices distances = infinity
    double inf = std::numeric_limits<double>::infinity();
    
    //map representing previous vertex in the path
    std::map<std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>> prev;

    //map representing distances from source vertex
    std::map<std::shared_ptr<Vertex<T>>, double> dist;

    //set do przechowywania posortowanych wierzchołków
    typedef std::pair<std::shared_ptr<Vertex<T>>, double> pair_sharedptr_double;
    std::multiset<pair_sharedptr_double, myComp> Q;

    //helper set to store visited vertcies
    std::map<std::shared_ptr<Vertex<T>>, bool> visited;
    
    dist[start] = 0;

    for (auto& vertex_iterator : vertices) {
        if (vertex_iterator != start) {
            dist[vertex_iterator] = inf;
            //dist[vertex_iterator] = rand() % 10 + 1;
            prev[vertex_iterator] = nullptr;
            visited[vertex_iterator] = false;
        }
        
        pair_sharedptr_double p(vertex_iterator, dist[vertex_iterator]);
        Q.insert(p);
     }
    
    //wypisanie kolejki
    //std::set<pair_sharedptr_double, myComp> temp = Q;
    for (auto& el : Q) {
        Vertex<T>* v = el.first.get();
        //std::cout << v->get_data() << ": "  << el.second << std::endl;
    }

    while (!Q.empty()) {
        auto it = Q.begin(); // iterator na 1 element
        std::pair<std::shared_ptr<Vertex<T>>, double> top = *it;

        //Vertex<T>* u = u.get();
        //std::cout << "Current:" <<  u->get_data()<< std::endl;

        std::shared_ptr<Vertex<T>> u = top.first;

        visited[u] = true;

        if (u == end) {
            std::cout << "END" << std::endl;
            break;
        }

        std::set<std::shared_ptr<Vertex<T>>> neighbours = get_neighbours(u);

        pair_sharedptr_double value(u, dist[u]);
        auto itr = Q.find(value);
        if (itr != Q.end()) {
            Q.erase(itr);
            Vertex<T>* vertex = u.get();
            //std::cout << "usunienie z kolejki vertexa: " << vertex->get_data() << std::endl;
        }

        for (auto& v : neighbours) {
            if (visited[v] == true) {
                Vertex<T>* vd = v.get();
                //std::cout << "Vertex: " << vd->get_data() << "odwiedzony" << std::endl;;
                continue;
            }
            
            if (dist[v] > dist[u] + get_edge_weight(u, v)) {
                //std::cout << std::endl<< "mniejsza" << " ";
                prev[v] = u;

                //Q.decrease_priority(v, alt)
                pair_sharedptr_double value(v, dist[v]);
                auto itr = Q.find(value);
                if (itr != Q.end()) {
                    Q.erase(itr);
                    dist[v] = dist[u] + get_edge_weight(u, v);
                    pair_sharedptr_double new_set_element(v, dist[v]);
                    Q.insert(new_set_element);

                }
                ////std::cout << "Wypisanie kolejki" << std::endl;
                //for (auto& el : Q) {
                //    Vertex<T>* v = el.first.get();
                //    //std::cout << v->get_data() << ": " << el.second << std::endl;
                //}
            }
            //std::cout << dist[v] << std::endl;
        }       
    }

    for (auto& el : dist) {

        //std::map<std::shared_ptr<Vertex<T>>, double> el;
        //std::cout << "dist" << std::endl;
        Vertex<T>* v1 = el.first.get();
        //std::cout << v1->get_data() << ": " << el.second << std::endl;
    }
        

    //std::cout << "prev: " << std::endl;
    //for (auto& el : prev) {

    //    //std::map<std::shared_ptr<Vertex<T>>, double> el;
    //    //std::cout << "dist" << std::endl;
    //    Vertex<T>* v1 = el.first.get(); 
    //    Vertex<T>* v2 = el.second.get();
    //    //std::cout << v1->get_data() << ": " << v2->get_data() << std::endl;
    //}

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
    std::cout << std::endl << "Cost:" << dist[end];

}