#pragma once
#include <memory>
#include <cstdlib>

#include "Algorithm.h"
#include "Graph.h"

template <typename T>
class Astar :
    public Algorithm
{

private:
    double calculate_heuristics(std::shared_ptr<Vertex<T>> current, std::shared_ptr<Vertex<T>> destination);
public: 
    std::string name = "A star";
    void find_path(Graph<T> G, std::shared_ptr<Vertex<T>> start, std::shared_ptr<Vertex<T>> end);
};

template <typename T>
double Astar<T>::calculate_heuristics(std::shared_ptr<Vertex<T>> current, std::shared_ptr<Vertex<T>> destination) {
    double h = 1;

    Vertex<T>* curr_ver = current.get();
    Localization curr_loc = curr_ver->get_loc();

    Vertex<T>* end_ver = destination.get();
    Localization end_loc = end_ver->get_loc();

    //std::cout << end_loc.x << curr_loc.x << end_loc.y << curr_loc.y;
    std::cout << "hstart";
    h = abs(end_loc.x - curr_loc.x) + abs(end_loc.y - curr_loc.y);
    std::cout << "hend";

    return h;
}

template <typename T>
void Astar<T>::find_path(Graph<T> G, std::shared_ptr<Vertex<T>> start, std::shared_ptr<Vertex<T>> end) {
    std::cout << "Astar" << std::endl;

    std::vector <std::shared_ptr<Vertex<T>>> vertices = G.get_vertices();
    Edges<T> edges = G.get_edges();

    std::shared_ptr<Vertex<T>> c = end;
    Vertex<T>* vy = c.get();

    //Set all vertices distances = infinity
    double inf = std::numeric_limits<double>::infinity();

    //map representing previous vertex in the path
    std::map<std::shared_ptr<Vertex<T>>, std::shared_ptr<Vertex<T>>> prev;

    //map representing distances from source vertex (cost)
    std::map<std::shared_ptr<Vertex<T>>, double> dist;

    //map representing distance + heuristics
    std::map<std::shared_ptr<Vertex<T>>, double> priority;

    //set do przechowywania posortowanych wierzcho³ków
    typedef std::pair<std::shared_ptr<Vertex<T>>, double> pair_sharedptr_double;
    std::multiset<pair_sharedptr_double, myComp> Q;

    //helper set to store visited vertcies
    std::map<std::shared_ptr<Vertex<T>>, bool> visited;

    dist[start] = 0;
    priority[start] = 0;

    for (auto& vertex_iterator : vertices) {
        if (vertex_iterator != start) {
            dist[vertex_iterator] = inf;
            priority[vertex_iterator] = inf;
            prev[vertex_iterator] = nullptr;
            visited[vertex_iterator] = false;
        }

        pair_sharedptr_double p(vertex_iterator, dist[vertex_iterator]);
        Q.insert(p);
    }

    //wypisanie kolejki
    std::multiset<pair_sharedptr_double, myComp> temp = Q;

    std::cout << "Wypisanie kolejki: " << std::endl;
    for (auto& el : Q) {
        Vertex<T>* v = el.first.get();
        std::cout << v->get_data() << ": "  << el.second << std::endl;
    }

    while (!Q.empty()) {
        auto it = Q.begin(); // iterator na 1 element
        std::pair<std::shared_ptr<Vertex<T>>, double> top = *it;

        std::shared_ptr<Vertex<T>> u = top.first;

        visited[u] = true;

        if (u == end) {
            std::cout << "END" << std::endl;
            break;
        }

        std::set<std::shared_ptr<Vertex<T>>> neighbours = G.get_neighbours(u);

        pair_sharedptr_double value(u, dist[u]);
        auto itr = Q.find(value);
        if (itr != Q.end()) {
            Q.erase(itr);
            Vertex<T>* vertex = u.get();
        }

        for (auto& v : neighbours) {
            if (visited[v] == true) {
                Vertex<T>* vd = v.get();
                continue;
            }

            if (dist[v] > dist[u] + G.get_edge_weight(u, v)) {
                prev[v] = u;

                //Q.decrease_priority(v, alt)
                pair_sharedptr_double value(v, dist[v]);
                auto itr = Q.find(value);
                if (itr != Q.end()) {
                    Q.erase(itr);
                    dist[v] = dist[u] + G.get_edge_weight(u, v);

                    std::cout << "h: " << calculate_heuristics(v, end) << std::endl;

                    priority[v] = dist[v] + calculate_heuristics(v, end);
                    pair_sharedptr_double new_set_element(v, priority[v]);
                    Q.insert(new_set_element);
                }

                //std::cout << "Wypisanie kolejki" << std::endl;
                std::cout << std::endl << "Wypisywanie kolejki: " << std::endl;
                for (auto& el : Q) {
                    Vertex<T>* v = el.first.get();
                    std::cout << v->get_data() << ": " << el.second << std::endl;
                }
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
    std::cout << std::endl << "Cost:" << dist[end];

}
