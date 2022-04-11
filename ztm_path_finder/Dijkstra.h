#pragma once
#include <memory>

#include "Algorithm.h"
#include "Graph.h"

template <typename T>
class Dijkstra :
    public Algorithm
{
public:
    std::string name;
    virtual void find_path(Graph<T> G, std::shared_ptr<Vertex<T>> start, std::shared_ptr<Vertex<T>> end);
};

template <typename T>
void Dijkstra<T>::find_path(Graph<T> G, std::shared_ptr<Vertex<T>> start, std::shared_ptr<Vertex<T>> end) {
    std::cout << "Dijkstra" << std::endl;

    std::vector <std::shared_ptr<Vertex<T>>> vertices = G.get_vertices();
    Edges<T> edges = G.get_edges();

    std::shared_ptr<Vertex<T>> c = end;
    Vertex<T>* vy = c.get();
    //std::cout << vy->get_data() << ", ";

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

        std::set<std::shared_ptr<Vertex<T>>> neighbours = G.get_neighbours(u);

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

            if (dist[v] > dist[u] + G.get_edge_weight(u, v)) {
                //std::cout << std::endl<< "mniejsza" << " ";
                prev[v] = u;

                //Q.decrease_priority(v, alt)
                pair_sharedptr_double value(v, dist[v]);
                auto itr = Q.find(value);
                if (itr != Q.end()) {
                    Q.erase(itr);
                    dist[v] = dist[u] + G.get_edge_weight(u, v);
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
    std::cout << std::endl << "Cost:" << dist[end] <<std::endl;
}