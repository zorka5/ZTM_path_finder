// ztm_path_finder.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <memory>
#include <vector>
#include <unordered_map>
#include <utility>

#include "Graph.h"
#include "Vertex.h"
#include "Dijkstra.h"
#include "Algorithm.h"

int main()
{
	
	std::shared_ptr<Vertex<int>> v1(new Vertex<int>(1));
	std::shared_ptr<Vertex<int>> v2(new Vertex<int>(2));
	std::shared_ptr<Vertex<int>> v3(new Vertex<int>(3));
	std::shared_ptr<Vertex<int>> v4(new Vertex<int>(4));
	std::shared_ptr<Vertex<int>> v5(new Vertex<int>(5));
	std::shared_ptr<Vertex<int>> v6(new Vertex<int>(6));
	std::shared_ptr<Vertex<int>> v7(new Vertex<int>(7));

	std::vector<std::shared_ptr<Vertex<int>>> vertices_;
	vertices_.push_back(v1);
	vertices_.push_back(v2);
	vertices_.push_back(v3);
	vertices_.push_back(v4);
	vertices_.push_back(v5);
	vertices_.push_back(v6);
	vertices_.push_back(v7);

	for (auto& it : vertices_) {
		std::cout << it->get_data() << " ";
	}

	Edges<int> edges_;

	EdgeKey<int> p12(vertices_[0], vertices_[1]);
	double w12 = 4;

	EdgeKey<int> p13(vertices_[0], vertices_[2]);
	double w13 = 3;

	EdgeKey<int> p21(vertices_[1], vertices_[0]);
	double w21 = 4;

	EdgeKey<int> p25(vertices_[1], vertices_[4]);
	double w25 = 12;

	EdgeKey<int> p26(vertices_[1], vertices_[5]);
	double w26 = 5;

	EdgeKey<int> p31(vertices_[2], vertices_[0]);
	double w31 = 3;

	EdgeKey<int> p34(vertices_[2], vertices_[3]);
	double w34 = 7;

	EdgeKey<int> p35(vertices_[2], vertices_[4]);
	double w35 = 10;

	EdgeKey<int> p43(vertices_[3], vertices_[2]);
	double w43 = 7;

	EdgeKey<int> p45(vertices_[3], vertices_[4]);
	double w45 = 2;

	EdgeKey<int> p52(vertices_[4], vertices_[1]);
	double w52 = 12;

	EdgeKey<int> p53(vertices_[4], vertices_[2]);
	double w53 = 10;

	EdgeKey<int> p54(vertices_[4], vertices_[3]);
	double w54 = 2;

	EdgeKey<int> p57(vertices_[4], vertices_[6]);
	double w57 = 5;

	EdgeKey<int> p62(vertices_[5], vertices_[1]);
	double w62 = 5;

	EdgeKey<int> p67(vertices_[5], vertices_[6]);
	double w67 = 16;

	EdgeKey<int> p75(vertices_[6], vertices_[4]);
	double w75 = 5;

	EdgeKey<int> p76(vertices_[6], vertices_[5]);
	double w76 = 16;


	edges_[p12] = w12;
	edges_[p13] = w13;
	edges_[p21] = w21;
	edges_[p25] = w25;
	edges_[p26] = w26;
	edges_[p31] = w31;
	edges_[p34] = w34;
	edges_[p35] = w35;
	edges_[p43] = w43;
	edges_[p45] = w45;
	edges_[p52] = w52;
	edges_[p53] = w53;
	edges_[p54] = w54;
	edges_[p57] = w57;
	edges_[p62] = w62;
	edges_[p67] = w67;
	edges_[p75] = w75;
	edges_[p76] = w76;


	Graph<int> G1(vertices_, edges_);
	//G1.print_graph();
	//std::cout << "V count:" << G1.get_vertices_count() <<std::endl;
	
	//std::cout << v1->get_data() << " " << v2->get_data() << std::endl;
	Dijkstra<int> D;
	D.find_path(G1, v1, v7);

	return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
