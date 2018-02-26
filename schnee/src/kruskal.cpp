#include "kruskal.h"

#include <cassert>

bool cmp_edges(Kruskal::KEdge a, Kruskal::KEdge b)
{
    return (a.weight<b.weight);
}

Kruskal::Kruskal()
{
	this->graph.E = 0;
	this->graph.V = 0;
}

void Kruskal::setEdges(size_t E)
{
    this->graph.E = E;
}

void Kruskal::setVertices(size_t V)
{
    this->graph.V = V;
}

bool Kruskal::addEdge(size_t src, size_t dst, float weight)
{
	assert(src != dst);
	// Toujours trier les index pour eviter les doublons
	if(src > dst)
		std::swap(src, dst);

	auto it = graph.edges.begin();
	auto end = graph.edges.end();
	while(it != end)
	{
		const KEdge & ke = *it;
		if(ke.src == src && ke.dst == dst)
			return false;
		if(ke.weight > weight) // We add !
			break;
		it++;
	}

    KEdge edge;
    edge.src = src;
    edge.dst = dst;
    edge.weight = weight;
	graph.edges.insert(it, edge);
    return true;
}

std::vector<Kruskal::KEdge> Kruskal::MST()
{
    std::vector<KEdge> result;
    size_t i = 0;

	// Should be already sorted
    //std::sort(this->graph.edges.begin(),this->graph.edges.end(),cmp_edges);

    /*
    std::cout << this->graph.E << std::endl;
    std::cout << this->graph.V << std::endl;
    */

    std::vector<KSubset> subsets;
    subsets.resize(this->graph.V);

    for (size_t v = 0; v < this->graph.V; ++v)
    {
        subsets[v].parent = v;
        subsets[v].rank = 0;
    }
    while (result.size() < this->graph.V - 1)
    {
        assert(i < this->graph.edges.size());
        KEdge next_edge = this->graph.edges[i++];

        size_t x = find(subsets, next_edge.src);
        size_t y = find(subsets, next_edge.dst);
        if (x != y)
        {
            result.push_back(next_edge);
            set_union(&subsets, x, y);
        }
    }
    /*
    std::cout << "MST :" << std::endl;
    for (i = 0; i < result.size(); ++i)
        std::cout << result[i].src << " -- "
                  << result[i].dst << " = "
                  << result[i].weight << std::endl;
    */
    return result;
}

size_t Kruskal::find(std::vector<KSubset> subsets, size_t i)
{
	assert(i < subsets.size());
    if (subsets[i].parent != i)
        subsets[i].parent = find(subsets, subsets[i].parent);

    return subsets[i].parent;
}

void Kruskal::set_union(std::vector<KSubset> *subsets, size_t x, size_t y)
{
    size_t xroot = find(*subsets, x);
    size_t yroot = find(*subsets, y);

    if ((*subsets)[xroot].rank < (*subsets)[yroot].rank)
        (*subsets)[xroot].parent = yroot;
    else if ((*subsets)[xroot].rank > (*subsets)[yroot].rank)
        (*subsets)[yroot].parent = xroot;

    else
    {
        (*subsets)[yroot].parent = xroot;
        (*subsets)[xroot].rank++;
    }
}
