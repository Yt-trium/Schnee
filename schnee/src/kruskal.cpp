#include "kruskal.h"

bool cmp_edges(Kruskal::KEdge a, Kruskal::KEdge b)
{
    return (a.weight<b.weight);
}

Kruskal::Kruskal()
{
    this->graph.E = 0;
    this->graph.V = 0;
}

void Kruskal::setEdges(int E)
{
    this->graph.E = E;
}

void Kruskal::setVertices(int V)
{
    this->graph.V = V;
}

bool Kruskal::addEdge(int src, int dst, float weight)
{
    for(int i = 0;i < this->graph.edges.size();i++)
    {
        if(this->graph.edges[i].dst == src && this->graph.edges[i].src == dst)
            return false;
        if(this->graph.edges[i].dst == dst && this->graph.edges[i].src == src)
            return false;
    }

    KEdge edge;
    edge.src = src;
    edge.dst = dst;
    edge.weight = weight;
    this->graph.edges.push_back(edge);
    return true;
}

std::vector<Kruskal::KEdge> Kruskal::MST()
{
    std::vector<KEdge> result;
    int i = 0;

    std::sort(this->graph.edges.begin(),this->graph.edges.end(),cmp_edges);

    /*
    std::cout << this->graph.E << std::endl;
    std::cout << this->graph.V << std::endl;
    */

    std::vector<KSubset> subsets;
    subsets.resize(this->graph.V);

    for (int v = 0; v < this->graph.V; ++v)
    {
        subsets[v].parent = v;
        subsets[v].rank = 0;
    }
    while (result.size() < this->graph.V - 1)
        {
            KEdge next_edge = this->graph.edges[i++];

            int x = find(subsets, next_edge.src);
            int y = find(subsets, next_edge.dst);
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

int Kruskal::find(std::vector<KSubset> subsets, int i)
{
    if (subsets[i].parent != i)
        subsets[i].parent = find(subsets, subsets[i].parent);

    return subsets[i].parent;
}

void Kruskal::set_union(std::vector<KSubset> *subsets, int x, int y)
{
    int xroot = find(*subsets, x);
    int yroot = find(*subsets, y);

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
