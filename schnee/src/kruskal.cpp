#include "kruskal.h"

int max(int a, int b)
{
    return (a>b?a:b);
}

Kruskal::Kruskal()
{
    this->graph.E = 0;
    this->graph.V = 0;
}

void Kruskal::addEdge(int src, int dst, float weight)
{
    KEdge edge;
    edge.src = src;
    edge.dst = dst;
    edge.weight = weight;
    this->graph.edges.push_back(edge);
    this->graph.E = max(this->graph.E,max(src,dst));
    this->graph.V++;
}
