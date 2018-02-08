#include "kruskal.h"

int max(int a, int b)
{
    return (a>b?a:b);
}

bool cmp_edges(Kruskal::KEdge a, Kruskal::KEdge b)
{
    return (a.weight<b.weight);
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

void Kruskal::MST()
{
    KGraph result;

    std::sort(this->graph.edges.begin(),this->graph.edges.end(),cmp_edges);

    std::cout << this->graph.E << std::endl;
    std::cout << this->graph.V << std::endl;

    std::vector<KSubset> subsets;
    subsets.resize(this->graph.V+1);

    for (int v = 0; v < this->graph.V+1; ++v)
    {
        subsets[v].parent = v;
        subsets[v].rank = 0;
    }
    int i = 0, e = 0;
    while (this->graph.E < this->graph.V)
        {
            KEdge next_edge = this->graph.edges[i++];

            int x = find(subsets, next_edge.src);
            int y = find(subsets, next_edge.dst);
            if (x != y)
            {
                result.edges[e++] = next_edge;
                set_union(subsets, x, y);
            }
        }
}

int Kruskal::find(std::vector<KSubset> subsets, int i)
{
    if (subsets[i].parent != i)
        subsets[i].parent = find(subsets, subsets[i].parent);

    return subsets[i].parent;
}

void Kruskal::set_union(std::vector<KSubset> subsets, int x, int y)
{
    int xroot = find(subsets, x);
    int yroot = find(subsets, y);

    if (subsets[xroot].rank < subsets[yroot].rank)
        subsets[xroot].parent = yroot;
    else if (subsets[xroot].rank > subsets[yroot].rank)
        subsets[yroot].parent = xroot;

    else
    {
        subsets[yroot].parent = xroot;
        subsets[xroot].rank++;
    }
}
