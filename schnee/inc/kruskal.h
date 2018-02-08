#ifndef KRUSKAL_H
#define KRUSKAL_H

#include <vector>
#include <algorithm>

class Kruskal
{
public:
    Kruskal();

    void addEdge(int src, int dst, float weight);
    void MST();

    /**
     * @brief The KEdge class : Edge for the Kruskal MST algorithm
     */
    class KEdge
    {
    public:
        int src;
        int dst;
        float weight;
    };
    /**
     * @brief The KGraph class : Graph for the Kruskal MST algorithm
     */
    class KGraph
    {
    public:
        int V;  // number of vertices
        int E;  // number of edges
        std::vector<KEdge> edges;
    };
    /**
     * @brief The KSubset class : Subset for the Kruskal MST algorithm
     */
    class KSubset
    {
    public:
        int parent;
        int rank;
    };

private:
    KGraph graph;

    int find(KSubset subsets[], int i);
    void set_union(KSubset subsets[], int x, int y);
};

int max(int a, int b);
bool cmp_edges(Kruskal::KEdge a, Kruskal::KEdge b);

#endif // KRUSKAL_H
