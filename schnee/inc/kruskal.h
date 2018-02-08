#ifndef KRUSKAL_H
#define KRUSKAL_H

#include <vector>

int max(int a, int b);

class Kruskal
{
public:
    Kruskal();

    void addEdge(int src, int dst, float weight);

private:
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

    KGraph graph;
};

#endif // KRUSKAL_H
