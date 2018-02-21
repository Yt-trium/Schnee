#ifndef KRUSKAL_H
#define KRUSKAL_H

#include <iostream>
#include <vector>
#include <algorithm>

/**
 * @brief The Kruskal class, generic Kruskal's Algorithm implementation
 */
class Kruskal
{
public:
    Kruskal();

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

    /**
     * @brief setEdges number
     * @param E
     */
    void setEdges(int E);
    /**
     * @brief setVertices number
     * @param V
     */
    void setVertices(int V);
    /**
     * @brief addEdge
     * @param src
     * @param dst
     * @param weight
     */
    bool addEdge(int src, int dst, float weight);
    /**
     * @brief Compute the Minimum Spanning Tree
     */
    std::vector<KEdge> MST();

private:
    KGraph graph;

    /**
     * @brief find set of an element
     * @param subsets
     * @param i
     * @return
     */
    int find(std::vector<KSubset> subsets, int i);
    /**
     * @brief set_union union of two sets
     * @param subsets
     * @param x
     * @param y
     */
    void set_union(std::vector<KSubset> *subsets, int x, int y);
};

/**
 * @brief cmp_edges compare Kruskal::KEdge for std::sort
 * @param a
 * @param b
 * @return
 */
bool cmp_edges(Kruskal::KEdge a, Kruskal::KEdge b);

#endif // KRUSKAL_H
