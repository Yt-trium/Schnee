#include "orientationfixer.h"

/* For each point in the cloud
 * Find the neighbor (oi and oj are close)
 * Create the graph with weight = 1 - ni * nj
 * Creating MST
 * Fix planes orientation
 */

void orientationFixer(PlaneCloud &plc, const plane_cloud_index &index, const int &k)
{
    Kruskal kruskal;
    std::vector<Kruskal::KEdge> r;

    kruskal.setVertices(plc.planes.size());
    kruskal.setEdges(plc.planes.size()*k);

    // Nbhd variables
    const size_t        num_results = k + 1;
    std::vector<size_t> ret_index(num_results);
    std::vector<float>  out_squared_dist(num_results);
    float               kd_query[3];
    size_t              nbhd_count;

    float w;

    for(int i = 0; i < plc.planes.size(); ++i)
    {
        kd_query[0] = plc.planes.at(i)->center->x;
        kd_query[1] = plc.planes.at(i)->center->y;
        kd_query[2] = plc.planes.at(i)->center->z;

        nbhd_count = index.knnSearch(&kd_query[0], num_results, &ret_index[0], &out_squared_dist[0]);

        assert(nbhd_count == num_results);
        assert(ret_index[0] == i);

        sVector3 n1 = plc.planes.at(i)->normal;

        for(int j = 1; j <= k; ++j)
        {
            sVector3 n2 = plc.planes.at(ret_index[j])->normal;

            w = 1 - Vector3::dot((*n1),(*n2));

            kruskal.addEdge(i,ret_index[j],w);
        }
    }
    /*
    kruskal.addEdge(0,1,10);
    kruskal.addEdge(0,2,6);
    kruskal.addEdge(0,3,5);
    kruskal.addEdge(1,3,15);
    kruskal.addEdge(2,3,4);

    r = kruskal.MST();
    */
}
