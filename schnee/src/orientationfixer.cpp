#include "orientationfixer.h"

/* For each point in the cloud
 * Find the neighbor (oi and oj are close)
 * Create the graph with weight = 1 - ni * nj
 * Creating MST
 * Fix planes orientation
 */

void orientationFixer(PlaneCloud &plc, const plane_cloud_index &index, const int &k)
{
    // std::cout << "orientationFixer " << plc.planes.size() << std::endl;

    Kruskal kruskal;
    std::vector<Kruskal::KEdge> r;

    kruskal.setVertices(plc.planes.size());


    // Nbhd variables
    const size_t        num_results = k + 1;
    std::vector<size_t> ret_index(num_results);
    std::vector<float>  out_squared_dist(num_results);
    float               kd_query[3];
    size_t              nbhd_count;

    int e = 0, id;
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

            w = 1 - abs(Vector3::dot((*n1),(*n2)));

            if(kruskal.addEdge(i,ret_index[j],w))
                e++;
        }
    }

    kruskal.setEdges(e);
    r = kruskal.MST();

    // search for the max z plane
    id = maxZ(plc);

    if(plc.planes.at(id)->normal->z < 0)
    {
        // *(plc.planes[id]->normal) *= -1;
        flip (plc.planes[id]->normal);
    }

    // parcourir tout le graph
    depthSearchFix(r,plc,id,id);
}

int maxZ(const PlaneCloud &plc)
{
    int r = 0;
    float max = plc.planes.at(0)->center->z;

    for(int i = 1; i < plc.planes.size(); ++i)
    {
        if(max < plc.planes.at(i)->center->z)
        {
            max = plc.planes.at(i)->center->z;
            r = i;
        }
    }

    return r;
}

void flip(sVector3 &v)
{
    *(v) *= -1;
}

void depthSearchFix(const std::vector<Kruskal::KEdge> &r, const PlaneCloud &plc, int current, int from)
{
    // std::cout << from << " " << current << std::endl;
    std::vector<int> nbh;

    nbh = getNBH(r,current);

    // fix les voisins (pas from)
    for(int i = 0; i < nbh.size();i++)
    {
        int id = nbh.at(i);
        if(Vector3::dot(*(plc.planes.at(current)->normal),*(plc.planes.at(id)->normal)) < 0)
            flip(plc.planes.at(id)->normal);
    }

    // appel les voisins.
    for(int i = 0; i < nbh.size();i++)
    {
        if(nbh.at(i) != from)
            depthSearchFix(r,plc,nbh.at(i),current);
    }

}

std::vector<int> getNBH(const std::vector<Kruskal::KEdge> &r, int current)
{
    std::vector<int> nbh;

    for(int i = 0; i < r.size();i++)
    {
        if(r.at(i).dst == current)
            nbh.push_back(r.at(i).src);

        if(r.at(i).src == current)
            nbh.push_back(r.at(i).dst);
    }

    return nbh;
}
