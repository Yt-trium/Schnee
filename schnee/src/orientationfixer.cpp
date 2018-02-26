#include "orientationfixer.h"
#include <ctime>

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

    size_t e = 0, id;
    float w;

	int start_graph = clock();
    for(size_t i = 0; i < plc.planes.size(); ++i)
    {
		const sPlane & plane = plc.planes.at(i);
        kd_query[0] = plane->center->x;
        kd_query[1] = plane->center->y;
        kd_query[2] = plane->center->z;

        nbhd_count = index.knnSearch(&kd_query[0], num_results, &ret_index[0], &out_squared_dist[0]);

        assert(nbhd_count == num_results);

        sVector3 n1 = plc.planes.at(i)->normal;
		assert(i >= 0);

        for(size_t j = 0; j <= k; ++j)
        {
			if(ret_index[j] == i) continue;
            sVector3 n2 = plc.planes.at(ret_index[j])->normal;

            w = 1.0f - abs(Vector3::dot((*n1),(*n2)));

			assert(ret_index[j] >= 0);
            if(kruskal.addEdge(i,ret_index[j],w))
                e++;
        }
    }
	int end_graph = clock();

    kruskal.setEdges(e);

	int start_mst = clock();
    r = kruskal.MST();
	int end_mst = clock();

    // search for the max z plane
    id = maxZ(plc);

	int start_traversal = clock();
    if(plc.planes.at(id)->normal->z < 0)
    {
        // *(plc.planes[id]->normal) *= -1;
        flip (plc.planes[id]->normal);
    }

    // parcourir tout le graph
    depthSearchFix(r,plc,id,id);
	int end_traversal = clock();

	std::cout << "\nNORMAL FIXER EXECUTION TIMES:\n";
	std::cout << "BUILD GRAPH: \t" << (end_graph-start_graph)/double(CLOCKS_PER_SEC) << " s" << std::endl;
	std::cout << "BUILD MST: \t" << (end_mst-start_mst)/double(CLOCKS_PER_SEC) << " s" << std::endl;
	std::cout << "TRAVERSE GRAPH: \t" << (end_traversal-start_traversal)/double(CLOCKS_PER_SEC) << " s" << std::endl;
	std::cout << "\n";
}

size_t maxZ(const PlaneCloud &plc)
{
    size_t r = 0;
    float max = plc.planes.at(0)->center->z;

    for(size_t i = 1; i < plc.planes.size(); ++i)
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

void depthSearchFix(const std::vector<Kruskal::KEdge> &r, const PlaneCloud &plc, size_t current, size_t from)
{
    // std::cout << from << " " << current << std::endl;
    std::vector<size_t> nbh;

    nbh = getNBH(r,current);

    // fix les voisins (pas from)
    for(size_t i = 0; i < nbh.size();i++)
    {
        size_t id = nbh.at(i);
        if(Vector3::dot(*(plc.planes.at(current)->normal),*(plc.planes.at(id)->normal)) < 0)
            flip(plc.planes.at(id)->normal);
    }

    // appel les voisins.
    for(size_t i = 0; i < nbh.size();i++)
    {
        if(nbh.at(i) != from)
            depthSearchFix(r,plc,nbh.at(i),current);
    }

}

std::vector<size_t> getNBH(const std::vector<Kruskal::KEdge> &r, size_t current)
{
    std::vector<size_t> nbh;

    for(size_t i = 0; i < r.size();i++)
    {
        if(r.at(i).dst == current)
            nbh.push_back(r.at(i).src);

        if(r.at(i).src == current)
            nbh.push_back(r.at(i).dst);
    }

    return nbh;
}
