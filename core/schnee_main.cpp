#include "vector.h"
#include "plane.h"
#include "file_loader.h"
#include "nanoflann.hpp"

#include <iostream>
#include <string>
#include <vector>

void dump_mem_usage()
{
	FILE* f = fopen("/proc/self/statm","rt");
	if (!f) return;
	char str[300];
	size_t n = fread(str, 1, 200, f);
	str[n] = 0;
	printf("MEM: %s\n", str);
	fclose(f);
}

struct PointCloud
{
	std::vector<sVector3> points;

	inline size_t kdtree_get_point_count() const { return points.size(); }

	inline float kdtree_get_pt(const size_t idx, int dim) const
	{
		if(dim == 0)
			return points[idx]->x;
		else if(dim == 1)
			return points[idx]->y;
		else
			return points[idx]->z;
	}

	template<class BBOX>
	bool kdtree_get_bbox(BBOX&) const { return false; }
};

int main(int argc, const char * argv[])
{
	if(argc < 3)
	{
		std::cout << argv[0] << " off_input_file k" << std::endl;
		exit(2);
	}
	dump_mem_usage();

	// In off file
	std::string pin = argv[1];
	int k = std::stoi(argv[2]);
	std::cout << "IN FILE: " << pin << "\n";
	std::cout << "K: " << k << std::endl;

	// Create empty point cloud
	PointCloud pc;

	// Get points
	if(!FL_OFF_load_points(pin, pc.points))
	{
		pc.points.clear();
		exit(3);
	}

	// Create nearest neighbour tree
	typedef nanoflann::KDTreeSingleIndexAdaptor<
	        nanoflann::L2_Simple_Adaptor<float, PointCloud>,
	        PointCloud,
	        3
	        > sc_kd_tree;

	sc_kd_tree index(3, pc,
	                 nanoflann::KDTreeSingleIndexAdaptorParams(10));
	dump_mem_usage();
	index.buildIndex();
	dump_mem_usage();

	std::vector<sPlane> planes;
	float kd_query[3] = {0, 0, 0};
	kd_query[0] = pc.points[0]->x+0.0001f;
	kd_query[1] = pc.points[0]->y;
	kd_query[2] = pc.points[0]->z;
	// Process tangents
	/*
    const size_t num_results = k;
	size_t ret_index;
	float out_squared_dist;
	nanoflann::KNNResultSet<float> result_set(num_results);
	result_set.init(&ret_index, &out_squared_dist);
	index.findNeighbors(result_set, &kd_query[0], nanoflann::SearchParams(10));
	std::cout << "knnSearch(nn=" << num_results << "):\n";
	std::cout << "ret_index=" << ret_index << " out_squared_dist=" << out_squared_dist << std::endl;
	*/
	size_t num_results = k;
    std::vector<size_t>   ret_index(num_results);
    std::vector<float> out_dist_sqr(num_results);

    num_results = index.knnSearch(&kd_query[0], num_results, &ret_index[0], &out_dist_sqr[0]);

    // In case of less points in the tree than requested:
    ret_index.resize(num_results);
    out_dist_sqr.resize(num_results);

    std::cout << "knnSearch(): num_results=" << num_results << "\n";
    for (size_t i = 0; i < num_results; i++)
        std::cout << "idx["<< i << "]=" << ret_index[i] << " dist["<< i << "]=" << out_dist_sqr[i] << std::endl;
	std::cout << "\n";
	for(int i = 0; i < pc.points.size(); ++i)
	{
		// Find k nearest points
		// UNIMPLEMENTED
		sPlane plane = std::make_shared<Plane>();
		plane->center = pc.points[i];
		plane->normal = std::make_shared<Vector3>(Vector3::zup());
		planes.push_back(plane);
	}



	return 0;
}
