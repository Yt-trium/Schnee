#include "vector.h"
#include "plane.h"
#include "file_loader.h"
#include "file_saver.h"
#include "point_cloud.h"

#include <iostream>
#include <string>
#include <vector>
#include <cassert>

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


int main(int argc, const char * argv[])
{
	if(argc < 4)
	{
		std::cout << "USAGE:\n";
		std::cout << argv[0] << " off_input_file off_out_file k" << std::endl;
		exit(2);
	}

	// In off file
	std::string pin = argv[1];
	std::string pout = argv[2];
	int k = std::stoi(argv[3]);
	assert(k > 1);
	std::cout << "IN FILE: " << pin << "\n";
	std::cout << "OUT FILE: " << pout << "\n";
	std::cout << "K: " << k << std::endl;

	// Create empty point cloud
	PointCloud pc;

	// Get points
	if(!FL_OFF_load_points(pin, pc.points))
		exit(3);

	std::vector<sPlane> planes;
	PC_build_planes(pc, planes, k);

	std::vector<sVector3> origins;
	for(int i = 0; i < planes.size(); i++)
	{
		origins.push_back(planes[i]->center);
	}

	//FS_OFF_save_points(pout, origins);
	FS_OFF_save_planes(pout, planes, 0.05f);

	return 0;
}
	// Search closest to a point
	/*
    const size_t num_results = k;
	size_t ret_index[k];
	float out_squared_dist[k];
	nanoflann::KNNResultSet<float> result_set(num_results);
	result_set.init(ret_index, out_squared_dist);
	result_set.
	index.findNeighbors(result_set, &kd_query[0], nanoflann::SearchParams());
	std::cout << "knnSearch(nn=" << num_results << "):\n";
	for(size_t i = 0; i < result_set.size(); i++)
	{
		std::cout << "idx[" << i << "]=" << ret_index[i] << " dist=" << out_squared_dist[i] << std::endl;

	}
	*/
	/*
	// Search closests to a point
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
	*/
