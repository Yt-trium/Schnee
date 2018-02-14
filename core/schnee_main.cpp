#include "vector.h"
#include "plane.h"
#include "file_loader.h"
#include "file_saver.h"
#include "cloud.h"

#include <iostream>
#include <string>
#include <vector>
#include <cassert>
#include <cmath>

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
	if(argc < 5)
	{
		std::cout << "USAGE:\n";
		std::cout << argv[0] << " off_input_file off_out_file k cell_size" << std::endl;
		exit(2);
	}

	// In off file
	std::string pin = argv[1];
	std::string pout = argv[2];
	int k = std::stoi(argv[3]);
	float cell_size = std::stof(argv[4]);
	assert(k > 1);
	assert(cell_size > 0.0f);
	std::cout << "IN FILE: " << pin << "\n";
	std::cout << "OUT FILE: " << pout << "\n";
	std::cout << "K: " << k << std::endl;
	std::cout << "CELL SIZE: " << cell_size << std::endl;

	// Create empty point cloud
	PointCloud pc;

	// Get points
	if(!FL_OFF_load_points(pin, pc.points))
		exit(3);

	// Buil planes
	std::vector<sPlane> planes;
	PTC_build_planes(pc, planes, k);

	// Fix planes orientation
	PlaneCloud plc;
	plc.planes = planes;
	plane_cloud_index index(3, plc, nanoflann::KDTreeSingleIndexAdaptorParams(10));
	index.buildIndex();

	// Get bounding box
	Vector3 bbox_min;
	Vector3 bbox_max;
	PLC_get_bounds(plc,
	               bbox_min.x, bbox_min.y, bbox_min.z,
	               bbox_max.x, bbox_max.y, bbox_max.z);
	std::cout << "BBOX MIN " << bbox_min << "\n";
	std::cout << "BBOX MAX " << bbox_max << "\n";

	// Calculate "cells"
	float nb_cell_x = std::ceil((bbox_max.x - bbox_min.x ) / cell_size + 1);
	float nb_cell_y = std::ceil((bbox_max.y - bbox_min.y ) / cell_size + 1);
	float nb_cell_z = std::ceil((bbox_max.z - bbox_min.z ) / cell_size + 1);
	std::cout << "NBCELLS : " << nb_cell_x << " / " << nb_cell_y << " / " << nb_cell_z << "\n";

	// Debug "cells"
	std::vector<sVector3> corners(nb_cell_x * nb_cell_y * nb_cell_z);
	Vector3 cell_center;
	assert(bbox_min.x + cell_size * nb_cell_x >= bbox_max.x);
	assert(bbox_min.y + cell_size * nb_cell_y >= bbox_max.y);
	assert(bbox_min.z + cell_size * nb_cell_z >= bbox_max.z);
	std::cout << "TOTAL NB CELLS: " << corners.size() << "\n";
	std::cout << "TOTAL NB CELLS2: " <<
	             (nb_cell_z - 1) * (nb_cell_x * nb_cell_y) + (nb_cell_y - 1) * nb_cell_x + nb_cell_x << "\n";
	std::cout << std::endl;
	assert(corners.size() ==
	             (nb_cell_z - 1) * (nb_cell_x * nb_cell_y) + (nb_cell_y - 1) * nb_cell_x + nb_cell_x);

	for(float z = 0; z < nb_cell_z; z+=1)
	{
        cell_center.z = bbox_min.z + z * cell_size;
        for(float y = 0; y < nb_cell_y; y+=1)
        {
            cell_center.y = bbox_min.y + y * cell_size;
            for(float x = 0; x < nb_cell_x; x+=1)
            {
                cell_center.x = bbox_min.x + x * cell_size;
                corners[z * (nb_cell_x * nb_cell_y) + y * nb_cell_x + x] = std::make_shared<Vector3>(cell_center);
            }
        }
	}

	// Calculate signed distance function
	//std::vector<float> signedFunctions()
	std::vector<float> distances;
	PLC_compute_signed_distances(plc, index, distances, bbox_min, bbox_max, cell_size, nb_cell_x, nb_cell_y, nb_cell_z);

	// Debug
	std::vector<sVector3> origins;
	for(int i = 0; i < planes.size(); i++)
	{
		origins.push_back(planes[i]->center);
	}

    FS_OFF_save_points("/tmp/out.cells.corners.off", corners);
	FS_OFF_save_points("/tmp/out.plane.centers.off", origins);
	FS_OFF_save_planes("/tmp/out.planes.faces.off", planes, 0.05f);
	FS_OFF_save_planes_normals("/tmp/out.planes.normals.off", planes, 5, 0.06f);
	FS_OFF_save_grid_distances("/tmp/out.grid.distances.off", corners, distances);

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
